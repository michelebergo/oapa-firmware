/**
 * OAPA reference firmware - FYSETC E4 (ESP32 + dual TMC2209)
 *
 * Implements the OAPA wire protocol for the NINA Three Point Polar Alignment
 * plugin. Protocol home and documentation:
 *   https://github.com/michelebergo/oapa-firmware
 *
 * Wire discipline (fixed by the plugin, do not change):
 *   - "?"  -> exactly two lines: status frame, then "ok"
 *   - any other non-empty command -> exactly one reply line
 *   - empty input -> no reply
 *   - commands are newline-terminated text at 115200 baud
 *
 * 1.2.1: the F feed value in $J= jogs now sets the max speed for that move
 * (clamped, steps/s; absent -> default 2000), and "!" decelerates both axes
 * to a stop.
 *
 * Status frame format (verbatim):
 *   <Status|MPos:x.xx,y.yy,0.00|V:FW_VERSION|>
 * where Status is Idle, Run or Home. The plugin polls "?" every ~300 ms and
 * detects motion completion by watching MPos converge - positions reported
 * here must always be the stepper's real position, never a cached value.
 *
 * Axis convention: X = altitude, Y = azimuth. Endstops are optional per
 * axis (see the endstop section): an axis with one homes against it on $H,
 * an axis without one zeroes in place. Gear ratios and backlash are NOT
 * configured here: the plugin's on-sky self-calibration measures them,
 * whatever the mechanics.
 */

#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>

// Reported in the status frame (V: field) so the plugin can detect outdated
// firmware. Bump on every protocol-visible change.
#define FW_VERSION "1.2.2"

// ---------------------------------------------------------------------------
// Board wiring (FYSETC E4 v1.3)
// ---------------------------------------------------------------------------

#define ENABLE_PIN 25 // one enable line drives both TMC2209s

#define X_STEP_PIN 27
#define X_DIR_PIN 26
#define Y_STEP_PIN 33
#define Y_DIR_PIN 32

// Endstops - optional, per axis, all DISABLED by default: without a switch
// an axis simply zeroes in place on $H. The FYSETC E4 exposes X-min (GPIO34)
// and Y-min (GPIO35) if your build has reference switches.
//
// ESP32 hardware note: GPIO34-39 are input-only pins WITHOUT internal pull
// resistors - INPUT_PULLUP silently does nothing there. Before enabling an
// endstop, wire an external pull-up (switch to GND, INVERT=true) or
// pull-down (switch to 3V3, INVERT=false); a floating pin false-triggers.
#define X_ENDSTOP_ENABLED false
#define X_ENDSTOP_PIN 34
#define X_ENDSTOP_INVERT false  // true for a normally-closed switch
#define X_HOMING_DIR -1         // sign of motion toward the switch

#define Y_ENDSTOP_ENABLED false // enable if your azimuth has a reference switch
#define Y_ENDSTOP_PIN 35
#define Y_ENDSTOP_INVERT false
#define Y_HOMING_DIR -1

// Soft-limit guard during normal moves (outside $H): stops an axis that is
// driving into its triggered endstop. Off by default - enable only after the
// external pull resistor is verified, or a floating pin will halt real moves.
#define ENDSTOP_GUARD_ENABLED false

// Both TMC2209s share one UART (half-duplex on GPIO15), addressed 1 and 3.
#define DRIVER_SERIAL Serial1
#define DRIVER_UART_RX 15
#define DRIVER_UART_TX 15
#define R_SENSE 0.11f
#define X_DRIVER_ADDR 1
#define Y_DRIVER_ADDR 3

// ---------------------------------------------------------------------------
// Axes - each axis owns its driver, its stepper and its electrical settings.
// The protocol layer only ever talks to an Axis through the helpers below.
// ---------------------------------------------------------------------------

TMC2209Stepper tmcX(&DRIVER_SERIAL, R_SENSE, X_DRIVER_ADDR);
TMC2209Stepper tmcY(&DRIVER_SERIAL, R_SENSE, Y_DRIVER_ADDR);
AccelStepper stepX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

struct Axis {
  const char *name;
  TMC2209Stepper &driver;
  AccelStepper &stepper;
  int runCurrent_mA;
  float holdMultiplier;
  int microsteps;
  // endstop (optional)
  bool endstopEnabled;
  int endstopPin;
  bool endstopInvert;
  int homingDirection;
};

// Hold defaults to 25% (was 50% before 1.2.2): the hold current flows in the
// coils continuously from power-on - including the whole window before the
// plugin connects and pushes the user's values - and heat goes with I^2, so a
// polar-alignment platform (usually self-locking mechanics) is better served
// by a cool motor than by holding torque it rarely needs.
Axis xAxis = {"altitude", tmcX, stepX, 600, 0.25f, 16,
              X_ENDSTOP_ENABLED, X_ENDSTOP_PIN, X_ENDSTOP_INVERT, X_HOMING_DIR};
Axis yAxis = {"azimuth", tmcY, stepY, 600, 0.25f, 16,
              Y_ENDSTOP_ENABLED, Y_ENDSTOP_PIN, Y_ENDSTOP_INVERT, Y_HOMING_DIR};

// Returns nullptr for anything that is not an axis letter.
Axis *axisByLetter(char letter) {
  if (letter == 'X' || letter == 'x') return &xAxis;
  if (letter == 'Y' || letter == 'y') return &yAxis;
  return nullptr;
}

void applyDriverCurrent(Axis &axis) {
  axis.driver.rms_current(axis.runCurrent_mA, axis.holdMultiplier);
}

// ---------------------------------------------------------------------------
// Machine state
// ---------------------------------------------------------------------------

bool homed = false;
bool homingInProgress = false;
String lineBuffer = ""; // partial command, filled one char per loop() pass

const int HOMING_SPEED = 800;   // steps/s toward the endstop
const int HOMING_BACKOFF = 50;  // steps to retreat after triggering
// Safety net for misconfiguration: if an enabled endstop is never seen
// within this travel (e.g. switch not actually wired on a two-motor-only
// build), homing gives up and zeroes in place instead of seeking forever.
//
// Sizing: 200000 steps = ~62 motor revolutions at 16 microsteps (~4 min at
// HOMING_SPEED). How much platform travel that is depends on your gear
// reduction: plenty for low ratios (~15 steps/arcmin -> hundreds of
// degrees), but only ~3.4 deg at extreme reductions (~970 steps/arcmin).
// If your switch sits farther than that, raise this limit accordingly.
const long HOMING_MAX_TRAVEL = 200000;

bool endstopTriggered(const Axis &axis) {
  if (!axis.endstopEnabled) return false;
  return digitalRead(axis.endstopPin) == (axis.endstopInvert ? LOW : HIGH);
}

// ---------------------------------------------------------------------------
// Status frame
// ---------------------------------------------------------------------------

// "?" is the plugin's heartbeat: discovery probe during the COM scan and
// completion polling during moves. Two lines out, always.
void handleStatusQuery() {
  const char *status = "Idle";
  if (homingInProgress) {
    status = "Home";
  } else if (xAxis.stepper.isRunning() || yAxis.stepper.isRunning()) {
    status = "Run";
  }

  Serial.print("<");
  Serial.print(status);
  Serial.print("|MPos:");
  Serial.print((float)xAxis.stepper.currentPosition(), 2);
  Serial.print(",");
  Serial.print((float)yAxis.stepper.currentPosition(), 2);
  Serial.print(",0.00|V:");
  Serial.print(FW_VERSION);
  Serial.println("|>");
  Serial.println("ok");
}

// ---------------------------------------------------------------------------
// Motion commands
// ---------------------------------------------------------------------------

// Extracts the signed number following `letter` in a jog spec, e.g. "X-42.5"
// out of "G91G21X-42.5F800". Returns false when the letter is absent.
bool readAxisValue(const String &spec, char letter, float &value) {
  int at = spec.indexOf(letter);
  if (at < 0) return false;
  int end = at + 1;
  while (end < (int)spec.length()) {
    char c = spec.charAt(end);
    if (!isdigit(c) && c != '.' && c != '-') break;
    end++;
  }
  value = spec.substring(at + 1, end).toFloat();
  return true;
}

// Motion profile bounds (steps/s). The F feed value was ignored before 1.2.1;
// now it sets the max speed for that jog. The ceiling keeps the step rate
// within what loop()-driven AccelStepper can generate reliably on the ESP32
// while also servicing serial I/O; the floor keeps a typo from freezing an
// axis at a glacial rate. F absent or out of grammar -> DEFAULT_MAX_SPEED,
// exactly the pre-1.2.1 behavior. Acceleration stays fixed.
const float DEFAULT_MAX_SPEED = 2000;
const float JOG_SPEED_MIN = 50;
const float JOG_SPEED_MAX = 3000;

float jogSpeedFrom(const String &spec) {
  float feed;
  if (!readAxisValue(spec, 'F', feed) || feed <= 0) return DEFAULT_MAX_SPEED;
  return constrain(feed, JOG_SPEED_MIN, JOG_SPEED_MAX);
}

// $J=G91G21X<n>F<f> (relative) / $J=G53X<n>F<f> (absolute). Targets are
// rounded with lround() - truncating would lose up to 0.99 steps per command,
// which accumulates into real drift at high gear ratios.
String handleJog(const String &spec) {
  bool relative = spec.indexOf("G91") >= 0;
  bool absolute = spec.indexOf("G53") >= 0;
  if (!relative && !absolute) return "ok";

  float speed = jogSpeedFrom(spec);
  float value;
  if (readAxisValue(spec, 'X', value)) {
    xAxis.stepper.setMaxSpeed(speed);
    if (relative) xAxis.stepper.move(lround(value));
    else xAxis.stepper.moveTo(lround(value));
  }
  if (readAxisValue(spec, 'Y', value)) {
    yAxis.stepper.setMaxSpeed(speed);
    if (relative) yAxis.stepper.move(lround(value));
    else yAxis.stepper.moveTo(lround(value));
  }
  return "ok";
}

// Bare "X800" / "Y-200": relative move in whole steps. Carries no feed value,
// so the profile is reset to the default rather than inheriting whatever F the
// previous jog happened to use.
String handleDirectMove(Axis &axis, const String &command) {
  axis.stepper.setMaxSpeed(DEFAULT_MAX_SPEED);
  axis.stepper.move(command.substring(1).toInt());
  return "ok";
}

// ---------------------------------------------------------------------------
// Driver configuration (type-first grammar: C=run current mA, H=hold percent,
// S=microsteps; second char selects the axis, e.g. CX600, HY50, SX16).
// Deprecated in the protocol spec - kept for wire compatibility.
// ---------------------------------------------------------------------------

String handleDriverConfig(char type, const String &command) {
  Axis *axis = axisByLetter(command.charAt(1));
  if (axis == nullptr) axis = &yAxis; // historical fallback, kept as-is
  int value = command.substring(2).toInt();

  if (type == 'C' || type == 'c') {
    axis->runCurrent_mA = value;
    applyDriverCurrent(*axis);
  } else if (type == 'H' || type == 'h') {
    axis->holdMultiplier = value / 100.0f;
    applyDriverCurrent(*axis);
  } else if (type == 'S' || type == 's') {
    axis->driver.microsteps(value);
    axis->microsteps = value;
  }
  return "ok";
}

// ---------------------------------------------------------------------------
// Homing ($H) - per-axis: an axis with an endstop seeks the switch, backs
// off and zeroes there; an axis without one zeroes in place. Blocking on
// purpose: the plugin never issues $H during an alignment; this exists for
// bench setup from a terminal.
// ---------------------------------------------------------------------------

void homeAxis(Axis &axis) {
  if (!axis.endstopEnabled) {
    Serial.print("Homing: ");
    Serial.print(axis.name);
    Serial.println(" has no endstop, zeroed in place");
    axis.stepper.setCurrentPosition(0);
    return;
  }

  Serial.print("Homing: ");
  Serial.print(axis.name);
  Serial.println(" toward endstop...");
  long start = axis.stepper.currentPosition();
  axis.stepper.setSpeed(axis.homingDirection * HOMING_SPEED);
  while (!endstopTriggered(axis)) {
    axis.stepper.runSpeed();
    if (labs(axis.stepper.currentPosition() - start) > HOMING_MAX_TRAVEL) {
      axis.stepper.stop();
      Serial.print("Homing: ");
      Serial.print(axis.name);
      Serial.println(" endstop not found within travel limit - check wiring/config, zeroed in place");
      axis.stepper.setCurrentPosition(0);
      return;
    }
  }
  axis.stepper.stop();
  delay(100);

  axis.stepper.move(-axis.homingDirection * HOMING_BACKOFF);
  while (axis.stepper.distanceToGo() != 0) {
    axis.stepper.run();
  }
  axis.stepper.setCurrentPosition(0);
  Serial.print("Homing: ");
  Serial.print(axis.name);
  Serial.println(" zeroed at endstop");
}

void handleHoming() {
  homingInProgress = true;
  homeAxis(xAxis);
  homeAxis(yAxis);
  homed = true;
  homingInProgress = false;
  Serial.println("Homing complete");
  Serial.println("ok");
}

// ---------------------------------------------------------------------------
// Command dispatch - one line in, reply lines out (see wire discipline above).
// Handlers that write their own reply lines return "" so dispatch stays quiet.
// ---------------------------------------------------------------------------

String dispatchCommand(String input) {
  input.trim();
  if (input.length() == 0) return "";

  if (input.charAt(0) == '?') {
    handleStatusQuery();
    return "";
  }
  // "!" - stop: decelerate both axes to a halt (AccelStepper::stop keeps the
  // position counter true, so MPos stays honest). New in 1.2.1; the plugin's
  // STOP button sends this. Not reachable during $H (homing is blocking).
  if (input.charAt(0) == '!') {
    xAxis.stepper.stop();
    yAxis.stepper.stop();
    return "ok";
  }
  if (input.startsWith("$H")) {
    handleHoming();
    return "";
  }
  if (input.startsWith("$J=")) {
    return handleJog(input.substring(3));
  }
  if (input.length() < 2) return "error";

  char first = input.charAt(0);
  Axis *axis = axisByLetter(first);
  char second = input.charAt(1);
  if (axis != nullptr && (isdigit(second) || second == '-')) {
    return handleDirectMove(*axis, input);
  }
  if (input.length() > 2 &&
      (first == 'C' || first == 'c' || first == 'H' || first == 'h' ||
       first == 'S' || first == 's')) {
    return handleDriverConfig(first, input);
  }

  // Unknown command: acknowledge and do nothing - never silence, never a
  // crash. The plugin must never be left waiting for a reply.
  return "ok";
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  DRIVER_SERIAL.begin(115200, SERIAL_8N1, DRIVER_UART_RX, DRIVER_UART_TX);

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  // Plain INPUT: GPIO34/35 have no internal pulls (see endstop note above) -
  // the external pull resistor defines the idle level.
  for (Axis *axis : {&xAxis, &yAxis}) {
    if (axis->endstopEnabled) pinMode(axis->endstopPin, INPUT);
  }

  for (Axis *axis : {&xAxis, &yAxis}) {
    axis->driver.begin();
    axis->driver.toff(5);
    axis->driver.microsteps(axis->microsteps);
    axis->driver.pwm_autoscale(true);
    applyDriverCurrent(*axis);
    axis->stepper.setMaxSpeed(2000);
    axis->stepper.setAcceleration(1000);
  }

  // Boot banner. The plugin discards and retries past this (it clears the
  // input buffer before probing), but a human on a terminal gets oriented.
  Serial.println("\n--- OAPA controller ready ---");
  Serial.print("firmware ");
  Serial.print(FW_VERSION);
  Serial.println(" | protocol: github.com/michelebergo/oapa-firmware");
  Serial.println("$H homes axes with an endstop, zeroes the others in place");
  Serial.println("Waiting for commands...");
}

void loop() {
  // Soft-limit guard (opt-in, see ENDSTOP_GUARD_ENABLED): stop an axis that
  // is moving toward its triggered endstop. Requires verified wiring with an
  // external pull resistor - a floating GPIO34/35 would false-trigger.
#if ENDSTOP_GUARD_ENABLED
  if (!homingInProgress) {
    for (Axis *axis : {&xAxis, &yAxis}) {
      if (endstopTriggered(*axis) &&
          axis->stepper.speed() * axis->homingDirection > 0) {
        axis->stepper.stop();
      }
    }
  }
#endif

  // AccelStepper generates steps from run(): the protocol layer must never
  // starve it. That is why serial input is consumed one character per pass
  // instead of blocking on a full line.
  xAxis.stepper.run();
  yAxis.stepper.run();

  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineBuffer.length() > 0) {
        String reply = dispatchCommand(lineBuffer);
        if (reply.length() > 0) Serial.println(reply);
        lineBuffer = "";
      }
    } else {
      lineBuffer += c;
    }
  }
}
