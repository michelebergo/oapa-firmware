/**
 * OAPA reference firmware - FYSETC E4 (ESP32 + dual TMC2209), or any ESP32
 * with plain STEP/DIR drivers (see DRIVER_TMC2209 below)
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
 * Axis convention: X = azimuth, Y = altitude (as the N.I.N.A. plugin uses them). Endstops are optional per
 * axis (see the endstop section): an axis with one homes against it on $H,
 * an axis without one zeroes in place. The board measures its own steps per
 * arcminute and backlash (calibration) and runs the alignment itself on the
 * polar error it is given (by N.I.N.A.'s TPPA plugin or by ASIAIR).
 *
 * 1.3.0: the board runs the alignment. Error sources: N.I.N.A. over USB or
 * WiFi (TCP 2323, bridge commands $E= $F= $A= $B= $T= $M= $C= $K? $L?) and
 * ASIAIR's polar alignment; automatic calibration of factors and backlash;
 * phone web page; firmware update over WiFi. The 1.2.3 protocol region
 * (src/oapa_protocol.inc) is unchanged, STEP/DIR driver builds included.
 */

#include <Arduino.h>
#include <AccelStepper.h>
#include <WiFi.h>

// Driver family. 1 (default) = TMC2209 in UART mode as on the FYSETC E4: run
// current, hold current and microsteps are set by the host over the serial
// link. 0 = plain STEP/DIR drivers (A4988, DRV8825, LV8729 and the like): the
// TMCStepper library is not needed, current comes from the Vref trimmer and
// microsteps from the MS jumpers on the driver, and the C/H/S commands are
// still acknowledged with "ok" but have no effect. Motion is identical: it
// goes through STEP/DIR pulses either way.
#ifndef DRIVER_TMC2209
#define DRIVER_TMC2209 1
#endif

#if DRIVER_TMC2209
#include <TMCStepper.h>
#endif

// Reported in the status frame (V: field). Must stay purely numeric: the
// plugin's status regex rejects any suffix.
#define FW_VERSION "1.3.0"

// Verbatim 1.2.3 protocol region - guarded by tools/check_verbatim.py.
#include "oapa_protocol.inc"
#include "nina_bridge.h"

#include "web_bridge.h"
#include "web_dispatch.inc"
#include "status_json.h"
#include "shared_state.h"
#include "net.h"
#include "web_server.h"
#include "loop_service.h"
#include "device_config.h"
#include "device_state.h"
#include "asiair_client.h"

// ---------------------------------------------------------------------------
// 1.3.0 web side, serviced from loop() on core 1
// ---------------------------------------------------------------------------

bool anySerialByte = false;       // N.I.N.A. (or a terminal) has spoken since boot
unsigned long lastSerialMs = 0;   // millis() of the last received serial byte
unsigned long lastServiceMs = 0;
unsigned long lastPublishMs = 0;
const unsigned long PUBLISH_EVERY_MS = 20;
unsigned long lastLoopTickMs = 0;
bool appliedInvert[2] = {false, false};

// The saved motor direction reaches an axis only while it stands still: flipping DIR
// mid-move would turn the motor back on the spot.
void applyMotorDirection() {
  Axis *axes[2] = {&xAxis, &yAxis};
  for (int i = 0; i < 2; ++i) {
    bool wanted = device::motorInverted(i == 0 ? 'X' : 'Y');
    if (wanted == appliedInvert[i] || axes[i]->stepper.isRunning()) continue;
    axes[i]->stepper.setPinsInverted(wanted, false, false);
    appliedInvert[i] = wanted;
  }
}
const unsigned long LOOP_TICK_MS = 50;
bool lastNinaActive = false;
bool storedDriversApplied = false;

// At most once per millisecond, so the step loop stays as tight as in 1.2.2.
void serviceWebSide() {
  unsigned long now = millis();
  if (now == lastServiceMs) return;
  lastServiceMs = now;

  if (shared::takeStopRequest()) runWebCommand("!");

  char cmd[web::kCommandLen];
  if (shared::popCommand(cmd)) {
    // Re-checked here: N.I.N.A. may have taken over after the page queued it.
    if (!web::ninaActive(anySerialByte, now, lastSerialMs)) {
      if (web::isDriverCommand(cmd)) {
        runWebDriverCommand(cmd);
      } else {
        runWebCommand(cmd);
      }
    }
  }

  bool nina = web::ninaActive(anySerialByte, now, lastSerialMs);
  if (nina != lastNinaActive) {
    lastNinaActive = nina;
    device::logEvent("pc", nina ? "A PC is controlling OAPA" : "The PC released OAPA");
  }
  if (!storedDriversApplied && !nina) {
    storedDriversApplied = true;
    if (device::driversStored()) {
      char commands[3][12];
      for (char axis : {'X', 'Y'}) {
        if (devset::driverCommands(axis, device::driverConfig(axis), commands)) {
          for (const char *command : {commands[0], commands[1], commands[2]}) runWebDriverCommand(command);
        }
      }
      device::logEvent("drivers", "saved driver settings applied");
    }
  }

  if (now - lastLoopTickMs >= LOOP_TICK_MS) {
    lastLoopTickMs = now;
    applyMotorDirection();
    const char *loopCommand = loopservice::tick(now, web::ninaActive(anySerialByte, now, lastSerialMs),
                                                xAxis.stepper.currentPosition(), yAxis.stepper.currentPosition(),
                                                xAxis.stepper.isRunning(), yAxis.stepper.isRunning());
    if (loopCommand) runWebCommand(loopCommand);
  }

  if (now - lastPublishMs >= PUBLISH_EVERY_MS) {
    lastPublishMs = now;
    StatusSnapshot s;
    s.x = xAxis.stepper.currentPosition();
    s.y = yAxis.stepper.currentPosition();
    s.running = xAxis.stepper.isRunning() || yAxis.stepper.isRunning();
    s.homing = homingInProgress;
    s.ninaActive = web::ninaActive(anySerialByte, now, lastSerialMs);
    shared::publish(s);
    devset::DriverConfig activeX, activeY;
    activeX.runMa = xAxis.runCurrent_mA;
    activeX.holdPct = static_cast<int>(lround(xAxis.holdMultiplier * 100));
    activeX.microsteps = xAxis.microsteps;
    activeY.runMa = yAxis.runCurrent_mA;
    activeY.holdPct = static_cast<int>(lround(yAxis.holdMultiplier * 100));
    activeY.microsteps = yAxis.microsteps;
    device::publishActiveDrivers(activeX, activeY);
  }
}

// Serial lines from N.I.N.A.: the bridge commands first (nina_bridge.h), then
// the 1.2.2 dispatcher, untouched. A jog or STOP from N.I.N.A. ends a run that
// N.I.N.A. is feeding: the user took the axes back.
String dispatchSerialLine(String line) {
  line.trim();
  ninabridge::Command command = ninabridge::parse(line.c_str());
  switch (command.kind) {
    case ninabridge::Kind::None:
      break;
    case ninabridge::Kind::Reading:
      loopservice::ninaReading(command.a, command.b, millis());
      return "ok";
    case ninabridge::Kind::Factors:
      loopservice::ninaFactors(command.a, command.b);
      return "ok";
    case ninabridge::Kind::CalibrateStart:
      loopservice::ninaCalibrateStart(millis());
      return "ok";
    case ninabridge::Kind::CalibrateOnly:
      loopservice::ninaCalibrateStart(millis(), false);
      return "ok";
    case ninabridge::Kind::CalibrateStop:
      loopservice::ninaCalibrateStop();
      return "ok";
    case ninabridge::Kind::CalibrationQuery: {
      char status[400];
      loopservice::ninaCalibrationStatus(status, sizeof status);
      return String(status);
    }
    case ninabridge::Kind::Tolerance:
      loopservice::ninaTolerance(command.a);
      return "ok";
    case ninabridge::Kind::MoveCap:
      loopservice::ninaMoveCap(command.a);
      return "ok";
    case ninabridge::Kind::Backlash:
      loopservice::ninaBacklash(command.axis, command.mode, command.a, command.b);
      return "ok";
    case ninabridge::Kind::Start:
      loopservice::ninaStart();
      return "ok";
    case ninabridge::Kind::Stop:
      loopservice::ninaStop();
      return "ok";
    case ninabridge::Kind::StatusQuery: {
      char status[400];
      loopservice::ninaStatus(status, sizeof status);
      return String(status);
    }
    case ninabridge::Kind::Invalid:
      return "error";
  }
  bool directMove = line.length() > 1 && axisByLetter(line.charAt(0)) != nullptr &&
                    (isdigit(line.charAt(1)) || line.charAt(1) == '-');
  if (line.startsWith("$J=") || line.startsWith("!") || directMove) loopservice::ninaUserMotion();
  return dispatchCommand(line);
}

// The same line protocol over WiFi, TCP port 2323, for a plugin that connects
// by address instead of by COM port. One client at a time and the newest
// connection wins: a client that vanished without closing (WiFi drop) must not
// keep the board unreachable. Serviced from loop(), once per millisecond, so
// commands run on the same thread as the USB ones.
const uint16_t NINA_TCP_PORT = 2323;
const size_t NINA_TCP_LINE_MAX = 128;
WiFiServer ninaTcpServer(NINA_TCP_PORT);
WiFiClient ninaTcpClient;
String ninaTcpLine = "";
bool ninaTcpListening = false;
unsigned long lastNinaTcpMs = 0;

// "?" and "$H" print their replies to USB inside the 1.2.2 region, so the TCP
// link answers them itself; everything else takes the USB path.
String dispatchTcpLine(String line) {
  line.trim();
  if (line.startsWith("?")) {
    const char *state = homingInProgress ? "Home"
                        : (xAxis.stepper.isRunning() || yAxis.stepper.isRunning()) ? "Run" : "Idle";
    char frame[96];
    ninabridge::formatStatusFrame(state, xAxis.stepper.currentPosition(), yAxis.stepper.currentPosition(), FW_VERSION,
                                  frame, sizeof frame);
    return String(frame) + "\r\nok";
  }
  if (line.startsWith("$H")) {
    handleHoming();
    return "ok";
  }
  return dispatchSerialLine(line);
}

void serviceNinaTcp() {
  unsigned long now = millis();
  if (now == lastNinaTcpMs) return;
  lastNinaTcpMs = now;

  if (!ninaTcpListening) {
    if (WiFi.status() != WL_CONNECTED) return;
    ninaTcpServer.begin();
    ninaTcpServer.setNoDelay(true);
    ninaTcpListening = true;
  }
  if (ninaTcpServer.hasClient()) {
    WiFiClient incoming = ninaTcpServer.available();
    if (ninaTcpClient) ninaTcpClient.stop();
    ninaTcpClient = incoming;
    ninaTcpClient.setNoDelay(true);
    ninaTcpLine = "";
    device::logEvent("pc", "TCP client %s connected", ninaTcpClient.remoteIP().toString().c_str());
  }
  if (!ninaTcpClient || !ninaTcpClient.connected()) return;

  // Bounded per pass, so a flood cannot starve the steppers.
  for (int budget = 64; budget > 0 && ninaTcpClient.available(); --budget) {
    char c = ninaTcpClient.read();
    anySerialByte = true;  // N.I.N.A. over WiFi counts as N.I.N.A. for the page
    lastSerialMs = millis();
    if (c == '\n' || c == '\r') {
      if (ninaTcpLine.length() > 0) {
        String reply = dispatchTcpLine(ninaTcpLine);
        if (reply.length() > 0) ninaTcpClient.print(reply + "\r\n");
        ninaTcpLine = "";
      }
    } else if (ninaTcpLine.length() < NINA_TCP_LINE_MAX) {
      ninaTcpLine += c;
    }
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  // ESP-IDF components (WiFi, NVS) log to UART0 too; the USB line belongs to
  // N.I.N.A., so they stay silent. Diagnostics live on the web page instead.
  esp_log_level_set("*", ESP_LOG_NONE);
  Serial.begin(115200);
#if DRIVER_TMC2209
  DRIVER_SERIAL.begin(115200, SERIAL_8N1, DRIVER_UART_RX, DRIVER_UART_TX);
#endif

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  // Plain INPUT: GPIO34/35 have no internal pulls (see endstop note above) -
  // the external pull resistor defines the idle level.
  for (Axis *axis : {&xAxis, &yAxis}) {
    if (axis->endstopEnabled) pinMode(axis->endstopPin, INPUT);
  }

  for (Axis *axis : {&xAxis, &yAxis}) {
#if DRIVER_TMC2209
    axis->driver.begin();
    axis->driver.toff(5);
    axis->driver.pwm_autoscale(true);
#endif
    applyDriverMicrosteps(*axis);
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

  shared::begin();
  net::begin();
  device::begin();
  device::logEvent("boot", "firmware %s, boot %lu, reset %s", FW_VERSION, static_cast<unsigned long>(net::bootCount()),
                   net::resetReasonName());
  webserver::begin(FW_VERSION);
  asiairclient::begin();
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
  unsigned long passStartUs = micros();
  serviceWebSide();
  loopservice::notePassMicros(static_cast<uint32_t>(micros() - passStartUs));
  serviceNinaTcp();

  if (Serial.available()) {
    anySerialByte = true;
    lastSerialMs = millis();
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineBuffer.length() > 0) {
        String reply = dispatchSerialLine(lineBuffer);
        if (reply.length() > 0) Serial.println(reply);
        lineBuffer = "";
      }
    } else {
      lineBuffer += c;
    }
  }
}
