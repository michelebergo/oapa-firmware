# OAPA Firmware

Reference firmware for **OAPA-compatible automated polar alignment platforms**. It works with the [N.I.N.A.](https://nighttime-imaging.eu/) Three Point Polar Alignment (TPPA) plugin and with ASIAIR's polar alignment.

An OAPA platform sits between tripod and mount and adjusts the mount's **altitude and azimuth** with two stepper motors. Since 1.3.0 the **board runs the alignment itself**: it is given the polar alignment error (by TPPA over USB or WiFi, or by ASIAIR), decides every move, and stops at the tolerance. It also calibrates itself: steps per arcminute and backlash of each axis.

## Supported platforms

The firmware is **platform-agnostic**: it drives two stepper motors in steps and learns how they move the mount. Any design that converts two motor rotations into altitude and azimuth adjustments of the mount can be used, for example:

- **Wedge-style platforms** — a tilting plate for altitude and a rotating base for azimuth, placed between tripod and mount
- **Lead-screw / linear-actuator designs** — a screw pushing the altitude plate, a second one (or a rotary stage) for azimuth
- **Worm-gear or belt-reduction rotary stages** on both axes
- **Motorized alt-az adjuster retrofits** — motors replacing the manual alt/az adjustment knobs of an existing wedge or mount base

None of the mechanical parameters need to be configured: the board's **calibration** measures the steps per arcminute and the backlash of each axis on the polar alignment readings, and the correction steps scale with the measured error. High gear reductions (1:20, 1:100, …) for heavy payloads work out of the box. If the reduction is known, the TPPA plugin can also compute the factor from it.

Practical requirements for the mechanics:

- Both axes must be able to move **in both directions** while under the full load of the mount and rig
- A total adjustment range of **±1-2°** per axis is plenty (get within that range by hand first)
- Backlash is acceptable — it is measured by the calibration and compensated — but the axes must not slip or shift under load
- The azimuth motor goes on the X axis and the altitude motor on the Y axis
- Endstops are optional on both axes (used only for homing) and **disabled by default**: an axis without one simply zeroes in place

## Hardware

The reference implementation targets:

| Component | Reference part |
|---|---|
| Controller board | FYSETC E4 v1.3 (ESP32) |
| Stepper drivers | 2× TMC2209 (UART mode) — or any STEP/DIR driver, see below |
| Motors | 2× NEMA17 stepper (or similar), one per axis |
| Endstop | Optional, per axis — see below; a two-motor build needs none |

Pinout (FYSETC E4 v1.3):

| Signal | GPIO |
|---|---|
| Enable | 25 |
| X step / dir | 27 / 26 |
| Y step / dir | 33 / 32 |
| X endstop (azimuth, optional) | 34 (X-min port) |
| Y endstop (altitude, optional) | 35 (Y-min port) |
| TMC UART | 15 |

### Plain STEP/DIR drivers (A4988, DRV8825, LV8729, ...)

The firmware only uses the TMC2209 UART to set run current, hold current and microsteps. Motion itself is STEP/DIR pulses, the same for every driver family, so a bare ESP32 dev board with StepStick-style drivers works too. Build the `esp32_stepdir` environment (it sets `DRIVER_TMC2209=0`): the `TMCStepper` library is then not used, and the `C`/`H`/`S` commands are still answered with `ok` but have no effect.

What moves to the hardware in that case:

- **Run current**: the Vref trimmer on each driver module. Neither driver lowers the current when idle the way the TMC2209 does, so the motors sit at run current whenever the board is powered — set Vref on the conservative side.
- **Microsteps**: the MS/MD jumpers on the module. 16 is the firmware default and the A4988 maximum; the calibration measures the real steps per arcminute anyway, so any setting works as long as it does not change afterwards.
- **Logic level**: the ESP32 drives 3.3 V signals. Power the A4988's logic supply (`VDD`) from the 3.3 V rail, not 5 V, otherwise the step pulses sit too close to its input threshold. The LV8729 and DRV8825 accept 3.3 V logic directly.
- **Enable**: active LOW on these modules, as on the TMC2209, so `ENABLE_PIN` needs no change.

## Web page and WiFi

The board serves a phone page for use with ASIAIR: live error and chart, start/stop of the automatic alignment, an arrow pad for hand moves, calibration, settings, motor direction per axis, event log and firmware update.

- **First start**: the board opens a WiFi network named `OAPA-setup`; join it and enter the network the ASIAIR uses (2.4 GHz only). The board then joins that network; its address shows in the router's device list and in the page's event log.
- **Motor direction**: each axis can be set to turn the other way (System tab), like swapping two wires of its motor. It is saved on the board and applies to every move.
- **Firmware update over WiFi**: from the System tab, protected by a password set on the page.

## Serial protocol

115200 baud, 8N1, newline-terminated commands, GRBL-flavoured. The same protocol is served over WiFi on **TCP port 2323** (one client at a time, the newest connection wins).

| Command | Meaning |
|---|---|
| `?` | Status query — replies with the status frame below |
| `!` | Stop — decelerates both axes to a halt (since 1.2.1); also ends an alignment the host is feeding |
| `$H` | Home each axis: against its endstop if it has one, otherwise zero in place |
| `$J=G91G21X<n>F<f>` | Relative jog (steps); `F` sets the step rate for that move |
| `$J=G53X<n>F<f>` | Absolute jog (steps); `F` sets the step rate for that move |
| `X<n>` / `Y<n>` | Direct relative move in steps |
| `CX<mA>` / `CY<mA>` | Set run current (milliamps) |
| `HX<pct>` / `HY<pct>` | Set hold current (percent of run current) |
| `SX<n>` / `SY<n>` | Set microsteps |

Note the **type-first** shape of the driver commands: the letter that selects *what* comes before the letter that selects *which axis* (`CX600`, not `XC600`). Anything unknown is acknowledged with `ok` and ignored, which makes a wrong shape invisible on the wire.

Alignment commands (since 1.3.0), used by the TPPA plugin:

| Command | Meaning |
|---|---|
| `$E=<az>,<alt>` | One polar error reading, arcminutes. The first reading of a stream starts an alignment (after a calibration if the board has no factors) |
| `$F=<fx>,<fy>` | Steps per azimuth / altitude arcminute, when the host knows them |
| `$B=<axis>,<mode>,<plus>,<minus>` | Backlash of one axis: mode `O`ff, `S`oft, `F`ull or `U`nidirectional; arcminutes entering the positive / negative direction |
| `$T=<arcmin>` | Alignment tolerance (the host's own, so both stop at the same error) |
| `$A=1` / `$A=0` | Start / stop an alignment |
| `$C=1` / `$C=2` / `$C=0` | Calibrate, then align / calibrate only / stop a calibration |
| `$L?` | Alignment status, one line: `<L|phase:...|outcome:...|moves:...|az:...|alt:...|plan:...|reason:...|>` |
| `$K?` | Calibration status and result: `<K|state:...|x:<factor>,<sign>|y:...|xplay:<+>,<->,<mode>|yplay:...|reason:...|>` |

A malformed alignment command replies `error`. The `F` feed value is honored from **1.2.1** onward (clamped to 50–3000 steps/s). Driver settings are **not persisted**: they return to the defaults below on every power-up, so a host should push its values after connecting.

Status frame:

```
<Idle|MPos:123.00,-45.00,0.00|V:1.3.0|>
```

- `Idle` / `Run` / `Home` — machine state
- `MPos` — X, Y, Z positions in steps (Z unused)
- `V` — firmware version (since 1.1.0). The plugin reads this field and warns when the connected firmware is older than the version it expects.

## Endstops

Both axes can have a homing switch, and both are **disabled by default** — a two-motor build needs no wiring and no changes. Enable one per axis at the top of `src/oapa_protocol.inc` (`X_ENDSTOP_ENABLED`, pin, `INVERT`, homing direction).

⚠️ **ESP32 hardware note**: GPIO34–39 are input-only and have **no internal pull resistors**, so `INPUT_PULLUP` silently does nothing there. Before enabling an endstop on those pins, wire an external pull-up (switch to GND, `INVERT=true`) or pull-down (switch to 3V3, `INVERT=false`) — a floating pin false-triggers.

Homing is bounded by `HOMING_MAX_TRAVEL` (200 000 steps): a switch that is enabled but never seen makes the axis give up and zero in place instead of seeking forever.

## Defaults

| Setting | Default | Notes |
|---|---|---|
| Run current | 600 mA | per axis, set with `CX`/`CY` |
| Hold current | 25% of run | lowered from 50% in 1.2.2 — it flows continuously from power-up, and heating goes with the square of the current |
| Microsteps | 16 | set with `SX`/`SY` |
| Step rate | 2000 steps/s | used when a jog carries no `F` value |

## Building and flashing

This is a [PlatformIO](https://platformio.org/) project:

```
pio run -e fysetc_e4 -t upload        # FYSETC E4, TMC2209 drivers
pio run -e esp32_stepdir -t upload    # any ESP32 with STEP/DIR drivers
```

Once the board is on the network, later versions can be uploaded from the page (System tab) with the `.pio/build/<env>/firmware.bin` file. Host tests: `tests\host\run_tests.bat` (MSVC). `tools/check_verbatim.py` checks that the serial protocol region is unchanged from 1.2.3.

The single-file `oapa.ino` for the Arduino IDE ends at 1.2.3; it stays in the history of this repository.

## Versioning

`FW_VERSION` in `src/main.cpp` is bumped on every change in what the board does. The version is reported in the status frame so the host can detect outdated firmware.

| Version | Changes |
|---|---|
| 1.3.0 | The board runs the alignment: error from TPPA over USB or WiFi (TCP 2323) or from ASIAIR; calibration of factors and backlash; phone web page with arrow pad and motor direction; firmware update over WiFi. PlatformIO project; serial protocol of 1.2.3 unchanged |
| 1.2.3 | `DRIVER_TMC2209` build switch: plain STEP/DIR drivers (A4988, DRV8825, LV8729, ...) without the TMCStepper library; protocol unchanged |
| 1.2.2 | Hold current default lowered to 25% |
| 1.2.1 | `F` feed value sets the step rate of a jog (clamped 50–3000); new `!` stop command |
| 1.2.0 | Axis-first restructure; endstops optional per axis and disabled by default; homing travel limit; opt-in soft-limit guard |
| 1.1.0 | `V:` version field added to the status frame |

## Using it with N.I.N.A.

1. Install the Three Point Polar Alignment plugin
2. In the plugin options, select **OAPA** as the alignment system; in the OAPA panel choose USB (the plugin scans the COM ports) or WiFi (the board's address) and connect
3. Press **Calibrate** once, with the camera pointed at the sky (or let the board calibrate by itself on the first alignment)
4. Start TPPA with **Controller aligns** on — the board drives the platform until TPPA's tolerance is reached

## Using it with ASIAIR

1. Put the board on the ASIAIR's network (see *Web page and WiFi*)
2. Start the polar alignment on ASIAIR and open the board's page on the phone
3. Calibrate once from the page, then press **Start automatic** when ASIAIR reaches the adjustment step

## License

MIT — see [LICENSE](LICENSE).
