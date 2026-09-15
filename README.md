# OAPA Firmware

Reference firmware for **OAPA-compatible automated polar alignment platforms**, driven by the [N.I.N.A.](https://nighttime-imaging.eu/) Three Point Polar Alignment (TPPA) plugin.

An OAPA platform sits between tripod and mount and adjusts the mount's **altitude and azimuth** with two stepper motors. TPPA measures the polar alignment error by plate solving and drives the platform automatically until the configured tolerance is reached — fully hands-off.

## Supported platforms

The firmware is **platform-agnostic**: it drives two stepper motors in steps and knows nothing about the mechanics attached to them. Any design that converts two motor rotations into altitude and azimuth adjustments of the mount can be used, for example:

- **Wedge-style platforms** — a tilting plate for altitude and a rotating base for azimuth, placed between tripod and mount
- **Lead-screw / linear-actuator designs** — a screw pushing the altitude plate, a second one (or a rotary stage) for azimuth
- **Worm-gear or belt-reduction rotary stages** on both axes
- **Motorized alt-az adjuster retrofits** — motors replacing the manual alt/az adjustment knobs of an existing wedge or mount base

None of the mechanical parameters (gear ratio, screw pitch, reduction, microstepping) need to be configured in the firmware: the plugin's **Self-Calibration** measures the actual steps-to-sky response and the backlash of each axis directly on the sky, and the correction step sizes scale automatically with the measured error. This also means high gear reductions (1:20, 1:100, …) for heavy payloads work out of the box — they only change the calibrated response, which is learned, not configured.

Practical requirements for the mechanics:

- Both axes must be able to move **in both directions** while under the full load of the mount and rig
- A total adjustment range of **±1-2°** per axis is plenty (TPPA gets you within that range by hand first)
- Backlash is acceptable — it is measured during Self-Calibration and compensated — but the axes must not slip or shift under load
- The azimuth motor goes on the X axis and the altitude motor on the Y axis — the convention the TPPA plugin uses
- Endstops are optional on both axes (used only for homing) and **disabled by default**: an axis without one simply zeroes in place

## Hardware

The reference implementation targets:

| Component | Reference part |
|---|---|
| Controller board | FYSETC E4 v1.3 (ESP32) |
| Stepper drivers | 2× TMC2209 (UART mode) |
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

Different boards, drivers, motors, and gear ratios work as long as the serial protocol below is implemented. The plugin's **Self-Calibration** measures the actual gear response and backlash on the sky, so no mechanical parameters need to be configured in the firmware.

## Serial protocol

115200 baud, 8N1, newline-terminated commands. GRBL-flavoured.

| Command | Meaning |
|---|---|
| `?` | Status query — replies with the status frame below |
| `!` | Stop — decelerates both axes to a halt (since 1.2.1) |
| `$H` | Home each axis: against its endstop if it has one, otherwise zero in place |
| `$J=G91G21X<n>F<f>` | Relative jog (steps); `F` sets the step rate for that move |
| `$J=G53X<n>F<f>` | Absolute jog (steps); `F` sets the step rate for that move |
| `X<n>` / `Y<n>` | Direct relative move in steps |
| `CX<mA>` / `CY<mA>` | Set run current (milliamps) |
| `HX<pct>` / `HY<pct>` | Set hold current (percent of run current) |
| `SX<n>` / `SY<n>` | Set microsteps |

Note the **type-first** shape of the driver commands: the letter that selects *what* comes before the letter that selects *which axis* (`CX600`, not `XC600`). Anything else is acknowledged with `ok` and ignored, which makes a wrong shape invisible on the wire — check the driver actually changed rather than trusting the reply.

The `F` feed value is honored from **1.2.1** onward (clamped to 50–3000 steps/s); earlier firmware accepted and ignored it, running every move at a fixed profile. Driver settings are **not persisted**: they return to the defaults below on every power-up, so a host should push its values after connecting.

Status frame:

```
<Idle|MPos:123.00,-45.00,0.00|V:1.2.2|>
```

- `Idle` / `Run` / `Home` — machine state
- `MPos` — X, Y, Z positions in steps (Z unused)
- `V` — firmware version (since 1.1.0). The plugin reads this field and warns when the connected firmware is older than the version it expects.

All commands answer `ok` (or `error`).

## Endstops

Both axes can have a homing switch, and both are **disabled by default** — a two-motor build needs no wiring and no changes. Enable one per axis at the top of `oapa.ino` (`X_ENDSTOP_ENABLED`, pin, `INVERT`, homing direction).

⚠️ **ESP32 hardware note**: GPIO34–39 are input-only and have **no internal pull resistors**, so `INPUT_PULLUP` silently does nothing there. Before enabling an endstop on those pins, wire an external pull-up (switch to GND, `INVERT=true`) or pull-down (switch to 3V3, `INVERT=false`) — a floating pin false-triggers.

Homing is bounded by `HOMING_MAX_TRAVEL` (200 000 steps): a switch that is enabled but never seen makes the axis give up and zero in place instead of seeking forever. At extreme reductions that limit is only a few degrees of platform travel — raise it if your switch sits farther away.

## Defaults

| Setting | Default | Notes |
|---|---|---|
| Run current | 600 mA | per axis, set with `CX`/`CY` |
| Hold current | 25% of run | lowered from 50% in 1.2.2 — it flows continuously from power-up, and heating goes with the square of the current |
| Microsteps | 16 | set with `SX`/`SY` |
| Step rate | 2000 steps/s | used when a jog carries no `F` value |

## Flashing

Open `oapa.ino` in the Arduino IDE (or PlatformIO), select your ESP32 board, and upload. Required libraries: `TMCStepper`, `AccelStepper`.

The source is deliberately **plain ASCII with no byte-order mark**: unzipping on Windows and opening in the Arduino IDE can re-encode anything else into bytes the compiler rejects (`stray '\255' in program`), and whether it happens depends on the local environment. Keep it that way when editing.

## Versioning

`FW_VERSION` in `oapa.ino` is bumped on every protocol-visible change. The version is reported in the status frame so the host can detect outdated firmware.

| Version | Changes |
|---|---|
| 1.2.2 | Hold current default lowered to 25% |
| 1.2.1 | `F` feed value sets the step rate of a jog (clamped 50–3000); new `!` stop command |
| 1.2.0 | Axis-first restructure; endstops optional per axis and disabled by default; homing travel limit; opt-in soft-limit guard |
| 1.1.0 | `V:` version field added to the status frame |

## Using it with N.I.N.A.

1. Install the Three Point Polar Alignment plugin
2. In the plugin options, select **OAPA** as the alignment system and connect (the plugin scans COM ports automatically)
3. Run the built-in **Self-Calibration** once, pointing toward the celestial pole
4. Start TPPA — the correction phase drives the platform automatically

## License

MIT — see [LICENSE](LICENSE).
