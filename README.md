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
- An endstop on the altitude axis is optional (used only for homing); azimuth may rotate freely

## Hardware

The reference implementation targets:

| Component | Reference part |
|---|---|
| Controller board | FYSETC E4 v1.3 (ESP32) |
| Stepper drivers | 2× TMC2209 (UART mode) |
| Motors | 2× NEMA17 stepper (or similar), one per axis |
| Endstop | 1× on the altitude (X) axis for homing; azimuth (Y) rotates freely |

Pinout (FYSETC E4 v1.3):

| Signal | GPIO |
|---|---|
| Enable | 25 |
| X step / dir | 27 / 26 |
| Y step / dir | 33 / 32 |
| X endstop | 34 (X-min port) |
| TMC UART | 15 |

Different boards, drivers, motors, and gear ratios work as long as the serial protocol below is implemented. The plugin's **Self-Calibration** measures the actual gear response and backlash on the sky, so no mechanical parameters need to be configured in the firmware.

## Serial protocol

115200 baud, 8N1, newline-terminated commands. GRBL-flavoured.

| Command | Meaning |
|---|---|
| `?` | Status query — replies with the status frame below |
| `$H` | Home the altitude (X) axis against its endstop |
| `$J=G91G21X<n>F<f>` | Relative jog (steps), `F` = max speed in steps/s |
| `$J=G53X<n>F<f>` | Absolute jog (steps), `F` = max speed in steps/s |
| `X<n>` / `Y<n>` | Direct relative move in steps, at the default speed |
| `!` | Stop — decelerates both axes to a halt (since 1.2.1) |
| `CX<mA>` / `CY<mA>` | Set run current (milliamps) |
| `HX<pct>` / `HY<pct>` | Set hold current (percent of run current) |
| `SX<n>` / `SY<n>` | Set microsteps |

Status frame:

```
<Idle|MPos:123.00,-45.00,0.00|V:1.1.0|>
```

- `Idle` / `Run` / `Home` — machine state
- `MPos` — X, Y, Z positions in steps (Z unused)
- `V` — firmware version (since 1.1.0). The plugin reads this field and warns when the connected firmware is older than the version it expects.

All commands answer `ok` (or `error`).

## Flashing

Open `oapa.ino` in the Arduino IDE (or PlatformIO), select your ESP32 board, and upload. Required libraries: `TMCStepper`, `AccelStepper`.

## Versioning

`FW_VERSION` in `oapa.ino` is bumped on every protocol-visible change. The version is reported in the status frame so the plugin can detect outdated firmware.

| Version | Change |
|---|---|
| **1.2.2** | Hold current defaults to 25% of run current instead of 50%. Hold current flows continuously from power-on — including the whole window before the plugin connects and pushes the user's values — and heat goes with the square of it. A polar-alignment platform is usually self-locking mechanics, so it is better served by a cool motor than by holding torque it rarely needs. |
| **1.2.1** | The `F` feed value in `$J=` jogs now sets the maximum speed for that move (clamped to 50–3000 steps/s; absent or malformed → 2000, exactly the previous behaviour). Acceleration stays fixed. Direct `X<n>`/`Y<n>` moves carry no feed value, so they reset the profile to the default instead of inheriting whatever `F` the previous jog used. Adds `!`, which decelerates both axes to a halt while keeping the position counter honest — the plugin's STOP button sends it. |
| **1.2.0** | Homing travel limit: bounded seek, and an axis whose enabled endstop is never found zeroes in place instead of running away. |
| **1.1.0** | `V:` version field added to the status frame. |

## Using it with N.I.N.A.

1. Install the Three Point Polar Alignment plugin
2. In the plugin options, select **OAPA** as the alignment system and connect (the plugin scans COM ports automatically)
3. Run the built-in **Self-Calibration** once, pointing toward the celestial pole
4. Start TPPA — the correction phase drives the platform automatically

## License

MIT — see [LICENSE](LICENSE).
