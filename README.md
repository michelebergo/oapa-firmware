# OAPA Firmware

Reference firmware for **OAPA-compatible automated polar alignment platforms**, driven by the [N.I.N.A.](https://nighttime-imaging.eu/) Three Point Polar Alignment (TPPA) plugin.

An OAPA platform sits between tripod and mount and adjusts the mount's **altitude and azimuth** with two stepper motors. TPPA measures the polar alignment error by plate solving and drives the platform automatically until the configured tolerance is reached — fully hands-off.

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
| `$J=G91G21X<n>F<f>` | Relative jog (steps) |
| `$J=G53X<n>F<f>` | Absolute jog (steps) |
| `X<n>` / `Y<n>` | Direct relative move in steps |
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

## Using it with N.I.N.A.

1. Install the Three Point Polar Alignment plugin
2. In the plugin options, select **OAPA** as the alignment system and connect (the plugin scans COM ports automatically)
3. Run the built-in **Self-Calibration** once, pointing toward the celestial pole
4. Start TPPA — the correction phase drives the platform automatically

## License

MIT — see [LICENSE](LICENSE).
