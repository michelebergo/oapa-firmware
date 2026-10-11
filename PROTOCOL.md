# OAPA protocol — version 1

The line protocol between a host (the TPPA plugin for N.I.N.A.) and a device that
corrects polar alignment. The OAPA firmware in this repository is one implementation;
any device that answers the commands below can be driven by the same host, without
having to look like the OAPA firmware or report its version numbers.

## Transport

- USB serial: 115200 baud, 8N1.
- Or TCP, port 2323 by default (the host connects to `host` or `host:port`). One client
  at a time.

## Framing

- Every command is one line of ASCII ending with a newline (`\n`; a preceding `\r` is
  ignored).
- Every reply is one line ending with `\r\n`, with one exception: the reply to `?` is the
  status frame line followed by an `ok` line.
- Plain replies are `ok` or `error`. Structured replies are framed `<X|key:value|...|>`,
  where `X` names the reply; a value never contains `|`.
- A command the device does not know is answered `ok` and ignored. A host therefore
  detects an optional command by asking: a plain `ok` where a framed reply was expected
  means "not implemented".
- A malformed command (bad number, value out of range) is answered `error` and changes
  nothing. A lost character costs one line.

## Protocol version

`$P?` answers `<P|protocol:1|>`. The host decides what the device can do from this
number, not from the firmware version. The protocol version is raised only when a host
would have to change; optional commands added later are found by asking, as above.

A device that does not answer `$P?` is not driven for alignment; the host still offers
hand moves.

## Status

`?` answers the status frame, then `ok`:

```
<Idle|MPos:123.00,-45.00,0.00|V:1.3.3|>
ok
```

- State: `Idle`, `Run` (a move in progress) or `Home` (homing).
- `MPos`: X (azimuth) and Y (altitude) positions in motor steps, then 0.00 (Z, unused).
- `V`: the device's own firmware version, in the device's own format (any text without
  `|`, `<` or `>`), for display and support. Optional; the host decides nothing from it,
  and a device that reports the protocol is not warned about its firmware number.

The host polls `?` while a hand move runs and treats `Idle` as the end of the move.

## Alignment: what a device must implement

With "Controller aligns" on, the host only measures; the device decides every move.

| Command | Reply | Meaning |
|---|---|---|
| `$P?` | `<P|protocol:1|>` | Protocol version. |
| `$E=<az>,<alt>` | `ok` | One polar alignment error reading from TPPA: azimuth and altitude error of the mount's polar axis, arcminutes, as TPPA reports them, a few seconds apart while TPPA measures. The device must not assume what a positive value means for its own axes: the OAPA firmware learns how each axis moves the error with its first two probe moves. A reading after a silence longer than 30 s starts a new alignment; readings that keep coming after an alignment ended do not restart it. |
| `$T=<arcmin>` | `ok` | The tolerance of the running TPPA instruction, 0 < t ≤ 60. The device finishes at it, like TPPA. Sent before a reading when it changes. |
| `$A=0` | `ok` | Stop the alignment at once (the user turned "Controller aligns" off). `$A=1` starts one without waiting for a reading. |
| `$L?` | `<L|phase:...|outcome:...|source:...|moves:N|az:...|alt:...|plan:x,y|reason:...|>` | Alignment status, read after every reading and written to the N.I.N.A. log. `phase`: `idle`, `waiting`, `moving_x`, `moving_y`, `ended`. `outcome`: `none`, `finished`, `finished_best_effort`, `halted_calibration_suspect`, `halted_estimate_drift`, `halted_runaway`, `halted_unresponsive`, `stopped_by_user`, `stopped_by_nina`, `stopped_no_source`, `stopped_source_ended`. `az`/`alt`: the reading the last decision used, or `-`. `plan`: the last correction, arcminutes. `reason`: free text for the user. |
| `!` | `ok` | Stop both axes now. Also ends an alignment the host is feeding. |

## Parameters the host pushes

Sent on connection and whenever the user changes them. A device that has no use for
one acknowledges it with `ok`.

| Command | Meaning |
|---|---|
| `$F=<fx>,<fy>` | Steps per arcminute of error, azimuth and altitude, when the user has them (measured or typed in). Not sent otherwise: the device calibrates by itself. |
| `$B=<axis>,<mode>,<plus>,<minus>` | Backlash of one axis (`X` or `Y`): mode `O`ff, `S`oft, `F`ull or `U`nidirectional; arcminutes entering the positive / negative direction. |
| `$M=<arcmin>` | Largest single correction, from 1 up to the device's limit. |
| `SX<n>`, `SY<n>`, `CX<mA>`, `CY<mA>`, `HX<pct>`, `HY<pct>` | Microsteps, run current, hold current per axis (stepper drivers). |

## Optional commands

Found by asking; a plain `ok` means not implemented.

| Command | Reply | Meaning |
|---|---|---|
| `$M?` | `<M|max:180|>` | The largest move cap `$M=` accepts. Without it the host assumes 120. |
| `$C=1` / `$C=2` / `$C=0` | `ok` | Calibrate on the readings that follow, then align / calibrate only (the readings are field displacements the host measures itself, in the same arcminute units) / stop a calibration. |
| `$K?` | `<K|state:...|x:<factor>,<sign>|y:...|xplay:<+>,<->,<mode>|yplay:...|reason:...|>` | Calibration state (`idle`, `preloading`, `baseline`, `probing`, `measuring`, `restoring`, `done`, `failed`, `stopped`) and result; `-` for an axis without one. The host stores a `done` result as the user's factors and backlash. |
| `$G=<seq>` | `<G|seq:N|last:M|ms:T|code:...|text:...|>` or `<G|seq:-|last:M|>` | The device's event log, one event per command: the oldest event after `seq`, and the newest number. `last` below the `seq` asked for means the device restarted. The host copies the events into the N.I.N.A. log. |

## Hand moves

| Command | Meaning |
|---|---|
| `$J=G91G21X<n>F<f>` / `...Y<n>F<f>` | Relative move of one axis, in steps, at `f` steps per second. |
| `$J=G53X<n>F<f>` / `...Y<n>F<f>` | Move one axis to a position, in steps. |
| `$H` | Home: against an endstop if the axis has one, otherwise set the position to zero. |

A hand move or a stop from the host ends an alignment the host is feeding.

## Known quirks

Kept for compatibility; candidates for a later protocol version:

- the reply to `?` is two lines (frame and `ok`), every other command one;
- the hand-move and driver commands (`X800`, `CX600`, `$J=...`) come from GRBL and have
  no common start character, unlike the `$` commands;
- an unknown command is acknowledged `ok` rather than refused, so a misspelled command
  goes unnoticed on the wire.
