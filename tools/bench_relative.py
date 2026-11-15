#!/usr/bin/env python3
"""Bench check of the 1.3.7 relative moves over the OAPA protocol (TCP 2323 or a serial-over-TCP
bridge): $I?, $K? after $F=, then $R= moves watched through $R? and the step counter.

    python tools/bench_relative.py 192.168.1.29 --factors 59.61,59.44

The motors move: +1' and -2' on azimuth, +1.5' on altitude, a refused move beyond the cap, a
refused move while one runs. Exit code 0 when every expectation holds.
"""
import argparse
import socket
import sys
import time


class Board:
    def __init__(self, address):
        host, _, port = address.partition(":")
        self.sock = socket.create_connection((host, int(port or 2323)), timeout=5)
        self.buffer = b""
        time.sleep(0.5)
        self.sock.settimeout(0.3)
        try:
            while self.sock.recv(4096):
                pass
        except (socket.timeout, OSError):
            pass

    def line(self, timeout=3.0):
        self.sock.settimeout(timeout)
        while b"\n" not in self.buffer:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise RuntimeError("the link closed")
            self.buffer += chunk
        text, _, self.buffer = self.buffer.partition(b"\n")
        return text.decode(errors="replace").rstrip("\r")

    def command(self, text, lines=1):
        self.sock.sendall((text + "\n").encode())
        return [self.line() for _ in range(lines)]

    def frame(self, text):
        reply = self.command(text)[0]
        if not reply.startswith("<"):
            return {"_": reply}
        parts = reply.strip("<>|").split("|")
        d = dict(f.split(":", 1) for f in parts[1:] if ":" in f)
        d["_name"] = parts[0]
        return d

    def steps(self):
        frame, _ok = self.command("?", lines=2)
        fields = dict(f.split(":", 1) for f in frame.strip("<>|").split("|")[1:] if ":" in f)
        x, y, _z = (float(v) for v in fields["MPos"].split(","))
        return frame.strip("<").split("|")[0], x, y


failures = []


def expect(label, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + label + (f"  ({detail})" if detail else ""))
    if not ok:
        failures.append(label)


def wait_done(board, timeout=30):
    end = time.time() + timeout
    while time.time() < end:
        r = board.frame("$R?")
        if r.get("state") in ("done", "failed", "idle"):
            return r
        time.sleep(0.25)
    return board.frame("$R?")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("address")
    p.add_argument("--factors", default="59.61,59.44", help="steps per arcminute az,alt to push with $F=")
    args = p.parse_args()
    fx, fy = (float(v) for v in args.factors.split(","))
    b = Board(args.address)

    print("protocol:", b.command("$P?")[0])
    ident = b.frame("$I?")
    expect("$I? names the board", ident.get("_name") == "I" and ident.get("fw", "").startswith("1.3.7"), str(ident))

    print("$K? before:", b.frame("$K?"))
    b.command(f"$F={fx},{fy}")
    k = b.frame("$K?")
    expect("$K? carries the pushed factors", k.get("x", "-") != "-" and k.get("y", "-") != "-", str(k))
    m = b.frame("$M?")
    expect("$M? carries the board's own cap", "cap" in m, str(m))
    cap = float(m.get("cap", 30))
    b.command("$M=5")  # a pushed correction cap must not bound $R=

    state, x0, y0 = b.steps()
    r = b.command("$R=1,0")[0]
    expect("$R=1,0 accepted", r == "ok", r)
    r = b.command("$R=0,1")[0]
    expect("$R= while moving is refused", r == "error", r)
    done = wait_done(b)
    state, x1, y1 = b.steps()
    expect("+1' az moved +fx steps", abs((x1 - x0) - round(fx)) <= 1 and y1 == y0, f"dx={x1 - x0:.0f} want {round(fx)}")
    expect("$R? reports done with the commanded az", done.get("state") == "done" and abs(float(done.get("az", "0")) - 1.0) < 0.02, str(done))

    r = b.command("$R=-2,1.5")[0]
    expect("$R=-2,1.5 accepted", r == "ok", r)
    done = wait_done(b)
    state, x2, y2 = b.steps()
    expect("-2' az moved -2fx steps", abs((x2 - x1) + round(2 * fx)) <= 1, f"dx={x2 - x1:.0f} want {-round(2 * fx)}")
    expect("+1.5' alt moved 1.5fy steps", abs((y2 - y1) - round(1.5 * fy)) <= 1, f"dy={y2 - y1:.0f} want {round(1.5 * fy)}")
    expect("$R? done with both amounts", done.get("state") == "done" and abs(float(done.get("alt", "0")) - 1.5) < 0.02, str(done))

    r = b.command(f"$R={cap + 1:.0f},0")[0]
    expect("a move beyond the board's own cap is refused", r == "error", r)
    if cap > 6:
        r = b.command("$R=6,0")[0]
        expect("a pushed $M= does not bound $R=", r == "ok", r)
        wait_done(b)
    r = b.command("$R=0,0")[0]
    expect("a zero move is accepted and done at once", r == "ok" and wait_done(b, 3).get("state") == "done", r)

    b.command("$R=5,0")
    time.sleep(0.2)
    b.command("!")
    time.sleep(0.5)
    r = b.frame("$R?")
    expect("a stop ends the move as failed/stopped", r.get("state") == "failed" and r.get("reason") == "stopped", str(r))

    print("\n%d failure(s)" % len(failures))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
