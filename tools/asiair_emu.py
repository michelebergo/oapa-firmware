#!/usr/bin/env python3
"""Stand-in for ASIAIR's event port 4700 on the bench. Replays the PA event
sequence of the 13/09/2026 night capture and computes the error from the OAPA
board's real step counts, with a chosen factor, sign and noise. At the end
(--duration or Ctrl+C) it sends the "aborted" solve, code 253, as ASIAIR does.

    python3 asiair_emu.py --board 192.168.1.15 --factor 60 --sign-x -1 --duration 900
"""
import argparse
import json
import math
import random
import signal
import socket
import threading
import time
import urllib.request

clients = []
lock = threading.Lock()


def broadcast(event):
    line = (json.dumps(event, separators=(",", ":")) + "\n").encode()
    with lock:
        for conn in list(clients):
            try:
                conn.sendall(line)
            except OSError:
                clients.remove(conn)
                conn.close()


def accept_loop(server):
    while True:
        conn, addr = server.accept()
        print("board connected from", addr[0], flush=True)
        with lock:
            clients.append(conn)


def board_steps(board):
    with urllib.request.urlopen(f"http://{board}/api/status", timeout=3) as response:
        status = json.loads(response.read())
    return status["x"], status["y"]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--board", required=True)
    p.add_argument("--port", type=int, default=4700)
    p.add_argument("--factor", type=float, default=60, help="true steps per arcminute on both axes")
    p.add_argument("--sign-x", type=int, default=1)
    p.add_argument("--sign-y", type=int, default=1)
    p.add_argument("--az", type=float, default=30, help="starting azimuth error, arcmin")
    p.add_argument("--alt", type=float, default=-20, help="starting altitude error, arcmin")
    p.add_argument("--noise", type=float, default=0.05, help="arcmin, per reading")
    p.add_argument("--period", type=float, default=4.0, help="seconds per frame")
    p.add_argument("--duration", type=float, default=1800)
    a = p.parse_args()
    # A background process ignores Ctrl+C: SIGTERM (pkill) also ends the PA with the 253 solve.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", a.port))
    server.listen(4)
    threading.Thread(target=accept_loop, args=(server,), daemon=True).start()
    print("waiting for the board", flush=True)
    while not clients:
        time.sleep(0.2)

    x0, y0 = board_steps(a.board)
    t0 = time.time()

    def stamp():
        return f"{time.time() - t0 + 100:.9f}"

    def pa_event(state, code, retry, error=None):
        event = {"Event": "3PPA", "Timestamp": stamp(), "state": state, "state_code": code, "auto_move": True,
                 "auto_update": state == "calc4", "paused": False, "detail": {}}
        if error is not None:
            az, alt = error
            event["pa_error"] = {"total": round(math.hypot(az, alt) / 60, 6), "x": round(az / 60, 6),
                                 "y": round(alt / 60, 6), "total_arcsec": round(math.hypot(az, alt) * 60, 6),
                                 "x_arcsec": round(az * 60, 6), "y_arcsec": round(alt * 60, 6)}
        event["lapse_ms"] = int((time.time() - t0) * 1000)
        event["retry_cnt"] = retry
        return event

    for state, code in (("start", 1), ("move1", 5), ("delay1", 6), ("move2", 10), ("delay2", 11)):
        broadcast(pa_event(state, code, 0))
        time.sleep(1)

    frame = 0
    try:
        while time.time() - t0 < a.duration:
            broadcast({"Event": "Exposure", "Timestamp": stamp(), "page": "pa", "tag": "3PPA", "pa_state": "exp4",
                       "pa_state_code": 15, "state": "start", "exp_us": 1000000, "gain": 28})
            x, y = board_steps(a.board)  # the frame sees the platform when its exposure starts
            az = a.az + a.sign_x * (x - x0) / a.factor + random.gauss(0, a.noise)
            alt = a.alt + a.sign_y * (y - y0) / a.factor + random.gauss(0, a.noise)
            time.sleep(a.period * 0.6)
            broadcast({"Event": "PlateSolve", "Timestamp": stamp(), "page": "pa", "tag": "3PPA", "pa_state": "solve4",
                       "pa_state_code": 16, "state": "complete"})
            state, code = ("calc3", 14) if frame == 0 else ("calc4", 17)
            broadcast(pa_event(state, code, 1, (az, alt)))
            print(f"{state} az {az:+.2f}' alt {alt:+.2f}'", flush=True)
            frame += 1
            if frame % 3 == 0:
                broadcast({"Event": "PiStatus", "Timestamp": stamp(), "is_overtemp": False, "temp": 40.0,
                           "is_undervolt": False, "is_over_current": False})
            time.sleep(a.period * 0.4)
    except KeyboardInterrupt:
        pass
    broadcast({"Event": "PlateSolve", "Timestamp": stamp(), "page": "pa", "tag": "3PPA", "pa_state": "solve4",
               "pa_state_code": 16, "error": "aborted", "code": 253, "state": "fail", "result": {"duration_ms": 900}})
    print("sent aborted (253)", flush=True)
    time.sleep(0.5)


if __name__ == "__main__":
    main()
