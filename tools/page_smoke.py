#!/usr/bin/env python3
"""Round-trips the page endpoints on the board. Moves X by +1' and back.

    python tools/page_smoke.py 192.168.1.15
"""
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.15"
failures = 0


def call(path, **fields):
    data = urllib.parse.urlencode(fields).encode() if fields else None
    request = urllib.request.Request(f"http://{HOST}{path}", data=data, method="POST" if fields else "GET")
    try:
        with urllib.request.urlopen(request, timeout=8) as response:
            body = response.read().decode()
            return response.status, (json.loads(body) if body.startswith("{") else body)
    except urllib.error.HTTPError as error:
        return error.code, error.read().decode()


def check(name, condition, detail=""):
    global failures
    failures += 0 if condition else 1
    print(("PASS " if condition else "FAIL ") + name, detail)


def wait_idle():
    for _ in range(100):
        if call("/api/status")[1]["state"] == "Idle":
            return
        time.sleep(0.2)


code, settings = call("/api/settings", factorX=60, factorY=60)
check("settings saved", code == 200 and settings["factorX"] == 60, str(settings))
code, body = call("/api/settings", tolerance=20)
check("invalid tolerance refused", code == 400, str(body))
code, drivers = call("/api/drivers")
check("drivers readable", code == 200 and "x" in drivers, str(drivers))
code, body = call("/api/drivers", axis="X", run=drivers["x"]["run"], hold=drivers["x"]["hold"], micro=drivers["x"]["micro"])
check("drivers re-applied with the same values", code == 202, str(body))
time.sleep(0.5)
check("active drivers unchanged", call("/api/drivers")[1] == drivers)
wait_idle()
before = call("/api/status")[1]["x"]
code, body = call("/api/move", axis="X", arcmin=1)
time.sleep(0.3)
wait_idle()
after = call("/api/status")[1]["x"]
check("move 1' = 60 steps", code == 202 and after - before == 60, f"{body} delta={after - before}")
call("/api/move", axis="X", arcmin=-1)
time.sleep(0.3)
wait_idle()
code, events = call("/api/events?since=0")
codes = {e["code"] for e in events["events"]}
check("events include boot, settings and drivers", {"boot", "settings", "drivers"} <= codes, str(sorted(codes)))
code, history = call("/api/history?since=0")
check("history endpoint answers", code == 200 and "samples" in history, f"{len(history['samples'])} samples")
print("ALL PASS" if failures == 0 else f"{failures} FAILED")
sys.exit(1 if failures else 0)
