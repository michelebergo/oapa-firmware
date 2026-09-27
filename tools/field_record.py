#!/usr/bin/env python3
"""Records what the board publishes during a field session, without taking a
client slot on ASIAIR's port 4700 (it accepts only a couple of them).

    python tools/field_record.py 192.168.1.15 --out captures/field_board.log

One line per poll: isoTimestamp<TAB>kind<TAB>json. Events and history are
written once each, as they appear.
"""
import argparse
import json
import sys
import time
import urllib.error
import urllib.request


def get(host, path):
    try:
        with urllib.request.urlopen(f"http://{host}{path}", timeout=6) as response:
            return json.loads(response.read())
    except (urllib.error.URLError, OSError, ValueError):
        return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument("board")
    p.add_argument("--out", default="captures/field_board.log")
    p.add_argument("--period", type=float, default=2.0)
    p.add_argument("--duration", type=float, default=10800)
    a = p.parse_args()

    last_event = 0
    last_sample = 0
    started = time.time()
    with open(a.out, "a", encoding="utf-8") as log:
        def write(kind, value):
            log.write(f"{time.strftime('%Y-%m-%dT%H:%M:%S')}\t{kind}\t{json.dumps(value, separators=(',', ':'))}\n")
            log.flush()

        print(f"recording {a.board} to {a.out}, Ctrl+C to stop", flush=True)
        while time.time() - started < a.duration:
            for kind, path in (("status", "/api/status"), ("asiair", "/api/asiair"), ("loop", "/api/loop"),
                               ("calibrate", "/api/calibrate")):
                body = get(a.board, path)
                if body is not None:
                    write(kind, body)
            events = get(a.board, f"/api/events?since={last_event}")
            if events and events["events"]:
                last_event = events["last"]
                for event in events["events"]:
                    write("event", event)
                    print("event:", event["code"], event["text"], flush=True)
            history = get(a.board, f"/api/history?since={last_sample}")
            if history and history["samples"]:
                last_sample = history["last"]
                for sample in history["samples"]:
                    write("sample", sample)
            time.sleep(a.period)
    return 0


if __name__ == "__main__":
    sys.exit(main())
