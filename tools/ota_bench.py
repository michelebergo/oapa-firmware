#!/usr/bin/env python3
"""Checks the WiFi firmware update on the board, then uploads firmware.bin.
The board reboots into the other app slot; the motors do not move.

    python tools/ota_bench.py 192.168.1.15 --password <pw> --firmware .pio/build/fysetc_e4/firmware.bin
    python tools/ota_bench.py 192.168.1.15 --password <pw> --expect-busy   # while N.I.N.A. polls the board
"""
import argparse
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

FAKE_IMAGE = b"\xe9" + b"\x00" * 1023  # enough for the first-chunk checks; never completes an update


def get(host, path):
    with urllib.request.urlopen(f"http://{host}{path}", timeout=8) as response:
        return json.loads(response.read())


def post_form(host, path, **fields):
    request = urllib.request.Request(f"http://{host}{path}", data=urllib.parse.urlencode(fields).encode(),
                                     method="POST")
    return send(request)


def upload(host, image, password, timeout=120):
    boundary = "oapaotaboundary"
    body = (f"--{boundary}\r\nContent-Disposition: form-data; name=\"firmware\"; filename=\"firmware.bin\"\r\n"
            f"Content-Type: application/octet-stream\r\n\r\n").encode() + image + f"\r\n--{boundary}--\r\n".encode()
    request = urllib.request.Request(f"http://{host}/api/ota", data=body, method="POST", headers={
        "Content-Type": f"multipart/form-data; boundary={boundary}", "X-OTA-Password": password})
    return send(request, timeout)


def send(request, timeout=8):
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return response.status, response.read().decode()
    except urllib.error.HTTPError as error:
        return error.code, error.read().decode()


def check(ok, text):
    print(("PASS " if ok else "FAIL ") + text)
    return 0 if ok else 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument("board")
    p.add_argument("--password", required=True)
    p.add_argument("--firmware")
    p.add_argument("--expect-busy", action="store_true")
    a = p.parse_args()
    failures = 0

    if a.expect_busy:
        code, text = upload(a.board, FAKE_IMAGE, a.password)
        return check(code == 409, f"refused while busy: {code} {text}")

    state = get(a.board, "/api/ota")
    print("ota:", state)
    code, text = post_form(a.board, "/api/ota/password", current=a.password, password=a.password)
    failures += check(code == 200, f"password set/confirmed: {code} {text}")

    for attempt in range(3):
        code, text = upload(a.board, FAKE_IMAGE, "wrong-password")
        failures += check(code == 401, f"wrong password #{attempt + 1}: {code} {text}")
    code, text = upload(a.board, FAKE_IMAGE, a.password)
    failures += check(code == 429, f"right password while locked: {code} {text}")
    print("waiting 61 s for the lock to expire")
    time.sleep(61)
    code, text = upload(a.board, b"not a firmware image", a.password)
    failures += check(code == 400, f"non-ESP32 image refused: {code} {text}")
    if failures or not a.firmware:
        return failures

    before = get(a.board, "/api/ota")["partition"]
    with open(a.firmware, "rb") as f:
        image = f.read()
    started = time.time()
    code, text = upload(a.board, image, a.password)
    failures += check(code == 200, f"upload {len(image)} bytes in {time.time() - started:.1f} s: {code} {text}")
    if failures:
        return failures
    time.sleep(8)
    for _ in range(60):
        try:
            status = get(a.board, "/api/status")
            if status["uptime"] < 120:
                break
        except (urllib.error.URLError, OSError, ValueError):
            pass
        time.sleep(1)
    after = get(a.board, "/api/ota")["partition"]
    failures += check(after != before, f"running slot {before} -> {after}, fw {status['fw']}, boot {status['boots']}")
    print("ALL PASS" if failures == 0 else f"{failures} FAILED")
    return failures


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
