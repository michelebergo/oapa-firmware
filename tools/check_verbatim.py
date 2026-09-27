#!/usr/bin/env python3
"""Fails unless src/oapa_protocol.inc is byte-identical (LF-normalized)
to lines 55-412 of oapa-firmware 1.2.3 (oapa.ino). That region is everything N.I.N.A.
talks to; 1.3.0 must not change a byte of it."""
import hashlib
import pathlib
import sys

EXPECTED_SHA256 = "70954a20c0d11cf7899e521552d64950fbc9b153937877e68bf844a27afde57b"
INC = pathlib.Path(__file__).resolve().parent.parent / "src" / "oapa_protocol.inc"


def main():
    if not INC.exists():
        print(f"FAIL: {INC} missing")
        return 1
    text = INC.read_bytes().replace(b"\r\n", b"\n")
    actual = hashlib.sha256(text).hexdigest()
    if actual != EXPECTED_SHA256:
        print(f"FAIL: protocol region changed\n  expected {EXPECTED_SHA256}\n  actual   {actual}")
        return 1
    print("OK: protocol region identical to 1.2.3")
    return 0


if __name__ == "__main__":
    sys.exit(main())
