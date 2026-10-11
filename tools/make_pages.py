#!/usr/bin/env python3
"""Builds the firmware installer site (GitHub Pages) for one release.

    python tools/make_pages.py 1.3.2 --apps <dir> --out <site dir>

<dir> holds the release images, oapa-firmware-<version>-<env>.bin. The bootloader
and the partition table come from the local PlatformIO build of each env, and
boot_app0 from the Arduino core: they do not change between releases. The site
gets:

  index.html                     the installer page (pages/index.html)
  manifest-<env>.json            what ESP Web Tools writes, and where
  firmware/<version>/<env>/*.bin the four images
  versions.json                  the latest version and its app image per env,
                                 read by the board's own page to offer the update

Parts are written separately rather than as one image from 0x0, so the NVS
partition (0x9000, the board's settings) is left alone unless the user chooses
to erase the device.
"""
import argparse
import json
import pathlib
import shutil
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
ENVS = ["fysetc_e4", "esp32_stepdir"]
BOOT_APP0 = pathlib.Path.home() / ".platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin"
# ESP32 with the min_spiffs table: bootloader, partition table, otadata (boot_app0), app0.
OFFSETS = {"bootloader.bin": 0x1000, "partitions.bin": 0x8000, "boot_app0.bin": 0xE000, "firmware.bin": 0x10000}
NOTES = "https://github.com/michelebergo/oapa-firmware/releases/tag/{version}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("version")
    parser.add_argument("--apps", required=True, type=pathlib.Path)
    parser.add_argument("--out", required=True, type=pathlib.Path)
    args = parser.parse_args()
    version, out = args.version, args.out

    boards = {}
    for env in ENVS:
        build = ROOT / ".pio" / "build" / env
        sources = {
            "bootloader.bin": build / "bootloader.bin",
            "partitions.bin": build / "partitions.bin",
            "boot_app0.bin": BOOT_APP0,
            "firmware.bin": args.apps / f"oapa-firmware-{version}-{env}.bin",
        }
        missing = [str(p) for p in sources.values() if not p.exists()]
        if missing:
            sys.exit("missing: " + ", ".join(missing))
        target = out / "firmware" / version / env
        target.mkdir(parents=True, exist_ok=True)
        for name, source in sources.items():
            shutil.copyfile(source, target / name)
        manifest = {
            "name": "OAPA firmware",
            "version": version,
            "new_install_prompt_erase": True,
            "builds": [{
                "chipFamily": "ESP32",
                "parts": [{"path": f"firmware/{version}/{env}/{name}", "offset": offset} for name, offset in OFFSETS.items()],
            }],
        }
        (out / f"manifest-{env}.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        boards[env] = f"firmware/{version}/{env}/firmware.bin"

    versions = {"latest": version, "notes": NOTES.format(version=version), "boards": boards}
    (out / "versions.json").write_text(json.dumps(versions, indent=2) + "\n", encoding="utf-8")
    page = (ROOT / "pages" / "index.html").read_text(encoding="utf-8")
    page = page.replace("{{VERSION}}", version).replace("{{NOTES_URL}}", NOTES.format(version=version))
    (out / "index.html").write_text(page, encoding="utf-8")
    (out / ".nojekyll").write_text("", encoding="utf-8")
    print(f"site for {version} in {out}")


if __name__ == "__main__":
    main()
