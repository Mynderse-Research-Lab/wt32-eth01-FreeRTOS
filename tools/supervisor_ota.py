#!/usr/bin/env python3
"""Dual-OTA flash through the supervisor's developer toolbox HTTP API.

Does what the toolbox page does, without the browser:
  1. log in to /dev/login (password from $ADSB_DEV_PASS, or a prompt),
  2. send `stop`, then `status`, and refuse to flash unless the motors are
     disabled and nothing is busy,
  3. POST the image to /api/ota and follow /api/ota/status,
  4. wait for the gantry to come back and check that the running slot changed
     and the uptime restarted.

The password is never printed or written anywhere.

Usage:
  set ADSB_DEV_PASS in the environment (or add --prompt to type it), then
  python tools/supervisor_ota.py [idf/build/wt32_eth01_gantry.bin] [--host 10.168.40.1]
         [--transport cellnet|tcp]

The supervisor sends over CellNet L2 by default (since 2026-11-14); --transport
tcp uses its TCP relay to the gantry's port 8032 instead.
"""
from __future__ import annotations

import argparse
import getpass
import http.cookiejar
import json
import os
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

DEFAULT_HOST = "10.168.40.1"
DEFAULT_BIN = os.path.join("idf", "build", "wt32_eth01_gantry.bin")
PASSWORD_ENV = "ADSB_DEV_PASS"
CONSOLE_TIMEOUT_S = 15.0
UPLOAD_TIMEOUT_S = 300.0
OTA_DONE_TIMEOUT_S = 240.0
REBOOT_TIMEOUT_S = 120.0


class Toolbox:
    def __init__(self, host: str) -> None:
        self.base = f"http://{host}"
        self.jar = http.cookiejar.CookieJar()
        self.opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(self.jar))

    def login(self, password: str) -> None:
        body = urllib.parse.urlencode({"password": password}).encode()
        req = urllib.request.Request(f"{self.base}/dev/login", data=body, method="POST")
        req.add_header("Content-Type", "application/x-www-form-urlencoded")
        with self.opener.open(req, timeout=10) as resp:
            page = resp.read().decode("utf-8", errors="replace")
        if not any(True for _ in self.jar) or "Toolbox password" in page:
            raise SystemExit("[ERROR] toolbox login refused (wrong password?)")

    def console(self, line: str, timeout: float = CONSOLE_TIMEOUT_S) -> str:
        body = json.dumps({"line": line}).encode()
        req = urllib.request.Request(f"{self.base}/api/console", data=body, method="POST")
        req.add_header("Content-Type", "application/json")
        try:
            with self.opener.open(req, timeout=timeout) as resp:
                return json.loads(resp.read().decode("utf-8", errors="replace")).get("text", "")
        except urllib.error.HTTPError as exc:
            if exc.code == 401:
                raise SystemExit("[ERROR] toolbox session expired (401)") from exc
            raise
        except (TimeoutError, urllib.error.URLError) as exc:
            return f"__NO_REPLY__ {exc}"

    def ota_status(self) -> dict:
        with self.opener.open(f"{self.base}/api/ota/status", timeout=10) as resp:
            return json.loads(resp.read().decode("utf-8", errors="replace"))

    def upload(self, image: bytes, transport: str | None = None) -> str:
        url = f"{self.base}/api/ota" + (f"?transport={transport}" if transport else "")
        req = urllib.request.Request(url, data=image, method="POST")
        req.add_header("Content-Type", "application/octet-stream")
        with self.opener.open(req, timeout=UPLOAD_TIMEOUT_S) as resp:
            return resp.read().decode("utf-8", errors="replace")


def field(text: str, name: str) -> str | None:
    m = re.search(rf"{re.escape(name)}\s*:\s*([^\r\n]+)", text)
    return m.group(1).strip() if m else None


def slot_and_uptime(tb: Toolbox) -> tuple[str | None, int | None]:
    text = tb.console("ota")
    slot = field(text, "Running Slot")
    m = re.search(r"uptime (\d+) s", text)
    return (slot.split()[0] if slot else None), (int(m.group(1)) if m else None)


def preflight(tb: Toolbox, discard_unsaved: bool) -> None:
    """stop, then status: motors must be disabled and nothing busy."""
    print("> stop")
    print("  " + (re.search(r"OK[^\r\n]*|ERROR[^\r\n]*", tb.console("stop")) or [""])[0])
    time.sleep(2.0)
    status = tb.console("status")
    enabled = field(status, "Motor Enabled")
    busy = field(status, "Busy")
    print(f"> status\n  Motor Enabled: {enabled} | Busy: {busy} | Alarm: {field(status, 'Alarm')}")
    if enabled != "No" or busy != "No":
        raise SystemExit("[ERROR] refusing to flash: motors enabled or gantry busy")
    unsaved = field(status, "Unsaved parameters")
    if unsaved not in (None, "0") and not discard_unsaved:
        raise SystemExit(f"[ERROR] {unsaved} unsaved parameter(s) would be lost: "
                         "run 'cfg save', or pass --discard-unsaved")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("binary", nargs="?", default=DEFAULT_BIN)
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--transport", choices=("cellnet", "tcp"), default=None,
                    help="OTA transport; default: the supervisor's (CellNet L2)")
    ap.add_argument("--prompt", action="store_true",
                    help=f"ask for the toolbox password when {PASSWORD_ENV} is not set")
    ap.add_argument("--discard-unsaved", action="store_true",
                    help="flash even though parameters changed since the last 'cfg save'")
    args = ap.parse_args()

    with open(args.binary, "rb") as f:
        image = f.read()
    print(f"image {args.binary}: {len(image)} bytes, "
          f"built {time.strftime('%Y-%m-%d %H:%M', time.localtime(os.path.getmtime(args.binary)))}")

    password = os.environ.get(PASSWORD_ENV)
    if not password:
        # Git Bash reports a console even when nobody is there to type: only
        # prompt when asked, so unattended runs fail fast instead of hanging.
        if not args.prompt:
            raise SystemExit(f"[ERROR] set {PASSWORD_ENV}, or pass --prompt to type it")
        password = getpass.getpass("Toolbox password: ")

    tb = Toolbox(args.host)
    tb.login(password)
    del password
    print(f"logged in to {tb.base}/dev")

    preflight(tb, args.discard_unsaved)
    old_slot, old_up = slot_and_uptime(tb)
    print(f"running slot {old_slot}, uptime {old_up} s")

    print(f"uploading {len(image)} bytes ...")
    reply = tb.upload(image, args.transport)
    print("  " + reply.strip()[:200])

    deadline = time.time() + OTA_DONE_TIMEOUT_S
    last = ""
    while time.time() < deadline:
        st = tb.ota_status()
        line = f"{st.get('state')} {st.get('sent')}/{st.get('total')} {st.get('message', '')}"
        if line != last:
            print("  " + line)
            last = line
        if st.get("total") == len(image) and st.get("state") == "ok":
            break
        if st.get("state") in ("error", "failed"):
            raise SystemExit("[ERROR] OTA failed: " + str(st.get("message")))
        time.sleep(1.0)
    else:
        raise SystemExit("[ERROR] OTA did not complete in time")

    print("waiting for the gantry to reboot ...")
    deadline = time.time() + REBOOT_TIMEOUT_S
    time.sleep(15.0)
    while time.time() < deadline:
        slot, up = slot_and_uptime(tb)
        if slot and up is not None and (old_up is None or up < old_up):
            print(f"running slot {slot}, uptime {up} s")
            if slot == old_slot:
                print("[WARN] slot did not change: the new image may have rolled back")
                return 1
            print("OK flashed and rebooted")
            return 0
        time.sleep(3.0)
    raise SystemExit("[ERROR] gantry did not come back with a fresh uptime")


if __name__ == "__main__":
    sys.exit(main())
