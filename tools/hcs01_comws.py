#!/usr/bin/env python3
"""HCS01 Service Tool COMWS client (engineering HTTP).

Default target is 192.168.1.22. Does not write CIP/FKM IP (P-0-4089.0.13).
"""

from __future__ import annotations

import socket
import subprocess
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Callable

DEFAULT_HOST = "192.168.1.22"
DEFAULT_CIP_HOST = "192.168.1.23"
ADMIN_PASSWD_HASH = "526545e524f08d95e0514ab5fe3aac46"

# Never write these via set helpers used by the eng CLI.
CIP_IP_IDNS = frozenset(
    {
        "P-0-4089.0.13",
        "P-0-4089.0.14",
        "P-0-4089.0.15",
    }
)

READY_NAMES = {
    0: "not-ready",
    1: "Bb",
    2: "Ab",
    3: "AF",
}


class ComwsError(RuntimeError):
    """COMWS / SERCOS command failure."""


def normalize_idn(idn: str) -> str:
    """Ensure IDN has a .0.0 (or EIDN) element suffix when callers pass bare names."""
    s = idn.strip()
    if not s:
        raise ValueError("empty IDN")
    # Already has element path (e.g. S-0-0095.0.0 or P-0-4089.0.13)
    if s.count(".") >= 2:
        return s
    return f"{s}.0.0"


def parse_status_word(raw: str) -> int:
    """Parse COMWS status/value: decimal, 0b…, or 0x…."""
    s = (raw or "").strip().replace(" ", "")
    if not s:
        return 0
    if s.lower().startswith("0b"):
        return int(s[2:], 2)
    if s.lower().startswith("0x"):
        return int(s, 16)
    if set(s) <= set("01") and len(s) >= 4:
        return int(s, 2)
    try:
        return int(s, 10)
    except ValueError:
        return 0


def decode_control_word(raw: int) -> str:
    bits = []
    if raw & (1 << 0):
        bits.append("accept")
    if raw & (1 << 1):
        bits.append("OM")
    if raw & (1 << 5):
        bits.append("C0500")
    # Bit13: 0 = Drive Halt active, 1 = Halt inactive (motion allowed)
    if raw & (1 << 13):
        bits.append("HaltInactive")
    else:
        bits.append("HaltActive")
    if raw & (1 << 14):
        bits.append("DriveEnable")
    if raw & (1 << 15):
        bits.append("DriveON")
    return " ".join(bits) if bits else "(none)"


def decode_status_word(raw: int) -> str:
    ready = (raw >> 14) & 0x3
    om_ack = raw & 0x3
    parts = [
        f"ready={ready}({READY_NAMES.get(ready, '?')})",
        f"om={om_ack}",
    ]
    if raw & (1 << 2):
        parts.append("in-ref")
    if raw & (1 << 3):
        parts.append("standstill")
    if raw & (1 << 4):
        parts.append("reached")
    if raw & (1 << 7):
        parts.append("not-following")
    if raw & (1 << 11):
        parts.append("class3")
    if raw & (1 << 12):
        parts.append("class2")
    if raw & (1 << 13):
        parts.append("class1")
    return " ".join(parts)


def ping_host(host: str, timeout_ms: int = 800) -> bool:
    r = subprocess.run(
        ["ping", "-n", "1", "-w", str(timeout_ms), host],
        capture_output=True,
        text=True,
    )
    out = (r.stdout or "") + (r.stderr or "")
    return "ttl=" in out.lower()


def tcp_probe(host: str, port: int, timeout: float = 1.5) -> str:
    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect((host, port))
        s.close()
        return "OPEN"
    except Exception as e:
        return type(e).__name__


class Hcs01Comws:
    """Session against IndraDrive Service Tool COMWS on the engineering IP."""

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        cip_host: str = DEFAULT_CIP_HOST,
        timeout: float = 15.0,
        log: Callable[[str], None] | None = print,
    ) -> None:
        self.host = host
        self.cip_host = cip_host
        self.timeout = timeout
        self.base = f"http://{host}"
        self.login_url = (
            f"{self.base}/login.cgi?name=administrator&passwd={ADMIN_PASSWD_HASH}"
        )
        self._log = log if log is not None else (lambda _m: None)

    def _say(self, msg: str) -> None:
        self._log(msg)

    def http_get(self, url: str, timeout: float | None = None) -> str:
        t = self.timeout if timeout is None else timeout
        with urllib.request.urlopen(url, timeout=t) as r:
            return r.read().decode("latin-1", errors="replace")

    def http_post(self, url: str, data: str, timeout: float | None = None) -> str:
        t = self.timeout if timeout is None else timeout
        req = urllib.request.Request(
            url,
            data=data.encode("ascii"),
            method="POST",
            headers={"Content-Type": "application/x-www-form-urlencoded"},
        )
        with urllib.request.urlopen(req, timeout=t) as r:
            return r.read().decode("latin-1", errors="replace")

    def login(self) -> str:
        body = self.http_get(self.login_url).strip()
        self._say(f"login {body[:80]}")
        return body

    def _var_key(self, idn: str) -> str:
        n = normalize_idn(idn)
        if n.startswith("0,1,"):
            return n
        return f"0,1,{n}"

    def getvar(self, idn: str) -> str:
        key = self._var_key(idn)
        q = urllib.parse.quote(key, safe=",.")
        body = self.http_get(f"{self.base}/getvar.cgi?var1={q}")
        if "=" in body:
            return body.split("=", 1)[1].strip()
        return body.strip()

    def getlst(self, idn: str, count: int = 16) -> str:
        key = self._var_key(idn)
        q = urllib.parse.quote(key, safe=",.")
        return self.http_get(
            f"{self.base}/getlst.cgi?var={q}&offset=0&count={count}"
        ).strip()

    def setvar(self, idn: str, value: str) -> str:
        key = self._var_key(idn)
        bare = key.split(",", 1)[-1] if key.startswith("0,1,") else key
        for blocked in CIP_IP_IDNS:
            if bare.startswith(blocked) or blocked in bare:
                raise ComwsError(
                    f"refusing to write {bare}: CIP IP IDNs are off-limits"
                )
        # COMWS wants literal commas in var=; do not percent-encode them.
        val = urllib.parse.quote(value, safe="xXb0-|.")
        return self.http_post(
            f"{self.base}/setvar.cgi", f"var={key}&value={val}"
        ).strip()

    def setlst(self, idn: str, items: list[str]) -> str:
        key = self._var_key(idn)
        joined = "|".join(items)
        return self.http_post(
            f"{self.base}/setlst.cgi", f"var={key}&values={joined}"
        ).strip()

    def setvar_try_element7(self, idn: str, value: str) -> str:
        """Write via .7 first (writable element), fall back to bare IDN."""
        n = normalize_idn(idn)
        r = self.setvar(f"{n}.7", value)
        if "error" in r.lower():
            r = self.setvar(n, value)
        return r

    def run_command(self, idn: str, timeout_s: float = 20.0) -> str:
        """SERCOS procedure command: clear, set+enable (bits 0+1), poll .8, clear.

        Live HCS01 .8 status (LSB): 0=set, 1=enabled, 2=executing, 3=error.
        Observed C0500: 7 (0b0111 = set+en+exec) then 3 (0b0011 = set+en done).
        Wait while executing; complete when set+enabled and not executing.
        """
        n = normalize_idn(idn)
        if n.endswith(".7"):
            op = n
            st = n[:-2] + ".8"
        else:
            op = f"{n}.7"
            st = f"{n}.8"

        self._say(f"  command {n} ...")
        self._say(f"   clear {self.setvar(op, '0b0000000000000000')}")
        self._say(f"   set  {self.setvar(op, '0b0000000000000011')}")

        deadline = time.time() + timeout_s
        last = ""
        while time.time() < deadline:
            last = self.getvar(st)
            sw = parse_status_word(last)
            set_bit = bool(sw & (1 << 0))
            enabled = bool(sw & (1 << 1))
            executing = bool(sw & (1 << 2))
            error = bool(sw & (1 << 3))
            self._say(f"   status {last} (0x{sw:04X})")

            if error:
                diag = self.getvar("S-0-0095.0.0")
                self._say(f"   diagnostic {diag}")
                self.setvar(op, "0b0000000000000000")
                raise ComwsError(f"command {n} error: {diag}")

            # Done: set+enabled, not executing (e.g. decimal 3 after 7)
            if set_bit and enabled and not executing:
                break
            if "ok" in last.lower() and not executing:
                break
            time.sleep(0.3)
        else:
            self.setvar(op, "0b0000000000000000")
            raise ComwsError(f"command {n} timeout (last status {last})")

        self._say(f"   clear {self.setvar(op, '0b0000000000000000')}")
        return last

    def wait_http(self, timeout_s: float = 90.0) -> bool:
        """Poll until engineering HTTP answers after a reboot."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                self.http_get(self.login_url, timeout=2.0)
                return True
            except (urllib.error.URLError, TimeoutError, OSError):
                time.sleep(1.0)
        return False

    def probe_network(self) -> dict[str, str | bool]:
        return {
            "ping_eng": ping_host(self.host),
            "ping_cip": ping_host(self.cip_host),
            "http80": tcp_probe(self.host, 80),
            "cip44818": tcp_probe(self.cip_host, 44818),
        }
