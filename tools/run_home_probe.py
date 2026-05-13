"""Issue `home` over the live console and capture the resulting log lines.

Captures every line from the firmware to debug-06947d.log as NDJSON, with
special-case parsing for setDirectionPin, [X_MOVE], and MCP register dumps.
Also writes a raw verbatim copy to home_probe_raw.log for backup.
"""
import json
import re
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
LOG = sys.argv[3] if len(sys.argv) > 3 else "debug-06947d.log"
RAW = "home_probe_raw.log"
SESSION = "06947d"
RUN = "home-probe-postfix"
HYP = "S-7"

re_setdir = re.compile(r"setDirectionPin: logical=(\d)")
re_xmove = re.compile(r"\[X_MOVE\] (.+)")
re_dump_a = re.compile(r"MCP dump port A:.*GPIOA=0x([0-9A-Fa-f]+).*OLATA=0x([0-9A-Fa-f]+)", re.IGNORECASE)
re_alarm = re.compile(r"ALARM|stopAllMotion|abort|FAIL", re.IGNORECASE)
re_home_seed = re.compile(r"\[HOME\] seeded position tracker to (\d+)")
re_live_pos = re.compile(r"LIVE POS: x_cmd=(-?[\d.]+) mm, x_enc=(-?[\d.]+) mm")
re_home_done = re.compile(r"OK Homing|already at MIN/home switch|Homing did not start|Homing complete", re.IGNORECASE)


def emit(loc, msg, data, hyp=HYP):
    rec = {
        "sessionId": SESSION,
        "runId": RUN,
        "hypothesisId": hyp,
        "location": loc,
        "message": msg,
        "data": data,
        "timestamp": int(time.time() * 1000),
    }
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(json.dumps(rec) + "\n")


def main():
    s = serial.Serial(PORT, 115200, timeout=0.05)
    s.reset_input_buffer()
    raw = open(RAW, "wb")

    def drain(secs, label=""):
        deadline = time.time() + secs
        buf = b""
        while time.time() < deadline:
            chunk = s.read(8192)
            if not chunk:
                continue
            raw.write(chunk); raw.flush()
            buf += chunk
            *lines, buf = buf.split(b"\n")
            for ln in lines:
                txt = ln.decode("utf-8", errors="replace")
                # strip ANSI
                txt = re.sub(r"\x1b\[[0-9;]*m", "", txt).strip()
                if not txt:
                    continue
                m = re_setdir.search(txt)
                if m:
                    emit("PulseMotor.cpp:setDirectionPin",
                         "DIR pin write captured",
                         {"logical_high": int(m.group(1)), "label": label, "raw": txt})
                    continue
                m = re_xmove.search(txt)
                if m:
                    emit("Gantry.cpp:startXAxisMotion",
                         "[X_MOVE] line", {"line": m.group(1), "label": label})
                    continue
                m = re_dump_a.search(txt)
                if m:
                    gpioa = int(m.group(1), 16)
                    olata = int(m.group(2), 16)
                    emit("MCP23S17:port_a",
                         f"PA dump {label}",
                         {
                             "gpioa": f"0x{gpioa:02x}",
                             "olata": f"0x{olata:02x}",
                             "pa0_dir": (olata >> 0) & 1,
                             "pa1_en": (olata >> 1) & 1,
                             "pa3_max": (gpioa >> 3) & 1,
                             "pa2_min": (gpioa >> 2) & 1,
                             "label": label,
                         })
                    continue
                m = re_home_seed.search(txt)
                if m:
                    emit("Gantry.cpp:home_seed",
                         "[HOME] seed-fix line captured",
                         {"seeded_to": int(m.group(1)), "label": label})
                    continue
                m = re_live_pos.search(txt)
                if m:
                    emit("GantryConsole:live_pos",
                         "live position",
                         {"x_cmd_mm": float(m.group(1)),
                          "x_enc_mm": float(m.group(2)),
                          "label": label})
                    continue
                m = re_home_done.search(txt)
                if m:
                    emit("GantryConsole:home_outcome",
                         "homing outcome line",
                         {"line": txt, "label": label})
                    continue
                if re_alarm.search(txt):
                    emit("firmware",
                         "alarm/abort signal",
                         {"line": txt, "label": label})
        return buf

    emit("run_home_probe.py:start", "begin", {"port": PORT, "secs": SECS})

    # Phase 1: pre-snapshot
    s.write(b"mcp_dump a\r\n")
    drain(1.5, "pre")

    # Phase 2: issue home
    emit("run_home_probe.py:issue_home", "sending home", {})
    s.write(b"home\r\n")

    # Phase 3: capture during home (most of the budget)
    drain(max(SECS - 4.0, 6.0), "during")

    # Phase 4: post-snapshot
    s.write(b"mcp_dump a\r\n")
    drain(1.5, "post")
    s.write(b"status\r\n")
    drain(1.5, "post-status")

    emit("run_home_probe.py:end", "complete", {})
    raw.close()
    s.close()
    print("done")


if __name__ == "__main__":
    main()
