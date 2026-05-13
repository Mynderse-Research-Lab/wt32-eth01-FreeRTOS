"""Poll the X-MAX limit sensor (MCP23S17 PA3) over the live console.

Sends `mcp_dump a` every ~600 ms, parses GPIOA, and writes one NDJSON line
to debug-06947d.log on every PA2/PA3 state transition. Also records start/end
markers so we can see how long the run was.
"""
import json
import re
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
LOG = sys.argv[3] if len(sys.argv) > 3 else "debug-06947d.log"
SESSION = "06947d"
RUN = "maxlimit-probe-1"
HYP = "F-A"

re_dump = re.compile(r"MCP dump port A:.*GPIOA=0x([0-9A-Fa-f]+)")


def emit(location, message, data):
    rec = {
        "sessionId": SESSION,
        "runId": RUN,
        "hypothesisId": HYP,
        "location": location,
        "message": message,
        "data": data,
        "timestamp": int(time.time() * 1000),
    }
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(json.dumps(rec) + "\n")


def main():
    s = serial.Serial(PORT, 115200, timeout=0.05)
    s.reset_input_buffer()

    emit("poll_max_limit.py:start", "begin polling", {"port": PORT, "secs": SECS})

    deadline = time.time() + SECS
    buf = b""
    last_state = None
    last_dump = 0.0
    DUMP_INTERVAL = 0.6

    while time.time() < deadline:
        if time.time() - last_dump >= DUMP_INTERVAL:
            s.write(b"mcp_dump a\r\n")
            last_dump = time.time()

        chunk = s.read(8192)
        if not chunk:
            continue

        buf += chunk
        # Process only complete lines, keep partial in buf.
        *lines, buf = buf.split(b"\n")
        for raw in lines:
            txt = raw.decode("utf-8", errors="replace")
            m = re_dump.search(txt)
            if not m:
                continue
            gpioa = int(m.group(1), 16)
            pa2 = (gpioa >> 2) & 1
            pa3 = (gpioa >> 3) & 1
            state = (pa2, pa3)
            if state != last_state:
                emit(
                    "poll_max_limit.py:transition",
                    f"PA2={pa2} PA3={pa3}",
                    {
                        "gpioa": f"0x{gpioa:02x}",
                        "pa2_min_open": pa2,
                        "pa3_max_open": pa3,
                        "min_logical": "open" if pa2 else "ACTIVE",
                        "max_logical": "open" if pa3 else "ACTIVE",
                    },
                )
                last_state = state
        # Cap unparsed-tail growth.
        if len(buf) > 8192:
            buf = buf[-1024:]

    emit("poll_max_limit.py:end", "polling complete", {})
    s.close()
    print("done")


if __name__ == "__main__":
    main()
