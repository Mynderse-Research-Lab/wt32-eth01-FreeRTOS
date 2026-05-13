"""Issue `home` then `calibrate` and capture every relevant log line.

Used to verify S-8 fix. Captures:
  - [HOME] seeded position tracker        (S-7 — should still fire)
  - setDirectionPin: logical=...          (motion direction)
  - OK Homing started / Homing skipped    (home outcome)
  - CTRL FLIP: min_limit / max_limit / busy
  - [CAL] travel measurement complete: axisLength=...mm   (S-8 success path)
  - [CAL] inner loop exited but xMaxSwitch_ not active    (S-8 genuine failure)
  - Calibration failed                    (console layer)
"""
import json
import re
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
LOG = sys.argv[3] if len(sys.argv) > 3 else "debug-06947d.log"
RAW = "calibrate_probe_raw.log"
SESSION = "06947d"
RUN = "calibrate-probe-postfix"
HYP = "S-8"

re_setdir = re.compile(r"setDirectionPin: logical=(\d)")
re_home_seed = re.compile(r"\[HOME\] seeded position tracker to (\d+)")
re_cal_done = re.compile(r"\[CAL\] travel measurement complete: axisLength=(-?\d+) mm")
re_cal_unexp = re.compile(r"\[CAL\] inner loop exited but xMaxSwitch_ not active")
re_ctrl_flip = re.compile(r"CTRL FLIP: (\w+) (-?\d+) -> (-?\d+)")
re_outcome = re.compile(
    r"OK Homing started|OK Homing skipped|Homing did not start|"
    r"OK Calibrated length:|Calibration failed|Calibration aborted",
    re.IGNORECASE,
)


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
                txt = re.sub(r"\x1b\[[0-9;]*m", "", txt).strip()
                if not txt:
                    continue

                m = re_home_seed.search(txt)
                if m:
                    emit("Gantry.cpp:home_seed",
                         "[HOME] seed", {"seeded_to": int(m.group(1)), "label": label})
                    continue
                m = re_cal_done.search(txt)
                if m:
                    emit("Gantry.cpp:calibrate_success",
                         "[CAL] success",
                         {"axis_length_mm": int(m.group(1)), "label": label})
                    continue
                if re_cal_unexp.search(txt):
                    emit("Gantry.cpp:calibrate_unexpected",
                         "[CAL] unexpected non-active MAX",
                         {"label": label, "raw": txt})
                    continue
                m = re_setdir.search(txt)
                if m:
                    emit("PulseMotor.cpp:setDirectionPin",
                         "DIR write",
                         {"logical_high": int(m.group(1)), "label": label})
                    continue
                m = re_ctrl_flip.search(txt)
                if m:
                    emit("GantryConsole:ctrl_flip",
                         f"flip {m.group(1)}",
                         {"name": m.group(1), "from": int(m.group(2)),
                          "to": int(m.group(3)), "label": label})
                    continue
                if re_outcome.search(txt):
                    emit("GantryConsole:outcome",
                         "outcome line",
                         {"line": txt, "label": label})
        return buf

    emit("run_calibrate_probe.py:start", "begin", {"port": PORT, "secs": SECS})

    # Phase 1: home
    s.write(b"home\r\n")
    drain(8.0, "home")

    # Phase 2: calibrate
    s.write(b"calibrate\r\n")
    drain(max(SECS - 9.0, 12.0), "calibrate")

    # Phase 3: status
    s.write(b"status\r\n")
    drain(1.5, "post-status")

    emit("run_calibrate_probe.py:end", "complete", {})
    raw.close()
    s.close()
    print("done")


if __name__ == "__main__":
    main()
