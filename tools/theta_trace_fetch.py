#!/usr/bin/env python3
"""Fetch the gantry's last `thtrace` (1 kHz theta move trace) into a CSV.

Arm and move first, in the toolbox console:
    thtrace arm
    move <x> <z> <theta>

Usage:
    ADSB_DEV_PASS=... python tools/theta_trace_fetch.py --host 10.42.0.1 -o trace.csv

The toolbox password is read from ADSB_DEV_PASS only. The CSV holds one row
per sample: t_ms (from the move command), rel_deg (actual - start angle),
vel_deg_s (drive S-0-0040), status (P-0-4078, hex). The summary and meta
lines go into leading '#' comment rows.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import supervisor_ota as so  # noqa: E402

RAW_PER_CALL = 300  # firmware kRawMaxPerCall
SAMPLE_RE = re.compile(r"(-?\d+):(-?\d+(?:\.\d+)?):(-?\d+(?:\.\d+)?):([0-9A-Fa-f]{4})")
LINE_RE = re.compile(r"\btt (\d+)((?: \S+)+)")


def fetch(tb: so.Toolbox) -> tuple[str, str, list[tuple[int, int, float, float, str]]]:
    summary = tb.console("thtrace", timeout=30)
    m = re.search(r"thtrace move=\d+ n=(\d+)", summary)
    if not m:
        raise SystemExit(f"[ERROR] no trace summary:\n{summary.strip()}")
    n = int(m.group(1))
    meta = ""
    rows: dict[int, tuple[int, int, float, float, str]] = {}
    for start in range(0, n, RAW_PER_CALL):
        text = tb.console(f"thtrace raw {start} {RAW_PER_CALL}", timeout=60)
        if not meta:
            mm = re.search(r"thtrace meta .*", text)
            meta = mm.group(0).strip() if mm else ""
        for line in text.splitlines():
            lm = LINE_RE.search(line)
            if not lm:
                continue
            idx = int(lm.group(1))
            for k, sm in enumerate(SAMPLE_RE.finditer(lm.group(2))):
                rows[idx + k] = (idx + k, int(sm.group(1)), float(sm.group(2)),
                                 float(sm.group(3)), sm.group(4).upper())
    missing = [i for i in range(n) if i not in rows]
    if missing:
        print(f"[WARN] {len(missing)} of {n} samples missing (first {missing[0]})",
              file=sys.stderr)
    return summary.strip(), meta, [rows[i] for i in sorted(rows)]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="10.42.0.1")
    ap.add_argument("-o", "--out", type=Path, required=True)
    args = ap.parse_args()
    password = os.environ.get("ADSB_DEV_PASS")
    if not password:
        raise SystemExit("[ERROR] set ADSB_DEV_PASS")
    tb = so.Toolbox(args.host)
    tb.login(password)
    summary, meta, rows = fetch(tb)
    with args.out.open("w", encoding="utf-8", newline="") as f:
        for line in summary.splitlines():
            f.write(f"# {line}\n")
        if meta:
            f.write(f"# {meta}\n")
        f.write("i,t_ms,rel_deg,vel_deg_s,status\n")
        for i, t_us, rel, vel, st in rows:
            f.write(f"{i},{t_us / 1000.0:.3f},{rel:.4f},{vel:.1f},{st}\n")
    print(summary)
    print(f"{len(rows)} samples -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
