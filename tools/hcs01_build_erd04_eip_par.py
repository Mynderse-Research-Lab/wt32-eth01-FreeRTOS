#!/usr/bin/env python3
"""Recreate a full HCS01 SERCOS-ASCII parameter file from the live dump.

Reads tools/hcs01_Parameter_all_1_cip22.par (MPB-20V30 / W0005, original block
order) and writes the complete file with overlays:

- CIP / engineering IP 192.168.1.22, protocol 3, profile 0xFFFE, gw 0.0.0.0
- SCHUNK ERD04 Motordatenblatt motor / encoder / loop values
- FO assemblies 101/102 process-data maps (P-0-4081 / P-0-4080)
- P-0-0300 duplicate P-0-0222 removed (C0242)

Does not copy the SCHUNK 2013 W0054 dump (wrong drive, protocol 6, 192.168.0.1).
"""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "tools" / "hcs01_Parameter_all_1_cip22.par"
OUT = ROOT / "tools" / "hcs01_ERD04_eip_192_168_1_22.par"
OUT_ALIAS = ROOT / "tools" / "hcs01_cip_ip_192_168_1_22.par"

HEADER = """\
SERCOS-ASCII
2026-08-18 00:33:00
ERD04-eip-192.168.1.22
FWA-INDRV*-MPB-20V30-D5-1-NNN-NN | HCS01.1E-W0005-A-03-B-ET-EC-NN-NN-NN-FW | ERD04 | CSB02.1C-ET-EC-NN-NN-NN-FW
1
"""

# IDNs whose values are overlaid; all other dump blocks are copied unchanged.
OVERLAY_IDNS = [
    "P-0-4089.0.1",
    "P-0-4084.0.0",
    "P-0-4089.0.13",
    "P-0-4089.0.14",
    "P-0-4089.0.15",
    "S-0-1020.0.0",
    "S-0-1021.0.0",
    "S-0-1022.0.0",
    "S-0-0141.0.0",
    "S-0-0109.0.0",
    "S-0-0111.0.0",
    "S-0-0113.0.0",
    "S-0-0092.0.0",
    "P-0-0109.0.0",
    "P-0-0051.0.0",
    "P-0-0018.0.0",
    "P-0-4014.0.0",
    "P-0-4016.0.0",
    "P-0-4017.0.0",
    "P-0-4048.0.0",
    "P-0-4005.0.0",
    "P-0-4013.0.0",
    "P-0-0512.0.0",
    "S-0-0201.0.0",
    "S-0-0204.0.0",
    "P-0-4034.0.0",
    "P-0-4035.0.0",
    "S-0-0277.0.0",
    "S-0-0278.0.0",
    "S-0-0049.0.0",
    "S-0-0050.0.0",
    "P-0-0077.0.0",
    "S-0-0103.0.0",
    "S-0-0100.0.0",
    "S-0-0101.0.0",
    "S-0-0104.0.0",
    "S-0-0106.0.0",
    "S-0-0107.0.0",
    "P-0-0004.0.0",
    "P-0-4080.0.0",
    "P-0-4081.0.0",
    "P-0-0300.0.0",
]


def parse_blocks(text: str, nl: str) -> dict[str, str]:
    parts = text.split(f"{nl}|{nl}")
    blocks: dict[str, str] = {}
    for part in parts[1:]:
        body = part.strip("\r\n")
        if not body:
            continue
        first = body.split(nl, 1)[0].strip()
        if first.startswith(("S-", "P-", "E-")):
            blocks[first] = body
    return blocks


def set_last_line(block: str, nl: str, value: str) -> str:
    lines = block.split(nl)
    lines[-1] = value
    return nl.join(lines)


def set_ip(block: str, nl: str, octets: list[int]) -> str:
    lines = block.split(nl)
    if len(lines) < 6:
        raise ValueError(f"IP block too short: {lines[0]}")
    # Trailing 6 lines: 4, 4, o, o, o, o  (some blocks have extra -- before that)
    lines[-6:] = ["4", "4", *[str(x) for x in octets]]
    return nl.join(lines)


def set_list_items(block: str, nl: str, nbytes: int, nmax: int, items: list[str]) -> str:
    lines = block.split(nl)
    # Keep IDN, name, attr, unit, min, max (first 6 lines of a typical list).
    head = lines[:6]
    tail = [str(nbytes), str(nmax), *items]
    return nl.join(head + tail)


def main() -> None:
    raw = SRC.read_bytes()
    nl = "\r\n" if b"\r\n" in raw[:200] else "\n"
    text = raw.decode("latin-1")
    blocks = parse_blocks(text, nl)
    missing = [idn for idn in OVERLAY_IDNS if idn not in blocks]
    if missing:
        raise SystemExit(f"IDNs missing from dump: {missing}")

    b = dict(blocks)
    overlay_set = set(OVERLAY_IDNS)

    # CIP / engineering — keep protocol 3 and .22; drop W5500 as gateway.
    b["S-0-1022.0.0"] = set_ip(b["S-0-1022.0.0"], nl, [0, 0, 0, 0])

    # ERD04 Motordatenblatt (IndraDrive CS) + catalog In=0.43 A.
    b["S-0-0141.0.0"] = set_last_line(b["S-0-0141.0.0"], nl, "5 80 ERD04")
    b["S-0-0109.0.0"] = set_last_line(b["S-0-0109.0.0"], nl, "1.290")
    b["S-0-0111.0.0"] = set_last_line(b["S-0-0111.0.0"], nl, "0.430")
    b["S-0-0113.0.0"] = set_last_line(b["S-0-0113.0.0"], nl, "600.0000")
    b["S-0-0092.0.0"] = set_last_line(b["S-0-0092.0.0"], nl, "322.5")
    b["P-0-0109.0.0"] = set_last_line(b["P-0-0109.0.0"], nl, "322.5")
    b["P-0-0051.0.0"] = set_last_line(b["P-0-0051.0.0"], nl, "0.90")
    b["P-0-0018.0.0"] = set_last_line(b["P-0-0018.0.0"], nl, "7")
    b["P-0-4014.0.0"] = set_last_line(b["P-0-4014.0.0"], nl, "0000.0010.0000.0000")
    b["P-0-4016.0.0"] = set_last_line(b["P-0-4016.0.0"], nl, "26.900")
    b["P-0-4017.0.0"] = set_last_line(b["P-0-4017.0.0"], nl, "26.900")
    b["P-0-4048.0.0"] = set_last_line(b["P-0-4048.0.0"], nl, "70.000")
    b["P-0-4005.0.0"] = set_last_line(b["P-0-4005.0.0"], nl, "0.000")
    b["P-0-0512.0.0"] = set_last_line(b["P-0-0512.0.0"], nl, "3")
    b["S-0-0201.0.0"] = set_last_line(b["S-0-0201.0.0"], nl, "85.0")
    b["S-0-0204.0.0"] = set_last_line(b["S-0-0204.0.0"], nl, "90.0")
    b["P-0-4035.0.0"] = set_last_line(b["P-0-4035.0.0"], nl, "20.0")
    # Travel: S-0-0278=180 caused F2057 at joint +10 from HIPERFACE ~178.5°.
    # Restore the command-span range; software limits ±180 match thetalim.
    b["S-0-0278.0.0"] = set_last_line(b["S-0-0278.0.0"], nl, "36000.0000")
    b["S-0-0049.0.0"] = set_last_line(b["S-0-0049.0.0"], nl, "180.0000")
    b["S-0-0050.0.0"] = set_last_line(b["S-0-0050.0.0"], nl, "-180.0000")
    b["S-0-0100.0.0"] = set_last_line(b["S-0-0100.0.0"], nl, "0.030")
    b["S-0-0101.0.0"] = set_last_line(b["S-0-0101.0.0"], nl, "10.0")
    b["S-0-0106.0.0"] = set_last_line(b["S-0-0106.0.0"], nl, "40.00")
    b["S-0-0107.0.0"] = set_last_line(b["S-0-0107.0.0"], nl, "0.5")
    b["P-0-0004.0.0"] = set_last_line(b["P-0-0004.0.0"], nl, "500")

    # Instance 102 actual: P-0-4078 + S-0-0051 + S-0-0040 + S-0-0390 (14 B).
    b["P-0-4080.0.0"] = set_list_items(
        b["P-0-4080.0.0"], nl, 16, 64,
        ["P-0-4078", "S-0-0051", "S-0-0040", "S-0-0390"],
    )
    # Instance 101 command: P-0-4077 + S-0-0282 + S-0-0259 + S-0-0260 + S-0-0359 (18 B).
    b["P-0-4081.0.0"] = set_list_items(
        b["P-0-4081.0.0"], nl, 20, 64,
        ["P-0-4077", "S-0-0282", "S-0-0259", "S-0-0260", "S-0-0359"],
    )

    di = b["P-0-0300.0.0"].split(nl)
    # Replace duplicate P-0-0222 (index after counts) with S-0-0000.
    replaced = 0
    seen_0222 = False
    for i, line in enumerate(di):
        if line == "P-0-0222":
            if seen_0222:
                di[i] = "S-0-0000"
                replaced += 1
            else:
                seen_0222 = True
    if replaced != 1:
        raise SystemExit(f"P-0-0300 duplicate fix: expected 1 replacement, got {replaced}")
    b["P-0-0300.0.0"] = nl.join(di)

    parts = text.split(f"{nl}|{nl}")
    header = HEADER.replace("\n", nl).rstrip(nl)
    out_parts = [header]
    seen: set[str] = set()
    for part in parts[1:]:
        body = part.strip("\r\n")
        if not body:
            continue
        first = body.split(nl, 1)[0].strip()
        if first in overlay_set:
            out_parts.append(b[first])
            seen.add(first)
        else:
            out_parts.append(body)
    if seen != overlay_set:
        raise SystemExit(f"overlay mismatch: missing {overlay_set - seen}")
    out = f"{nl}|{nl}".join(out_parts) + f"{nl}|{nl}"
    payload = out.encode("latin-1")
    OUT.write_bytes(payload)
    OUT_ALIAS.write_bytes(payload)
    print(
        f"wrote {OUT.name} and {OUT_ALIAS.name} "
        f"({OUT.stat().st_size} bytes, {len(out_parts) - 1} IDNs, {len(seen)} overlays)"
    )


if __name__ == "__main__":
    main()
