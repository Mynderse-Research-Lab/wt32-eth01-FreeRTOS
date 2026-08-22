#!/usr/bin/env python3
"""Split Kinetix5100 user manual PDF into chapter-level PDFs using pikepdf."""

import pikepdf
import os

INPUT = r"e:\Projects\wt32-eth01-base\driver_datasheets_and_calculations\AB_Kinetix5100_user_manual.pdf"
OUTPUT_DIR = r"e:\Projects\wt32-eth01-base\driver_datasheets_and_calculations\UM004D"

chapters = [
    ("00_Cover_and_TOC",               1,   12),
    ("01_Preface",                     13,  16),
    ("02_Ch01_Start",                  17,  30),
    ("03_Ch02_Plan_and_Install",       31,  49),
    ("04_Ch03_Connector_Data",         50,  78),
    ("05_Ch04_Connect_the_Drive",      79,  111),
    ("06_Ch05_Setup_EtherNetIP",       112, 116),
    ("07_Ch06_Keypad_Interface",       117, 128),
    ("08_Ch07_KNX5100C_Software",      129, 188),
    ("09_Ch08_Studio_5000",            189, 199),
    ("10_Ch09_Tuning",                 200, 233),
    ("11_Ch10_Modes_of_Operation",     234, 285),
    ("12_Ch11_Motion_Control_PR_Mode", 286, 370),
    ("13_Ch12_Motion_Control_Apps",    371, 409),
    ("14_Ch13_Safe_Torque_Off",        410, 422),
    ("15_Ch14_Absolute_Pos_Recovery",  423, 430),
    ("16_Ch15_Programming_Parameters", 431, 448),
    ("17_Ch16_Troubleshooting",        449, 456),
    ("18_AppA_Interconnect_Diagrams",  457, 476),
    ("19_AppB_Upgrade_Firmware",       477, 488),
    ("20_AppC_AddOn_Instructions",     489, 525),
    ("21_AppD_Full_Closed_Loop",       526, 528),
    ("22_AppE_Scope_Function",         529, 538),
    ("23_AppF_Automatic_Device_Config",539, 544),
    ("24_AppG_History_of_Changes",     545, 546),
    ("25_Index",                       547, 556),
]

os.makedirs(OUTPUT_DIR, exist_ok=True)

src = pikepdf.Pdf.open(INPUT)
total = len(src.pages)
print(f"Total pages in source PDF: {total} (expected 556)")
print(f"Source size: {os.path.getsize(INPUT) / 1_000_000:.1f} MB")

for name, start, end in chapters:
    dst = pikepdf.Pdf.new()
    for i in range(start - 1, min(end, total)):
        dst.pages.append(src.pages[i])
    out_path = os.path.join(OUTPUT_DIR, f"{name}.pdf")
    dst.save(out_path, compress_streams=True)
    size_mb = os.path.getsize(out_path) / 1_000_000
    pages = end - start + 1
    print(f"  {name}.pdf  pages {start}-{end}  ({pages}p)  {size_mb:.1f} MB")

print(f"\nDone: {len(chapters)} files written to {OUTPUT_DIR}")
