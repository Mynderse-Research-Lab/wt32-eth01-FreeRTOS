#!/usr/bin/env python3
"""
Generates two publication-grade PDFs for Grasp Timing Mathematics:
1. docs/Grasp_Timing_Executive_Brief.pdf (Presentable Executive & Engineering Briefing)
2. docs/Grasp_Timing_Mathematical_Specification.pdf (Rigorous Mathematical Specification & Derivations)
"""

import os
import sys
import base64
import subprocess
import time

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DOCS_DIR = os.path.join(REPO_ROOT, "docs")
FIG_DIR = os.path.join(DOCS_DIR, "figures_pdf")
CHROME_PATH = r"C:\Program Files\Google\Chrome\Application\chrome.exe"

def img_to_b64(fname):
    path = os.path.join(FIG_DIR, fname)
    if not os.path.exists(path):
        return ""
    with open(path, "rb") as f:
        return f"data:image/png;base64,{base64.b64encode(f.read()).decode('utf-8')}"

# Pre-load base64 images
b64_kinematics = img_to_b64("kinematic_frames.png")
b64_trajectories = img_to_b64("synchronized_timing_trajectories.png")
b64_clearance = img_to_b64("separable_axis_clearance.png")
b64_jitter = img_to_b64("jitter_distributions.png")

print(f"Loaded figures: kinematics={len(b64_kinematics)}, traj={len(b64_trajectories)}, clear={len(b64_clearance)}, jitter={len(b64_jitter)}")
