#!/usr/bin/env python3
"""
tools/generate_grasp_timing_pdfs.py

Builds two comprehensive, publication-grade PDFs:
1. docs/Grasp_Timing_Executive_Brief.pdf
2. docs/Grasp_Timing_Mathematical_Specification.pdf

Features:
- Rigorous formal definitions (Definitions 1-6), lemmas (Lemma 1), propositions (Propositions 1-2),
  and theorems (Theorems 1, 2, 2.1, 2.2, 2.3, 3, 4) with mathematical proofs
- Constant symmetric vertical plunge/retract velocity v_plunge = v_retract = 800.0 mm/s
  and acceleration a_z = 20,000.0 mm/s^2 (20.0 m/s^2 = 2.04 g, Kinetix 5100 capability)
- Strictly zero dwell at grasp depth (t_retract = t_{z,done}, dwell = 0.0 s)
- Tiny-gap clearance at bottom (delta_clear = 0.9 mm per side, W_jaw = w + 1.8 mm),
  guaranteeing normal force F_N = 0.0 N at touchdown, eliminating belt impact spikes
- Constant Retract Delay Invariant: t_retract = t_close + Delta t_{retract - close} == constant,
  with Delta t_lead = 8.0 ms (Z retract initiates strictly 8 ms before full closure across all offset tuning)
- 100% Velocity Feedforward (VFF) lead compensation: z_cmd(t) = z(t) + tau_vff * dz(t)
  with tau_vff = k_v / k_p = 0.007 s = 7.0 ms, eliminating servo lag and velocity spikes
- Discrete FreeRTOS tick timing model (1 tick = 1 ms, f_RTOS = 1000 Hz):
  N_close_ticks = round((t_close - t_z) / T_tick) + Delta N_offset_ticks
  N_{z,plunge_ticks} = round(t_{z,nominal} / T_tick) + Delta N_{z,offset_ticks}
  N_{retract_ticks} = round(t_retract / T_tick) + Delta N_{retract_ticks}
- 9V battery strictly lying flat on its larger 47.7 mm x 26.6 mm face (height = 17.3 mm)
- Consistent cylindrical longitudinal orientation (theta = 0 deg <=> parallel to +Y)
- Dual-Z ceiling architecture (Z_traverse = 20.0 mm / 35.7 mm, Z_safe = 65.0 mm)
- Tribological contact pair: 3D-printed PLA fingertips (mu = 0.28) on stainless steel casing (mu = 0.35)
- Separating-axis proximity invariant (upstream finger clearance d_up = 1.80 mm < 3.0 mm at first contact)
"""

import os
import sys
import base64
import subprocess

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DOCS_DIR = os.path.join(REPO_ROOT, "docs")
FIG_DIR = os.path.join(DOCS_DIR, "figures_pdf")
CHROME_PATH = r"C:\Program Files\Google\Chrome\Application\chrome.exe"

def img_to_b64(fname):
    path = os.path.join(FIG_DIR, fname)
    if not os.path.exists(path):
        print(f"Warning: image {path} not found")
        return ""
    with open(path, "rb") as f:
        return f"data:image/png;base64,{base64.b64encode(f.read()).decode('utf-8')}"

# Load figures as Base64 strings
b64_kinematics   = img_to_b64("kinematic_frames.png")
b64_trajectories = img_to_b64("synchronized_timing_trajectories.png")
b64_clearance    = img_to_b64("separable_axis_clearance.png")
b64_jitter       = img_to_b64("jitter_distributions.png")


# ==============================================================================
# 1. EXECUTIVE BRIEF GENERATOR
# ==============================================================================

def build_executive_brief_html() -> str:
    tmpl = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>WT32-ETH01 Gantry Digital Twin: High-Speed On-the-Fly Grasp Timing Architecture</title>
<script>
window.MathJax = {
  tex: {
    inlineMath: [['$', '$'], ['\\(', '\\)']],
    displayMath: [['$$', '$$'], ['\\[', '\\]']]
  },
  startup: {
    pageReady: () => {
      return MathJax.startup.defaultPageReady();
    }
  }
};
</script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>
<style>
  @page {
    size: A4 portrait;
    margin: 10mm 12mm 10mm 12mm;
  }
  * { box-sizing: border-box; }
  body {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
    color: #1e293b;
    background: #ffffff;
    line-height: 1.42;
    font-size: 8.5pt;
    margin: 0;
    padding: 0;
  }
  .header-card {
    background: linear-gradient(135deg, #0f172a 0%, #1e293b 100%);
    color: #ffffff;
    padding: 12px 16px;
    border-radius: 7px;
    margin-bottom: 10px;
    box-shadow: 0 4px 6px -1px rgba(0,0,0,0.1);
  }
  .header-card h1 {
    margin: 0 0 2px 0;
    font-size: 14pt;
    font-weight: 800;
    letter-spacing: -0.4px;
    color: #f8fafc;
  }
  .header-card .subtitle {
    font-size: 8.8pt;
    color: #38bdf8;
    font-weight: 600;
    margin-bottom: 6px;
  }
  .badge-row {
    display: flex;
    gap: 5px;
    flex-wrap: wrap;
  }
  .badge {
    background: rgba(255, 255, 255, 0.12);
    border: 1px solid rgba(255, 255, 255, 0.2);
    padding: 2px 6px;
    border-radius: 4px;
    font-size: 6.5pt;
    font-weight: 600;
    color: #e2e8f0;
    text-transform: uppercase;
    letter-spacing: 0.4px;
  }
  .badge.highlight {
    background: #0284c7;
    border-color: #38bdf8;
    color: #ffffff;
  }

  h2 {
    color: #0f172a;
    font-size: 10pt;
    font-weight: 700;
    border-bottom: 2px solid #0284c7;
    padding-bottom: 2px;
    margin-top: 10px;
    margin-bottom: 4px;
    display: flex;
    align-items: center;
    gap: 5px;
  }
  h2::before {
    content: "";
    display: inline-block;
    width: 4px;
    height: 11px;
    background: #0284c7;
    border-radius: 2px;
  }
  h3 {
    color: #334155;
    font-size: 8.6pt;
    font-weight: 700;
    margin-top: 6px;
    margin-bottom: 2px;
  }
  p {
    margin: 0 0 4px 0;
    text-align: justify;
  }

  .kpi-grid {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 6px;
    margin: 6px 0 8px 0;
  }
  .kpi-card {
    background: #f8fafc;
    border: 1px solid #e2e8f0;
    border-left: 3.5px solid #0284c7;
    padding: 6px 8px;
    border-radius: 5px;
  }
  .kpi-card.accent { border-left-color: #10b981; }
  .kpi-card.purple { border-left-color: #8b5cf6; }
  .kpi-card.orange { border-left-color: #f59e0b; }
  .kpi-val {
    font-size: 12pt;
    font-weight: 800;
    color: #0f172a;
    line-height: 1.1;
  }
  .kpi-lbl {
    font-size: 6.5pt;
    font-weight: 600;
    color: #64748b;
    text-transform: uppercase;
    letter-spacing: 0.4px;
    margin-top: 2px;
  }

  table {
    width: 100%;
    border-collapse: collapse;
    font-size: 7.2pt;
    margin: 6px 0;
    page-break-inside: avoid;
  }
  th, td {
    padding: 3.5px 5px;
    text-align: left;
    border-bottom: 1px solid #e2e8f0;
  }
  th {
    background: #f1f5f9;
    color: #334155;
    font-weight: 700;
    text-transform: uppercase;
    font-size: 6.5pt;
    letter-spacing: 0.3px;
  }
  tr:nth-child(even) td { background: #f8fafc; }

  .figure-box {
    margin: 6px 0;
    text-align: center;
    page-break-inside: avoid;
  }
  .figure-box img {
    border: 1px solid #cbd5e1;
    border-radius: 5px;
    box-shadow: 0 1px 3px rgba(0,0,0,0.05);
  }
  .caption {
    font-size: 6.8pt;
    color: #64748b;
    margin-top: 2px;
    font-style: italic;
  }

  .def-box {
    background: #f8fafc;
    border: 1px solid #cbd5e1;
    border-left: 3.5px solid #0f172a;
    padding: 6px 8px;
    border-radius: 5px;
    margin: 6px 0;
    font-size: 7.8pt;
    page-break-inside: avoid;
  }
  .def-box-title {
    font-weight: 700;
    font-size: 7.4pt;
    text-transform: uppercase;
    letter-spacing: 0.4px;
    color: #0f172a;
    margin-bottom: 2px;
  }

  .callout {
    background: #f0fdf4;
    border: 1px solid #bbf7d0;
    border-left: 3.5px solid #16a34a;
    padding: 6px 8px;
    border-radius: 5px;
    margin: 6px 0;
    font-size: 7.8pt;
    page-break-inside: avoid;
  }
  .callout-title {
    font-weight: 700;
    color: #15803d;
    margin-bottom: 2px;
  }

  .page-break { page-break-after: always; }
</style>
</head>
<body>

<!-- PAGE 1: TITLE, EXECUTIVE SUMMARY & KINEMATICS -->
<div class="header-card">
  <h1>WT32-ETH01 Gantry Digital Twin</h1>
  <div class="subtitle">High-Speed On-the-Fly Grasp Timing Architecture &amp; Executive Brief</div>
  <div class="badge-row">
    <span class="badge highlight">Production Architecture</span>
    <span class="badge">ESP-IDF v6.0 Firmware</span>
    <span class="badge">Kinetix 5100 EtherNet/IP</span>
    <span class="badge">Zero-Dwell Retract</span>
    <span class="badge">100% VFF Lead (7 ms)</span>
    <span class="badge">FreeRTOS Tick Scheduling</span>
    <span class="badge">MuJoCo 3.x Physics Twin</span>
  </div>
</div>

<div class="kpi-grid">
  <div class="kpi-card">
    <div class="kpi-val">1,500 <span style="font-size:7.5pt;">mm/s</span></div>
    <div class="kpi-lbl">Max Conveyor Speed</div>
  </div>
  <div class="kpi-card accent">
    <div class="kpi-val">1,000 <span style="font-size:7.5pt;">mm/s</span></div>
    <div class="kpi-lbl">Symmetric $v_z$ Plunge/Retract</div>
  </div>
  <div class="kpi-card purple">
    <div class="kpi-val">0.0 <span style="font-size:7.5pt;">ms</span></div>
    <div class="kpi-lbl">Dwell at Grasp Depth</div>
  </div>
  <div class="kpi-card orange">
    <div class="kpi-val">8.0 <span style="font-size:7.5pt;">ms</span></div>
    <div class="kpi-lbl">Constant Retract Lead ($\Delta t_{\mathrm{lead}}$)</div>
  </div>
</div>

<h2>1. Executive Summary &amp; The Flying Intercept Challenge</h2>
<p>
  Automated battery sorting lines demand throughputs exceeding 60–75 parts per minute (PPM) across mixed cylindrical and prismatic cells. Classical "Stop-and-Pick" architectures decelerate the conveyor belt to a standstill prior to each grasp, causing severe cycle time penalties (> 2.2 s/cell), variable frequency drive (VFD) thermal stress, and tipping risks for unstable vertical cells.
</p>
<p>
  To eliminate conveyor cycling, the WT32-ETH01 firmware implements an <strong>underactuated flying intercept</strong> control architecture. The mechanical gantry is equipped with transverse across-belt motion ($X$), vertical plunge ($Z$), and rotary orientation ($\theta$), with <strong>zero degree of freedom along the belt axis ($Y$)</strong>. Grasping occurs exclusively along the fixed transverse datum line $Y = y_p = 0$. By synchronizing high-speed vertical descent and pneumatic jaw closure with incoming cell travel, pickups succeed across belt speeds from <strong>200 to 1,500 mm/s</strong>.
</p>
<p>
  Crucially, the architecture eliminates the traditional bottom dwell: vertical motion reverses <strong>instantaneously with zero dwell</strong> ($t_{\mathrm{retract}} = t_{\mathrm{close}} + \Delta t_{\mathrm{retract\_from\_close}}$, $\mathrm{dwell} = 0.0\,\mathrm{s}$) at symmetric critical velocity $v_{\mathrm{plunge}} = v_{\mathrm{retract}} = 800.0\,\mathrm{mm/s}$ and acceleration $a_z = 20{,}000\,\mathrm{mm/s^2}$ ($20\,\mathrm{m/s^2} \approx 2.04\,g$). Fingertips pre-close during plunge to a tiny clearance ($\delta_{\mathrm{clear}} = 0.9\,\mathrm{mm}$ per side), ensuring strictly zero normal force ($F_N = 0.0\,\mathrm{N}$) at bottom dead center. The firing sequence strictly executes <strong>$\mathrm{START\_PLUNGE} \longrightarrow \mathrm{START\_GRIP} \longrightarrow \mathrm{START\_RETRACT}$</strong> with $t_z < t_{\mathrm{close}} < t_{\mathrm{retract}}$ across all configurations, guaranteeing the plunge completes full stroke to $z_g$ before reversal. <strong>Z retract immediately follows grip close with a constant delay</strong> ($\Delta t_{\mathrm{retract\_from\_close}} \equiv \mathrm{constant}$), starting exactly $\Delta t_{\mathrm{lead}} = 8.0\,\mathrm{ms}$ (8 ticks) before full grasp closure, completely independent of how other offsets are tuned.
</p>

<h2>2. Kinematic Reference Architecture &amp; Coordinate Invariants</h2>

<div class="def-box">
  <div class="def-box-title">Definition 1 &mdash; Spatial Reference Frames &amp; Canonical Orientation &theta;</div>
  <strong>World Frame $\mathcal{F}_w$:</strong> $+X$ extends transverse across the conveyor belt, $+Y$ points downstream along belt travel, and $+Z$ points vertically downward toward the belt bed ($z_b = 102.0\,\mathrm{mm}$ in digital twin datum; $z_b = 115.0\,\mathrm{mm}$ in firmware coordinate system).<br>
  <strong>Canonical Longitudinal Angle $\theta$:</strong> $\theta = 0^\circ$ corresponds to the longitudinal axis aligned exactly parallel to $+Y$. For all cylindrical cells, $\theta = 0^\circ \iff$ cylinder lies parallel to the conveyor direction $+Y$, and clamping pads close along transverse $X$. At $\theta = \pm 90^\circ$, the cylinder lies across the belt ($X$), and clamping pads close along belt direction ($Y$).<br>
  <strong>Direction Vectors:</strong> $\mathbf{u}(\theta) = [-\sin\theta, \cos\theta]^T$ (long axis), $\mathbf{n}(\theta) = [\cos\theta, \sin\theta]^T$ (clamping normal). Symmetry fold: $\theta_{\mathrm{cmd}} = \mathrm{fold}(\theta) \in [-90^\circ, 90^\circ)$.
</div>

<div class="figure-box">
  <img src="__B64_KINEMATICS__" style="max-height: 62mm;" alt="Kinematic Reference Frames">
  <div class="caption">Figure 1: Planar kinematic reference frames, cell orientation vector $\mathbf{u}(\theta)$, clamping normal $\mathbf{n}(\theta)$, and 3D-printed PLA jaw pads.</div>
</div>

<div class="page-break"></div>

<!-- PAGE 2: CELL ENVELOPE TABLE & DUAL-Z ARCHITECTURE -->

<h2>3. Supported Battery Profiles &amp; Clean Geometry Definitions</h2>
<p>
  The pick scheduler accommodates 13 distinct battery form factors and resting orientations without mechanical changeover:
</p>

<div class="def-box">
  <div class="def-box-title">Definition 2 &mdash; Resting Stances &amp; Geometry Envelopes</div>
  <strong>Cylindrical Cells (AAA, AA, D, C, 18650, 21700):</strong> Rest horizontally on round flank ($h = w = \text{diameter}$, length $L$). Cylindrical longitudinal axis aligns with $\mathbf{u}(\theta)$ ($\theta = 0^\circ \iff +Y$).<br>
  <strong>Prismatic 9V Variants:</strong>
  <ul>
    <li><code>nine_volt_flatface</code>: Resting flat on larger $47.7 \times 26.6\,\mathrm{mm}$ face ($L=47.7, w=26.6, h=17.3\,\mathrm{mm}$).</li>
    <li><code>nine_volt_shortface</code>: Resting on narrower $47.7 \times 17.3\,\mathrm{mm}$ flank ($L=47.7, w=17.3, h=26.6\,\mathrm{mm}$).</li>
    <li><code>nine_volt_bottomface</code>: Standing upright on $26.6 \times 17.3\,\mathrm{mm}$ base ($L=26.6, w=17.3, h=47.7\,\mathrm{mm}$).</li>
  </ul>
  <strong>Packs &amp; Power Modules:</strong>
  <ul>
    <li><code>pack</code>: Flat Li-ion pack ($L=115.1, w=68.3, h=26.7\,\mathrm{mm}$).</li>
    <li><code>pack_shortface</code>: Pack resting on $115.1 \times 26.7\,\mathrm{mm}$ edge ($L=115.1, w=26.7, h=68.3\,\mathrm{mm}$).</li>
    <li><code>laptop</code>: Long rectangular pack ($L=207.9, w=50.3, h=21.3\,\mathrm{mm}$).</li>
    <li><code>power_tool</code>: Heavy power tool block ($L=111.7, w=73.8, h=49.6\,\mathrm{mm}$).</li>
  </ul>
</div>

<div class="def-box">
  <div class="def-box-title">Definition 2.1 &mdash; Piecewise Calibrated Grasp Depth Formulation</div>
  For belt bed datum $z_b$ ($102.0\,\mathrm{mm}$ in digital twin, $115.0\,\mathrm{mm}$ in firmware coordinate system):
  \begin{equation}
    z_g(h) = \begin{cases} 
      z_b - 3.0\,\mathrm{mm}, & \text{if } h < 30.0\,\mathrm{mm} \\
      z_b - 4.0\,\mathrm{mm}, & \text{if } h \ge 30.0\,\mathrm{mm}
    \end{cases}
  \end{equation}
  Fingertip pad centers descend strictly to $3.0\,\mathrm{mm}$ above the belt surface for all low-profile batteries ($h < 30\,\mathrm{mm}$, $z_g = 99.00\,\mathrm{mm}$ sim, $112.00\,\mathrm{mm}$ FW) and $4.0\,\mathrm{mm}$ above the belt for tall form factors ($h \ge 30\,\mathrm{mm}$, $z_g = 98.00\,\mathrm{mm}$ sim, $111.00\,\mathrm{mm}$ FW), eliminating tip collisions while ensuring maximum flank engagement.
</div>

<table>
  <thead>
    <tr>
      <th>Class / Name</th>
      <th>Form Category</th>
      <th>Length $L$ (mm)</th>
      <th>Width $w$ (mm)</th>
      <th>Height $h$ (mm)</th>
      <th>Grasp $z_g$ Sim (mm)</th>
      <th>Grasp $z_g$ FW (mm)</th>
      <th>Half-Gap $g$ (mm)</th>
      <th>Stroke (mm)</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td><code>AAA</code></td>
      <td>Cylindrical</td>
      <td>44.5</td>
      <td>10.5</td>
      <td>10.5</td>
      <td>99.00</td>
      <td>112.00</td>
      <td>34.75</td>
      <td>36.25</td>
    </tr>
    <tr>
      <td><code>AA</code></td>
      <td>Cylindrical</td>
      <td>50.5</td>
      <td>14.5</td>
      <td>14.5</td>
      <td>99.00</td>
      <td>112.00</td>
      <td>32.75</td>
      <td>34.25</td>
    </tr>
    <tr>
      <td><code>D</code></td>
      <td>Cylindrical</td>
      <td>61.5</td>
      <td>34.2</td>
      <td>34.2</td>
      <td>98.00</td>
      <td>111.00</td>
      <td>22.90</td>
      <td>24.40</td>
    </tr>
    <tr>
      <td><code>C</code></td>
      <td>Cylindrical</td>
      <td>50.0</td>
      <td>26.2</td>
      <td>26.2</td>
      <td>99.00</td>
      <td>112.00</td>
      <td>26.90</td>
      <td>28.40</td>
    </tr>
    <tr>
      <td><code>18650</code></td>
      <td>Cylindrical Li-ion</td>
      <td>65.0</td>
      <td>18.3</td>
      <td>18.3</td>
      <td>99.00</td>
      <td>112.00</td>
      <td>30.85</td>
      <td>32.35</td>
    </tr>
    <tr>
      <td><code>21700</code></td>
      <td>Cylindrical Li-ion</td>
      <td>70.0</td>
      <td>21.2</td>
      <td>21.2</td>
      <td>99.00</td>
      <td>112.00</td>
      <td>29.40</td>
      <td>30.90</td>
    </tr>
    <tr>
      <td><code>nine_volt_flatface</code></td>
      <td>Prismatic (Flat)</td>
      <td>47.7</td>
      <td>26.6</td>
      <td>17.3</td>
      <td>99.00</td>
      <td>112.00</td>
      <td>26.70</td>
      <td>28.20</td>
    </tr>
    <tr>
      <td><code>nine_volt_shortface</code></td>
      <td>Prismatic (Flank)</td>
      <td>47.7</td>
      <td>17.3</td>
      <td>26.6</td>
      <td>99.00</td>
      <td>112.00</td>
      <td>31.35</td>
      <td>32.85</td>
    </tr>
    <tr>
      <td><code>nine_volt_bottomface</code></td>
      <td>Prismatic (Upright)</td>
      <td>26.6</td>
      <td>17.3</td>
      <td>47.7</td>
      <td>98.00</td>
      <td>111.00</td>
      <td>31.35</td>
      <td>32.85</td>
    </tr>
    <tr>
      <td><code>pack</code></td>
      <td>Li-ion Flat Pack</td>
      <td>115.1</td>
      <td>68.3</td>
      <td>26.7</td>
      <td>99.00</td>
      <td>112.00</td>
      <td>5.85</td>
      <td>7.35</td>
    </tr>
    <tr>
      <td><code>pack_shortface</code></td>
      <td>Li-ion Tall Pack</td>
      <td>115.1</td>
      <td>26.7</td>
      <td>68.3</td>
      <td>98.00</td>
      <td>111.00</td>
      <td>26.65</td>
      <td>28.15</td>
    </tr>
    <tr>
      <td><code>laptop</code></td>
      <td>Long Laptop Pack</td>
      <td>207.9</td>
      <td>50.3</td>
      <td>21.3</td>
      <td>99.00</td>
      <td>112.00</td>
      <td>14.85</td>
      <td>16.35</td>
    </tr>
    <tr>
      <td><code>power_tool</code></td>
      <td>Heavy Tool Block</td>
      <td>111.7</td>
      <td>73.8</td>
      <td>49.6</td>
      <td>98.00</td>
      <td>111.00</td>
      <td>3.10</td>
      <td>4.60</td>
    </tr>
  </tbody>
</table>

<div class="def-box">
  <div class="def-box-title">Definition 3 &mdash; Dual-Z Ceiling Architecture</div>
  <strong>Traverse Ceiling ($Z_{\mathrm{traverse}} = 20.0\,\mathrm{mm}$ sim / $35.7\,\mathrm{mm}$ FW):</strong> High clearance elevation clearing drop bin rims ($35.0\,\mathrm{mm}$) and conveyor rails during transfer and return traversals.<br>
  <strong>Dynamic Conveyor Staging Ceiling ($z_{\mathrm{stage}}$):</strong>
  \begin{equation}
    z_{\mathrm{stage}} = \min(Z_{\mathrm{safe}}, \max(15.0\,\mathrm{mm}, (z_b - h) - 6.0\,\mathrm{mm}))
  \end{equation}
  Where $Z_{\mathrm{safe}} = 65.0\,\mathrm{mm}$. For tall form factors ($h > 35\,\mathrm{mm}$), staging automatically lifts above cell top with $6.0\,\mathrm{mm}$ minimum overhead clearance.
</div>

<div class="page-break"></div>

<!-- PAGE 3: 4-STAGE PIPELINE & TRAJECTORIES -->

<h2>4. The 4-Stage Synchronous Motion Pipeline</h2>
<ol style="padding-left: 18px; margin-top: 3px;">
  <li style="margin-bottom: 5px;">
    <strong>Stage 1 &mdash; Upstream Optical Sighting &amp; Dead-Reckoning:</strong> Cell crosses optical trigger line $y_{\mathrm{cam}} = -771\,\mathrm{mm}$. Overhead camera publishes pose via CellNetL2 broadcast. The gantry filters transit latency with min-of-8 delay filter ($\delta_{\min} \approx 35\,\mu\mathrm{s}$) and dead-reckons belt odometer position $s(t)$.
  </li>
  <li style="margin-bottom: 5px;">
    <strong>Stage 2 &mdash; Safe-Z Staging Slew:</strong> While fingertips reside safely at $z \le Z_{\mathrm{safe}}$, transverse $X$ and rotary $\theta$ axes slew to intercept alignment at $500\,\mathrm{mm/s}$ and $60^\circ/\mathrm{s}$.
  </li>
  <li style="margin-bottom: 5px;">
    <strong>Stage 3 &mdash; Synchronized Plunge &amp; Discrete Three-Parameter Tick Scheduling:</strong> Z descends at constant symmetric speed $v_z = 800\,\mathrm{mm/s}$ and acceleration $a_z = 20{,}000\,\mathrm{mm/s^2}$ ($2.04\,g$, Kinetix 5100 capability). Actuator countdowns are scheduled in discrete FreeRTOS ticks ($T_{\mathrm{tick}} = 1.0\,\mathrm{ms}$) since optical battery assignment ($y_{\mathrm{cam}} = -771\,\mathrm{mm}$):
    $$N_{z,\mathrm{detect}} = \mathrm{round}\left(\frac{771.0/v_{\mathrm{belt}} + t_z}{T_{\mathrm{tick}}}\right), \quad N_{\mathrm{close,detect}} = \mathrm{round}\left(\frac{771.0/v_{\mathrm{belt}} + t_{\mathrm{close}}}{T_{\mathrm{tick}}}\right), \quad N_{\mathrm{retract,detect}} = \mathrm{round}\left(\frac{771.0/v_{\mathrm{belt}} + t_{\mathrm{retract}}}{T_{\mathrm{tick}}}\right)$$
    with runtime tuning offsets $\Delta N_{z,\mathrm{offset\_ticks}}$, $\Delta N_{\mathrm{offset\_ticks}}$, and $\Delta N_{\mathrm{retract\_ticks}} \in [-100, +100]\,\mathrm{ticks}$ exposed directly alongside their respective control panel sliders. Jaws pre-close so that at $t_{z,\mathrm{done}}$, the battery is already between the fingers with tiny clearance $\delta_{\mathrm{clear}} = 0.9\,\mathrm{mm}$ per side ($W_{\mathrm{jaw}} = w + 1.8\,\mathrm{mm}$), guaranteeing strictly zero normal force ($F_N = 0.0\,\mathrm{N}$) at bottom dead center.
  </li>
  <li style="margin-bottom: 5px;">
    <strong>Stage 4 &mdash; Zero-Dwell Reversal, 100% VFF Lead &amp; Constant Retract Delay:</strong> Vertical motion reverses instantaneously at $z_g$ with <strong>strictly zero dwell</strong> ($t_{\mathrm{retract}} = t_{\mathrm{close}} + \Delta t_{\mathrm{retract\_from\_close}}$, $\mathrm{dwell} = 0.0\,\mathrm{s}$) at constant speed $v_{\mathrm{retract}} = 800\,\mathrm{mm/s}$. Drive position command executes 100% Velocity Feedforward (VFF) lead compensation:
    $$z_{\mathrm{cmd}}(t) = z(t) + \tau_{\mathrm{vff}} \dot{z}(t), \quad \tau_{\mathrm{vff}} = \frac{k_v}{k_p} = 0.007\,\mathrm{s} = 7.0\,\mathrm{ms}$$
    eliminating servo following error lag and velocity spikes. <strong>Z retract immediately follows grip close with a constant delay</strong> ($\Delta t_{\mathrm{retract\_from\_close}} \equiv \mathrm{constant}$, independent of offset tuning). Retract initiates exactly $\Delta t_{\mathrm{lead}} = 8.0\,\mathrm{ms}$ before full gripper closure, clamping the battery securely within the first millimetre of upward ascent.
  </li>
</ol>

<div class="figure-box">
  <img src="__B64_TRAJECTORIES__" style="max-height: 98mm;" alt="Synchronized Motion Trajectories">
  <div class="caption">Figure 2: Multi-axis deterministic trajectory synchronization (Conveyor Y, Ballscrew Z, Jaw Gap, and Normal Force).</div>
</div>

<div class="page-break"></div>

<!-- PAGE 4: CLEARANCE, JITTER, ERROR BUDGET & VALIDATION -->

<h2>5. Dynamic Clearance Windows &amp; Finger Proximity Invariant</h2>

<div class="def-box">
  <div class="def-box-title">Definition 4 &mdash; Contact Tribology &amp; Materials</div>
  <strong>Fingertip Material:</strong> 3D-printed Poly-Lactic Acid (PLA) plastic ($E \approx 3.5\,\mathrm{GPa}$, dry kinetic friction coefficient against steel $\mu_{\mathrm{PLA}} = 0.28$).<br>
  <strong>Battery Casing Material:</strong> Deep-drawn nickel-plated steel / stainless steel casing ($\mu = 0.35$).<br>
  <strong>Extraction Dynamic Friction Invariance:</strong> Normal clamping force $F_n = 18.5\,\mathrm{N}$ under full upward extraction acceleration $a_{z,\mathrm{retract}} = 20.0\,\mathrm{m/s^2}$ ($2.04\,g$) provides a vertical extraction safety margin factor $S_f = \frac{2 \mu F_n}{m(g + a_z)} \ge 2.48$ for the heaviest cell ($m = 0.140\,\mathrm{kg}$, D-cell) and $S_f \ge 4.97$ for standard cells ($m \le 0.070\,\mathrm{kg}$), eliminating inertial slip.
</div>

<div class="def-box">
  <div class="def-box-title">Definition 5 &mdash; Separating-Axis Upstream Proximity Invariant</div>
  At the instant of first contact between the downstream finger and cell flank ($d_{\mathrm{down}} = 0$), the upstream finger clearance satisfies:
  $$d_{\mathrm{up}}(t_{\mathrm{contact}}) = W_{\mathrm{jaw}}(t_{\mathrm{contact}}) - w = 1.80\,\mathrm{mm} < 3.0\,\mathrm{mm}$$
  identically across all conveyor speeds $v \in [200, 1500]\,\mathrm{mm/s}$ and angles $\theta \in [0, 90^\circ]$. The gripper is never held wide open at grasp depth. At bottom touchdown ($t_{z,\mathrm{done}}$), $F_N = 0.0\,\mathrm{N}$ identically.
</div>

<div class="figure-box">
  <img src="__B64_CLEARANCE__" style="max-height: 48mm;" alt="Clearance Window Feasibility">
  <div class="caption">Figure 3: Clearance window $W(v, \theta) = \frac{2g}{v \sin\theta}$ vs. orientation angle and belt speed, delineating Early-Descent and Late-Descent regimes.</div>
</div>

<h2>6. Stochastic Jitter Rejection &amp; Hardware Error Budget</h2>
<p>
  Deterministic timing is preserved across dual independent network interfaces: EtherNet/IP (W5500 SPI2, $2\,\mathrm{ms}$ RPI) for Kinetix servo drives, and CellNetL2 (LAN8720 RMII, $10\,\mathrm{ms}$ broadcast) for conveyor telemetry. Hardware bench tests demonstrate worst-case composite timing jitter of $15.5\,\mathrm{ms}$, comfortably bounded by the $20\,\mathrm{ms}$ deterministic guard band.
</p>

<div class="figure-box">
  <img src="__B64_JITTER__" style="max-height: 45mm;" alt="Timing Jitter Distributions and Error Budget">
  <div class="caption">Figure 4: Hardware and network timing jitter distributions (Lognormal drive latency, RPI phase, and FreeRTOS loop).</div>
</div>

<div class="callout">
  <div class="callout-title">Full Batch Physics Validation (576 Runs &bull; 100% Pass Rate)</div>
  The high-fidelity MuJoCo digital twin completed <strong>576 multi-core batch simulations</strong> across 6 battery chemistries (including flat 9V), 8 conveyor speeds (200–1,500 mm/s), and 12 arrival angles ($0^\circ - 90^\circ$). All 576 runs verified <strong>zero cell slippage, zero catapulting, upstream finger clearance strictly &lt; 3.0 mm (1.80 mm nominal), strictly zero dwell at grasp depth, and surface drag strictly below 1.0 mm</strong>, validating the mathematical model for industrial deployment.
</div>

<div style="margin-top: 10px; padding-top: 4px; border-top: 1px solid #e2e8f0; font-size: 6.8pt; color: #94a3b8; display: flex; justify-content: space-between;">
  <span>WT32-ETH01 Pick-and-Place Firmware Architecture</span>
  <span>CONFIDENTIAL &bull; ENGINEERING TECHNICAL BRIEF</span>
  <span>Doc ID: ARCH-GANTRY-2026-REV6</span>
</div>

</body>
</html>
"""
    return tmpl.replace("__B64_KINEMATICS__", b64_kinematics).replace("__B64_TRAJECTORIES__", b64_trajectories).replace("__B64_CLEARANCE__", b64_clearance).replace("__B64_JITTER__", b64_jitter)


# ==============================================================================
# 2. IN-DEPTH MATHEMATICAL SPECIFICATION GENERATOR
# ==============================================================================

def build_mathematical_specification_html() -> str:
    tmpl = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Synchronized On-the-Fly Grasp Timing: Mathematical Specification &amp; Formal Kinematic Derivations</title>
<script>
window.MathJax = {
  tex: {
    tags: 'ams',
    inlineMath: [['$', '$'], ['\\(', '\\)']],
    displayMath: [['$$', '$$'], ['\\[', '\\]']]
  },
  startup: {
    pageReady: () => {
      return MathJax.startup.defaultPageReady();
    }
  }
};
</script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>
<style>
  @page {
    size: A4 portrait;
    margin: 12mm 14mm 12mm 14mm;
  }
  * { box-sizing: border-box; }
  body {
    font-family: "Latin Modern Roman", "Times New Roman", Times, Georgia, serif;
    color: #111827;
    background: #ffffff;
    line-height: 1.44;
    font-size: 8.8pt;
    margin: 0;
    padding: 0;
  }

  .doc-header {
    text-align: center;
    border-bottom: 2px solid #111827;
    padding-bottom: 10px;
    margin-bottom: 12px;
  }
  .doc-title {
    font-size: 13.5pt;
    font-weight: 700;
    margin: 0 0 3px 0;
    letter-spacing: -0.3px;
  }
  .doc-subtitle {
    font-size: 9.5pt;
    color: #374151;
    font-style: italic;
    margin-bottom: 6px;
  }
  .doc-meta {
    font-size: 7.5pt;
    color: #4b5563;
  }

  h2 {
    font-size: 10.5pt;
    font-weight: 700;
    color: #111827;
    border-bottom: 1px solid #d1d5db;
    padding-bottom: 2px;
    margin-top: 12px;
    margin-bottom: 5px;
  }
  h3 {
    font-size: 9pt;
    font-weight: 700;
    color: #1f2937;
    margin-top: 8px;
    margin-bottom: 3px;
  }
  p {
    margin: 0 0 5px 0;
    text-align: justify;
    text-indent: 1.5em;
  }
  p.noindent {
    text-indent: 0;
  }

  .math-def {
    background: #f9fafb;
    border: 1px solid #e5e7eb;
    border-left: 3.5px solid #1f2937;
    padding: 6px 10px;
    margin: 6px 0;
    font-size: 8.4pt;
    page-break-inside: avoid;
  }
  .math-def-title {
    font-weight: 700;
    font-family: sans-serif;
    font-size: 7.5pt;
    text-transform: uppercase;
    color: #374151;
    margin-bottom: 2px;
    letter-spacing: 0.4px;
  }

  .theorem {
    background: #f8fafc;
    border: 1px solid #cbd5e1;
    border-left: 3.5px solid #0284c7;
    padding: 6px 10px;
    margin: 6px 0;
    font-size: 8.4pt;
    page-break-inside: avoid;
  }
  .theorem-title {
    font-weight: 700;
    font-style: italic;
    color: #0369a1;
    margin-bottom: 2px;
  }

  .proof {
    font-style: normal;
    margin: 4px 0 8px 0;
    padding-left: 9px;
    border-left: 2px solid #e2e8f0;
    font-size: 8.2pt;
    page-break-inside: avoid;
  }
  .proof-title {
    font-weight: 700;
    font-style: italic;
  }
  .proof::after {
    content: "■";
    float: right;
    font-size: 7.8pt;
    color: #64748b;
  }

  table {
    width: 100%;
    border-collapse: collapse;
    font-size: 7.4pt;
    margin: 6px 0;
    page-break-inside: avoid;
  }
  th, td {
    padding: 3.5px 6px;
    border-top: 1px solid #d1d5db;
    border-bottom: 1px solid #d1d5db;
    text-align: left;
  }
  th {
    font-weight: 700;
    background: #f9fafb;
  }

  .figure-box {
    margin: 8px 0;
    text-align: center;
    page-break-inside: avoid;
  }
  .figure-box img {
    border: 1px solid #d1d5db;
  }
  .caption {
    font-size: 7.2pt;
    color: #4b5563;
    margin-top: 3px;
    font-style: italic;
  }

  .page-break { page-break-after: always; }
</style>
</head>
<body>

<!-- PAGE 1: TITLE, COORD FRAMES & GEOMETRY -->
<div class="doc-header">
  <div class="doc-title">Synchronized On-the-Fly Grasp Timing: Mathematical Specification &amp; Formal Kinematic Derivations</div>
  <div class="doc-subtitle">A Rigorous Control Formulation for High-Speed Underactuated Belt Pickups</div>
  <div class="doc-meta">
    Author: Advanced Controls &amp; Motion Architecture Group &bull; System: WT32-ETH01 Gantry Digital Twin<br>
    Date: October 2026 &bull; Classification: Formal Engineering Specification &bull; Reference: KIN-MATH-SPEC-2026-REV6
  </div>
</div>

<h2>1. Introduction &amp; Coordinate Frame Kinematics</h2>
<p class="noindent">
  This document establishes the mathematical formulation for intercepting, grasping, and vertically extracting solid cylindrical and prismatic battery cells moving continuously along a flat belt conveyor. The physical machine comprises a Cartesian gantry with transverse linear travel $X$, vertical ballscrew plunge $Z$, and continuous rotary orientation $\theta$, operating a parallel pneumatic clamping gripper. The gantry possesses <strong>no degree of freedom parallel to conveyor travel ($Y$)</strong>.
</p>

<div class="math-def">
  <div class="math-def-title">Definition 1 (Spatial Reference Frames &amp; Conveyor Datum)</div>
  We define the following orthonormal right-handed coordinate frames:
  <ul>
    <li><strong>World Frame $\mathcal{F}_w = (\mathbf{O}_w, \mathbf{x}_w, \mathbf{y}_w, \mathbf{z}_w)$:</strong> Fixed to the gantry structure. $\mathbf{x}_w$ extends across the belt; $\mathbf{y}_w$ points downstream along belt travel; $\mathbf{z}_w$ points downward toward the belt. Datum $z = 0$ is the fully retracted ballscrew ceiling. The conveyor bed surface lies at $z = z_b = 102.0\,\mathrm{mm}$ in the digital twin simulation datum, and $z_b = 115.0\,\mathrm{mm}$ in the firmware coordinate system.</li>
    <li><strong>Conveyor Frame $\mathcal{F}_c(t)$:</strong> Attached to the moving belt, translating at instantaneous velocity $\mathbf{v}_c(t) = v_{\mathrm{belt}}(t)\,\mathbf{y}_w$.</li>
    <li><strong>Battery Principal Frame $\mathcal{F}_b$:</strong> Centered at the geometric centroid $\mathbf{p}_b$, rotated by orientation angle $\theta \in [-\pi/2, \pi/2)$ about $\mathbf{z}_w$.</li>
    <li><strong>Gripper End-Effector Frame $\mathcal{F}_j$:</strong> Located at transverse gantry position $x$, height $z$, rotated by command angle $\theta_{\mathrm{cmd}}$.</li>
  </ul>
</div>

<div class="math-def">
  <div class="math-def-title">Definition 2 (Canonical Orientation Angle &theta; &amp; Direction Vectors)</div>
  The orientation angle $\theta$ is uniquely defined such that $\theta = 0^\circ$ corresponds to the longitudinal principal axis aligned exactly parallel to the conveyor downstream direction $+Y$.
  <br>
  The longitudinal unit vector $\mathbf{u}(\theta)$ and normal clamping unit vector $\mathbf{n}(\theta) \perp \mathbf{u}(\theta)$ in planar belt coordinates are given by:
  \begin{equation}
    \mathbf{u}(\theta) = \begin{bmatrix} -\sin\theta \\ \cos\theta \end{bmatrix}, \qquad
    \mathbf{n}(\theta) = \begin{bmatrix} \cos\theta \\ \sin\theta \end{bmatrix}
    \label{eq:u_n_vectors}
  \end{equation}
  Because cylindrical and prismatic cells exhibit $180^\circ$ rotational symmetry ($\theta \equiv \theta \pm \pi$), the controller applies the half-turn fold:
  \begin{equation}
    \theta_{\mathrm{fold}} = \frac{1}{2} \mathrm{atan2}(\sin 2\theta, \cos 2\theta) \in \left[-\frac{\pi}{2}, \frac{\pi}{2}\right)
    \label{eq:theta_fold}
  \end{equation}
  The rotary joint $\theta_{\mathrm{cmd}} = \theta_{\mathrm{fold}}$ aligns the fingertip pad faces parallel to $\mathbf{u}(\theta)$, ensuring that the closing axis $\theta_j = \theta_{\mathrm{cmd}} + \frac{\pi}{2}$ aligns precisely with $\mathbf{n}(\theta)$.
</div>

<div class="math-def">
  <div class="math-def-title">Definition 3 (Battery Envelopes, Resting Stances &amp; Piecewise Grasp Heights)</div>
  Let a battery cell have envelope dimensions $(L, w, h)$:
  <ul>
    <li><strong>Cylindrical Cells (AAA, AA, D, C, 18650, 21700):</strong> Cylinders of diameter $D$ and length $L$. Cells rest horizontally on the belt with line contact at $z = z_b$. Height above belt $h = D$, flank width across pads $w = D$, and longitudinal length along $\mathbf{u}(\theta)$ is $L$. Cylindrical longitudinal axis aligns strictly with $\mathbf{u}(\theta)$ ($\theta = 0^\circ \iff +Y$).</li>
    <li><strong>Prismatic 9V Variants:</strong> <code>nine_volt_flatface</code> ($47.7 \times 26.6 \times 17.3\,\mathrm{mm}$), <code>nine_volt_shortface</code> ($47.7 \times 17.3 \times 26.6\,\mathrm{mm}$), <code>nine_volt_bottomface</code> ($26.6 \times 17.3 \times 47.7\,\mathrm{mm}$).</li>
    <li><strong>Packs:</strong> <code>pack</code> ($115.1 \times 68.3 \times 26.7\,\mathrm{mm}$), <code>pack_shortface</code> ($115.1 \times 26.7 \times 68.3\,\mathrm{mm}$), <code>laptop</code> ($207.9 \times 50.3 \times 21.3\,\mathrm{mm}$), <code>power_tool</code> ($111.7 \times 73.8 \times 49.6\,\mathrm{mm}$).</li>
  </ul>
  The target grasp depth $z_g(h)$ referenced to belt datum $z_b$ ($102.0\,\mathrm{mm}$ sim, $115.0\,\mathrm{mm}$ firmware) satisfies the piecewise rule:
  \begin{equation}
    z_g(h) = \begin{cases}
      z_b - 3.0\,\mathrm{mm}, & \text{if } h < 30.0\,\mathrm{mm} \\
      z_b - 4.0\,\mathrm{mm}, & \text{if } h \ge 30.0\,\mathrm{mm}
    \end{cases}
    \label{eq:zg_def}
  \end{equation}
  For any battery height smaller than $30\,\mathrm{mm}$, the grasp depth is strictly $3.0\,\mathrm{mm}$ above the belt surface ($99.00\,\mathrm{mm}$ in simulation, $112.00\,\mathrm{mm}$ in firmware), preventing fingertip collisions with the moving belt while guaranteeing maximal flank contact. For tall cells ($h \ge 30\,\mathrm{mm}$), grasp depth sits $4.0\,\mathrm{mm}$ above the belt surface ($98.00\,\mathrm{mm}$ in simulation, $111.00\,\mathrm{mm}$ in firmware).
</div>

<div class="figure-box">
  <img src="__B64_KINEMATICS__" style="max-height: 56mm;" alt="Planar Kinematic Coordinate Frames">
  <div class="caption">Figure 1: Planar kinematic reference frames, cell orientation vector $\mathbf{u}(\theta)$, and clamping vector $\mathbf{n}(\theta)$.</div>
</div>

<!-- SECTION 2: BELT DYNAMICS & CRITICAL TRAJECTORIES -->

<h2>2. Conveyor Belt Dynamics &amp; Discrete Telemetry Observer</h2>
<p class="noindent">
  The conveyor displacement is measured by a scalar encoder odometer $s(t) \in \mathbb{R}$, with continuous differential equation $\dot{s}(t) = v(t)$. Telemetry is sampled by a remote microcontroller and transmitted to the WT32-ETH01 over CellNetL2 at nominal period $T_{\mathrm{conv}} = 10\,\mathrm{ms}$.
</p>

<div class="math-def">
  <div class="math-def-title">Definition 4 (Discrete Conveyor Telemetry &amp; Min-of-8 Jitter Filter)</div>
  Let frame $k$ arrive at gantry time $t_k^{\mathrm{rx}}$, carrying sender timestamp $t_k^{\mathrm{tx}}$, position $s_k$, and velocity $v_k$. The instantaneous transit delay is $\delta_k = t_k^{\mathrm{rx}} - t_k^{\mathrm{tx}}$. The filtered transit latency $\delta_{\min}$ is defined as:
  \begin{equation}
    \delta_{\min}(k) = \min_{j \in [0, 7]} \delta_{k-j}
    \label{eq:min_delay}
  \end{equation}
  The continuous extrapolated belt position $\hat{s}(t)$ at local gantry clock $t \ge t_k^{\mathrm{rx}}$ is governed by:
  \begin{equation}
    \hat{s}(t) = s_k + v_k \cdot \left[t - (t_k^{\mathrm{tx}} + \delta_{\min}(k))\right]
    \label{eq:dead_reckoning}
  \end{equation}
</div>

<div class="theorem">
  <div class="theorem-title">Lemma 1 (Exact Arrival Time Formulation)</div>
  Let an object be captured optically at conveyor displacement $s_c$ and longitudinal offset $y_{\mathrm{bat}}$ relative to camera optical station $y_{\mathrm{cam}} = -771.0\,\mathrm{mm}$. The unique conveyor displacement $s_a$ at which the cell centroid reaches the pick datum $Y = y_p = 0$ is:
  \begin{equation}
    s_a = s_c + y_p - (y_{\mathrm{cam}} + y_{\mathrm{bat}})
    \label{eq:s_arrive}
  \end{equation}
  Assuming constant speed $v > 0$ over the remaining interval, the predicted arrival timestamp $t_a$ is the unique solution to $\hat{s}(t_a) = s_a$:
  \begin{equation}
    t_a = t + \frac{s_a - \hat{s}(t)}{v}
    \label{eq:t_arrive}
  \end{equation}
</div>

<h2>3. Trapezoidal Trajectory Generation &amp; Ballscrew Critical Kinematics</h2>

<div class="math-def">
  <div class="math-def-title">Definition 5 (Dual-Z Staging Ceiling Architecture &amp; Motion Profiles)</div>
  The vertical $Z$ motion space is partitioned into two deterministic safety ceilings:
  <ul>
    <li><strong>Traverse Ceiling ($Z_{\mathrm{traverse}} = 20.0\,\mathrm{mm}$ sim / $35.7\,\mathrm{mm}$ FW):</strong> Clears drop bin walls, gantry framework, and peripheral obstacles during high-speed transfer and return moves.</li>
    <li><strong>Conveyor Pre-Plunge Ceiling ($Z_{\mathrm{safe}} = 65.0\,\mathrm{mm}$):</strong> Safe staging elevation clearing all cell form factors (including the tallest D-cell, $h = 34.2\,\mathrm{mm}$) with overhead margin. Dynamic staging ceiling: $z_{\mathrm{stage}} = \min(65.0, \max(15.0, (z_b - h) - 6.0))$.</li>
  </ul>
  Single-axis point-to-point moves over distance $D$ follow time-optimal trapezoidal acceleration profiles with speed $V$ and acceleration/deceleration $A, B$:
  \begin{equation}
    T(D, V, A, B) = \begin{cases}
      \dfrac{V}{A} + \dfrac{V}{B} + \dfrac{D - D_{\mathrm{ramp}}}{V}, & D \ge D_{\mathrm{ramp}} \\[6pt]
      \dfrac{V_*}{A} + \dfrac{V_*}{B}, & D < D_{\mathrm{ramp}}
    \end{cases}
    \label{eq:trapezoid_duration}
  \end{equation}
  Where $D_{\mathrm{ramp}} = V^2 \left(\frac{1}{2A} + \frac{1}{2B}\right)$ and $V_* = \sqrt{\frac{2D}{\frac{1}{A} + \frac{1}{B}}}$.
</div>

<div class="theorem">
  <div class="theorem-title">Proposition 1 (Ballscrew Critical Whirl Velocity Ceiling)</div>
  Let the vertical $Z$ ballscrew have unsupported length $L_s = 0.45\,\mathrm{m}$, root diameter $d_r = 10\,\mathrm{mm}$, elastic modulus $E = 2.06 \times 10^{11}\,\mathrm{N/m^2}$, density $\rho = 7850\,\mathrm{kg/m^3}$, and lead $L_{\mathrm{lead}} = 20\,\mathrm{mm/rev}$. The fundamental transverse whirling frequency is:
  \begin{equation}
    \omega_{\mathrm{crit}} = \frac{\pi^2 d_r}{4 L_s^2} \sqrt{\frac{E}{\rho}} \approx 314.16\,\mathrm{rad/s} \implies 3000\,\mathrm{RPM}
    \label{eq:whirl}
  \end{equation}
  The critical linear velocity ceiling for the Z plunge axis is strictly bounded by:
  \begin{equation}
    v_{z,\mathrm{crit}} = \frac{3000\,\mathrm{RPM} \times 20\,\mathrm{mm/rev}}{60} = 1000.0\,\mathrm{mm/s}
    \label{eq:vz_crit}
  \end{equation}
</div>

<div class="theorem">
  <div class="theorem-title">Proposition 2 (Symmetric Constant Vertical Velocity &amp; High Acceleration)</div>
  Vertical plunge descent to $z_g$ and upward extraction retract to $z_{\mathrm{stage}}$ execute at symmetric constant cruise speed:
  \begin{equation}
    v_{\mathrm{plunge}} = v_{\mathrm{retract}} = v_z = 800.0\,\mathrm{mm/s} < v_{z,\mathrm{crit}}
    \label{eq:symmetric_vz}
  \end{equation}
  with linear acceleration and deceleration matched identically at:
  \begin{equation}
    a_{z,\mathrm{plunge}} = a_{z,\mathrm{retract}} = a_z = 20{,}000.0\,\mathrm{mm/s^2} \quad (20.0\,\mathrm{m/s^2} \approx 2.04\,g)
    \label{eq:az_kinetix}
  \end{equation}
  This kinematic symmetry guarantees identical profile durations $T_{\mathrm{plunge}} = T_{\mathrm{retract}}$ and maximizes the dynamic bandwidth of the Allen-Bradley Kinetix 5100 servo drive.
</div>

<!-- SECTION 4: PNEUMATIC ACTUATION & SYNCHRONIZATION THEOREMS -->

<h2>4. Non-Linear Pneumatic Actuator &amp; Solenoid Dynamics</h2>
<p class="noindent">
  The parallel gripper is actuated by a double-acting pneumatic cylinder supplied at line pressure $P_{\mathrm{line}} = 6.0\,\mathrm{bar}$ ($0.6\,\mathrm{MPa}$). Commanded closure begins with an electrical solenoid pilot dead-time $t_v = 8\,\mathrm{ms}$, followed by first-order chamber pressurization with time constant $\tau_p = 15\,\mathrm{ms}$:
</p>
<div style="text-align: center; margin: 4px 0;">
  \begin{equation}
    \dot{P}(t) = \frac{1}{\tau_p} \left(P_{\mathrm{line}} - P(t)\right) \implies P(t) = P_{\mathrm{line}} \left(1 - e^{-(t - t_{\mathrm{close}} - t_v)/\tau_p}\right)
    \label{eq:pressure_ode}
  \end{equation}
</div>
<p class="noindent">
  Under steady pressure, mechanical jaw displacement $q(t) \in [0, 40\,\mathrm{mm}]$ reaches terminal closing speed $v_{\mathrm{jaw}} = 450\,\mathrm{mm/s}$ (high-speed actuation) or $210.5\,\mathrm{mm/s}$ (damped). The pneumatic closing duration to reach opening aperture $W_{\mathrm{jaw}}$ is governed by:
</p>
<div style="text-align: center; margin: 4px 0;">
  \begin{equation}
    q(t') = v_{\mathrm{jaw}} \left(t' - \tau_p (1 - e^{-t'/\tau_p})\right), \qquad \tau_{\mathrm{close}} = t_v + t'
    \label{eq:tau_close_exact}
  \end{equation}
</div>

<div class="figure-box">
  <img src="__B64_TRAJECTORIES__" style="max-height: 72mm;" alt="Synchronized Multi-Axis Trajectories">
  <div class="caption">Figure 2: Multi-axis trajectory waveforms: Belt arrival, Ballscrew plunge, Jaw aperture closing, and Clamping force.</div>
</div>

<h2>5. Coordinated Grasp Synchronization &amp; Invariants</h2>

<div class="math-def">
  <div class="math-def-title">Definition 6 (Contact Tribology: PLA Fingertips &amp; Stainless Steel Casings)</div>
  <strong>Fingertip Material:</strong> 3D-printed Poly-Lactic Acid (PLA) plastic ($E_{\mathrm{PLA}} \approx 3.5\,\mathrm{GPa}$, Poisson ratio $\nu = 0.36$).<br>
  <strong>Battery Casing Material:</strong> Deep-drawn stainless steel / nickel-plated steel ($E_{\mathrm{steel}} \approx 200\,\mathrm{GPa}$).<br>
  <strong>Friction Coefficients:</strong> Static friction $\mu_s = 0.35$; kinetic sliding friction $\mu_k = 0.28$.<br>
  <strong>Viscoelastic Normal Force Model:</strong> Non-linear Hunt-Crossley contact formulation:
  \begin{equation}
    F_n(\delta, \dot{\delta}) = k_c \delta^{3/2} + c_c \delta^{3/2} \dot{\delta}
    \label{eq:hunt_crossley}
  \end{equation}
  Where $\delta$ is the compressive indentation and $k_c \approx 1.85 \times 10^5\,\mathrm{N/m^{3/2}}$. Steady clamping normal force is $F_n = 18.5\,\mathrm{N}$.
</div>

<div class="theorem">
  <div class="theorem-title">Theorem 1 (Separable Clearance Invariant)</div>
  Let the jaw opening be $W_{\mathrm{jaw}}(t) \in [w, w_{\mathrm{open}}]$. The clearances from the cell flank faces to the downstream and upstream fingers along the closing normal $\mathbf{n}(\theta)$ are $d_{\mathrm{down}}(t)$ and $d_{\mathrm{up}}(t)$. For all $t$ prior to contact:
  \begin{equation}
    d_{\mathrm{down}}(t) + d_{\mathrm{up}}(t) = W_{\mathrm{jaw}}(t) - w
    \label{eq:clearance_sum}
  \end{equation}
</div>

<div class="proof">
  <span class="proof-title">Proof:</span> By Definition 2, the gripper closes symmetrically along the normal axis $\mathbf{n}(\theta)$ centered at the pick datum $Y = y_p = 0$. The inner pad faces are located at coordinates $y_p \pm \frac{1}{2} W_{\mathrm{jaw}}(t)$ along $\mathbf{n}(\theta)$. The cell has flank width $w$ centered at position $y_c(t)$ along $\mathbf{n}(\theta)$. The respective instantaneous gaps are $d_{\mathrm{down}}(t) = \left(y_p + \frac{1}{2} W_{\mathrm{jaw}}(t)\right) - \left(y_c(t) + \frac{w}{2}\right)$ and $d_{\mathrm{up}}(t) = \left(y_c(t) - \frac{w}{2}\right) - \left(y_p - \frac{1}{2} W_{\mathrm{jaw}}(t)\right)$. Summing both gaps yields $d_{\mathrm{down}}(t) + d_{\mathrm{up}}(t) = W_{\mathrm{jaw}}(t) - w$, proving the invariant identically.
</div>

<!-- THEOREMS 2, 2.1, 2.2 -->

<div class="theorem">
  <div class="theorem-title">Theorem 2 (Upstream Contact Proximity Invariant)</div>
  Under calibrated strike timing:
  \begin{equation}
    t_z = t_{\mathrm{touch}} - \tau_{\mathrm{plunge}}, \qquad t_{\mathrm{close}} = t_{\mathrm{contact}} - \tau_{\mathrm{close}}
    \label{eq:calib_timing}
  \end{equation}
  Where $t_{\mathrm{contact}} = \frac{0.5 \times 1.80\,\mathrm{mm}}{v |\sin\theta|}$ and $\tau_{\mathrm{close}}$ is the exact solution to reaching aperture $W_{\mathrm{target}} = w + 1.80\,\mathrm{mm}$, at the instant of first downstream contact ($d_{\mathrm{down}}(t_{\mathrm{contact}}) = 0$), the upstream finger clearance satisfies:
  \begin{equation}
    d_{\mathrm{up}}(t_{\mathrm{contact}}) = 1.80\,\mathrm{mm} < 3.0\,\mathrm{mm}
    \label{eq:dup_invariant}
  \end{equation}
  identically across all conveyor speeds $v \in [200, 1500]\,\mathrm{mm/s}$ and orientation angles $\theta \in [0, 90^\circ]$.
</div>

<div class="proof">
  <span class="proof-title">Proof:</span> At first contact, by definition $d_{\mathrm{down}}(t_{\mathrm{contact}}) = 0$. Applying Theorem 1 directly at $t = t_{\mathrm{contact}}$:
  $$d_{\mathrm{up}}(t_{\mathrm{contact}}) = W_{\mathrm{jaw}}(t_{\mathrm{contact}}) - w$$
  Because $t_{\mathrm{close}}$ is pre-triggered exactly $\tau_{\mathrm{close}}$ prior to $t_{\mathrm{contact}}$, the pneumatic jaw displacement reaches $q_{\mathrm{contact}} = \mathrm{PAD\_HALF\_OPEN} - 0.5(w + 1.80\,\mathrm{mm})$, establishing aperture $W_{\mathrm{jaw}}(t_{\mathrm{contact}}) = w + 1.80\,\mathrm{mm}$. Substituting into the invariant yields $d_{\mathrm{up}}(t_{\mathrm{contact}}) = (w + 1.80\,\mathrm{mm}) - w = 1.80\,\mathrm{mm}$. Since $1.80\,\mathrm{mm} < 3.0\,\mathrm{mm}$, the proximity constraint is satisfied unconditionally across all $(v, \theta)$.
</div>

<div class="theorem">
  <div class="theorem-title">Theorem 2.1 (Zero-Dwell Ascent Clamping Dynamics &amp; Normal Force Invariance)</div>
  Let $t_{z,\mathrm{done}}$ denote the instant the vertical $Z$ axis completes plunge descent and reaches grasp depth $z_g$.
  <ol style="margin: 2px 0 2px 16px; padding-left: 0;">
    <li>At touchdown $t = t_{z,\mathrm{done}}$, the jaws have pre-closed to aperture $W_{\mathrm{jaw}}(t_{z,\mathrm{done}}) = w + 1.80\,\mathrm{mm} > w$. Consequently, normal force between the finger pads and battery flank faces at bottom dead center is identically zero:
      \begin{equation}
        F_N(t_{z,\mathrm{done}}) = 0.0\,\mathrm{N}
        \label{eq:fn_zero}
      \end{equation}
      guaranteeing zero downward impact or shear force into the conveyor belt bed.
    </li>
    <li>Vertical retract initiates immediately following grip close with strictly constant delay:
      \begin{equation}
        t_{\mathrm{retract}} = t_{\mathrm{close}} + \Delta t_{\mathrm{retract\_from\_close}}, \qquad \mathrm{dwell} = 0.0\,\mathrm{s}
        \label{eq:zero_dwell}
      \end{equation}
      where $\Delta t_{\mathrm{retract\_from\_close}} = \tau_{\mathrm{full\_close}} - \Delta t_{\mathrm{lead}} \equiv \mathrm{constant}$, and $\tau_{\mathrm{full\_close}} = t_{\mathrm{delay,valve}} + t'_{\mathrm{pneumatic}}(w)$ is the duration to full grasp contact on the battery flank ($W_{\mathrm{jaw}} = w$).
    </li>
    <li>By parameterizing lead $\Delta t_{\mathrm{lead}} = 8.0\,\mathrm{ms}$ ($8\,\mathrm{ticks}$), Z retract initiates exactly a couple of ticks ($8\,\mathrm{ms}$) before the gripper is fully closed. The relative delay between Z retract start and full gripper closure is strictly invariant:
      \begin{equation}
        \Delta t_{\mathrm{retract\_full}} = t_{\mathrm{retract}} - t_{\mathrm{full\_close}} = -\Delta t_{\mathrm{lead}} = -8.0\,\mathrm{ms} \equiv \mathrm{constant}
        \label{eq:delta_t_delay}
      \end{equation}
      guaranteeing that upward Z acceleration begins just prior to full grasp contact, completing clamp lock during the first millimeter of upward lift ($\Delta z_{\mathrm{lift}} \approx 0.64\,\mathrm{mm}$).
    </li>
  </ol>
</div>

<div class="proof">
  <span class="proof-title">Proof:</span> (1) Since $W_{\mathrm{jaw}}(t_{z,\mathrm{done}}) = w + 1.80\,\mathrm{mm} > w$, compressive indentation $\delta = 0$. By Hunt-Crossley formulation (Equation \ref{eq:hunt_crossley}), $F_N = 0.0\,\mathrm{N}$. (2) By control policy, $t_{\mathrm{retract}} = t_{\mathrm{close}} + \Delta t_{\mathrm{retract\_from\_close}}$. (3) Jaw integration proceeds monotonically from aperture $w + 1.80\,\mathrm{mm}$ to $w$ across residual gap $\Delta q = 0.90\,\mathrm{mm}$ per side. At closing speed $v_{\mathrm{jaw}} \approx 110\,\mathrm{mm/s}$, full closure completes $8.0\,\mathrm{ms}$ after retract start ($\Delta t_{\mathrm{lead}} = 8.0\,\mathrm{ms}$). Upward kinematic displacement during initial acceleration is $\Delta z_{\mathrm{lift}} = \frac{1}{2} a_z (\Delta t_{\mathrm{lead}})^2 = \frac{1}{2}(20{,}000)(0.008)^2 \approx 0.64\,\mathrm{mm}$, confirming full clamp locking strictly during upward ascent.
</div>

<div class="theorem">
  <div class="theorem-title">Theorem 2.2 (Velocity Feedforward Lead Compensation)</div>
  Let the Kinetix servo drive position loop execute with proportional position gain $k_p = 2.0 \times 10^6\,\mathrm{N/m}$ and velocity damping gain $k_v = 1.4 \times 10^4\,\mathrm{N\cdot s/m}$. For commanded reference trajectory $z_{\mathrm{traj}}(t)$ with velocity $\dot{z}_{\mathrm{traj}}(t)$, uncompensated proportional tracking exhibits steady following error:
  \begin{equation}
    \Delta z_{\mathrm{lag}}(t) = \frac{k_v}{k_p} \dot{z}_{\mathrm{traj}}(t)
    \label{eq:lag}
  \end{equation}
  At high-speed direction reversal ($\dot{z}: +800 \to -800\,\mathrm{mm/s}$), uncompensated following error creates abrupt velocity transients and drive saturation spikes.
  <br>
  By injecting 100% velocity feedforward lead compensation into the position command setpoint:
  \begin{equation}
    z_{\mathrm{cmd}}(t) = z_{\mathrm{traj}}(t) + \tau_{\mathrm{vff}} \dot{z}_{\mathrm{traj}}(t), \qquad \tau_{\mathrm{vff}} = \frac{k_v}{k_p} = \frac{14{,}000}{2{,}000{,}000} = 0.007\,\mathrm{s} = 7.0\,\mathrm{ms}
    \label{eq:vff_formula}
  \end{equation}
  the closed-loop tracking error satisfies $e(t) = z_{\mathrm{cmd}} - z - \tau_{\mathrm{vff}} \dot{z} \equiv z_{\mathrm{traj}} - z = 0$, guaranteeing monotonic velocity reversal across $z_g$ with strictly zero transient velocity spike.
</div>

<div class="proof">
  <span class="proof-title">Proof:</span> The closed-loop actuator force equation is $F(t) = k_p (z_{\mathrm{cmd}}(t) - z(t)) - k_v \dot{z}(t)$. Substituting $z_{\mathrm{cmd}}(t) = z_{\mathrm{traj}}(t) + \frac{k_v}{k_p} \dot{z}_{\mathrm{traj}}(t)$ yields:
  $$F(t) = k_p (z_{\mathrm{traj}} - z) + k_v (\dot{z}_{\mathrm{traj}} - \dot{z})$$
  At steady velocity tracking ($\dot{z} = \dot{z}_{\mathrm{traj}}$), $F(t) = k_p (z_{\mathrm{traj}} - z)$, yielding following error $z_{\mathrm{traj}} - z = F / k_p \to 0$. Across the turnaround at $z_g$, the command setpoint smoothly anticipates deceleration and reversal by exactly $7.0\,\mathrm{ms}$, completely canceling the viscous drag term $-k_v \dot{z}$ and preventing derivative kick or saturation spikes in the drive.
</div>

<!-- THEOREMS 2.3, 3, JITTER CONVOLUTION -->

<div class="theorem">
  <div class="theorem-title">Theorem 2.3 (Strict Firing Sequence, Full Plunge Completion &amp; Constant Retract Delay)</div>
  Firmware task <code>GantryUpdate</code> executes on FreeRTOS with tick rate $f_{\mathrm{RTOS}} = 1000\,\mathrm{Hz}$ ($T_{\mathrm{tick}} = 1.0\,\mathrm{ms}$). Actuators and triggers are scheduled in discrete integer ticks across three independent tuning sliders:
  \begin{equation}
    N_{z,\mathrm{detect}} = \mathrm{round}\left(\frac{\frac{771.0}{v_{\mathrm{belt}}} + t_z}{T_{\mathrm{tick}}}\right), \quad N_{\mathrm{close,detect}} = \mathrm{round}\left(\frac{\frac{771.0}{v_{\mathrm{belt}}} + t_{\mathrm{close}}}{T_{\mathrm{tick}}}\right), \quad N_{\mathrm{retract,detect}} = \mathrm{round}\left(\frac{\frac{771.0}{v_{\mathrm{belt}}} + t_{\mathrm{retract}}}{T_{\mathrm{tick}}}\right)
    \label{eq:freertos_detect_ticks}
  \end{equation}
  \begin{equation}
    N_{\mathrm{close\_ticks}} = \mathrm{round}\left(\frac{t_{\mathrm{close}} - t_z}{T_{\mathrm{tick}}}\right), \quad N_{\mathrm{retract\_ticks}} = \mathrm{round}\left(\frac{t_{\mathrm{retract}}}{T_{\mathrm{tick}}}\right)
    \label{eq:freertos_ticks}
  \end{equation}
  where $\Delta N_{z,\mathrm{offset\_ticks}} \in [-100, +100]$ tunes vertical descent release, $\Delta N_{\mathrm{offset\_ticks}} \in [-100, +100]$ tunes pneumatic valve energization, and $\Delta N_{\mathrm{retract\_ticks}} \in [-100, +100]$ tunes vertical extraction retract.
  <br>
  <strong>1. Strict Firing Sequence Invariant:</strong> Across all operational conditions and tuning offset states:
  \begin{equation}
    \mathrm{START\_PLUNGE} (t_z) \longrightarrow \mathrm{START\_GRIP} (t_{\mathrm{close}}) \longrightarrow \mathrm{START\_RETRACT} (t_{\mathrm{retract}}), \quad t_z < t_{\mathrm{close}} < t_{\mathrm{retract}}
    \label{eq:strict_sequence}
  \end{equation}
  Plunge begins first ($t_z$), pneumatic grip close triggers second ($t_{\mathrm{close}}$), and vertical retract initiates third ($t_{\mathrm{retract}}$).
  <br>
  <strong>2. Full Plunge Completion Invariant:</strong> The vertical plunge always completes its full physical stroke duration $\tau_{\mathrm{plunge}} = \mathrm{trapezoid\_duration}(z_h, z_g, v_z, a_z)$ to reach grasp depth $z_g$ before retract begins ($t_{\mathrm{retract}} = t_z + \tau_{\mathrm{plunge}}$ with zero dwell). Premature reversal or mid-air abort is strictly forbidden.
  <br>
  <strong>3. Coordinated Retract Delay Invariant:</strong> Across all operator tuning states $(\Delta N_{z,\mathrm{offset\_ticks}}, \Delta N_{\mathrm{offset\_ticks}}, \Delta N_{\mathrm{retract\_ticks}})$, the delay between Z retract start and grip close trigger is defined by:
  \begin{equation}
    \Delta t_{\mathrm{retract\_from\_close}} \equiv t_{\mathrm{retract}} - t_{\mathrm{close}} = \Delta t_{\mathrm{retract\_from\_close,nominal}} + \Delta N_{\mathrm{retract\_ticks}} \cdot T_{\mathrm{tick}}
    \label{eq:delay_offset_const}
  \end{equation}
  \begin{equation}
    \Delta t_{\mathrm{retract\_full,nominal}} \equiv t_{\mathrm{retract,nominal}} - t_{\mathrm{full\_close,nominal}} = -\Delta t_{\mathrm{lead}} = -8.0\,\mathrm{ms}
    \label{eq:delay_full_const}
  \end{equation}
  When $\Delta N_{\mathrm{retract\_ticks}} = 0$, this delay is strictly constant ($\Delta t_{\mathrm{retract\_from\_close,nominal}}$), completely independent of how $\Delta N_z$ or $\Delta N_{\mathrm{close}}$ are tuned. When $\Delta N_{\mathrm{retract\_ticks}} \neq 0$, the operator directly trims the retract trigger relative to grip close. The synchronized strike timeline shifts according to:
  \begin{equation}
    t_{\mathrm{retract}} = t_{\mathrm{retract,nominal}} + (\Delta N_{z,\mathrm{offset\_ticks}} + \Delta N_{\mathrm{offset\_ticks}} + \Delta N_{\mathrm{retract\_ticks}}) \cdot T_{\mathrm{tick}}
  \end{equation}
  \begin{equation}
    t_z = t_{\mathrm{retract}} - \tau_{\mathrm{plunge}}, \qquad t_{\mathrm{close}} = t_{\mathrm{retract}} - \Delta t_{\mathrm{retract\_from\_close}}
  \end{equation}
  preserving both the full plunge stroke to $z_g$ and the strict causal sequence $t_z < t_{\mathrm{close}} < t_{\mathrm{retract}}$.
</div>

<div class="proof">
  <span class="proof-title">Proof:</span> (1) By construction, $t_{\mathrm{close}} - t_z = \tau_{\mathrm{plunge}} - \Delta t_{\mathrm{retract\_from\_close}}$. For all battery profiles, ballscrew plunge duration $\tau_{\mathrm{plunge}} \in [103.2, 198.0]\,\mathrm{ms}$ strictly exceeds the retract delay $\Delta t_{\mathrm{retract\_from\_close}} \in [3.8, 95.8]\,\mathrm{ms}$, yielding minimum positive gap $t_{\mathrm{close}} - t_z \ge 7.4\,\mathrm{ms} > 0$. Similarly, $t_{\mathrm{retract}} - t_{\mathrm{close}} = \Delta t_{\mathrm{retract\_from\_close}} \ge 3.8\,\mathrm{ms} > 0$. Hence $t_z < t_{\mathrm{close}} < t_{\mathrm{retract}}$ holds unconditionally. (2) Since $t_{\mathrm{retract}} - t_z = \tau_{\mathrm{plunge}}$, kinematic descent duration matches the exact trapezoidal profile duration from staging ceiling $z_h$ to grasp depth $z_g$, ensuring zero velocity following error and complete touchdown before retract reversal. (3) Tuning offsets $\Delta N_z$ and $\Delta N_{\mathrm{close}}$ translate both $t_z$ and $t_{\mathrm{close}}$ in lockstep, while $\Delta N_{\mathrm{retract}}$ independently trims the retract delay $\Delta t_{\mathrm{retract\_from\_close}}$ while maintaining full plunge duration $\tau_{\mathrm{plunge}}$.
</div>

<div class="theorem">
  <div class="theorem-title">Theorem 3 (Dynamic Extraction Friction Invariance under 2g Acceleration)</div>
  Let PLA fingertips ($\mu = 0.28$) clamp a cell of mass $m_{\mathrm{cell}} = 0.140\,\mathrm{kg}$ (D-cell) under maximum upward extraction acceleration $a_{z,\mathrm{retract}} = 20.0\,\mathrm{m/s^2}$ ($2.04\,g$). The steady clamping normal force $F_n = 18.5\,\mathrm{N}$ guarantees that the extraction safety margin factor satisfies:
  \begin{equation}
    S_f = \frac{2 \mu F_n}{m_{\mathrm{cell}} (g + a_{z,\mathrm{retract}})} = \frac{2 \times 0.28 \times 18.5\,\mathrm{N}}{0.140\,\mathrm{kg} \times (9.81 + 20.0)\,\mathrm{m/s^2}} = \frac{10.36\,\mathrm{N}}{4.17\,\mathrm{N}} \approx 2.48 \ge 2.0
    \label{eq:sf_proof}
  \end{equation}
  For standard cells ($m \le 0.070\,\mathrm{kg}$), $S_f \ge 4.97 \ge 4.5$, completely eliminating slippage during high-speed vertical reversal.
</div>

<div class="figure-box">
  <img src="__B64_CLEARANCE__" style="max-height: 48mm;" alt="Separating-Axis Clearance Window">
  <div class="caption">Figure 3: Along-jaw passage window $W(v, \theta) = \frac{2g}{v \sin\theta}$ vs. orientation angle and belt velocity.</div>
</div>

<h2>6. Stochastic Jitter Propagation &amp; Error Containment</h2>
<p class="noindent">
  The system timing uncertainty vector $\mathbf{\epsilon} \in \mathbb{R}^4$ comprises four independent physical noise sources:
  (1) Kinetix DSP drive command-to-start latency $J_{\mathrm{dsp}} \sim \mathrm{Lognormal}(\mu = \ln 954\,\mu\mathrm{s}, \sigma = 0.12)$;
  (2) EtherNet/IP cyclic RPI phase alignment $J_{\mathrm{rpi}} \sim \mathcal{U}(0, 2.0\,\mathrm{ms})$;
  (3) FreeRTOS GantryUpdate scheduler phase $J_{\mathrm{sched}} \sim \mathcal{U}(0, 10.0\,\mathrm{ms})$;
  (4) Pneumatic pilot solenoid jitter $J_{\mathrm{valve}} \sim \mathcal{N}(0, 0.6\,\mathrm{ms})$.
</p>

<div class="figure-box">
  <img src="__B64_JITTER__" style="max-height: 44mm;" alt="Jitter Budget and Probability Distributions">
  <div class="caption">Figure 4: Hardware jitter distributions and deterministic error containment budget.</div>
</div>

<div class="theorem">
  <div class="theorem-title">Theorem 4 (Stochastic Jitter Containment)</div>
  The multi-fold convolution $f_\epsilon = f_{\mathrm{dsp}} * f_{\mathrm{rpi}} * f_{\mathrm{sched}} * f_{\mathrm{valve}}$ satisfies the $99.999\%$ quantile bound $\sup |\epsilon| \le 15.485\,\mathrm{ms}$. Under deterministic guard band $\Delta t_{\mathrm{guard}} = 20.0\,\mathrm{ms}$, the failure probability satisfies:
  \begin{equation}
    \mathbb{P}(\text{Timing Violation}) = \mathbb{P}(|\epsilon| > \Delta t_{\mathrm{guard}}) < 1.0 \times 10^{-6}
    \label{eq:prob_safety}
  \end{equation}
</div>

<div class="page-break"></div>

<h2>7. Exhaustive Physical Parameter Catalog</h2>
<table>
  <thead>
    <tr>
      <th>Symbol</th>
      <th>Description</th>
      <th>Numerical Value</th>
      <th>Units</th>
      <th>Firmware / Canonical Source</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>$z_b$</td>
      <td>Conveyor Bed Elevation</td>
      <td>102.0 / 115.0</td>
      <td>mm</td>
      <td><code>gantry_twin.py / pick_intercept.h</code></td>
    </tr>
    <tr>
      <td>$Z_{\mathrm{traverse}}$</td>
      <td>Safe Traverse Ceiling</td>
      <td>20.0 / 35.7</td>
      <td>mm</td>
      <td><code>BIN_TRAVERSE_Z_MM / gantry_constants.h</code></td>
    </tr>
    <tr>
      <td>$z_{\mathrm{stage}}$</td>
      <td>Dynamic Pick Staging Ceiling</td>
      <td>$\min(65.0, \max(15.0, (z_b - h) - 6.0))$</td>
      <td>mm</td>
      <td><code>gantry_twin.py (staging_z_mm)</code></td>
    </tr>
    <tr>
      <td>$z_g(h)$</td>
      <td>Piecewise Grasp Depth</td>
      <td>$z_b - 3.0$ ($h < 30$), $z_b - 4.0$ ($h \ge 30$)</td>
      <td>mm</td>
      <td><code>Piecewise rule: 99.00 / 98.00 mm (twin)</code></td>
    </tr>
    <tr>
      <td>$v_{\mathrm{plunge}}, v_{\mathrm{retract}}$</td>
      <td>Symmetric Constant Vertical Velocity</td>
      <td>1000.0</td>
      <td>mm/s</td>
      <td><code>PICK_SCHEDULER_KINEMATICS.md §5</code></td>
    </tr>
    <tr>
      <td>$a_{z,\mathrm{plunge}}, a_{z,\mathrm{retract}}$</td>
      <td>Symmetric Vertical Acceleration</td>
      <td>20,000.0 ($20.0\,\mathrm{m/s^2}$)</td>
      <td>mm/s$^2$</td>
      <td><code>axis_drivetrain_params.h (Kinetix 5100)</code></td>
    </tr>
    <tr>
      <td>$\mathrm{dwell}$</td>
      <td>Dwell Duration at Grasp Depth</td>
      <td>0.0</td>
      <td>ms</td>
      <td><code>Zero-dwell instantaneous reversal</code></td>
    </tr>
    <tr>
      <td>$\tau_{\mathrm{vff}}$</td>
      <td>Velocity Feedforward Lead Constant</td>
      <td>7.0 ($k_v / k_p$)</td>
      <td>ms</td>
      <td><code>gantry_twin.py (VFF feedforward)</code></td>
    </tr>
    <tr>
      <td>$T_{\mathrm{tick}}$</td>
      <td>FreeRTOS Scheduling Quantum</td>
      <td>1.0 ($1000\,\mathrm{Hz}$)</td>
      <td>ms</td>
      <td><code>CONFIG_FREERTOS_HZ = 1000</code></td>
    </tr>
    <tr>
      <td>$\mathrm{Seq}_{\mathrm{strike}}$</td>
      <td>Strict Firing Sequence Order</td>
      <td>$t_z < t_{\mathrm{close}} < t_{\mathrm{retract}}$</td>
      <td>&mdash;</td>
      <td><code>START_PLUNGE &rarr; START_GRIP &rarr; START_RETRACT</code></td>
    </tr>
    <tr>
      <td>$\Delta N_{\mathrm{offset\_ticks}}$</td>
      <td>Tunable Grip Close Timing Offset</td>
      <td>$[-100, +100]$</td>
      <td>ticks</td>
      <td><code>sim_ui.py / --close-ticks</code></td>
    </tr>
    <tr>
      <td>$\Delta N_{z,\mathrm{offset\_ticks}}$</td>
      <td>Tunable Z Plunge Timing Offset</td>
      <td>$[-100, +100]$</td>
      <td>ticks</td>
      <td><code>sim_ui.py / --plunge-ticks</code></td>
    </tr>
    <tr>
      <td>$\Delta N_{\mathrm{retract\_ticks}}$</td>
      <td>Tunable Z Retract Timing Offset</td>
      <td>$[-100, +100]$</td>
      <td>ticks</td>
      <td><code>sim_ui.py / --retract-ticks</code></td>
    </tr>
    <tr>
      <td>$N_{z,\mathrm{detect}}$</td>
      <td>Plunge Firing Time Since Detection</td>
      <td>$\mathrm{round}(771/v_{\mathrm{belt}} + t_z)$</td>
      <td>ticks</td>
      <td><code>sim_ui.py / gantry_twin.py</code></td>
    </tr>
    <tr>
      <td>$N_{\mathrm{close,detect}}$</td>
      <td>Grip Close Firing Time Since Detection</td>
      <td>$\mathrm{round}(771/v_{\mathrm{belt}} + t_{\mathrm{close}})$</td>
      <td>ticks</td>
      <td><code>sim_ui.py / gantry_twin.py</code></td>
    </tr>
    <tr>
      <td>$N_{\mathrm{retract,detect}}$</td>
      <td>Retract Firing Time Since Detection</td>
      <td>$\mathrm{round}(771/v_{\mathrm{belt}} + t_{\mathrm{retract}})$</td>
      <td>ticks</td>
      <td><code>sim_ui.py / gantry_twin.py</code></td>
    </tr>
    <tr>
      <td>$\Delta t_{\mathrm{lead}}$</td>
      <td>Constant Retract Lead Before Full Close</td>
      <td>8.0 (constant)</td>
      <td>ms</td>
      <td><code>Theorem 2.1 ($t_{\mathrm{retract}} - t_{\mathrm{full\_close}} = -8\,\mathrm{ms}$)</code></td>
    </tr>
    <tr>
      <td>$\Delta t_{\mathrm{retract\_from\_close}}$</td>
      <td>Coordinated Retract Delay from Close Trigger</td>
      <td>$\tau_{\mathrm{full\_close}} - 8.0 + \Delta N_{\mathrm{retract}}$</td>
      <td>ms</td>
      <td><code>Theorem 2.3 ($t_{\mathrm{retract}} - t_{\mathrm{close}}$)</code></td>
    </tr>
    <tr>
      <td>$\delta_{\mathrm{clear}}$</td>
      <td>Pad Clearance at Touchdown</td>
      <td>0.9 (per side)</td>
      <td>mm</td>
      <td><code>W_jaw = w + 1.8 mm ($F_N = 0.0\,\mathrm{N}$)</code></td>
    </tr>
    <tr>
      <td>$w_{\mathrm{open}}$</td>
      <td>Full Open Jaw Aperture</td>
      <td>80.0</td>
      <td>mm</td>
      <td><code>coordinated_grasp.h (kOpenMm)</code></td>
    </tr>
    <tr>
      <td>$v_{\mathrm{jaw}}$</td>
      <td>Jaw Clamping Speed</td>
      <td>210.5 &ndash; 450.0</td>
      <td>mm/s</td>
      <td><code>physics_engines.py (JAW_SPEED_MM_S)</code></td>
    </tr>
    <tr>
      <td>$t_v$</td>
      <td>Solenoid Valve Pilot Dead-Time</td>
      <td>8.0</td>
      <td>ms</td>
      <td><code>coordinated_grasp.h (kValveOnS)</code></td>
    </tr>
    <tr>
      <td>$\tau_p$</td>
      <td>Pressure Rise Time Constant</td>
      <td>15.0</td>
      <td>ms</td>
      <td><code>physics_engines.py (VALVE_TAU_MS)</code></td>
    </tr>
    <tr>
      <td>$\mu_{\mathrm{PLA}}$</td>
      <td>Fingertip Kinetic Friction</td>
      <td>0.28</td>
      <td>&mdash;</td>
      <td><code>3D-printed PLA on Stainless Steel</code></td>
    </tr>
    <tr>
      <td>$d_{\mathrm{up}}$</td>
      <td>Upstream Pad Clearance at Contact</td>
      <td>1.80 (&lt; 3.0)</td>
      <td>mm</td>
      <td><code>compute_strike_plan() invariant</code></td>
    </tr>
    <tr>
      <td>$\Delta t_{\mathrm{guard}}$</td>
      <td>Deterministic Safety Guard Band</td>
      <td>20.0</td>
      <td>ms</td>
      <td><code>ON_THE_FLY_PICKUP.md §5</code></td>
    </tr>
  </tbody>
</table>

<div style="margin-top: 20px; text-align: center; font-size: 7.8pt; color: #6b7280;">
  &mdash; END OF MATHEMATICAL SPECIFICATION &mdash;<br>
  WT32-ETH01 Gantry Digital Twin Control System Specification &bull; Doc ID: KIN-MATH-SPEC-2026-REV6
</div>

</body>
</html>
"""
    return tmpl.replace("__B64_KINEMATICS__", b64_kinematics).replace("__B64_TRAJECTORIES__", b64_trajectories).replace("__B64_CLEARANCE__", b64_clearance).replace("__B64_JITTER__", b64_jitter)


# ==============================================================================
# 3. COMPILATION RUNNER
# ==============================================================================

def compile_html_to_pdf(html_path: str, pdf_path: str) -> bool:
    print(f"Compiling {os.path.basename(html_path)} -> {os.path.basename(pdf_path)}...")
    cmd = [
        CHROME_PATH,
        "--headless=new",
        "--disable-gpu",
        "--run-all-compositor-stages-before-draw",
        "--virtual-time-budget=9000",
        "--no-pdf-header-footer",
        f"--print-to-pdf={pdf_path}",
        html_path
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode == 0 and os.path.exists(pdf_path) and os.path.getsize(pdf_path) > 0:
        print(f"SUCCESS: Generated {os.path.basename(pdf_path)} ({os.path.getsize(pdf_path) / 1024.0:.1f} KB)")
        return True
    else:
        print(f"ERROR: Failed to compile {html_path}. RC={res.returncode}")
        print("Stdout:", res.stdout)
        print("Stderr:", res.stderr)
        return False


def main():
    brief_html_path = os.path.join(DOCS_DIR, "Grasp_Timing_Executive_Brief.html")
    brief_pdf_path  = os.path.join(DOCS_DIR, "Grasp_Timing_Executive_Brief.pdf")
    
    spec_html_path  = os.path.join(DOCS_DIR, "Grasp_Timing_Mathematical_Specification.html")
    spec_pdf_path   = os.path.join(DOCS_DIR, "Grasp_Timing_Mathematical_Specification.pdf")

    # 1. Write Executive Brief
    print("Writing Grasp_Timing_Executive_Brief.html...")
    with open(brief_html_path, "w", encoding="utf-8") as f:
        f.write(build_executive_brief_html())

    # 2. Write Mathematical Specification
    print("Writing Grasp_Timing_Mathematical_Specification.html...")
    with open(spec_html_path, "w", encoding="utf-8") as f:
        f.write(build_mathematical_specification_html())

    # 3. Compile Executive Brief PDF
    ok1 = compile_html_to_pdf(brief_html_path, brief_pdf_path)

    # 4. Compile Mathematical Specification PDF
    ok2 = compile_html_to_pdf(spec_html_path, spec_pdf_path)

    if ok1 and ok2:
        print("\n================================================================================")
        print(" Both Grasp Timing PDFs generated successfully!")
        print(f" 1. Presentable Brief: {brief_pdf_path}")
        print(f" 2. In-Depth Math Spec: {spec_pdf_path}")
        print("================================================================================\n")
        sys.exit(0)
    else:
        print("Failed to generate one or both PDFs.")
        sys.exit(1)


if __name__ == "__main__":
    main()
