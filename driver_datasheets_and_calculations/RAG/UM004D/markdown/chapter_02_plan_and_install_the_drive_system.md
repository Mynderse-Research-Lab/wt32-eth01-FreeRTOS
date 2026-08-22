# Chapter 2: Plan and Install the Drive System

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `03_Ch02_Plan_and_Install.pdf` (19 pages)_

<!-- page 1 -->

This chapter describes system installation guidelines used in preparation for
mounting your Kinetix® 5100 drive components.
System Design Guidelines
Use the information in this section when you design your enclosure and plan
to mount your system components on the panel.
For online product selection and system configuration tools, including
AutoCAD (DXF) drawings of the product, see
http://www.rockwellautomation.com/global/support/selection.page.
System Mounting Requirements
•
To comply with UL and CE and UK requirements, the Kinetix 5100 drive
system must be mounted in a grounded conductive enclosure offering
protection as defined in standard EN/IEC 60529 to IP20 such that they
are not accessible to an operator or unskilled person.
•
To maintain the functional safety rating of the Kinetix 5100 drive system,
this enclosure must be appropriate for the environmental conditions of
the industrial location and provide a protection class of IP54 or higher.
•
The panel you install inside the enclosure for mounting your system
components must be on a flat, rigid, vertical surface that won’t be
subjected to shock, vibration, moisture, oil mist, dust, or corrosive vapors
in accordance with pollution degree 2 (EN/IEC 61800-5-1) because the
product is rated to protection class IP20 (EN 60529).
•
Size the drive enclosure so as not to exceed the maximum-ambient
temperature rating. Consider heat dissipation specifications for all drive
components.
Topic
Page
System Design Guidelines

Electrical Noise Reduction

Mount Your Kinetix 5100 Drive

ATTENTION: Plan the installation of your system so that you can cut, drill,
tap, and weld with the system removed from the enclosure. Because the
system is of the open type construction, be careful to keep any metal debris
from falling into it. Metal debris or other foreign matter can become lodged
in the circuitry, which can result in damage to components.

<!-- page 2 -->

•
Use high-frequency (HF) techniques for bonding to connect the
enclosure, machine frame, and motor housing, and to provide a lowimpedance return path for high-frequency (HF) energy and reduce
electrical noise.
Bond the Kinetix 5100 drive modules and line filter grounding screws by
using a braided ground strap as shown in Figure 52 on page 85.
See the System Design for Control of Electrical Noise Reference Manual,
publication GMC-RM001, to better understand the concept of electrical noise
reduction.
AC Line Filter Selection
An AC line filter is required to meet CE and UK requirements. Install an AC
line filter for input power as close to the 2198-Exxxx-ERS servo drive as
possible.
a
IMPORTANT
Kinetix 5100 servo drives support only grounded wye power
configurations. For facility power configuration examples, see
Determine the Input Power Configuration on page 80.

> **Table 4** — AC Line Filter Selection

Kinetix 5100 Drive
Cat. No.
Input Voltage (nom)
AC Line Filter Cat. No.
AC Line Filter Cat. No.
Single-phase Operation
Three-phase Operation
2198-E1004-ERS
120V single-phase
200…230V singlephase
230V three-phase
2198-DB111-F
2198-DB310-F
2198-DB310-F
2198-E1007-ERS
2198-DB127-F
2198-E1015-ERS
2198-DB324-F
2198-DB324-F
2198-E1020-ERS
2198-E2030-ERS
230V three-phase
—
2198-E2055-ERS
—
2198-DB335-F
2198-E2075-ERS
—
2198-DB356-F
2198-E2150-ERS
—
2198-DBR90-F
2198-E4004-ERS
480V three-phase
—
2198-DB418-F
2198-E4007-ERS
—
2198-E4015-ERS
—
2198-E4020-ERS
—
2198-E4030-ERS
—
2198-E4055-ERS
—
2198-DB433-F
2198-E4075-ERS
—
2198-E4150-ERS
—
2198-DBR40-F
IMPORTANT
Select 2198-DB310-F and 2198-DB324-F line filters for replacements in
existing installations and new systems of 2198-E10XX-ERS drives.

**Extracted table (page 2, #1):**

| Input Voltage (nom) | AC Line Filter Cat. No. |  |
| --- | --- | --- |
|  | Single-phase Operation |  |
| 120V single-phase 200…230V single- phase 230V three-phase | 2198-DB111-F | 2198-DB310-F |
|  | 2198-DB127-F |  |
|  |  | 2198-DB324-F |
| 230V three-phase | — |  |
|  | — |  |
|  | — |  |
|  | — |  |
| 480V three-phase | — |  |
|  | — |  |
|  | — |  |
|  | — |  |
|  | — |  |
|  | — |  |
|  | — |  |
|  | — |  |

<!-- page 3 -->

## Circuit Breaker/Fuse Selection

The Kinetix 5100 drives use internal solid-state motor short-circuit protection
and, when connected to a suitable branch circuit protection, are rated for use
on a circuit that can deliver up to 5000 A (fuses or circuit breakers).
Make sure the selected components are properly coordinated and meet
acceptable codes including any requirements for branch circuit protection.
Evaluation of the short-circuit available current is critical and must be kept
below the short-circuit current rating of the circuit breaker.
See the Kinetix Servo Drives Specifications Technical Data, publication
KNX-TD003, for input current and inrush current specifications for your
Kinetix 5100 drive.
Circuit Breaker/Fuse Specifications
Kinetix 5100 servo drives use internal solid-state motor short-circuit
protection and, when protected by suitable branch circuit protection, are rated
for use on a circuit capable of delivering up to 5000 A when protected by fuses
or circuit breakers. These fuses and Allen-Bradley® circuit breakers are
recommended for use with 2198-Exxxx-ERS drives.
IMPORTANT
Do not use circuit protection devices on the output of an AC drive as
an isolating disconnect switch or motor overload device. These
devices are designed to operate on sine-wave voltage and the drive's
PWM waveform does not allow it to operate properly. As a result,
damage to the device occurs.

> **Table 5** — Control Power Circuit-protection Specifications

Cat. No.
Fuse (Bussman)
Cat. No
Miniature CB
Cat. No.
2198-E1004-ERS
KTK-R-2 (2 A)
1489-M2D010
2198-E1007-ERS
2198-E1015-ERS
2198-E1020-ERS
1489-M2D020
2198-E2030-ERS
1489-M2D010
2198-E2055-ERS
KTK-R-3 (3 A)
1489-M2D016
2198-E2075-ERS
2198-E2150-ERS
KTK-R-5 (5 A)
1489-M2D030

<!-- page 4 -->

> **Table 6** — Input Power UL/CSA Circuit-protection Specifications

Kinetix 5100 Drive
Cat. No.
Drive Voltage
Fuses (Bussmann)
Cat. No.
Miniature CB (1)
Cat. No.
Molded Case CB
Cat. No.
2198-E1004-ERS
120V/230V, single-phase
KTK-R-15 (15 A)
1489-M2D100
–
230V, three-phase
KTK-R-10 (10 A)
1489-M3D100
–
2198-E1007-ERS
120V/230V, single-phase
KTK-R-20 (20 A)
1489-M2D200
–
230V, three-phase
KTK-R-15 (15 A)
1489-M3D130
–
2198-E1015-ERS
120V/230V, single-phase
KTK-R-30 (30 A)
1489-M2D300
–
230V, three-phase
KTK-R-25 (25 A)
1489-M3D200
–
2198-E1020-ERS
120V/230V, single-phase
LPJ-40SP (40 A)
1489-M2D400
–
230V, three-phase
LPJ-35SP (35 A)
1489-M3D300
–
2198-E2030-ERS
230V, three-phase
LPJ-50SP (50 A)
1489-M3D350
–
2198-E2055-ERS
LPJ-70SP (70 A)
1489-M3D600
–
2198-E2075-ERS
LPJ-80SP (80 A)
–
140G-G2C3-C70
2198-E2150-ERS
LPJ-125SP (125 A)
–
140G-G2C3-D12
2198-E4004-ERS
380…480V AC, three-phase
KTK-R-10 (10A)
1489-M3D100
–
2198-E4007-ERS
KTK-R-15 (15A)
1489-M3D100
–
2198-E4015-ERS
KTK-R-20 (20A)
1489-M3D150
–
2198-E4020-ERS
KTK-R-25 (25A)
1489-M3D200
–
2198-E4030-ERS
KTK-R-30 (30A)
1489-M3D300
–
2198-E4055-ERS
LPJ-35SP (35A)
1489-M3D350
–
2198-E4075-ERS
LPJ-45SP (45A)
–
140G-G6C3-C45
2198-E4150-ERS
LPJ-90SP (90A)
–
140G-G6C3-C60
(1)
There are no recommended motor-protection circuit breakers for the Kinetix 5100 servo drives.

> **Table 7** — Input Power IEC (non-UL/CSA) Circuit-protection Specifications

Kinetix 5100 Drive
Cat. No.
Drive Voltage
DIN gG Fuses
Amps, Max
Miniature CB (1)
Cat. No.
Molded Case CB
Cat. No.
2198-E1004-ERS
120V/230V, single-phase

1489-M2D100
–
230V, three-phase

1489-M3D100
–
2198-E1007-ERS
120V/230V, single-phase

1489-M2D200
–
230V, three-phase

1489-M3D130
–
2198-E1015-ERS
120V/230V, single-phase

1489-M2D300
–
230V, three-phase

1489-M3D200
–
2198-E1020-ERS
120V/230V, single-phase

1489-M2D400
–
230V, three-phase

1489-M3D300
–
2198-E2030-ERS
230V, three-phase

1489-M3D350
–
2198-E2055-ERS

1489-M3D600
–
2198-E2075-ERS

–
140G-G2C3-C70
2198-E2150-ERS

–
140G-G2C3-D12
2198-E4004-ERS
380…480V AC, three-phase

1489-M3D100
–
2198-E4007-ERS

1489-M3D100
–
2198-E4015-ERS

1489-M3D150
–
2198-E4020-ERS

1489-M3D200
–
2198-E4030-ERS

1489-M3D300
–
2198-E4055-ERS

1489-M3D350
–
2198-E4075-ERS

–
140G-G6C3-C45
2198-E4150-ERS

–
140G-G6C3-C60
(1)
There are no recommended motor protection circuit breakers for the Kinetix 5100 servo drives.

**Extracted table (page 4, #1):**

| Drive Voltage | Fuses (Bussmann) Cat. No. | Miniature CB (1) Cat. No. |
| --- | --- | --- |
| 120V/230V, single-phase | KTK-R-15 (15 A) | 1489-M2D100 |
| 230V, three-phase | KTK-R-10 (10 A) | 1489-M3D100 |
| 120V/230V, single-phase | KTK-R-20 (20 A) | 1489-M2D200 |
| 230V, three-phase | KTK-R-15 (15 A) | 1489-M3D130 |
| 120V/230V, single-phase | KTK-R-30 (30 A) | 1489-M2D300 |
| 230V, three-phase | KTK-R-25 (25 A) | 1489-M3D200 |
| 120V/230V, single-phase | LPJ-40SP (40 A) | 1489-M2D400 |
| 230V, three-phase | LPJ-35SP (35 A) | 1489-M3D300 |
| 230V, three-phase | LPJ-50SP (50 A) | 1489-M3D350 |
|  | LPJ-70SP (70 A) | 1489-M3D600 |
|  | LPJ-80SP (80 A) | – |
|  | LPJ-125SP (125 A) | – |
| 380…480V AC, three-phase | KTK-R-10 (10A) | 1489-M3D100 |
|  | KTK-R-15 (15A) | 1489-M3D100 |
|  | KTK-R-20 (20A) | 1489-M3D150 |
|  | KTK-R-25 (25A) | 1489-M3D200 |
|  | KTK-R-30 (30A) | 1489-M3D300 |
|  | LPJ-35SP (35A) | 1489-M3D350 |
|  | LPJ-45SP (45A) | – |
|  | LPJ-90SP (90A) | – |

**Extracted table (page 4, #2):**

| Drive Voltage | DIN gG Fuses Amps, Max | Miniature CB (1) Cat. No. |
| --- | --- | --- |
| 120V/230V, single-phase | 15 | 1489-M2D100 |
| 230V, three-phase | 10 | 1489-M3D100 |
| 120V/230V, single-phase | 20 | 1489-M2D200 |
| 230V, three-phase | 15 | 1489-M3D130 |
| 120V/230V, single-phase | 30 | 1489-M2D300 |
| 230V, three-phase | 25 | 1489-M3D200 |
| 120V/230V, single-phase | 40 | 1489-M2D400 |
| 230V, three-phase | 35 | 1489-M3D300 |
| 230V, three-phase | 50 | 1489-M3D350 |
|  | 70 | 1489-M3D600 |
|  | 80 | – |
|  | 125 | – |
| 380…480V AC, three-phase | 10 | 1489-M3D100 |
|  | 15 | 1489-M3D100 |
|  | 20 | 1489-M3D150 |
|  | 25 | 1489-M3D200 |
|  | 30 | 1489-M3D300 |
|  | 35 | 1489-M3D350 |
|  | 45 | – |
|  | 90 | – |

<!-- page 5 -->

## Transformer Selection

The Kinetix 5100 drive does not require an isolation transformer for threephase input power. However, a transformer can be required to match the
voltage requirements of the drive to the available service.
To size a transformer for the main AC power inputs, see Circuit Breaker/Fuse
Selection on page 33 and Kinetix Servo Drives Specifications Technical Data,
publication KNX-TD003.
Passive Shunt Considerations
See Table 8 for the 2198-Exxxx-ERS servo drives that include internal shunt
resistors. Bulletin 2198-Rxxx and 2097-Rx external passive shunts are available
to provide additional shunt capacity for applications where the internal shunt
capacity is exceeded or in applications requiring shunt capacity for drives
without an internal shunt.
IMPORTANT
Transformers (auto transformer is not supported) must have WYE
secondary with grounded neutral. Phase to neutral voltage must not
exceed the input voltage rating of the drive.
IMPORTANT
Use a factor of 1.5 for single and three-phase power (this factor is used
to compensate for transformer, drive, and motor losses, and to account
for utilization in the intermittent operating area of the torque speed
curve).
For example, size a transformer to the voltage requirements of catalog
number 2198-E2030-ERS = 3 kW continuous x 1.5 = 4.5 KVA transformer.
IMPORTANT
A line reactor must be used if the source transformer is greater than 150
KVA, max and 3% impedance, min.

> **Table 8** — External Passive-shunt Options

## Kinetix 5100 Servo Drive

Cat. No.
Internal Shunt
Resistor
Shunt Power
Capacity of
Resistor
External Shunt
Resistance, min
Bulletin 2198 External Shunt Module (1) Cat. No.
Ω
W
Ω
2198-R031
2198-R004
2097-R6
2097-R7
2198-E1004-ERS

–
–
X
X
2198-E1007-ERS

–
–
X
X
2198-E1015-ERS

X
X
X
X
2198-E1020-ERS

X
X
X
X
2198-E2030-ERS
X
X
X
X
2198-E2055-ERS
–
–

X
X
X
X
2198-E2075-ERS
–
–
X
X
X
X
2198-E2150-ERS
–
–

X
X
X
X
2198-E4004-ERS

–
–
–
X
2198-E4007-ERS

–
–
X
X
2198-E4015-ERS

–
–
X
X
2198-E4020-ERS
–
–
–
–
X
X
2198-E4030-ERS
–
–

X
X
X
X
2198-E4055-ERS
–
–

X
X
X
X
2198-E4075-ERS
–
–

X
X
X
X
2198-E4150-ERS
–
–

X
X
X
X
(1)
Shunt resistor selection is based on the needs of your actual hardware configuration.

**Extracted table (page 5, #1):**

| Internal Shunt Resistor | Shunt Power Capacity of Resistor | External Shunt Resistance, min |  |  |  |
| --- | --- | --- | --- | --- | --- |
| Ω | W | Ω | 2198-R031 | 2198-R004 | 2097-R6 |
| 100 | 5 | 60 | – | – | X |
|  | 14 |  | – | – | X |
|  |  | 30 | X | X | X |
| 20 | 20 | 15 | X | X | X |
|  |  |  | X | X | X |
| – | – | 10 | X | X | X |
| – | – |  | X | X | X |
| – | – | 5 | X | X | X |
| 80 | 10 | 80 | – | – | – |
|  | 10 | 60 | – | – | X |
|  | 10 | 40 | – | – | X |
| – | – |  | – | – | X |
| – | – | 30 | X | X | X |
| – | – | 20 | X | X | X |
| – | – | 15 | X | X | X |
| – | – | 12 | X | X | X |

<!-- page 6 -->

Catalog number 2198-R031 is composed of resistor coils that are housed inside
an enclosure. Catalog numbers 2198-R004, 2097-R6, and 2097-R7 are shunt
resistors without an enclosure.

> **Figure 9** — External Passive Shunts

How the Bulletin 2198-Rxxx and 2097-Rx shunts connect to the Kinetix 5100
drive is explained in External Passive-shunt Resistor Connections on page 108
and illustrated with interconnect diagrams in Passive Shunt Wiring Examples
on page 463.
Enclosure Selection
This example is provided to assist you in size selection for an enclosure for
your Kinetix 5100 drive system. You need heat dissipation data from all
components that are planned for your enclosure to calculate the enclosure size.
See Table 10 on page 37 for the Kinetix 5100 drive heat dissipation
specifications.
ATTENTION: See Table 8 for the minimum external shunt resistance.
Connecting an external shunt resistor of with resistance rating lower than
specified results in (drive-side) shunt circuitry damage.
2198-R031
Shunt Module
2198-R004,
2097-R6, and 2097-R7
Shunt Resistors

> **Table 9** — External Shunt Module Specifications

## Shunt Module

Cat. No.
Resistance
Ω
Continuous Power
W
Weight, approx
kg (lb)
2097-R6

0.3 (0.7)
2097-R7

0.2 (0.4)
2198-R004

1.8 (4.0)
2198-R031

3100
16.8 (37)
IMPORTANT
We recommend that new installations or field replacements for 2097-R6
and 2097-R7 shunt modules use 2198-R002 or 2198-R001 shunt resistors

**Extracted table (page 6, #1):**

| Resistance Ω | Continuous Power W |
| --- | --- |
| 75 | 150 |
| 150 | 80 |
| 33 | 400 |
| 33 | 3100 |

<!-- page 7 -->

With no active method of heat dissipation (such as fans or air conditioning),
either of the following approximate equations can be used.
If the maximum ambient rating of the Kinetix 5100 drive system is 50 °C
(122 °F) and if the maximum environmental temperature is 20 °C (68 °F), then
T=30. In this example, the total heat dissipation is 416 W (sum of all
components in enclosure). So, in the equation below, T=30 and Q=416.
In this example, the enclosure must have an exterior surface of at least 2.99 m2.
If any portion of the enclosure is not able to transfer heat, do not include that
value in the calculation.
Because the minimum cabinet depth to house the Kinetix 5100 system
(selected for this example) is 300 mm (11.8 in.), the cabinet needs to be
approximately 1500 x 700 x 300 mm (59.0 x 27.6 x 11.8 in.) HxWxD.
1.5 x (0.300 x 0.70) + 1.5 x (0.300 x 2.0) + 1.5 x (0.70 x 2.0) = 3.31 m2
Because this cabinet size is considerably larger than what is necessary to house
the system components, it can be more efficient to provide a means of cooling
in a smaller cabinet. Contact your cabinet manufacturer for options available
to cool your cabinet.
Table 10 provides total power dissipation for Kinetix 5100 drives, three-phase
operation, with 100% rated current and speed.
Metric
Standard English
Where T is temperature difference between inside air
and outside ambient (°C), Q is heat that is generated in
enclosure (Watts), and A is enclosure surface area (m2).
The exterior surface of all six sides of an enclosure is
calculated as
Where T is temperature difference between inside air
and outside ambient (°F), Q is heat that is generated in
enclosure (Watts), and A is enclosure surface area (ft2).
The exterior surface of all six sides of an enclosure is
calculated as
A = 2dw + 2dh + 2wh
A = (2dw + 2dh + 2wh) /144
Where d (depth), w (width), and h (height) are in meters. Where d (depth), w (width), and h (height) are in inches.
A =
0.38Q
1.8T - 1.1
A = 4.08Q
T - 1.1
A =
0.38 (416)
1.8 (30) - 1.1 = 2.99 m2

> **Table 10** — Power Dissipation Specifications

Kinetix 5100 (200V-class)
Drives
Cat. No.
Loss (230V), max
W
Kinetix 5100 (400V-class) Drives
Cat. No.
Loss (380V), max
W
Loss (480V), max
W
2198-E1004-ERS
38.06
2198-E4004-ERS

2198-E1007-ERS
66.33
2198-E4007-ERS

2198-E1015-ERS
87.23
2198-E4015-ERS

2198-E1020-ERS
139.83
2198-E4020-ERS

2198-E2030-ERS
179.53
2198-E4030-ERS

2198-E2055-ERS
328.52
2198-E4055-ERS

2198-E2075-ERS
372.33
2198-E4075-ERS

2198-E2150-ERS
648.55
2198-E4150-ERS

<!-- page 8 -->

## Minimum Clearance Requirements

This section provides information to assist you in sizing your cabinet and
positioning your Kinetix 5100 drive system:
•
Additional clearance is required for cables and wires connected to the
drive modules.
•
Additional clearance is required if other devices are installed above and/
or below the drive module and have clearance requirements of their own.
•
Additional clearance left and right of the drive module is required when
mounted adjacent to noise sensitive equipment or clean wireways.
•
Recommended minimum cabinet depth:
- 300 mm (11.81 in.) for 2198-E1004, 2198-E1007, 2198-E1015,
2198-E1020, 2198-E2030, 2198-E2055, and 2198-E2075 servo drives
- 300 mm (11.81 in.) for 2198-E4004, 2198-E4007, 2198-E4015,
2198-E4020, 2198-E4030, 2198-E4055, and 2198-E4075 servo drives
- 350 mm (13.78 in.) for 2198-E2150 and 2198-E4150 servo drives

<!-- page 9 -->

To maintain adequate ventilation:
•
Install cooling fans above servo drives inside the cabinet to remove
excess heat.
•
Keep servo drives away from heat sources.
•
Make sure that the ambient temperature at 5.0 cm (1.96 in.) beneath the
drives does not exceed the operating temperature range.

> **Figure 10** — Minimum Clearance Requirements

## See Kinetix Servo Drives Specifications Technical Data, publication

KNX-TD003 for Kinetix 5100 drive dimensions.
ATTENTION: To avoid damage to drives due to overheating, cooling
fans must be installed when 2198-E1004-ERS drives are mounted in the
cabinet. Make sure that there is a minimum of 0.5 m/s (1.6 ft/s) air flow
at 10 mm (0.4 in.) above the top-center of the drive.

(0.8)
Cabinet

(2.0)
D
80 (3.2)
Airflow

(4.0)

(0.8)
Airflow

(0.8)

(0.8)
D
D
50 (2.0)
Air Flow
Air Flow
Dimension D is determined
in Table 11.
Measure
Airflow Here
Cooling Fans

(0.4)
Dimensions are in mm (in.)

> **Table 11** — Dimension D

Kinetix 5100 Drive
Cat. No.
Temperature, Ambient Versus Dimension D
2198-E1004-ERS
2198-E1007-ERS
2198-E1015-ERS
2198-E1020-ERS
2198-E2030-ERS
2198-E2055-ERS
2198-E2075-ERS
2198-E2150-ERS
2198-E4004-ERS
2198-E4007-ERS
2198-E4015-ERS
2198-E4020-ERS
2198-E4030-ERS
2198-E4055-ERS
2198-E4075-ERS
2198-E4150-ERS
IMPORTANT
Mount the drive in an upright position as shown. Do not mount the
drive on its side.

Ta °F
Ta °C
D mm (in.)
(0.0)
(0.2)
(0.4)
(0.6)
(0.8)

<!-- page 10 -->

## Electrical Noise Reduction

This section outlines best practices that minimize the possibility of noiserelated failures as they apply specifically to Kinetix 5100 system installations.
For more information on the concept of high-frequency (HF) bonding, the
ground plane principle, and electrical noise reduction, see the System Design
for Control of Electrical Noise Reference Manual, publication GMC-RM001.
HF Bond the Drives
Bonding is the practice where you connect the metal chassis, assemblies,
frames, shields, and enclosures to reduce the effects of electromagnetic
interference (EMI).
Unless specified, most paints are not conductive and act as insulators. To
achieve a good bond between drive and the subpanel, surfaces must be paintfree or plated. Bonding the metal surfaces creates a low-impedance return
path for high-frequency energy.
Improper bonding of the metal surfaces blocks the direct return path and
allows high-frequency energy to travel elsewhere in the cabinet. Excessive
high-frequency energy can affect the operation of other microprocessor
controlled equipment.
These illustrations show recommended practices for bonding the painted
panels, enclosures, and brackets.
IMPORTANT
To improve the bond between the drive and subpanel, construct your
subpanel out of zinc-plated (paint-free) steel.

<!-- page 11 -->

> **Figure 11** — Recommended Bonding Practices for Painted Panels

Stud-mounting the Subpanel
to the Enclosure Back Wall
Stud-mounting a Ground Bus
or Chassis to the Subpanel
Subpanel
Welded Stud
Scrape Paint
Flat Washer
If the mounting bracket is coated with
a non-conductive material (anodized or
painted), scrape the material around
the mounting hole.
Star Washer
Nut
Nut
Flat Washer
Mounting Bracket or
Ground Bus
Use a wire brush to remove paint from
threads to maximize ground connection.
Back Wall of
Enclosure
Welded Stud
Subpanel
Star Washer
Use plated panels or scrape paint on
front of panel.
Subpanel
Nut
Nut
Star Washer
Flat Washer
Star Washer
Star Washer
Scrape paint on both sides of
panel and use star washers.
Tapped Hole
Bolt
Flat Washer
Ground Bus or
Mounting Bracket
If the mounting bracket is coated with
a non-conductive material (anodized or
painted), scrape the material around
the mounting hole.
Bolt-mounting a Ground Bus or Chassis to the Back-panel

<!-- page 12 -->

## HF Bond Multiple Subpanels

Bonding multiple subpanels creates a common low-impedance exit path for
the high frequency energy inside the cabinet. Subpanels that are not bonded
together do not necessarily share a common low-impedance path. This
difference in impedance can affect networks and other devices that span
multiple panels.
•
Bond the top and bottom of each subpanel to the cabinet by using
25.4 mm (1.0 in.) by 6.35 mm (0.25 in.) wire braid. As a rule, the wider and
shorter the braid is, the better the bond.
•
Scrape the paint from around each fastener to maximize metal-to-metal
contact.

> **Figure 12** — Multiple Subpanels and Cabinet Recommendations

Wire Braid.
25.4 mm (1.0 in.) by
6.35 mm (0.25 in.)
Remove paint
from cabinet.
Ground bus that is bonded
to the subpanel.
Wire Braid.
25.4 mm (1.0 in.) by
6.35 mm (0.25 in.)

<!-- page 13 -->

## Establish Noise Zones

Observe these guidelines when routing cables used in the Kinetix 5100 system:
•
The clean zone (C) is right of the drive system and includes the digital
inputs wiring and Ethernet cable (gray wireway).
•
The dirty zone (D) is left and below the drive system (black wireways) and
includes the circuit breakers, 24V DC power supply, safety, and motor
cables.
•
The very dirty zone (VD) is limited to where the AC (EMC) line filter VAC
output jumpers over to the DC-bus power supply. Shielded cable is
required only if the very dirty cables enter a wireway.

> **Figure 13** — Noise Zones

(1)
When space to the right of the module does not permit 150 mm (6.0 in.) segregation, use a grounded steel shield instead. For
examples, refer to the System Design for Control of Electrical Noise Reference Manual, publication GMC-RM001.
D
C
D
D
VD
C
C
D
5100

NET
MOD
CHARGE
I/0
AUX
U
V
W
DC+
ISH
ESH
P1
P2
DC–
L1
L2
L3
L1C
L2C
L1 L2 L3
Clean Wireway
Kinetix 5100
Servo Drive
Motor Power Cable
 Ethernet
(shielded)
Cable
(1)
Dirty Wireway
AC Line Filter
(required for CE and UK)
Route encoder/analog/registration
shielded cables.
Circuit
Protection
24V DC
Power Supply
Very Dirty Filter/AC Input
Connections Segregated
(not in wireway)
Route motor cables
in shielded cable.
Hardwired Safety Cable
Motor Feedback Cable
(1)

<!-- page 14 -->

Cable Categories for Kinetix 5100 Drive Systems
Table 12 indicates the zoning requirements of cables that connect to the
Kinetix 5100 drive components.
Noise Reduction Guidelines for Drive Accessories
See this section when mounting an AC line filter or shunt resistor module for
guidelines that are designed to reduce system failures caused by excessive
electrical noise.
AC Line Filters
Observe these guidelines when mounting your AC line filter:
•
If you are using a Bulletin 2198 line filter, mount the filter on the same
panel as the Kinetix 5100 drive, and as close to the drive as possible.
•
Good HF bonding to the panel is critical. For painted panels, see the
examples on page 41.
•
Segregate input and output wiring as far as possible.

> **Table 12** — Kinetix 5100 Drive Systems

Wire/Cable
Connector Function
Zone
Method
Very Dirty
Dirty
Clean
Ferrite
Sleeve
Shielded
Cable
L1, L2, L3 (shielded cable)
Mains input power
–
X
–
–
X
L1, L2, L3 (unshielded cable)
X
–
–
–
–
L1C, L2C (unshielded cable)
Control input power
–
X
–
–
–
U, V, W (motor power)
U, V, W (motor power)
–
X
–
–
X
Motor feedback (MFD)
Motor feedback (MFD)
–
X
–
–
X
DC+, ISH, ESH
Shunt resistor
–
X
–
–
–
24V DC
24V DC for Safe Torque Off (STO) feature and control
power on 2198-E4xxx-ERS (400V) drives
–
X
–
–
–
Digital and analog I/O
Registration and analog inputs/outputs (I/O)
–
–
X
–
X
Dedicated digital inputs
(other than registration inputs and other I/O signals)
–
X
–
–
–
Ethernet
Ethernet RJ45 (Port 1 and Port 2)
–
–
X
–
X

**Extracted table (page 14, #1):**

| Connector Function | Zone |  |  |  |
| --- | --- | --- | --- | --- |
|  | Very Dirty | Dirty | Clean | Ferrite Sleeve |
| Mains input power | – | X | – | – |
|  | X | – | – | – |
| Control input power | – | X | – | – |
| U, V, W (motor power) | – | X | – | – |
| Motor feedback (MFD) | – | X | – | – |
| Shunt resistor | – | X | – | – |
| 24V DC for Safe Torque Off (STO) feature and control power on 2198-E4xxx-ERS (400V) drives | – | X | – | – |
| Registration and analog inputs/outputs (I/O) | – | – | X | – |
| Dedicated digital inputs (other than registration inputs and other I/O signals) | – | X | – | – |
| Ethernet RJ45 (Port 1 and Port 2) | – | – | X | – |

<!-- page 15 -->

## External Passive Shunt Modules

Observe these guidelines when mounting your Bulletin 2198 and 2097 external
passive shunt outside of the drive system enclosure:
•
Mount the shunt module so that wiring routes in the very dirty zone
inside the drive system enclosure.
•
Keep unshielded wiring as short as possible, not to exceed 3 m (9.8 ft).
Keep shunt wiring as flat to the cabinet as possible.

> **Figure 14** — External Passive Shunt Module Mounted On Top of the Drive System Enclosure

D
C
D
D
VD
C
C
D
5100

NET
MOD
CHARGE
I/0
AUX
U
V
W
DC+
ISH
ESH
P1
P2
DC–
L1
L2
L3
L1C
L2C
L1 L2 L3
VD
Clean wireway
150 mm (6.0 in.)
clearance (min) on all four
sides of the shunt module.
Metal Conduit
(where required by local code)
No sensitive
equipment within
150 mm (6.0 in.).
Enclosure
Dirty wireway
Kinetix 5100
Servo Drive
Motor Power Cable
Digital Inputs and
Ethernet (shielded)
Cables
AC Line Filter
(required for CE and UK)
Circuit
Protection
24V DC
Power Supply
Very Dirty Filter/AC Input
Connections Segregated
(not in wireway)
Route motor cables
in shielded cable.
Hardwired Safety Cable
Motor Feedback Cable
Shunt Power Wiring Methods:
Twisted-pair in conduit (1st choice).
Twisted-pair, two twists per foot (min) (2nd choice).
Route registration and communication
signals in shielded cables.
610 mm (24 in.)
clearance (min) above the
shunt module.

<!-- page 16 -->

Observe these guidelines when mounting your Bulletin 2198 and 2097 external
passive shunt inside the drive system enclosure:
•
Mount the shunt resistors anywhere in the dirty zone, but as close to the
Kinetix 5100 power supply as possible.
•
Route the shunt power wires with other very dirty wires.
•
Keep unshielded wiring as short as possible, not to exceed 457 mm
(18 in.). Keep shunt wiring as flat to the cabinet as possible.
•
Separate shunt power cables from other sensitive low-voltage signal
cables.

> **Figure 15** — External Shunt Resistor Mounted Inside the Drive System Enclosure

D
C
D
D
VD
C
C
D
5100

NET
MOD
CHARGE
I/0
AUX
U
V
W
DC+
ISH
ESH
P1
P2
DC–
L1
L2
L3
L1C
L2C
L1 L2 L3
VD
Dirty Wireway
No sensitive
equipment within
150 mm (6.0 in.)
Clean Wireway
Enclosure
Shunt Power Wiring Methods:
Twisted-pair in conduit (1st choice).
Twisted-pair, two twists per foot (min) (2nd choice).
76 mm (3.0 in.)
clearance (min) below, left,
and right of the shunt resistor.
150 mm (6.0 in.)
clearance (min) above the
shunt resistor.
Kinetix 5100
Servo Drive
Motor Power Cable
Digital Inputs and
Ethernet (shielded)
Cables
AC Line Filter
(required for CE and UK)
Circuit
Protection
24V DC
Power Supply
Very Dirty Filter/AC Input
Connections Segregated
(not in wireway)
Route motor cables
in shielded cable.
Hardwired Safety Cable
Motor Feedback Cable
Route registration and communication
signals in shielded cables.

<!-- page 17 -->

Mount Your Kinetix 5100
Drive
This procedure assumes that you have prepared your panel and understand
how to bond your system. For installation instructions regarding other
equipment and accessories, see the instructions that came with those
products.
Drill-hole Patterns
The following views provide mounting-hole dimensions for the Kinetix 5100
servo drives.

> **Figure 16** — Mounting-hole Dimensions

ATTENTION: This drive contains electrostatic discharge (ESD) sensitive
parts and assemblies. You are required to follow static control precautions
when you install, test, service, or repair this assembly. If you do not follow
ESD control procedures, components can be damaged. If you are not
familiar with static control procedures, see Allen-Bradley publication
8000-4.5.2, Guarding Against Electrostatic Damage or any other applicable
ESD Protection Handbook.
7.0
(0.28)

(6.67)
81.0
(3.19)
7.0
(0.12)
5.0
(0.20)
3.0
(0.12)
7.0
(0.12)
5.5
(0.22)
2.0
(0.08)
8.2
(0.32)

(6.79)
5.5
(0.22)
2.0
(0.08)

(6.40)
5.5
(0.22)

(10.12)

(4.09)
8.0
(0.31)
8.0
 (0.31 )
8.0
 (0.31 )
244.3
(9.62)
8.0
(0.31)
8.0
 (0.31 )
8.0
 (0.31 )
8.0
(0.31)
8.0
(0.31)
8.0
(0.31)
94.5
(3.72)
8.0
(0.31)
8.0
(0.31)
8.0
(0.31)
2198-E1004-ERS
Kinetix 5100 Drive
2198-E1007-ERS
and 2198-E1015-ERS
Kinetix 5100 Drives
2198-E2055-ERS
2198-E4055-ERS
Kinetix 5100 Drive
2198-E4020-ERS
2198-E4030-ERS
Kinetix 5100 Drive
2198-E1020-ERS
2198-E2030-ERS, 2198-E4004-ERS,
2198-E4007-ERS, and 2198-E4015-ERS
Kinetix 5100 Drives
Dimensions are in mm (in.)

<!-- page 18 -->

> **Figure 17** — Mounting-hole Dimensions (continued)

Mount the Drive
Follow these steps to mount your Kinetix 5100 drive.
1.
Lay out the position for the Kinetix 5100 drive and accessories in the
enclosure.
See Establish Noise Zones on page 43 for panel layout recommendations.
2. Drill holes in the panel for mounting your servo drive.
Refer to Drill-hole Patterns on page 47. For drive dimensions, see the
Kinetix Servo Drives Specifications Technical Data, publication
KNX-TD003.
3.
Loosely attach the servo drive to the panel.
The recommended mounting hardware is M4 (#8-32) steel machine
screws. Observe bonding techniques as described in HF Bond the Drives
on page 40.
4. Tighten all mounting fasteners.
5.
Apply 2.0 N•m (17.7 lb•in) maximum torque to each fastener.

(4.92)

(11.69)
7.0
(0.28)
8.0
(0.31)

(14.41)

(6.46)
11.0
(0.43)
10.0
(0.39 )
8.0
(0.31)
8.0
(0.31)
8.0
(0.31)
11.0
(0.43)
11.0
(0.43)
11.0
(0.43)
7.0
(0.28)
10.0
(0.39 )
2198-E2075-ERS
2198-E4075-ERS
Kinetix 5100 Drive
2198-E2150-ERS
2198-E4150-ERS
Kinetix 5100 Drive
Dimensions are in mm (in.)
IMPORTANT
To improve the bond between the Kinetix 5100 drive and subpanel,
construct your subpanel out of zinc-plated (paint-free) steel.

<!-- page 19 -->

This chapter illustrates connectors and indicators for the Kinetix® 5100 servo
drives. Also included in this chapter are control/feedback signal specifications
and overviews of the functional safety feature and the Kinetix 5100 drive
modes of operation.
Topic
Page
Kinetix 5100 Connector Data

Control Signal Specifications

Feedback Specifications

Safe Torque Off Feature

Operation Modes
