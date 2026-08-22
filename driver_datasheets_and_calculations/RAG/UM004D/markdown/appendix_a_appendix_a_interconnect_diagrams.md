# Appendix A - Interconnect Diagrams

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `18_AppA_Interconnect_Diagrams.pdf` (20 pages)_

<!-- page 1 -->

Appendix A
Interconnect Diagrams
This appendix provides wiring examples to assist you in wiring the
Kinetix® 5100 drive system.
Interconnect Diagram
Notes
These notes apply to the wiring examples on the pages that follow.
Topic
Page
Interconnect Diagram Notes

Power Wiring Examples

Passive Shunt Wiring Examples

Kinetix 5100 Drive/Rotary Motor Wiring Examples

Kinetix 5100 Servo Drive and Linear Actuator Wiring Examples

System Block Diagram

> **Table 145** — Interconnect Diagram Notes

Note
Information

For power wiring specifications, see Wiring Requirements on page 86.

DC–, and P1, P2 terminals are not used. Do not remove the jumper between P1 and P2.

Single-phase control power is not phase limited. You can choose any two inputs (L1, L2, or L3), unless a fan or other item is powered on the AC line bus.

Only the 2198-E1xxxx-ERS, 2198-E2030-ERS, 2198-E4004-ERS, 2198-E4007-ERS, and 2198-E4015-ERS drives have an internal resistor for shunt purposes. A jumper
connects the internal shunt resistor, see Passive Shunt Wiring Examples on page 463. Remove jumper only when wiring to an external shunt resistor.

For input fuse and circuit breaker sizes, see Circuit Breaker/Fuse Selection on page 33.

Place the AC (EMC) line filters as close to the drive as possible and do not route very dirty wires in the same wireway. If routing in same wireway is unavoidable,
use shielded cable with shields grounded to the drive chassis and filter case. For AC line filter specifications, see Kinetix Servo Drives Specifications Technical
Data, publication KNX-TD003.

2198-TBIO I/O terminal block is required to make connections. Configure a digital output (OUTPUT1…OUTPUT6) as Brake Control in KNX5100C software. For digital
output specifications, see Digital Outputs on page 60.

The M1 contactor is optional - customer supplied. It is recommended when independent control of control power and main AC power is desired. Contactor coil
(M1) requires integrated surge suppressors for AC coil operation. See Kinetix Servo Drives Specifications Technical Data, publication
KNX-TD003.

See Brake Control Circuit Example on page 68 to size the customer-supplied interposing relay for your application. See Figure 30 on page 62 for the diode or MOV
suppression device for your application.

Servo On input must be removed when main power is removed or a drive fault occurs. A delay of at least 6.0 seconds must be observed before attempting to
enable the drive after main power is restored. The Kinetix 5100 drives are limited to 1 main power cycle per minute.

Ground plate connection must be used to meet CE and UK requirements. The motor ground termination has a direct path to the Kinetix 5100 drive for control of
common mode and EMI interference. However, we recommend this grounding practice regardless of CE and UK requirements. No external connection to ground
is required.

PE ground point is a mounting screw (see Connect the Braided Ground Strap Example on page 85).

For motor cable specifications, see the Kinetix Motion Accessories Technical Data, publication KNX-TD004.

MPL-B15xx-V/E…MPL-B2xx-V/E, MPL-B3xx-S/M…MPL-B9xx-S/M, MPL-A5xx, MPM-Bxx, MPM-A165xx…MPM-A215xx, MPF-Bxx, MPF-A5xx, and MPS-Bxxx encoders use
the +9V DC supply.

MPL-A/B15xx-H…MPL-A/B45xx-H, MPL-A15xx-V/E…MPL-A2xx-V/E, MPL-A3xx-S/M…MPL-A5xx-S/M, MPM-A115xx…MPM-A130xx, MPF-A3xx…MPF-A45xx, and MPS-Axxx
encoders use the +5V DC supply.

<!-- page 2 -->

Appendix A Interconnect Diagrams
Power Wiring Examples
You must supply input power components. The following diagrams illustrate
single-phase and three-phase input power and connections for control power,
motor power, AC line filters, and passive shunts.
In this example, the 2198-E1004-ERS, 2198-E1007-ERS, 2198-E1015-ERS, and
2198-E1020-ERS drives are wired for 120V or 230V single-phase operation.

> **Figure 238** — Kinetix 5100 Drive (120V or 230V Single-phase Input Power)

Motor brake connector pins are labeled plus (+) and minus (-) or F and G, respectively. Motor power connector pins are labeled U, V, W, and GND or A, B, C, and D,
respectively.

Kinetix LDAT linear thrusters do not have a brake option, so only the 2090-CPWM7DF-xxAAxx or 2090-CPWM7DF-xxAFxx motor power cables apply.

MPAS-Bxxxxx-VxxSxA (ballscrew) linear stages use the 9V supply. MPAS-Bxxxxx-ALMx2C (direct-drive) linear stages use the 5V supply.

> **Table 145** — Interconnect Diagram Notes (Continued)

Note
Information
L3
U
V
W
L2
L1
DC+
ISH
ESH
L1C
L2C
P1
P2
DC–
Single-phase AC Input
120V or 230V rms AC, 50/60 Hz
Notes 1, 3, 5
Bonded Cabinet
Ground Bus *
PE (mounting screw)
Note 12
Mains
AC Input Power
Connector
Circuit
Protection*
Passive Shunt
Connector
Note 4
Motor Power
Connector
2198-E1004-ERS, 2198-E1007-ERS,
2198-E1015-ERS, and 2198-E1020-ERS
Kinetix 5100 Servo Drives
* Indicates Customer Supplied Component
See table on page 457 for note information.
AC Line Filter
Note 6
Three-phase
Motor Power
Connections
Note 13
Ground Plate
Note 11
Internal Shunt
Resistor
Control Input Power
Connector
Note 2
Circuit
Protection*

**Extracted table (page 2, #1):**

| U |  |
| --- | --- |
| V |  |
| W |  |

**Extracted table (page 2, #2):**

| Circuit Protection* |  |  | L2 |
| --- | --- | --- | --- |
|  |  |  | L3 |
|  |  |  | L1C |
|  |  |  | L2C |
|  |  |  | P1 |
|  |  |  | P2 |
|  |  |  | DC– |

**Extracted table (page 2, #3):**

| DC+ |  |  |
| --- | --- | --- |
| ISH |  |  |
| ESH |  |  |

<!-- page 3 -->

Appendix A Interconnect Diagrams
In this example, the 2198-E1004-ERS, 2198-E1007-ERS, 2198-E1015-ERS,
2198-E1020-ERS, and 2198-E2030-ERS drives are wired for 230V three-phase
operation.

> **Figure 239** — Kinetix 5100 Drive (230V three-phase input power)

U
V
W
L3
L2
L1
DC+
ISH
ESH
L1C
L2C
P1
P2
DC–
Three-phase AC Input
230V rms AC, 50/60 Hz
Notes 1, 5
Bonded Cabinet
Ground Bus *
Mains
AC Input Power
Connector
Circuit
Protection*
Motor Power
Connector
2198-E1004-ERS, 2198-E1007-ERS,
2198-E1015-ERS, 2198-E1020-ERS,
and 2198-E2030-ERS
Kinetix 5100 Servo Drives
* Indicates Customer Supplied Component
See table on page 457 for note information.
AC Line Filter
Note 6
Three-phase
Motor Power
Connections
Note 13
Ground Plate
Note 11
PE (mounting screw)
Note 12
Note 2
Circuit
Protection*
Passive Shunt
Connector
Note 4
Internal Shunt
Resistor
Control Input Power
Connector

**Extracted table (page 3, #1):**

| U |  |
| --- | --- |
| V |  |
| W |  |

**Extracted table (page 3, #2):**

| AC Line Filter |  |  |  |  |  | L1 |
| --- | --- | --- | --- | --- | --- | --- |
| Note 6 |  |  |  |  |  | L2 |
|  |  |  |  |  |  | L3 |
|  |  | Circuit Protection* | Circuit Protection* |  |  |  |
|  |  |  |  |  |  | L1C |
|  |  |  |  |  |  | L2C |
|  |  |  |  |  |  | P1 |
|  |  |  |  |  |  | P2 |
|  |  |  |  |  |  | DC– |

**Extracted table (page 3, #3):**

| DC | + |  |
| --- | --- | --- |
| ISH |  |  |
| ESH |  |  |

<!-- page 4 -->

Appendix A Interconnect Diagrams
In this example, the 2198-E2055-ERS, 2198-E2075-ERS, and 2198-E2150-ERS
drives are wired for 230V three-phase operation.

> **Figure 240** — Kinetix 5100 Drive (230V three-phase input power)

U
V
W
DC+
ESH
L1
L2
L3
P1
P2
DC–
L1C
L2C
Three-phase AC Input
230V rms AC, 50/60 Hz
Notes 1, 5
Bonded Cabinet
Ground Bus *
Mains
AC Input Power
Connector
Circuit
Protection*
Motor Power
Connector
2198-E2055-ERS, 2198-E2075-ERS,
and 2198-E2150-ERS
Kinetix 5100 Servo Drives
* Indicates Customer Supplied Component
See table on page 457 for note information.
AC Line Filter
Note 6
Three-phase
Motor Power
Connections
Note 13
Ground Plate
Note 11
PE (mounting screw)
Note 12
Note 2
External
Passive Shunt
Connector
Circuit
Protection*
Control Input Power
Connector
Note 4

**Extracted table (page 4, #1):**

| U |  |
| --- | --- |
| V |  |
| W |  |

**Extracted table (page 4, #2):**

|  |  |  |  |  | L1C |
| --- | --- | --- | --- | --- | --- |
|  |  |  |  |  | L2C |
| AC Line Filter |  |  |  |  |  |
| Note 6 |  |  |  |  |  |
|  |  |  |  |  | L2 |
|  |  |  |  |  | L1 |

**Extracted table (page 4, #3):**

|  |  | P1 |
| --- | --- | --- |
|  |  | P2 |
|  |  | DC– |

<!-- page 5 -->

Appendix A Interconnect Diagrams
In this example, the 2198-E4004-ERS,2198-E4007-ERS, and 2198-E4015-ERS,
drives are wired for 480V three-phase operation.

> **Figure 241** — Kinetix 5100 Drive (480V three-phase input power)

U
V
W
L3
L2
L1
P1
P2
DC–
24V+
24VDC+
ESH
ISH
Three-phase AC Input
480V rms AC, 50/60 Hz
Notes 1, 5
Bonded Cabinet
Ground Bus *
Mains
AC Input Power
Connector
Circuit
Protection*
Motor Power
Connector
2198-E4004-ERS, 2198-E4007-ERS,
and 2198-E4015-ERS
Kinetix 5100 Servo Drives
* Indicates Customer Supplied Component
See table on page 457 for note information.
AC Line Filter
Note 6
Three-phase
Motor Power
Connections
Note 13
Ground Plate
Note 11
PE (mounting screw)
Note 12
Note 2
Control Input Power
Connector
Chassis
Customer Supplied
+24V DC
Power Supply *
Internal Shunt
Resistor
Passive Shunt
Connector
Note 4

**Extracted table (page 5, #1):**

| U |  |
| --- | --- |
| V |  |
| W |  |

**Extracted table (page 5, #2):**

| AC Line Filter |  |  | L1 |
| --- | --- | --- | --- |
| Note 6 |  |  |  |
|  |  |  | L2 |
|  |  |  | L3 |

**Extracted table (page 5, #3):**

| DC+ |  |  |
| --- | --- | --- |
| ISH |  |  |
| ESH |  |  |

**Extracted table (page 5, #4):**

|  |  | P1 |
| --- | --- | --- |
|  |  | P2 |
|  |  | DC– |

<!-- page 6 -->

Appendix A Interconnect Diagrams
In this example, the 2198-E4020-ERS,2198-E4030-ERS, 2198-E4055-ERS, 2198E4075-ERS, and 2198-E4150-ERS drives are wired for 480V three-phase
operation.

> **Figure 242** — Kinetix 5100 Drive (480V three-phase input power)

U
V
W
DC+
ESH
L3
L2
L1
P1
P2
DC–
24V+
24VThree-phase AC Input
480V rms AC, 50/60 Hz
Notes 1, 5
Bonded Cabinet
Ground Bus *
Mains
AC Input Power
Connector
Circuit
Protection*
Motor Power
Connector
2198-E4020-ERS, 2198-E4030-ERS,
2198-E4055-ERS, 2198-E4075-ERS,
and 2198-E4150-ERS
Kinetix 5100 Servo Drives
* Indicates Customer Supplied Component
See table on page 457 for note information.
AC Line Filter
Note 6
Three-phase
Motor Power
Connections
Note 13
Ground Plate
Note 11
PE (mounting screw)
Note 12
Note 2
External
Passive Shunt
Connector
Control Input Power
Connector
Note 4
Chassis
Customer Supplied
+24V DC
Power Supply *

**Extracted table (page 6, #1):**

| AC Line Filter |  |  | L1 |
| --- | --- | --- | --- |
| Note 6 |  |  | L2 |
|  |  |  | L3 |

**Extracted table (page 6, #2):**

|  |  | P1 |
| --- | --- | --- |
|  |  | P2 |
|  |  | DC– |

<!-- page 7 -->

Appendix A Interconnect Diagrams
Passive Shunt Wiring
Examples
Bulletin 2198-Rxxx shunts and 2097-Rx shunt resistors are available for the
Kinetix 5100 drives. See the Kinetix Servo Drives Specifications Technical
Data, publication KNX-TD003, for shunt specifications. See Passive Shunt
Considerations on page 35 for specifications specific to your Kinetix 5100 drive
application.
See the Kinetix 5700 Passive Shunt Modules Installation Instructions,
publication 2198-IN011, for installation information.

> **Figure 243** — Passive Shunt Wiring Examples

IMPORTANT
When wiring an external shunt to the 2198-E1xxx-ERS,
2198-E2030-ERS, 2198-E4004-ERS, 2198-E4007-ERS, and
2198-E4015-ERS drives you must remove the jumper between
terminals DC+ and ISH. Set the ID157 (P1.052) ShuntResistorValue and
ID158 (P1.053) ShuntResistorPower accordingly to make the external
shunt resistor take effect.
DC+
ESH
ISH
2198-E1xxx-ERS, 2198-E2030-ERS,
2198-E4004-ERS, 2198-E4007-ERS,
and 2198-E4015-ERS
Kinetix 5100 Drives
2198-Rxxx
Passive Shunts
DC+
ESH
2198-E4020-ERS
and 2198-E4030-ERS
Kinetix 5100 Drives
2198-Rxxx
Passive Shunts
Passive Shunt
Connections
Passive Shunt
Connections
(jumper removed)
DC+
ESH
2198-E2055-ERS, 2198-E2075-ERS,
2198-E2150-ERS, 2198-E4055-ERS,
2198-E4075-ERS, and 2198-E4150-ERS
Kinetix 5100 Drives
2198-Rxxx
Passive Shunts
Passive Shunt
Connections
ATTENTION: Kinetix 5100 drives are rated for minimum external
regenerative resistance. Shunt resistor used must have a rating above this
value. See Table 8 on page 35 for these ratings. Using an external shunt
resistor below the rated value can result in damage to the drive shunt
circuitry.

**Extracted table (page 7, #1):**

| DC+ |  |
| --- | --- |
| ISH |  |
| ESH |  |

<!-- page 8 -->

Appendix A Interconnect Diagrams
Kinetix 5100 Drive/Rotary
Motor Wiring Examples
These wiring diagrams apply to Kinetix 5100 drives with compatible rotary
motors.
In this example, the Kinetix TLP servo-motor with rectangular connectors uses
a power/brake cable and the motor brake is wired to a digital output. Flyinglead feedback connections to the 2198-K51CK-D15M feedback connector kit are
made by using bulk cable and building your own cables. See Build Your Own
Kinetix TLP Motor Cables Installation Instructions, publication 2090-IN048,
for more information.

> **Figure 244** — Kinetix 5100 Drives with Kinetix TLP-A/B046…TLP-A/B100 Servo Motors

BRBR+
W
V
U

T+
T–
BAT+
BAT–
GND
+5VDC
C
D
R
A
B
S

GND
W
V
U
SHIELD
L
+
–

OUTPUT6–
OUTPUT6+
SHIELD
GREEN/YELLOW
BLACK
WHITE
RED
WHITE
WHITE/RED
RED
BLACK
BROWN
BLUE
Motor
Brake
Motor Power
Connector
2198-Exxxx-ERS
Kinetix 5100 Drives
Motor Feedback
(MFB) Connector
Three-phase
Motor Power
Motor
Feedback
Customer
Supplied
24V DC
2198-K51CK-D15M
Connector Kit
See connector kit
illustration (below)
for proper ground
technique.
Use the 2198-K51CK-D15M
feedback connector kit when
building your own cable.
Ground Technique for
Feedback Cable Shield
360° exposed shield that is secured
under clamp.
Clamp Screws (2)
Clamp
See table on page 457 for note information.
2090-CTFB-MxDD-CFAxx (standard) and
2090-CTFB-MxDD-CFFxx (continuous-flex)
feedback cables do not require the
2198-K51CK-D15M feedback connector kit.
Notes 13
Ground Plate
Note 11
Note 9
2090-CTPB-MxDF-xxAxx
(standard) or
2090-CTPB-MxDF-xxFxx
(continuous-flex)
Motor Power Cable
Note 13
Tie Wrap
Ground Plate
Note 11
Relay
I/O Connector with
2198-TBIO Expansion Block
Note 7
TLP-A046, TLP-A/B070,
TLP-A/B090, and TLP-A100
Servo Motors with
High-resolution Feedback

> **Table 146** — Motor Power and Brake Cable Pinouts

## Motor Power/Brake Cable

Cat. No.
Motor Power
Motor Brake
Signal
Wire Color
Pin
Signal
Wire Color
Pin
2090-CTPx-MADF-16
U
V
W
PE
RED
WHITE
BLACK
GREEN/YELLOW

BR+
BR–
BROWN
BLUE

2090-CTPx-MADF-18

See Table 146 for
motor power and
brake pinouts
BROWN
BLUE
Refer to Kinetix 5100 Feedback Connector Kit
Installation Instructions, publication 2198-IN019,
for connector kit specifications.

**Extracted table (page 8, #1):**

|  | BLACK |  |  |  |
| --- | --- | --- | --- | --- |
|  | WHITE |  |  |  |
|  | RED |  |  |  |
|  |  |  |  | Se mo b |

<!-- page 9 -->

Appendix A Interconnect Diagrams
In this example, the Kinetix TLP servo-motor with military-style connectors
uses a power/brake cable and the motor brake is wired to a digital output.
Flying-lead feedback connections to the 2198-K51CK-D15M feedback connector
kit are made by using bulk cable and building your own cables. See Build Your
Own Kinetix TLP Motor Cables Installation Instructions, publication
2090-IN048, for more information.

> **Figure 245** — Kinetix 5100 Drives with Kinetix TLP-A/B115…TLP-A/B200 Servo Motors

BRBR+
W
V
U

T+
T–
BAT+
BAT–
GND
+5VDC
C
D
R
A
B
S

GND
W
V
U
SHIELD
L
+
–

OUTPUT6–
OUTPUT6+
SHIELD
GREEN/YELLOW
BLACK
WHITE
RED
WHITE
WHITE/RED
RED
BLACK
BROWN
BLUE
Motor
Brake
Motor Power
Connector
2198-Exxxx-ERS
Kinetix 5100 Drives
Motor Feedback
(MFB) Connector
Three-phase
Motor Power
Motor
Feedback
Customer
Supplied
24V DC
2198-K51CK-D15M
Connector Kit
See connector kit
illustration (below)
for proper ground
technique.
Use the 2198-K51CK-D15M
feedback connector kit when
building your own cable.
Ground Technique for
Feedback Cable Shield
360° exposed shield that is secured
under clamp.
Clamp Screws (2)
Clamp
See table on page 457 for note information.
2090-CTFB-MxDD-CFAxx (standard) and
2090-CTFB-MxDD-CFFxx (continuous-flex)
feedback cables do not require the
2198-K51CK-D15M feedback connector kit.
Notes 13
Ground Plate
Note 11
Note 9
2090-CTPB-MxDF-xxAxx
(standard) or
2090-CTPB-MxDF-xxFxx
(continuous-flex)
Motor Power Cable
Note 13
Tie Wrap
Ground Plate
Note 11
Relay
I/O Connector with
2198-TBIO Expansion Block
Note 7
See Table 147 for
motor power and
brake pinouts
TLP-A/B115, TLP-A/B145-050,
TLP-A145-090, TLP-A/B145-100,
TLP-A/B145-150, TLP-B145-200,
TLP-A/B145-250,
TLP-A200-200, TLP-A/B200-300,
TLP-A200-350, TLP-A/B200-450
Servo Motors with
High-resolution Feedback
BROWN/RED
BLUE/BLACK

> **Table 147** — Motor and Brake Cable Pinouts

## Motor Power/Brake Cable

Cat. No.
Motor Power
Motor Brake
Signal
Wire Color
Pin
Signal
Wire Color
Pin
2090-CTPx-MCDF-12
U
V
W
PE
RED
WHITE
BLACK
GREEN/YELLOW
F
I
B
E
BR+
BR–
RED
BLACK
G
H
2090-CTPx-MCDF-16
BR+
BR–
BROWN
BLUE
2090-CTPx-MDDF-08
U
V
W
PE
RED
WHITE
BLACK
GREEN/YELLOW
D
E
F
G
BR+
BR–
RED
BLACK
A
B
2090-CTPx-MDDF-12
Refer to Kinetix 5100 Feedback Connector Kit
Installation Instructions, publication 2198-IN019,
for connector kit specifications.

**Extracted table (page 9, #1):**

|  | BLACK |  |  |  |
| --- | --- | --- | --- | --- |
|  | WHITE |  |  |  |
|  | RED |  |  |  |
|  |  |  |  | See mot br |

**Extracted table (page 9, #2):**

| Signal | Wire Color |
| --- | --- |
| U V W PE | RED WHITE BLACK GREEN/YELLOW |
| U V W PE | RED WHITE BLACK GREEN/YELLOW |

<!-- page 10 -->

Appendix A Interconnect Diagrams
In this example, the Kinetix TLP servo motors have a separate brake (military
style) connector and brake cable. The motor brake is wired to a digital output.
Flying-lead feedback connections to the 2198-K51CK-D15M feedback connector
kit are made by using bulk cable and building your own cables. See Build Your
Own Kinetix TLP Motor Cables Installation Instructions, publication
2090-IN048, for more information.

> **Figure 246** — Kinetix 5100 Drives with Kinetix TLP-A/B200-550, TLP-A/B200-750, and TLP-A/B235

## Servo Motors

W
V
U

T+
T–
BAT+
BAT–
GND
+5VDC
C
D
R
A
B
S

GND
W
V
U
SHIELD
L
+
–

OUTPUT6–
OUTPUT6+
BRBR+
SHIELD
GREEN/YELLOW
BLACK
WHITE
RED
WHITE
WHITE/RED
RED
BLACK
BROWN
BLUE
Motor
Brake
Motor Power
Connector
2198-Exxxx-ERS
Kinetix 5100 Drives
Motor Feedback
(MFB) Connector
Three-phase
Motor Power
Motor
Feedback
Customer
Supplied
24V DC
2198-K51CK-D15M
Connector Kit
See connector kit
illustration (below)
for proper ground
technique.
Use the 2198-K51CK-D15M
feedback connector kit when
building your own cable.
Ground Technique for
Feedback Cable Shield
360° exposed shield that is secured
under clamp.
Clamp Screws (2)
Clamp
See table on page 457 for note information.
2090-CTFB-MxDD-CFAxx (standard) and
2090-CTFB-MxDD-CFFxx (continuous-flex)
feedback cables do not require the
2198-K51CK-D15M feedback connector kit.
Notes 13
Ground Plate
Note 11
Note 9
2090-CTPB-MxDF-xxAxx
(standard) or
2090-CTPB-MxDF-xxFxx
(continuous-flex)
Motor Power Cable
Note 13
Tie Wrap
Ground Plate
Note 11
Relay
I/O Connector with
2198-TBIO Expansion Block
Note 7
BROWN
BLUE
Refer to Kinetix 5100 Feedback Connector Kit
Installation Instructions, publication 2198-IN019,
for connector kit specifications.
TLP-A/B200-550, TLP-A/B200-750
TLP-A/B235-11K, TLP-A235-15K,
TLP-B235-14K
Servo Motors with
High-resolution Feedback

**Extracted table (page 10, #1):**

|  | BLACK |  |  |  |
| --- | --- | --- | --- | --- |
|  | WHITE |  |  |  |
|  | RED |  |  |  |

<!-- page 11 -->

Appendix A Interconnect Diagrams
In this example, the Kinetix MP motor brake is wired to a digital output.
2090-CFBM7DD feedback cables with premolded drive-end connector are
available. Flying-lead feedback connections are made with 2090-CFBM7DF
cables to the 2198-K51CK-D15M feedback connector kit. See Cable Preparation
for Kinetix MP Servo Motors on page 97 for more information.

> **Figure 247** — Kinetix 5100 Drives with Kinetix MP Rotary Servo Motors

D/
C/W
B/V
A/U
BRBR+
G/-
F/+
W
V
U
SIN+
SINCOS+
COSDATA+
DATA-
+5VDC
ECOM

+9VDC
TS

GND
BR+
BRF
G
W
V
U
GND
C
B
A
AM+
AMBM+
BMIM+
IM-
+5VDC
ECOM

S1
–
TS

S2
S3
COM

COM

OUTPUTx–
OUTPUTx+

W
V
U
SHIELD
GREEN/YELLOW
BLUE
BLACK
BROWN
GREEN
WHT/GREEN
GRAY
WHT/GRAY
BLACK
WHT/BLACK
RED
WHT/RED
ORANGE
WHT/ORANGE
GREEN
WHT/GREEN
GRAY
WHT/GRAY
BLACK
WHT/BLACK
RED
WHT/RED
ORANGE
WHT/ORANGE
YELLOW
WHT/YELLOW
WHT/BLUE
Motor
Brake
Note 7
Motor Power
Connector
2198-Exxxx-ERS
Kinetix 5100 Drives
Motor Feedback
(MFB) Connector
Three-phase
Motor Power
Motor
Feedback
Thermostat
Three-phase
Motor Power
Motor
Feedback
Thermostat
MPL-A/B15xx…MPL-A/B45xx
Servo Motors with
Incremental Feedback
2198-K51CK-D15M
Connector Kit
2198-K51CK-D15M
Connector Kit
See connector kit
illustration (lower left)
for proper ground technique.
See connector kit
illustration (left)
for proper ground
technique.
2198-K51CK-D15M
Feedback Connector Kit
Ground Technique for
Feedback Cable Shield
360° exposed shield that is secured
under clamp.
Clamp Screws (2)
Clamp
See table on page 457 for note information.
2090-CFBM7DF-CEAAxx (standard) or
2090-CFBM7DF-CEAFxx (continuous-flex)
(flying lead) Feedback Cable
Notes 13, 14, 15
2090-XXNFMF-Sxx (standard) or
2090-CFBM7DF-CDAFxx (continuous-flex)
(flying lead) Feedback Cable
Note 13
Note 16
Note 9
2090-CPxM7DF-xxAAxx
(standard) or
2090-CPxM7DF-xxAFxx
(continuous-flex)
Motor Power Cable
Note 13
Motor
Brake
Tie Wrap
Ground Plate
Note 11
2090-CFBM7DD-CEAAxx(standard) or
2090-CFBM7DD-CEAFxx (continuous-flex)
(drive-end connector) feedback cables
are also available.
Customer
Supplied
24V DC
Relay
I/O Connector with
2198-TBIO Expansion Block
MPL-A15xx…MPL-A5xx,
MPL-B15xx…MPL-B6xx,
MPM-A/Bxxx, MPF-A/Bxxx, and
MPS-A/Bxxx Servo Motors with
High-resolution Feedback
BLACK
WHITE

**Extracted table (page 11, #1):**

|  |  | BLUE |  |  | C/ |
| --- | --- | --- | --- | --- | --- |
|  |  | BLACK |  |  | B/ |
|  |  | BROWN |  |  | A/ |
|  |  |  |  |  | G/ F/ |

<!-- page 12 -->

Appendix A Interconnect Diagrams
These compatible Kinetix TL and TLY rotary motors have separate connectors
and cables for power/brake and feedback connections. See Cable Preparation
for Kinetix TL and TLY Motor Power Cables on page 98 for more information.

> **Figure 248** — Kinetix 5100 with Kinetix TLY Rotary Motors

W
V
U
+5VDC
ECOM
GREEN
WHT/GREEN
GRAY
WHT/GRAY
BLACK
WHT/BLACK
RED
WHT/RED

GND
W
V
U
Blue
Black
Brown
White
Black
Shield
Green/Yellow

BR+
BR-

GRAY
WHT/GRAY

GND
ORANGE
WHT/ORANGE

BAT+
BAT-
+5VDC
ECOM
SHIELD
BAT+
BATDATA+
DATAGREEN
WHT/GREEN

AM+
AMBM+
BMIM+
IM-

WHT/BLUE
YELLOW
WHT/YELLOW
S1
S2
S3
SHIELD

W
V
U

BR–
BR+

OUTPUTx–
OUTPUTx+
Motor Brake
Motor Brake
Connector
Motor Power
Connector
TLY-Axxxx-H (230V)
Servo Motors with
Incremental Feedback
Motor Feedback
(MFB) Connector
Three-phase
Motor Power
Motor
Feedback
Refer to connector kit
illustration (lower left)
for proper grounding technique.
Refer to table on page 457 for note information.
2090-CFBM6DF-CBAAxx
(flying-lead) Feedback Cable
Note 13
2090-CPBM6DF-16AAxx
Motor Power and Brake Cable
or
2090-CPWM6DF-16AAxx
(cable for non-brake
applications)
Cable Shield
Clamp
Note 11
2198-K51CK-D15M
Feedback Connector Kit
TLY-Axxxx-B (230V)
Servo Motors with
High-resolution Feedback
Motor Brake
Motor
Feedback
Three-phase
Motor Power
2090-CFBM6DF-CBAAxx (flying-lead) or
2090-CFBM6DD-CCAAxx
(with drive-end connector)
Feedback Cable
2198-K51CK-D15M
Feedback Connector Kit
2090-CFBM6DD-CCAAxx
(drive-end connector) feedback cable
is also available.
2198-Exxxx-ERS
Kinetix 5100 Drives
Note 7
Note 9
Customer
Supplied
24V DC
Relay
I/O Connector with
2198-TBIO Expansion Block
2198-K51CK-D15M
Feedback Connector Kit
Ground Technique for
Feedback Cable Shield
360° exposed shield that is secured
under clamp.
Clamp Screws (2)
Clamp
Tie Wrap

**Extracted table (page 12, #1):**

|  |  |  | 5 |  |
| --- | --- | --- | --- | --- |
| Blue |  |  | 3 |  |
| Black |  |  | 2 |  |
| Brown |  |  | 1 |  |
|  |  |  | 9 7 |  |

**Extracted table (page 12, #2):**

| 9 10 |  |
| --- | --- |
| 11 12 13 |  |
| 14 22 23 15 |  |
| 17 |  |
| 19 24 |  |

<!-- page 13 -->

Appendix A Interconnect Diagrams
The 2090-DANFCT-Sxx feedback cable is equipped with a drive-end connector
that is not compatible with the 15-pin (MFB) feedback connector. To provide
battery backup to the encoder, you can remove the drive-end connector and
prepare the cable shield and conductors for wiring to the 2198-K51CK-D15M
feedback connector kit. See Cable Preparation for Kinetix TL and TLY Motor
Power Cables on page 98 for more information.

> **Figure 249** — Kinetix 5100 with Kinetix TL Rotary Motors

W
V
U
GND
W
V
U
Blue
Black
Brown
White
Black
Shield
Green/Yellow

BR–
BR+

OUTPUTx–
OUTPUTx+
GRAY
WHT/GRAY

ORANGE
WHT/ORANGE

BAT+
BAT-
+5VDC
ECOM
SHIELD
BAT+
BATSD+
SDBROWN
WHT/BROWN

## Motor Brake

Motor Power
Connector
TL-Axxxx-B (230V)
Servo Motors with
High-resolution Feedback
Motor Feedback
(MFB) Connector
Three-phase
Motor Power
Motor
Feedback
Refer to connector kit
illustration (lower left)
Refer to table on page 457 for
note information.
2090-DANFCT-Sxx (standard)
Flying-lead Feedback Cable
(with drive-end connector removed)
2090-DANPT-16Sxx
Motor Power Cable
Note 13
Cable Shield
Clamp
2198-K51CK-D15M
Feedback Connector Kit
2090-DANBT-18Sxx
Motor Brake Cable
Note 13
2198-Exxxx-ERS
Kinetix 5100 Drives
Note 11
2198-K51CK-D15M
Feedback Connector Kit
Ground Technique for
Feedback Cable Shield
360° exposed shield that is secured
under clamp.
Clamp Screws (2)
Clamp
Tie Wrap
Note 7
Note 9
Customer
Supplied
24V DC
Relay
I/O Connector with
2198-TBIO Expansion Block

**Extracted table (page 13, #1):**

|  |  |  | 5 |  |
| --- | --- | --- | --- | --- |
| Blue |  |  | 3 |  |
| Black |  |  | 2 |  |
| Brown |  |  | 1 |  |
|  |  |  | 2 1 |  |

<!-- page 14 -->

Appendix A Interconnect Diagrams
Kinetix 5100 Servo Drive
and Linear Actuator Wiring
Examples
These compatible linear actuators have separate connectors and cables for
power/brake and feedback connections.

> **Figure 250** — Kinetix 5100 Drives with Kinetix LDAT Linear Thrusters

U
V
W
D+
D-

Brown
Black
Blue
Green/Yellow
Shield

GND
AM+
AMBM+
BMIM+
IM-
+5VDC
ECOM
WHITE/BLUE
GREEN
WHITE/GREEN
GRAY
WHITE/GRAY
BLACK
WHITE/BLACK
RED
WHITE/RED

S1
–
TS
ORANGE
WHITE/ORANGE

S2
S3
YELLOW
WHITE/YELLOW

COM
U
V
W
C
B
A
D
Universal Feedback
(UFB) Connector
Motor Power
(MP) Connector
Motor Feedback
(MF) Connector
Three-phase
Motor Power
Motor
Feedback
Thermostat
Refer to table on page 457 for note
Cable Shield
Clamp
Note 11
Power Connector
Feedback Connector
SpeedTec DIN
Motor Connectors
2090-CFBM7DF-CEAAxx (standard) or
2090-CFBM7DF-CEAFxx (continuous-flex)
(flying-lead) Feedback Cable
Note 13
2090-CPWM7DF-xxAAxx
(standard) or
2090-CPWM7DF-xxAFxx
(continuous-flex)
Motor Power Cable
Notes 13, 17
Refer to feedback kit
illustrations (lower left)
for proper grounding
technique.
2198-Exxx -ERSx
Kinetix 5100 Drives
See the Kinetix 5100 Feedback Connector Kit Installation Instructions,
publication 2198-IN019, for connector kit specifications.
LDAT-Sxxxxxx-xBx
Linear Thrusters with
Incremental Feedback
2198-K51CK-D15M Feedback
Connector Kit
2198-K51CK-D15M
Feedback Connector Kit
Ground Technique for
Feedback Cable Shield
360° exposed shield that is secured
under clamp.
Clamp Screws (2)
Clamp
Tie Wrap

**Extracted table (page 14, #1):**

|  |  |  |  | A |  |
| --- | --- | --- | --- | --- | --- |
|  | Black |  |  | B |  |
|  | Blue |  |  | C |  |
|  | Green/Yellow |  |  | D |  |

<!-- page 15 -->

Appendix A Interconnect Diagrams

> **Figure 251** — Kinetix 5700 Drives with Kinetix MPAS Linear Stages

C
B
A
MBRK+
MBRKF
G
U
V
W
SIN+
SINCOS+
COSDATA+
DATA-
–
ECOM
GREEN
WHT/GREEN
GRAY
WHT/GRAY
BLACK
WHT/BLACK
RED
WHT/RED

+9VDC
TS
ORANGE
WHT/ORANGE

GND
D
U
V
W

D+
D-

MBRK +
MBRK -
Brown
Black
Blue
Green/Yellow
White
Black
Shield

COM
AM+
AMBM+
BMIM+
IM-
+5VDC
ECOM
WHITE/BLUE
GREEN
WHITE/GREEN
GRAY
WHITE/GRAY
BLACK
WHITE/BLACK
RED
WHITE/RED

S1
–
TS
ORANGE
WHITE/ORANGE

S2
S3
YELLOW
WHITE/YELLOW

COM
U
V
W
C
B
A
D
Motor Brake
Motor Brake
(BC) Connector
Motor Power
(MP) Connector
MPAS-Bxxxxx-VxxSxA
Ballscrew Linear Stages with
High Resolution Feedback
Motor Feedback
(MF) Connector
Three-phase
Motor Power
Motor
Feedback
Thermostat
Refer to table on page 457 for note
Cable Shield
Clamp
Note 11
Power Connector
Feedback Connector
SpeedTec DIN
Motor Connectors
2090-CFBM7DF-CEAAxx (standard) or
2090-CFBM7DF-CEAFxx (continuous-flex)
(flying-lead) Feedback Cable
Notes 13, 18
2090-CPxM7DF-xxAAxx
(standard) or
2090-CPxM7DF-xxAFxx
(continuous-flex)
Motor Power Cable
Note 13
Refer to feedback kit
illustrations (lower left)
for proper grounding
technique.
2198-Exxx -ERSx
Kinetix 5100 Drives
MPAS-Bxxxxx-ALMx2C
Direct Drive Linear Stages with
Incremental Feedback
2198-K51CK-D15M
Feedback
Connector Kit
Refer to feedback connector
kit illustration (left)
for proper grounding
technique.
2090-XXNFMF-Sxx (standard) or
2090-CFBM7DF-CDAFxx (continuous-flex)
(flying-lead) Feedback Cable
Notes 13, 18
2198-K51CK-D15M Feedback
Connector Kit
Three-phase
Motor Power
Motor
Feedback
Thermostat
Universal Feedback
(UFB) Connector
See the Kinetix 5100 Feedback Connector Kit Installation Instructions,
publication 2198-IN019, for connector kit specifications.
2198-K51CK-D15M
Feedback Connector Kit
Ground Technique for
Feedback Cable Shield
360° exposed shield that is secured
under clamp.
Clamp Screws (2)
Clamp
Tie Wrap

**Extracted table (page 15, #1):**

|  |  |  |  |  | A | Thermo |
| --- | --- | --- | --- | --- | --- | --- |
|  |  | Brown |  |  |  |  |
| 3 |  | Black |  |  | B |  |
| 2 |  | Blue |  |  | C |  |
| 1 |  | Green/Yellow |  |  | D |  |
|  |  |  |  |  | F G |  |

**Extracted table (page 15, #2):**

| 1 2 |  |
| --- | --- |
| 3 4 5 |  |
| 6 9 10 11 13 14 12 |  |

<!-- page 16 -->

Appendix A Interconnect Diagrams

> **Figure 252** — Kinetix 5700 Drives with Kinetix MPAR and MPAI Electric Cylinders

C
B
A
MBRK+
MBRKF
G
U
V
W
SIN+
SINCOS+
COSDATA+
DATA-
–
ECOM
GREEN
WHT/GREEN
GRAY
WHT/GRAY
BLACK
WHT/BLACK
RED
WHT/RED

+9VDC
TS
ORANGE
WHT/ORANGE

GND
D
U
V
W

D+
D-

MBRK +
MBRK -
Brown
Black
Blue
Green/Yellow
White
Black
Shield

COM
Motor Brake
Motor Brake
(BC) Connector
Motor Power
(MP) Connector
MPAR-Bxxxxx and MPAI-Bxxxxx
Electric Cylinders with
High Resolution Feedback
Motor Feedback
(MF) Connector
Three-phase
Motor Power
Motor
Feedback
Thermostat
Grounding Techniques for Feedback Cable Shield
Refer to table on page 457 for note
Cable Shield
Clamp
Note 11
Refer to Table 148 for
(flying-lead) motor feedback
cable.
Refer to Table 148 for
motor power cable.
Notes 13
Power Connector
Feedback Connector
SpeedTec DIN
Motor Connectors
Refer to feedback kit
illustrations (lower left)
for proper grounding
technique.
2198-Exxx -ERSx
Kinetix 5100 Servo Drives
2198-K57CK-D15M Feedback
Connector Kit
Universal Feedback
(UFB) Connector
2198-K51CK-D15M
Feedback Connector Kit
360° exposed shield that is secured
under clamp.
Clamp Screws (2)
Clamp
Tie Wrap
See the Kinetix 5100 Feedback Connector Kit
Installation Instructions,
publication 2198-IN019, for connector kit
specifications.

> **Table 148** — Kinetix MPAR and MPAI Electric Cylinders Power and Feedback Cables

Kinetix MPAR and MPAI Electric Cylinders
Cat. No.
Frame Power Cable
Cat. No.
Feedback Cable
Cat. No.
MPAR-B1xxx (series A and B)

2090-XXNPMF-16Sxx (standard) or
2090-CPxM4DF-16AFxx (continuous-flex)
2090-XXNFMF-Sxx (standard) or
2090-CFBM4DF-CDAFxx (continuous-flex)
MPAR-B2xxx (series A and B)

MPAR-B1xxx (series B and C)

2090-CPxM7DF-16AAxx (standard) or
2090-CPxM7DF-16AFxx (continuous-flex)
2090-CFBM7DF-CEAAxx (standard) or
2090-CFBM7DF-CEAFxx (continuous-flex)
MPAR-B2xxx (series B and C)

MPAR-B3xxx

MPAI-B2xxxx

MPAI-B3xxxx

MPAI-B4xxxx

MPAI-B5xxxx

**Extracted table (page 16, #1):**

|  |  |  |  |  | A |
| --- | --- | --- | --- | --- | --- |
|  |  | Brown |  |  |  |
| 3 |  | Black |  |  | B |
| 2 |  | Blue |  |  | C |
| 1 |  | Green/Yellow |  |  | D |
|  |  |  |  |  | F G |
|  |  | Refer to Table148 for motor power cable. Notes 13 |  |  |  |

**Extracted table (page 16, #2):**

| 1 2 |  |
| --- | --- |
| 3 4 5 |  |
| 6 9 10 11 13 14 12 |  |

**Extracted table (page 16, #3):**

| U |  |  |
| --- | --- | --- |
| V |  |  |
| W |  |  |

**Extracted table (page 16, #4):**

| Frame | Power Cable Cat. No. |
| --- | --- |
| 32 | 2090-XXNPMF-16Sxx (standard) or 2090-CPxM4DF-16AFxx (continuous-fle |
| 40 |  |
| 32 | 2090-CPxM7DF-16AAxx (standard) or 2090-CPxM7DF-16AFxx (continuous-fle |
| 40 |  |
| 63 |  |
| 64 |  |
| 83 |  |
| 110 |  |
| 144 |  |

<!-- page 17 -->

Appendix A Interconnect Diagrams

> **Figure 253** — Kinetix 5100 Drives with Kinetix LDC Linear Motors (cable connectors)

(AM-)
(AM+)
(BM-)
(BM+)
IMIM+
ECOM
+5VDC
WHT/BLUE
WHT/GREEN
GREEN
WHT/GRAY
GRAY
WHT/BLACK
BLACK
WHT/RED
RED
TS
S1
S2
S3
YELLOW
WHT/YELLOW

SINSIN+
COSCOS+

WHT/ORANGE
SIN+
SINCOS+
COSIM+
IMPOWER
COM
(AM+)
(AM-)
(BM+)
(BM-)
U
V
W

Brown
Black
Blue
Green/Yellow
Shield

C
B
A
U
V
W
GND
D
D+
D-

## Motor Power

(MP) Connector
Kinetix LDC
LDC-Cxxxxxx-xHTx1
Linear Motor Coil with
Sin/Cos or TTL External Encoder
and Cable Connectors
Three-phase
Motor Power
Motor
Feedback
Thermostat
Refer to table on page 457 for note
2090-XXNFMF-Sxx (standard) or
2090-CFBM7DF-CDAFxx
(continuous-flex) (flying-lead)
Feedback Cable
2090-CPWM7DF-xxAAxx
(standard)
or 2090-CPWM7DF-xxAFxx
(continuous-flex)
Motor Power Cable
Notes 13
External
Sin/Cos or (TTL)
Encoder
Motor Feedback
(MF) Connector
Grounding Techniques for Feedback Cable Shield
Power Connector
Feedback Connector
SpeedTec DIN
Motor Connectors
2198-K51CK-D15M Feedback
Connector Kit
Refer to feedback connector kit
illustrations for proper grounding
technique.
Cable Shield
Clamp
Note 11
2198-Exxx -ERSx
Kinetix 5100 Servo Drives
Universal Feedback
(UFB) Connector
2198-K51CK-D15M
Feedback Connector Kit
360° exposed shield that is secured
under clamp.
Clamp Screws (2)
Clamp
Tie Wrap
See the Kinetix 5100 Feedback Connector Kit
Installation Instructions,
publication 2198-IN019, for connector kit
specifications.

**Extracted table (page 17, #1):**

|  |  |  |  |  |  |  |  | A |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| U |  |  | 3 |  | Black |  |  | B |  |
| V |  |  |  |  |  |  |  |  |  |
|  |  |  | 2 |  | Blue |  |  | C |  |
| W |  |  |  |  |  |  |  |  |  |
|  |  |  | 1 |  | Green/Yellow |  |  | D |  |

<!-- page 18 -->

Appendix A Interconnect Diagrams

> **Figure 254** — Kinetix 5700 Drives with Kinetix LDC Linear Motors (flying-lead cables)

GND

RED
WHITE
BLACK
GREEN/YELLOW
BLACK
BLACK
RED
WHITE
BLUE
ORANGE
BLACK
TS+
TSPOWER
S1
S2
S3
COM
SIN+
SINCOS+
COSIM+
IMPOWER
COM
TS+
S1
S2
S3

(AM+)
(AM-)
(BM+)
(BM-)

SIN+
SINCOS+
COSIM+
IMPOWER
COM
(AM+)
(AM-)
(BM+)
(BM-)
U
V
W
D+
D-

U
V
W
U
V
W
LDC-Cxxxxxx-xHTx0
Linear Motor Coil with
Sin/Cos or TTL External Encoder
and Flying-lead Cables
Three-phase
Motor Power
Motor Feedback
(MF) Connector
Thermostat
Refer to table on page 457 for note
External
Sin/Cos or (TTL)
Encoder
Hall Effect
Module
Wire as shown using
cable type appropriate for
your application.
Motor Power
(MP) Connector
Cable Shield
Clamp
Note 11
2198-Exxx -ERSx
Kinetix 5100 Servo Drives
Feedback
Connector
Grounding Techniques for Feedback Cable Shield
2198-K51CK-D15M
Feedback Connector Kit
360° exposed shield that is secured
under clamp.
Clamp Screws (2)
Clamp
Tie Wrap
See the Kinetix 5100 Feedback Connector Kit
Installation Instructions,
publication 2198-IN019, for connector kit
specifications.

**Extracted table (page 18, #1):**

| 3 |  |  | V WHITE |
| --- | --- | --- | --- |
| 2 |  |  | W BLACK |
| 1 |  |  | GREEN/YELL |
|  |  | Wire as shown using cable type appropriate for your application. |  |

**Extracted table (page 18, #2):**

| U |  |  |
| --- | --- | --- |
| V |  |  |
| W |  |  |

**Extracted table (page 18, #3):**

|  | 11 |  |  |
| --- | --- | --- | --- |
| 1 | 2 |  | S1 |
| 1 | 3 |  | S2 |
|  | 8 |  | S3 |
|  | 1 |  | SIN+ (AM+) |
|  | 2 3 4 5 10 14 6 |  |  |

<!-- page 19 -->

Appendix A Interconnect Diagrams
System Block Diagram
This power block diagram applies to all 2198-Exxxx-ERS servo drives.

> **Figure 255** — Power Block Diagram

L1
L2
L3
W
V
U
ESH
DC–
DC+
DC–
DC+
ISH
Chassis
Shunt/DC-bus Connector
Shunt
Transistor
Internal
Shunt
Resistor
Inverter Section
Three-phase Motor Output
IMPORTANT
Only 2198-E1004-ERS, 2198-E1007-ERS, 2198-E1015-ERS, 2198-E1020-ERS,
2198-E2030-ERS, 2198-E4004-ERS, 2198-E4007-ERS, and 2198-E4015ERS drives have an internal shunt and ISH terminal.
IMPORTANT
Only 2198-E1004-ERS, 2198-E1007-ERS, 2198-E1015-ERS, and
2198-E1020-ERS drives support both single-phase and three-phase
operation.

<!-- page 20 -->

Appendix A Interconnect Diagrams
Notes:
