# Chapter 4: Connect the Drive

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `05_Ch04_Connect_the_Drive.pdf` (33 pages)_

<!-- page 1 -->

This chapter provides procedures for wiring your Kinetix® 5100 drive system
and making cable connections.
Basic Wiring Requirements
This section contains basic information on how to wire the Kinetix 5100 drive.

Topic
Page
Basic Wiring Requirements

Determine the Input Power Configuration

Ground the Drive System

Wiring Requirements

Wiring Guidelines

Wire the Input Power Connectors

Wire the I/O Connector

Wire the Safe Torque Off Connector

Wire the Motor Power Connector

Motor Brake Connections

Wire the Motor Feedback Connector

External Passive-shunt Resistor Connections

Ethernet Cable Connections

ATTENTION: Plan the installation of your system so that you can cut, drill,
tap, and weld with the system removed from the enclosure. Because the
system is of the open type construction, be careful to keep any metal debris
from falling into it. Metal debris or other foreign matter can become lodged
in the circuitry, which can result in damage to components.
SHOCK HAZARD: To avoid hazard of electrical shock, mount and wire the
Kinetix 5100 drive before you apply power. Once power is applied, connector
terminals can have voltage present even when not in use.
IMPORTANT
This section contains common PWM servo system wire
configurations, size, and practices that can be used in most
applications. National Electrical Code, local electrical codes, special
operating temperatures, duty cycles, or system configurations take
precedence over the values and methods provided.

<!-- page 2 -->

Build Your Own Cables
•
Connect the cable shield to the connector shells on both ends of the cable
with a complete 360° connection.
•
Use twisted-pair cable whenever possible. Twist differential signals with
each other and twist single-ended signals with the appropriate ground
return.
When using Kinetix TLP servo motors, see Build Your Own Kinetix TLP Motor
Cables Installation Instructions, publication 2090-IN048, to attach motor-side
power and feedback connector kits to bulk cable.
When using other Allen-Bradley® servo motors and actuators compatible with
2090-CxxM7DF motor cables, see 2090-Series Circular-DIN Connector Kits,
Flange Kits, and Crimp Tools Installation Instructions, publication
2090-IN042, to attach motor-side power and feedback connector kits to bulk
cable.
Also, see Kinetix 5100 Feedback Connector Kit Installation Instructions,
publication 2198-IN019, to terminate the flying lead feedback-cable
connections.
Route Power and Signal Wiring
Be aware that when you route wiring on a machine or system, radiated noise
from nearby relays, transformers, and other electronic drives can be induced
into motor or encoder feedback signals, input/output communication, or
other sensitive low voltage signals. Radiated noise can cause system and
communication faults.
See Electrical Noise Reduction on page 40 for examples of routing high and
low voltage wiring. See the System Design for Control of Electrical Noise
Reference Manual, publication GMC-RM001, for more information.
Determine the Input Power
Configuration
Before wiring input power to your Kinetix 5100 drive, you must know the type
of input power within your facility. The drive is designed to operate with only
grounded wye input power.
This section contains examples of typical single-phase and three-phase input
power that is wired to single-phase and three-phase Kinetix 5100 drives.
The grounded power configuration lets you ground your single-phase or threephase power to a neutral point. When you use one of the examples, be certain
to include the grounded neutral connection.
For Kinetix 5100 drive power specifications, see Kinetix Servo Drives
Specifications Technical Data, publication KNX-TD003. For Kinetix 5100 drive
input-wiring diagrams, see Power Wiring Examples on page 458.
IMPORTANT
Factory-made cables are designed to minimize EMI and are
recommended over hand-built cables to optimize system performance.

<!-- page 3 -->

Three-phase Power Wired to Three-phase Drives
These examples illustrate grounded three-phase systems that are wired to
three-phase Kinetix 5100 drives.

> **Figure 45** — Three-phase (200…230V) Grounded Power Configuration (wye secondary)

> **Figure 46** — Three-phase (380…480V) Grounded Power Configuration (wye secondary)

ATTENTION: The power system must be center-grounded wye secondary
configuration for 230V AC and 480V AC mains.
L3
L2
L1
L3
L2
L1
E
L3
L2
L1
L3
L2
L1
2198-Exxxx-ERS
Servo Drives
L1C
L2C
Transformer (wye) Secondary
Mains AC Input
Power Connector
Bonded Cabinet Ground Bus
Ground Grid or
Power Distribution Ground
AC Line
Filter
Circuit
Protection
Feeder and branch short-circuit
protection is not illustrated.
Control AC Input
Power Connector
Circuit
Protection
L3
L2
L1
L3
L2
L1
E
L3
L2
L1
L3
L2
L1
2198-Exxxx-ERS
Servo Drives
24V24V+
Transformer (wye) Secondary
Mains AC Input
Power Connector
Bonded Cabinet Ground Bus
Ground Grid or
Power Distribution Ground
AC Line
Filter
Circuit
Protection
Feeder and branch short-circuit
protection is not illustrated.
Control Input
Power Connector
24V DC
Power Supply
(customer supplied)

**Extracted table (page 3, #1):**

|  |  |  | Circuit Protection |  |  |  |
| --- | --- | --- | --- | --- | --- | --- |
|  |  |  |  | Circuit Protection |  |  |
|  | L3 L3 AC Line L2 Filter L2 |  |  |  |  | L3 |
|  |  |  |  |  |  | L2 |
|  | L1 L1 |  |  |  |  |  |
|  |  |  |  |  |  | L1 |
|  | E |  |  |  |  |  |

**Extracted table (page 3, #2):**

|  | AC Line L2 Filter L2 |  |  | L3 |
| --- | --- | --- | --- | --- |
|  |  |  |  | L2 |
|  | L1 L1 |  |  |  |
|  |  |  |  | L1 |
|  | E |  |  |  |

<!-- page 4 -->

Single-phase Input Power used with Single-phase Drives
These examples illustrate grounded single-phase power that is wired to singlephase Kinetix 5100 drives.

> **Figure 47** — Single-phase (200…230V) Grounded Power Configuration

> **Figure 48** — Single-phase (120V) Grounded Power Configuration

> **Figure 49** — Single-phase (230V) Grounded Power Configuration

L2
L1
L2
L1
L2
L1
L2
L1
E
L1C
L2C
Transformer Secondary
Bonded Cabinet Ground Bus
Ground Grid or
Power Distribution Ground
AC Line
Filter
200…230V AC
Output
Circuit
Protection
Circuit
Protection
Mains AC Input
Power Connector
Control AC Input
Power Connector
2198-E1004-ERS, 2198-E1007-ERS,
2198-E1015-ERS, and 2198-E1020-ERS
Drives with Single-phase Operation
Reducing the transformer output reduces motor speed. Feeder and branch short-circuit protection is not illustrated.
L2
L1 (Neutral)
L2
L1
L2
L1
L2
L1
E
L1C
L2C
Transformer Secondary
Bonded Cabinet Ground Bus
Ground Grid or
Power Distribution Ground
AC Line
Filter
120V AC
Output
Circuit
Protection
Mains AC Input
Power Connector
Control AC Input
Power Connector
Circuit
Protection
2198-E1004-ERS, 2198-E1007-ERS,
2198-E1015-ERS, and 2198-E1020-ERS
Drives with Single-phase Operation
L2
L1 (Neutral)
L2
L1
L2
L1
L2
L1
E
L1C
L2C
Transformer Secondary
Bonded Cabinet Ground Bus
Ground Grid or
Power Distribution Ground
AC Line
Filter
230V AC
Output
Circuit
Protection
Mains AC Input
Power Connector
Control AC Input
Power Connector
Circuit
Protection
2198-E1004-ERS, 2198-E1007-ERS,
2198-E1015-ERS, and 2198-E1020-ERS
Drives with Single-phase Operation

**Extracted table (page 4, #1):**

| Circuit Protection |  |  |
| --- | --- | --- |
|  |  | L2 |
|  |  | L1 |

<!-- page 5 -->

> **Figure 50** — Single-phase (230V) with Three-phase AC Line Filter Grounded Power Configuration

Three-phase Input Power used with Single-phase Drives
This example illustrates grounded three-phase power that is wired to singlephase Kinetix 5100 drives when phase-to-phase voltage exceeds drive
specifications.
A neutral must be connected when single-phase drives are attached to a threephase isolating transformer secondary. It is not necessary that all three-phases
be loaded with drives, but each drive must have its power return via the neutral
connection.

> **Figure 51** — Single-phase Amplifiers (one AC line filter per drive)

L2
L1 (Neutral)
L3
L2
L1
E
L3
L2
L1
L3
L2
L1
L1C
L2C
Transformer Secondary
Bonded Cabinet Ground Bus
Ground Grid or
Power Distribution Ground
AC Line
Filter
230V AC
Output
Circuit
Protection
Mains AC Input
Power Connector
Control AC Input
Power Connector
Circuit
Protection
2198-E1004-ERS, 2198-E1007-ERS,
2198-E1015-ERS, and 2198-E1020-ERS
Drives with Single-phase Operation
ATTENTION: Failure to connect the neutral can result in supply voltage
swings at the individual drives. Voltage swings occur when the neutral point
changes as a result of load variations experienced by the individual drives.
The supply voltage swing can cause undervoltage and overvoltage trips on
the drives, and the drive can be damaged if the overvoltage limit is
exceeded.
L3
L2
L1
L2
L1
L2
L1
L2
L1
L2
L1
L2
L1
E
E
E
L2
L1
L2
L1
L2
L1
L2
L1
Bonded Cabinet Ground Bus
Transformer (wye) Secondary
Ground Grid or Power Distribution Ground
AC Line
Filter
AC Line
Filter
AC Line
Filter
Kinetix 5100 Drives
(System A)
Single-phase AC Input
 Grounded
Neutral
 Grounded Neutral
Kinetix 5100 Drives
(System B)
Single-phase AC Input
Kinetix 5100 Drives
(System C)
Single-phase AC Input
Circuit
Protection
2198-E1004-ERS, 2198-E1007-ERS,
2198-E1015-ERS, and 2198-E1020-ERS
Drives with Single-phase Operation

**Extracted table (page 5, #1):**

| L2 L2 AC Line Filter L1 E L1 |  |  |  |  | L2 |  |
| --- | --- | --- | --- | --- | --- | --- |
|  |  |  |  |  | L1 |  |
| L2 L2 AC Line Filter |  |  |  |  | L2 |  |
|  |  |  |  |  | L1 |  |
| L1 E L1 |  |  |  |  |  |  |

<!-- page 6 -->

Feeder and branch short-circuit protection is not illustrated.
If a three-phase line filter is used to feed multiple single-phase drives (not
recommended), it is important that the filter has a neutral connection as
shown in Figure 51. This neutral connection applies if three-phase power is
wired directly into the filter and no isolation transformer is present.
Voiding of CE and UK Compliance
The three-phase with neutral in-line filter applications that are described in
Three-phase Input Power used with Single-phase Drives are not adequate for
EMC and aspect for CE and UK compliance. Therefore, EMC validity and CE
and UKCA marking by Rockwell Automation is voided when three-phase and
neutral in-line filters are used.
Using Isolation Transformers with Grounded Power Configurations
When using an isolation transformer, attach a chassis ground wire to the
neutral connection. This grounded neutral connection does the following:
•
Prevents the system voltage from floating and avoids high voltages that
can otherwise occur, for example due to static electricity
•
Provides a solid earth path for fault conditions
IMPORTANT
An AC line filter for each drive is the preferred configuration, and
required for CE and UK compliance.
ATTENTION: The three-phase isolation transformers with neutral in-line
filter applications described in this document have not been tested for EMC
compliance and products that are used in such installations are not
considered CE and UK compliant.
If this three-phase isolation transformer and neutral in-line filter application
is used, you are responsible for EMC validation and CE and UKCA marking of
the system.
If CE and UK compliance is a customer requirement, single-phase or threephase line filters, tested by Rockwell Automation, and specified for the
product must be used. See Kinetix Servo Drives Specifications Technical
Data, publication KNX-TD003 for catalog numbers.
IMPORTANT
Transformers (auto transformers are not supported) must have a WYE
secondary with grounded neutral. Phase to neutral voltage must not
exceed the input voltage rating of the drive.

<!-- page 7 -->

Ground the Drive System
All equipment and components of a machine or process system must have a
common earth ground point connected to chassis. A grounded system
provides a ground path for protection against electrical shock. Grounding your
drives and panels minimize the shock hazard to personnel and damage to
equipment caused by short circuits, transient overvoltages, and accidental
connection of energized conductors to the equipment chassis.
Ground Your Drive to the System Subpanel
Ground Kinetix 5100 drives to a bonded cabinet ground bus with a braided
ground strap with at least 10 mm2 (0.0155 in2) cross-sectional area. Keep the
braided ground strap as short as possible for optimum bonding.

> **Figure 52** — Connect the Braided Ground Strap Example

Refer to the System Design for Control of Electrical Noise Reference Manual,
publication GMC-RM001, for more information.
IMPORTANT
To improve the bond between the Kinetix 5100 drive and subpanel,
construct your subpanel out of zinc-plated (paint-free) steel.
ATTENTION: The National Electrical Code contains grounding requirements,
conventions, and definitions. Follow all applicable local codes and
regulations to safely ground your system.
For CE and UK grounding requirements, refer to Agency Compliance on
page 29.
Item
Description

Ground lug attached to mounting fastener, 2.0 N•m (17.7 lb•in), max

Braided ground strap (customer supplied)

Ground grid or power distribution ground

Bonded cabinet ground bus (customer supplied)

Kinetix 5100
Servo Drive

<!-- page 8 -->

> **Figure 53** — Chassis Ground Configuration (multiple Kinetix 5100 drives on one panel)

## Ground Multiple Subpanels

In this figure, the chassis ground is extended to multiple subpanels.

> **Figure 54** — Subpanels Connected to a Single Ground Point

## Wiring Requirements

Wire must be copper with 75 °C (167 °F) minimum rating. Phasing of main AC
power is arbitrary and earth ground connection is required.
See Appendix A beginning on page 457 for interconnect diagrams.

I/0
AUX
5100
NET
MOD
CHARGE

I/0
AUX
5100
NET
MOD
CHARGE

I/0
AUX
5100
NET
MOD
CHARGE

I/0
AUX
5100
NET
MOD
CHARGE
Bonded Cabinet
Ground Bus
Ground Grid or Power
Distribution Ground
Always follow NEC and
applicable local codes.
Chassis Ground
Chassis Ground
Chassis Ground
Chassis Ground
Make braided ground straps with at least
10 mm2 (0.0155 in2) cross-sectional area.
Keep straps as short as possible.
Always follow NEC and
applicable local codes.
Ground Grid or Power
Distribution Ground
Bonded Ground
 Bus
IMPORTANT
The National Electrical Code and local electrical codes take precedence
over the values and methods provided.

<!-- page 9 -->

> **Table 45** — Wiring Requirements

Kinetix 5100 (200V) Drives
Cat. No.
Kinetix 5100 (400V) Drives
Cat. No.
Description
Connects to
Terminals
Recommended
Wire Size
mm2 (AWG)
Strip Length
mm (in.)
Torque Value
N•m (lb•in)
2198-E1004-ERS
2198-E1007-ERS
2198-E1015-ERS
–
Mains V AC
input power
Control V AC
input power
Control 24V DC
input power
L1
L2
L3
L1C (1)
L2C
24V+ (2)
24V0.20…3.31
(24…12)
11 (0.4)
N/A (3)
2198-E1020-ERS
2198-E2030-ERS
2198-E4004-ERS
2198-E4007-ERS
2198-E4015-ERS
0.20…5.26
(24…10)
13 (0.5)
N/A (3)
–
2198-E4020-ERS
2198-E4030-ERS
2198-E4055-ERS
0.82…8.36
(18…8)
1.6 (5)
(13.90)
2198-E2055-ERS
–
1.8 (5)
(15.49)
2198-E2075-ERS
2198-E4075-ERS
11 (0.4)
1.6 (5)
(13.90)
2198-E2150-ERS
2198-E4150-ERS
2.08…21.1
(14…4)
13 (0.5)
3.1 (5)
(27.44)
2198-E1004-ERS
2198-E1007-ERS
2198-E1015-ERS
–
Motor power (4)
U
V
W
0.20…3.31
(24…12)
11 (0.4)
N/A (3)
2198-E1020-ERS
2198-E2030-ERS
2198-E4004-ERS
2198-E4007-ERS
2198-E4015-ERS
0.20…5.26
(24…10)
13 (0.5)
N/A (3)
–
2198-E4020-ERS
2198-E4030-ERS
2198-E4055-ERS
0.82…8.36
(18…8)
1.6 (5)
(13.90)
2198-E2055-ERS
2198-E4075-ERS
2198-E4150-ERS
2.08…21.1
(14…4)
3.1 (5)
(27.44)
2198-E2075-ERS
2198-E2150-ERS
2198-E1004-ERS
2198-E1007-ERS
2198-E1015-ERS
–
Shunt resistor
DC+
ISH
ESH
0.20…3.31
(24…12)
11 (0.4)
N/A (3)
2198-E1020-ERS
2198-E2030-ERS
2198-E4004-ERS
2198-E4007-ERS
2198-E4015-ERS
0.20…5.26
(24…10)
13 (0.5)
N/A (3)
–
2198-E4020-ERS
2198-E4030-ERS
2198-E4055-ERS
0.82…8.36
(18…8)
1.6 (5)
(13.90)
2198-E2055-ERS
–
1.8 (5)
(15.49)
2198-E2075-ERS
2198-E4075-ERS
11 (0.4)
1.6 (5)
(13.90)
2198-E2150-ERS
2198-E4150-ERS
2.08…21.1
(14…4)
13 (0.5)
3.1 (5)
(27.44)
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
Safe Torque Off
SB+
SBS1
S1C
S2
S2C
SS+
SS0.75 (18)
(stranded wire with
ferrule)
1.5 (16)
(solid wire)
8.0 (0.31)
N/A (3)
(1)
Applies to 2198-E1xxx-ERS and 2198-E2xxx-ERS (200V) drives.
(2)
Applies to 2198-E4xxx-ERS (400V) drives.
(3)
This connector uses spring tension to hold wires in place.
(4)
Motor power wire size depends on drive and motor combination. See Kinetix 5100 Drive Systems Design Guide, publication KNX-RM011, for specific drive and motor combination.
(5)
Attach using a terminal crimp lug.

**Extracted table (page 9, #1):**

| Kinetix 5100 (400V) Drives Cat. No. | Description | Connects to Terminals | Recommended Wire Size mm2 (AWG) | Strip Length mm (in.) |
| --- | --- | --- | --- | --- |
| – | Mains V AC input power Control V AC input power Control 24V DC input power | L1 L2 L3 L1C (1) L2C 24V+ (2) 24V- | 0.20…3.31 (24…12) | 11 (0.4) |
| 2198-E4004-ERS 2198-E4007-ERS 2198-E4015-ERS |  |  | 0.20…5.26 (24…10) | 13 (0.5) |
| 2198-E4020-ERS 2198-E4030-ERS 2198-E4055-ERS |  |  | 0.82…8.36 (18…8) |  |
| – |  |  |  |  |
| 2198-E4075-ERS |  |  |  | 11 (0.4) |
| 2198-E4150-ERS |  |  | 2.08…21.1 (14…4) | 13 (0.5) |
| – | Motor power (4) | U V W | 0.20…3.31 (24…12) | 11 (0.4) |
| 2198-E4004-ERS 2198-E4007-ERS 2198-E4015-ERS |  |  | 0.20…5.26 (24…10) | 13 (0.5) |
| 2198-E4020-ERS 2198-E4030-ERS 2198-E4055-ERS |  |  | 0.82…8.36 (18…8) |  |
| 2198-E4075-ERS 2198-E4150-ERS |  |  | 2.08…21.1 (14…4) |  |
| – | Shunt resistor | DC+ ISH ESH | 0.20…3.31 (24…12) | 11 (0.4) |
| 2198-E4004-ERS 2198-E4007-ERS 2198-E4015-ERS |  |  | 0.20…5.26 (24…10) | 13 (0.5) |
| 2198-E4020-ERS 2198-E4030-ERS 2198-E4055-ERS |  |  | 0.82…8.36 (18…8) |  |
| – |  |  |  |  |
| 2198-E4075-ERS |  |  |  | 11 (0.4) |
| 2198-E4150-ERS |  |  | 2.08…21.1 (14…4) | 13 (0.5) |
| 2198-E4004-ERS 2198-E4007-ERS 2198-E4015-ERS 2198-E4020-ERS 2198-E4030-ERS 2198-E4055-ERS 2198-E4075-ERS 2198-E4150-ERS | Safe Torque Off | SB+ SB- S1 S1C S2 S2C SS+ SS- | 0.75 (18) (stranded wire with ferrule) 1.5 (16) (solid wire) | 8.0 (0.31) |

<!-- page 10 -->

## Wiring Guidelines

Use these guidelines when wiring the connectors on your Kinetix 5100 servo
drives.
Follow these steps when wiring the connectors on your Kinetix 5100 drive
modules.
1.
Prepare the wires by removing insulation as shown in Table 45.
2. Route the cable/wires as described in Chapter 2.
3.
Insert wires into connector plugs or the I/O terminal block.
See connector pinout tables in Chapter 3 and Appendix A beginning on
page 457 for interconnect diagrams.
- Tighten the terminal screws on 2198-E2055-ERS, 2198-E2075-ERS, and
2198-E2150-ERS (200V) drives and 2198-E4055-ERS, 2198-E4075-ERS,
and 2198-E4150-ERS (400V) drives to the specified torque value.
- Connectors on 2198-E1004-ERS, 2198-E1007-ERS, 2198-E1015-ERS,
2198-E1020-ERS, and 2198-E2030-ERS (200V) drives and
2198-E4004-ERS, 2198-E4007-ERS, 2198-E4015-ERS (400V) drives use
spring tension to hold wires in place.
SHOCK HAZARD: DC-bus capacitors can retain hazardous voltages after input
power has been removed. DO NOT touch the P1, P2, DC-, DC+, ISH or ESH
terminals within the capacitor discharge time (listed in Table 46). Before
working on the drive, measure the DC-bus voltage to verify that it has reached
a safe level. When DC-bus voltage is above 50V DC, the Charge status
indicator is on. Failure to observe this precaution could result in severe bodily
injury or loss of life.

> **Table 46** — DC-bus Capacitor Discharge Time

Kinetix 5100 (200V) Drives
Cat. No.
Capacitor Discharge Time
Minutes
Kinetix 5100 (400V) Drives
Cat. No.
Capacitor Discharge Time
Minutes
2198-E1004-ERS

2198-E4004-ERS

2198-E1007-ERS

2198-E4007-ERS

2198-E1015-ERS

2198-E4015-ERS

2198-E1020-ERS

2198-E4020-ERS

2198-E2030-ERS

2198-E4030-ERS

2198-E2055-ERS

2198-E4055-ERS

2198-E2075-ERS

2198-E4075-ERS

2198-E2150-ERS

2198-E4150-ERS

IMPORTANT
For connector locations of the Kinetix 5100 drives, see Kinetix 5100
Connector Data on page 50.
When you remove insulation or tighten screws to secure the wiring, see
the table on page 86 for torque values.
IMPORTANT
To improve system performance, run wires and cables in the wireways
as established in Establish Noise Zones on page 43.
IMPORTANT
Use caution not to nick, cut, or otherwise damage strands as
you remove the insulation.

<!-- page 11 -->

- 2198-E1004-ERS, 2198-E1007-ERS, and 2198-E1015-ERS drives include
connector plugs and a wiring tool.
4. Gently pull on each wire to make sure that it is secured in the terminal;
reinsert and/or tighten any loose wires.
5.
When the wiring is complete, plug the connector into the drive.
Wire the Input Power
Connectors
Input power connections are made at the input power connector on the
bottom of the drive or the terminal blocks on the front of the drive. This section
provides examples and guidelines to assist you in making connections to the
input power connector or I/O terminal block.

> **Figure 55** — 2198-E1004-ERS, 2198-E1007-ERS and 2198-E1015-ERS Servo Drives

> **Figure 56** — 2198-E1020-ERS, 2198-E2030-ERS, 2198-E2055-ERS, 2198-E2075-ERS, and

2198-E2150-ERS Servo Drives
1. Position the wiring tool to grip the orange tab.
2. Press lever firmly to disengage spring tension.
3. Insert (or remove) the wire.
4. Release lever to engage spring tension.
5. Reposition on the next orange tab and repeat.
Connector Plug
Lever
Upper Jaw Gripping
Orange Tab
Wiring Tool
Wire
L1 L2 L3
L1C L2C P1 P2 DC–
Control Input Power
Connector
Kinetix 5100 Servo Drives
(2198-E1004-ERS drive is shown)
Top View
Mains Input Power
Connector
P1
P2
DC–
L1
L2
L3
L1C
L2C
P1
P2
DC–
L1
L2
L3
L1C
L2C
NET
5100
MOD
CHARGE

I/O
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
2198-E1020-ERS and 2198E2030-ERS
Kinetix 5100 Servo Drives
Front View
2198-E2055-ERS, 2198-E2075-ERS, and 2198-E2150-ERS
Kinetix 5100 Servo Drives Front View
Kinetix 5100 Servo Drives
(2198-E1020-ERS drive is shown)
Front View
Control Input Power
Connections
Mains Input Power
Connections
Reserved
(not used)

<!-- page 12 -->

> **Figure 57** — 2198-E4020-ERS, 2198-E4030-ERS, 2198-E4055-ERS, 2198-E4075-ERS, and

2198-E4150-ERS Servo Drives
NET
MOD
CHARGE
I/O
AUX
U
V
W
DC+
ESH
P1
P2
DC–
L1
L2
L3
24V+
24V–
MFB

P1
P2
DC–
L1
L2
L3
24V+
24V–
Kinetix 5100 Servo Drives
(2198-E4020-ERS drive is shown)
Front View
Control Input Power
Connections
Mains Input Power
Connections
Reserved
(not used)

> **Table 47** — Input Power Connector Specifications

Kinetix 5100 Drive
Cat. No.
Connects to
Terminals
Recommended Wire Size
mm2 (AWG)
Strip Length
mm (in.)
Torque Value
N•m (lb•in)
2198-E1004-ERS
2198-E1007-ERS
L1
L2
L3
L1C
L2C
P1
P2
DC0.20…3.31
(24…12)
11 (0.4)
N/A (1)
(1)
This connector uses spring tension to hold wires in place.
2198-E1015-ERS
2198-E1020-ERS
2198-E2030-ERS
0.20…5.26
(24…10)
13 (0.5)
N/A(1)
2198-E2055-ERS
0.82…8.36
(18…8)
1.8 (2)
(15.49)
(2)
Attach using a terminal crimp lug.
2198-E2075-ERS
11 (0.4)
1.6 (2)
(13.90)
2198-E2150-ERS
2.08…21.1
(14…4)
13 (0.5)
3.1 (2)
(27.44)
2198-E4004-ERS
2198-E4007-ERS
2198-E4015-ERS
L1
L2
L3
24V+
24VP1
P2
DC0.20…5.26
(24…10)
N/A(1)
2198-E4020-ERS
2198-E4030-ERS
2198-E4055-ERS
0.82…8.36
(18…8)
1.6 (2)
(13.90)
2198-E4075-ERS
11 (0.4)
2198-E4150-ERS
2.08…21.1
(14…4)
13 (0.5)
3.1 (2)
(27.44)

**Extracted table (page 12, #1):**

| Connects to Terminals | Recommended Wire Si mm2 (AWG) | ze Strip Length mm (in.) |
| --- | --- | --- |
| L1 L2 L3 L1C L2C P1 P2 DC- | 0.20…3.31 (24…12) 0.20…5.26 (24…10) 0.82…8.36 (18…8) 2.08…21.1 (14…4) | 11 (0.4) |
|  |  | 13 (0.5) 11 (0.4) |
|  |  | 13 (0.5) 11 (0.4) |
| L1 L2 L3 24V+ 24V- P1 P2 DC- | 0.20…5.26 (24…10) 0.82…8.36 (18…8) 2.08…21.1 (14…4) |  |
|  |  | 13 (0.5) |

<!-- page 13 -->

Wire the I/O Connector
Connect your digital/analog inputs/outputs to the I/O connector by using the
2198-TBIO terminal expansion block. For the I/O terminal block pinout, see I/O
Connector Pinout on page 55. See the Kinetix 5100 I/O Terminal Expansion
Block Installation Instructions, publication 2198-IN020 for more information.

> **Figure 58** — Kinetix 5100 Drive (I/O connector and terminal block)

Wire the Safe Torque Off
Connector
For the Safe Torque Off (STO) connector pinouts, feature descriptions, and
wiring information, see Chapter 13 beginning on page 415.
Wire the Motor Power
Connector
Motor power connections are made at the motor power connector on the
bottom of the drive or the terminal blocks on the front of the drive. This section
provides examples and guidelines to assist you in making the motor power
connections.

> **Figure 59** — 2198-E1004-ERS, 2198-E1007-ERS and 2198-E1015-ERS Servo Drives

I/O
AUX

## I/O Connector

2198-TBIO
Terminal Expansion Block

> **Table 48** — I/O Terminal Expansion Block Specifications

## I/O Terminal

Expansion Block
Cat. No.
Recommended Wire Size
mm2 (AWG)
Strip Length
mm (in.)
Torque Value
N•m (lb•in)
2198-TBIO
1.5…0.05
(16…30)
6…7
(0.24…0.27)

(1.77)
U V W
Kinetix 5100 Servo Drives
(2198-E1004-ERS drive is shown)
Bottom View
Motor Power Connector

<!-- page 14 -->

> **Figure 60** — 2198-E1020-ERS, 2198-E2030-ERS, 2198-E2055-ERS, 2198-E2075-ERS, and

2198-E2150-ERS and 2198-E4xxx-ERS Servo Drives
U
V
W
NET
5100
MOD
CHARGE

I/O
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
U
V
W
2198-E1020-ERS, 2198-E2030-ERS,
2198-E4004-ERS, 2198-E4007-ERS,
2198-E4015-ERS
Kinetix 5100 Servo Drives (front view)
2198-E2055-ERS, 2198-E2075-ERS, 2198-E2150-ERS,
2198-E4020-ERS, 2198-E4030-ERS, 2198-E4055-ERS,
2198-E4075-ERS, and 2198-E4150-ERS
Kinetix 5100 Servo Drives (front view)
Kinetix 5100 Servo Drives
(2198-E1020-ERS drive is shown)
Front View
Motor Power
Connections
Motor Power
Connections

> **Table 49** — Motor Power Connector Specifications

Kinetix 5100 Drive
Cat. No.
Connects to
Terminals
Recommended Wire
Size
mm2 (AWG)
Strip Length
mm (in.)
Torque Value
N•m (lb•in)
2198-E1004-ERS
2198-E1007-ERS
2198-E1015-ERS
U
V
W
0.20…3.31 (1)
(24…12)
(1)
Motor power cable depends on motor/drive combination.
11 (0.4)
N/A (2)
(2)
This connector uses spring tension to hold wires in place.
2198-E1020-ERS
2198-E2030-ERS
0.20…5.26
(24…10)
13 (0.5)
N/A (2)
2198-E2055-ERS
2.08…21.1
(14…4)
3.1 (3)
(27.44)
(3)
Attach using a terminal crimp lug.
2198-E2075-ERS
2198-E2150-ERS
2198-E4004-ERS
2198-E4007-ERS
2198-E4015-ERS
U
V
W
0.20…5.26
(24…10)
13 (0.5)
N/A (2)
2198-E4020-ERS
2198-E4030-ERS
2198-E4055-ERS
0.82…8.36
(18…8)
1.6 (3)
(13.90)
2198-E4075-ERS
2198-E4150-ERS
2.08…21.1
(14…4)
3.1 (3)
(27.44)

**Extracted table (page 14, #1):**

| Connects to Terminals | Recommended Wire Size mm2 (AWG) | Strip Length mm (in.) |
| --- | --- | --- |
| U V W | 0.20…3.31 (1) (24…12) | 11 (0.4) |
|  | 0.20…5.26 (24…10) | 13 (0.5) |
|  | 2.08…21.1 (14…4) |  |
| U V W | 0.20…5.26 (24…10) | 13 (0.5) |
|  | 0.82…8.36 (18…8) |  |
|  | 2.08…21.1 (14…4) |  |

<!-- page 15 -->

Servo Motor and Motor Cable Compatibility
Kinetix 5100 drives are compatible with the following Allen-Bradley rotary and
linear products:
•
Kinetix TLP servo motors
•
Kinetix MP motor family includes:
- Kinetix MPL, MPM, MPF, and MPS servo motors
•
Kinetix TL and TLY servo motors
•
Kinetix MP linear actuator family includes:
- Kinetix MPAS, MPMA, MPAR, and MPAI linear actuators
•
Kinetix LDAT linear thrusters
•
Kinetix LDC and Kinetix LDL linear motors
See Install the Kinetix 5100 Add-On Profile on page 190 for information on
downloading the AOP.
IMPORTANT
To configure these motors with your Kinetix 5100 servo drive (see
Table 51, Table 52, and Table 53), you must have drive firmware
revision 1.xxx or 2.xxx. When using I/O mode with ControlLogix® or
CompactLogix™ controllers, see Table 50 to determine if you need
the revision of the Kinetix 5100 Add-on Profile.
IMPORTANT
Class 1 and Class 3 EtherNet/IP Connections do not support induction
motors and linear motors.

> **Table 50** — AOP Installation Requirement

## Drive Firmware Revision

Logix Designer Application Version
Kinetix 5100 AOP Needed?
1.xxx or 2.xxx
30, 31, or 32
Yes
33.00 or later
No

<!-- page 16 -->

Motor Power and Brake Cables
Kinetix TLP motors use power cables that can combine the power and brake
wiring (<4.5 kW with brake). The power/brake cable attaches to the ground
plate on the drive and power/brake conductors attach to the motor power and
I/O connectors respectively. Motors (with brake) and a power rating ≥5.5 kW
have separate power and brake cables.

> **Table 51** — Kinetix TLP Motor Power/Brake Cable Compatibility

Servo Motor Cat. No.
Motor Power Cat. No. (1)
(with brake wires)
Motor Power Cat. No. (1)
(without brake wires)
Brake Cat. No. (1)
TLP-A046-xxx, TLP-A070-xxx,
TLP-A090-xxx, TLP-A100-xxx
2090-CTPB-MADF-18Axx (standard) or
2090-CTPB-MADF-18Fxx (continuous-flex)
2090-CTPW-MADF-18Axx (standard) or
2090-CTPW-MADF-18Fxx (continuous-flex)
Not applicable. Brake
conductors are included in the
power cable.
TLP-A115-100,
TLP-A145-050, TLP-A145-100
2090-CTPB-MCDF-16Axx (standard) or
2090-CTPB-MCDF-16Fxx (continuous-flex)
2090-CTPW-MCDF-16Axx (standard) or
2090-CTPW-MCDF-16Fxx (continuous-flex)
TLP-A115-200,
TLP-A145-090, TLP-A145-150, TLP-A145-250
2090-CTPB-MCDF-12Axx (standard) or
2090-CTPB-MCDF-12Fxx (continuous-flex)
2090-CTPW-MCDF-12Axx (standard) or
2090-CTPW-MCDF-12Fxx (continuous-flex)
TLP-A200-200, TLP-A200-300,
TLP-A200-350
2090-CTPB-MDDF-12Axx (standard) or
2090-CTPB-MDDF-12Fxx (continuous-flex)
2090-CTPW-MDDF-12Axx (standard) or
2090-CTPW-MDDF-12Fxx (continuous-flex)
TLP-A200-450
2090-CTPB-MDDF-08Axx (standard) or
2090-CTPB-MDDF-08Fxx (continuous-flex)
2090-CTPW-MDDF-08Axx (standard) or
2090-CTPW-MDDF-08Fxx (continuous-flex)
TLP-A200-550, TLP-A200-750 (2)
TLP-A235-11K
–
2090-CTPW-MEDF-06Axx (standard) or
2090-CTPW-MEDF-06Fxx (continuous-flex)
2090-CTPB-MBDF-20Axx
(standard) or
2090-CTPB-MBDF-20Fxx
(continuous-flex)
TLP-A235-15K (2)
–
2090-CTPW-MEDF-04Axx (standard) or
2090-CTPW-MEDF-04Fxx (continuous-flex)
TLP-B070-040
TLP-B090-075
2090-CTPB-MADF-18Axx (standard) or
2090-CTPB-MADF-18Fxx (continuous-flex)
2090-CTPW-MADF-18Axx (standard) or
2090-CTPW-MADF-18Fxx (continuous-flex)
Not applicable. Brake
conductors are included in the
power cable.
TLP-B115-100, TLP-B115-200
TLP-B145-050, TLP-B145-100
TLP-B145-150, TLP-B145-200
2090-CTPB-MCDF-16Axx (standard) or
2090-CTPB-MCDF-16Fxx (continuous-flex)
2090-CTPW-MCDF-16Axx (standard) or
2090-CTPW-MCDF-16Fxx (continuous-flex)
TLP-B200-300, TLP-B200-450
2090-CTPB-MDDF-12Axx (standard) or
2090-CTPB-MDDF-12Fxx (continuous-flex)
2090-CTPW-MDDF-12Axx (standard) or
2090-CTPW-MDDF-12Fxx (continuous-flex)
TLP-B145-250
2090-CTPB-MCDF-12Axx (standard) or
2090-CTPB-MCDF-12Fxx (continuous-flex)
–
TLP-B200-550, TLP-B200-750
2090-CTPB-MDDF-08Axx (standard) or
2090-CTPB-MDDF-08Fxx (continuous-flex)
2090-CTPW-MDDF-08Axx (standard) or
2090-CTPW-MDDF-08Fxx (continuous-flex)
TLP-B235-11K, TLP-B235-14K(2)
–
2090-CTPW-MEDF-06Axx (standard) or
2090-CTPW-MEDF-06Fxx (continuous-flex)
2090-CTBK-MBDF-20Axx
(standard) or
2090-CTBK-MBDF-20Fxx
(continuous-flex)
(1)
Refer to the Kinetix Motion Accessories Specifications Technical Data, publication KNX-TD004, for cable specifications.
(2)
Only these motors have separate brake connectors and brake cables. All other motors have brake wires included with the power connectors.

> **Table 52** — Kinetix MP, LDAT, LDC/LDL Motor Power Cable Compatibility

Motor/Actuator
Cat. No.
Motor Power Cat. No. (1)
(with brake wires)
Motor Power Cat. No. (1)
(without brake wires)
MPL-A/B15xxx-xx7xAA, MPL-A/B2xxx-xx7xAA,
MPL-A/B3xxx-xx7xAA, MPL-A/B4xxx-xx7xAA,
MPL-A/B45xxx-xx7xAA, MPL-A/B5xxx-xx7xAA,
MPL-B6xxx-xx7xAA
2090-CPBM7DF-xxAAxx (standard) or
2090-CPBM7DF-xxAFxx (continuous-flex)
2090-CPWM7DF-xxAAxx (standard) or
2090-CPWM7DF-xxAFxx (continuous-flex)
MPM-A/Bxxxx, MPF-A/Bxxxx, MPS-A/Bxxxx
MPAS-A/Bxxxx1-V05SxA,
MPAS-A/Bxxxx2-V20SxA
MPAI-A/Bxxxx, MPAR-A/B3xxx,
MPAR-A/B1xxx and MPAR-A/B2xxx (series B)
MPAS-Bxxxxx-ALMx2C
LDAT-Sxxxxxx-xBx
LDC-Cxxxxxx
LDL-xxxxxxx
N/A
(these devices do not include a brake option)
(1)
Refer to the Kinetix Motion Accessories Specifications Technical Data, publication KNX-TD004, for cable specifications.

**Extracted table (page 16, #1):**

| Motor Power Cat. No. (1) (with brake wires) | Motor Power Cat. No. (1) (without brake wires) |
| --- | --- |
| 2090-CTPB-MADF-18Axx (standard) or 2090-CTPB-MADF-18Fxx (continuous-flex) | 2090-CTPW-MADF-18Axx (standard) or 2090-CTPW-MADF-18Fxx (continuous-flex) |
| 2090-CTPB-MCDF-16Axx (standard) or 2090-CTPB-MCDF-16Fxx (continuous-flex) | 2090-CTPW-MCDF-16Axx (standard) or 2090-CTPW-MCDF-16Fxx (continuous-flex) |
| 2090-CTPB-MCDF-12Axx (standard) or 2090-CTPB-MCDF-12Fxx (continuous-flex) | 2090-CTPW-MCDF-12Axx (standard) or 2090-CTPW-MCDF-12Fxx (continuous-flex) |
| 2090-CTPB-MDDF-12Axx (standard) or 2090-CTPB-MDDF-12Fxx (continuous-flex) | 2090-CTPW-MDDF-12Axx (standard) or 2090-CTPW-MDDF-12Fxx (continuous-flex) |
| 2090-CTPB-MDDF-08Axx (standard) or 2090-CTPB-MDDF-08Fxx (continuous-flex) | 2090-CTPW-MDDF-08Axx (standard) or 2090-CTPW-MDDF-08Fxx (continuous-flex) |
| – | 2090-CTPW-MEDF-06Axx (standard) or 2090-CTPW-MEDF-06Fxx (continuous-flex) |
| – | 2090-CTPW-MEDF-04Axx (standard) or 2090-CTPW-MEDF-04Fxx (continuous-flex) |
| 2090-CTPB-MADF-18Axx (standard) or 2090-CTPB-MADF-18Fxx (continuous-flex) | 2090-CTPW-MADF-18Axx (standard) or 2090-CTPW-MADF-18Fxx (continuous-flex) |
| 2090-CTPB-MCDF-16Axx (standard) or 2090-CTPB-MCDF-16Fxx (continuous-flex) | 2090-CTPW-MCDF-16Axx (standard) or 2090-CTPW-MCDF-16Fxx (continuous-flex) |
| 2090-CTPB-MDDF-12Axx (standard) or 2090-CTPB-MDDF-12Fxx (continuous-flex) | 2090-CTPW-MDDF-12Axx (standard) or 2090-CTPW-MDDF-12Fxx (continuous-flex) |
| 2090-CTPB-MCDF-12Axx (standard) or 2090-CTPB-MCDF-12Fxx (continuous-flex) | – |
| 2090-CTPB-MDDF-08Axx (standard) or 2090-CTPB-MDDF-08Fxx (continuous-flex) | 2090-CTPW-MDDF-08Axx (standard) or 2090-CTPW-MDDF-08Fxx (continuous-flex) |
| – | 2090-CTPW-MEDF-06Axx (standard) or 2090-CTPW-MEDF-06Fxx (continuous-flex) |

<!-- page 17 -->

> **Table 53** — Kinetix TL and TLY Motor Power/Brake Cable Compatibility

Refer to Table 49 on page 92 for the motor power connector specifications.
Maximum Cable Length
Maximum cable length depends on the feedback type and input voltage that is
used in the application.
Motor/Actuator
Cat. No.
Motor Power Cat. No. (1)
(with brake wires)
Motor Power Cat. No. (1)
(without brake wires)
Brake Cat. No. (1)
TLY-Axxxx
2090-CPBM6DF-16AAxx (standard)
2090-CPWM6DF-16AAxx (standard)
Not applicable. Brake conductors
are included in the power cable.
TL-Axxxx
–
2090-DANPT-16Sxx
2090-DANBT-18Sxx
(1)
Refer to the Kinetix Motion Accessories Specifications Technical Data, publication KNX-TD004, for cable specifications.

> **Table 54** — Legacy Motor Power Cables for Kinetix MP Servo Motors

## Motor Cable

Description
Motor Power Cat. No.
Standard
Power/brake, threaded
2090-XXNPMF-xxSxx
Power-only, bayonet
2090-XXNPMP-xxSxx
Continuous-flex
Power/brake, threaded
2090-CPBM4DF-xxAFxx
Power-only, threaded
2090-CPWM4DF-xxAFxx
Power-only, bayonet
2090-XXTPMP-xxSxx

> **Table 55** — Maximum Cable Lengths (200V-class) Motors

Compatible Motor and Actuator
Cat. No.
Feedback Type
Cable Length, max
m (ft)
TLP-Axxx-xxx-D
Nikon (24-bit) absolute high-resolution, multi-turn and single-turn
50 (164)
MPL-A15xxx-V/Ex7xAA
MPL-A2xxx-V/Ex7xAA
Hiperface, absolute high-resolution, multi-turn and single-turn
MPL-A3xxx-S/Mx7xAA, MPL-A4xxx-S/Mx7xAA
MPL-A45xxx-S/Mx7xAA, MPL-A5xxx-S/Mx7xAA
MPM-Axxxx-S/M
MPF-Axxxx-S/M
MPS-Axxxx-S/M
MPAR-A3xxxx-M
MPAS-Axxxx1-V05SxA (ballscrew)
MPAS-Axxxx2-V20SxA (ballscrew)
MPAR-A1xxxx-V and MPAR-A/B2xxxx-V (series B)
MPAI-AxxxxxM3
Absolute high-resolution, multi-turn
50 (164)
MPL-A15xxx-Hx7xAA
MPL-A2xxx-Hx7xAA
Incremental
30 (98.4)
MPL-A3xxx-Hx7xAA
MPL-A4xxx-Hx7xAA
MPL-A45xxx-Hx7xAA
MPAS-Axxxx-ALMx2C (direct drive)
Incremental, magnetic linear
TLY-Axxxx-B
Tamagawa (17-bit) absolute high-resolution, multi-turn
TL-Axxxx-B
TLY-Axxxx-H
Incremental
LDAT-Sxxxxxx-xBx
Incremental, magnetic scale
10 (33.1)
LDC-Cxxxxxx-xH, LDL-xxxxxxx-xH
Sin/Cos or TTL encoder

**Extracted table (page 17, #1):**

| Motor Power Cat. No. (1) (with brake wires) | Motor Power Cat. No. (1) (without brake wires) |
| --- | --- |
| 2090-CPBM6DF-16AAxx (standard) | 2090-CPWM6DF-16AAxx (standard) |
| – | 2090-DANPT-16Sxx |

<!-- page 18 -->

Cable Preparation for Kinetix TLP Servo Motors
Because the 2090-CTxx-MxDx motor cables are designed specifically for
Kinetix TLP motors, there is no preparation required.
•
2090-CTPW-MxDF flying lead power cables are equipped with ring lugs
where required, so no cable preparation is needed.
•
2090-CTFB-MxDD feedback cables are equipped with premolded
connectors on the drive end, so no cable preparation is needed.
If you are building your own cables, see Build Your Own Kinetix TLP Motor
Cables Installation Instructions, publication 2090-IN048, to attach motor-side
power and feedback connector kits to bulk cable. Also, see Kinetix 5100
Feedback Connector Kit Installation Instructions, publication 2198-IN019, to
terminate the flying lead feedback cable connections.

> **Table 56** — Maximum Cable Lengths (400V-class) Motors

Compatible Motor and Actuator
Cat. No.
Feedback Type
Cable Length, max
m (ft)
≤ 400V AC Input
480V AC Input
TLP-B200-xxx-D
Nikon (24-bit) absolute high-resolution, multi-turn and single-turn
50 (164)
15 (49.2)
TLP-B070-xxx-D, TLP-B090-xxx-D, TLP-B115-xxx-D,
TLP-B145-xxx-D, TLP-B235-xxx-D
50 (164)
MPL-B15xxx-V/Ex7xAA
MPL-B2xxx-V/Ex7xAA
Hiperface, absolute high-resolution, multi-turn and single-turn
50 (164)
20 (65.6)
MPL-B3xxx-S/Mx7xAA, MPL-B4xxx-S/Mx7xAA
MPL-B45xxx-S/Mx7xAA, MPL-B5xxx-S/Mx7xAA
MPL-B6xxx-S/Mx7xAA, MPL-B8xxx-S/Mx7xAA
MPL-B9xxx-S/Mx7xAA
50 (164)
MPM-Bxxxx-S/M
MPF-Bxxxx-S/M
MPS-Bxxxx-S/M
MPAR-B3xxxx-M
MPAS-Bxxxx1-V05SxA (ballscrew)
MPAS-Bxxxx2-V20SxA (ballscrew)
MPAR-B1xxxx-V and MPAR-A/B2xxxx-V (series B)
MPAI-BxxxxxM3
Absolute high-resolution, multi-turn
50 (164)
20 (65.6)
MPL-B15xxx-Hx7xAA
MPL-B2xxx-Hx7xAA
Incremental
30 (98.4)
20 (65.6)
MPL-B3xxx-Hx7xAA
MPL-B4xxx-Hx7xAA
MPL-B45xxx-Hx7xAA
30 (98.4)
MPAS-Bxxxx-ALMx2C (direct drive)
Incremental, magnetic linear
20 (65.6)
LDAT-Sxxxxxx-xBx
Incremental, magnetic scale
10 (33.1)
LDC-Cxxxxxx-xH, LDL-xxxxxxx-xH
Sin/Cos or TTL encoder

> **Table 57** — Maximum Cable Lengths, Third-Party Motors

## Motor Insulation Rating (1)

(1)
Motor Corona Inception Voltage (CIV) or Partial Discharge Inception Voltage (PDIV) ratings for
motor.
Cable Length, max (2) m (ft)
(2)
Cable lengths are estimated assuming nominal DC-bus voltage at nominal AC line input voltage.
≤ 400V AC Input
480V AC Input
1000V
10 (33.1)
1200V
50 (164)/30 (98.4) (3)
15 (49.2)
1488V
50 (164)/30 (98.4) (3)
(3)
Limited to 30 m (98.4 ft) for incremental encoders.
1600V

**Extracted table (page 18, #1):**

| Feedback Type |  |
| --- | --- |
|  | ≤ 400V AC Input |
| Nikon (24-bit) absolute high-resolution, multi-turn and single-turn | 50 (164) |
| Hiperface, absolute high-resolution, multi-turn and single-turn | 50 (164) |
| Absolute high-resolution, multi-turn | 50 (164) |
| Incremental | 30 (98.4) |
| Incremental, magnetic linear |  |
| Incremental, magnetic scale |  |
| Sin/Cos or TTL encoder |  |

<!-- page 19 -->

Cable Preparation for Kinetix MP Servo Motors
Motor power and brake conductors on 2090-CPBM7DF (series A) cables have
the following dimensions from the factory. If your cable is reused from an
existing application, the actual conductor lengths could be slightly different.

> **Figure 61** — 2090-CPBM7DF (series A) Power/brake Cable Dimensions

Motor power and brake conductors on 2090-CPBM7DF (series B) 12 and 10
AWG standard, non-flex cables provide (drive end) shield braid and conductor
preparation designed for compatibility with multiple Kinetix servo-drive
families, including Kinetix 5100 drives.

> **Figure 62** — 2090-CPBM7DF (series B, 10 or 12 AWG) Power/brake Cable Dimensions

See Build Your Own Kinetix TLP Motor Cables Installation Instructions,
publication 2090-IN048, to attach the proper ring lugs to PE, U, V, and W
conductors to 2090-CPBM7DF power cables when used with 2198-E2055-ERS,
2198-E2075-ERS, 2198-E2150-ERS, and 2198-4xxx-ERS servo drives.
635 (25)
102 (4.0)
150 (5.9)
Dimensions are in mm (in.)
Power Conductors
Brake
Conductors
Kinetix MP
Servo Motors
Brake Shield
Edge of
Cable Jacket
Overall Cable Shield
305 (12.0)
71 (2.80)
12.7 (0.50)
5.0 (0.20)
5.0 (0.20)
234 (9.20)
15.0 (0.59)
8.0 (0.31)
Dimensions are in mm (in.)
Power Conductors
Brake
Conductors
Kinetix MP
Servo Motors
Heat Shrink
Overall Cable Shield

<!-- page 20 -->

Cable Preparation for Kinetix TL and TLY Motor Power Cables
2090-CPBM6DF motor power cables, used with Kinetix TLY motors, require no
preparation. However, 2090-DANPT-16Sxx power cables, used with Kinetix TL
motors have a short pigtail cable that connects to the motor, but is not
shielded. The preferred method for grounding the Kinetix TL power cable on
the motor side is to expose a section of the cable shield and clamp it directly to
the machine frame.
The motor power cable also has a 150 mm (6.0 in.) shield termination wire with
a ring lug that connects to the closest earth ground. The termination wire can
be extended to the full length of the motor pigtail if necessary, but it is best to
connect the supplied wire directly to ground without lengthening.

> **Figure 63** — 2090-DANPT-16Sxx Cable Preparation

(1)
Remove paint from machine frame to provide HF-bond between machine frame, motor case, shield clamp, and ground stud.
Apply the Cable Shield Clamp
This procedure assumes that you have completed wiring your motor power
connector and are ready to apply the cable shield ground plate. Factorysupplied motor power cables for Kinetix TLP and Kinetix MP motors are
shielded. The braided cable shield must terminate at the drive during
installation.
•
2090-CTPx-MxDx motor power cables are designed specifically for
Kinetix TLP motors and require no preparation.
•
For 2090-CPxM7DF cables, used with Kinetix MP motors, ring lugs need
to be crimped to the PE, U, V, and W conductors when attaching to
2198-E2055-ERS, 2198-E2075-ERS, 2198-E2150-ERS, and
2198-E4020-ERS, 2198-E4030-ERS, 2198-E4055-ERS, 2198-E4075-ERS,
2198-E4150-ERS servo drives.
See Build Your Own Kinetix TLP Motor Cables Installation Instructions,
publication 2090-IN048, to attach the proper ring lugs to PE, U, V, and W
conductors to 2090-CPBM7DF power cables.
IMPORTANT
For Kinetix TL motors, connect the 150 mm (6.0 in.) termination wire to
the closest earth ground.
(1)
(1)
Pigtail Cable
Kinetix TL
Motor
Connectors
Motor Power Cable
Machine Frame
150 mm (6.0 in.) Termination
Cable Braid Clamped (1)
to Machine Frame
SHOCK HAZARD: To avoid hazard of electrical shock, make sure shielded
power cables are grounded according to recommendations.

<!-- page 21 -->

Follow these steps to apply the cable shield clamp.
1.
Route the conductors with service loops to provide stress relief to the
motor power and brake conductors.
2. Apply tie wraps through the ground plate slots and around the exposed
cable shield.
Make sure the cable clamp tightens around the cable shield and provides
a good bond between the cable shield and the drive chassis.
3.
Attach the motor-power ground wire to one of the PE ground screws.
Tighten the PE ground screw to a maximum torque value of 2.0 N•m
(17.7 lb•in).
Figure 64 displays examples of how the motor cable conductors and shield can
be routed and attached for each of the servo drives.

> **Figure 64** — Kinetix 5100 Drive Ground Plate Examples

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
Tie Wraps
PE Ground Screws
Grounding Plate
IMPORTANT
If the power/brake cable shield has a loose fit between the ground
plate and tie wraps, the cable shield ground is ineffective. When the
tie wraps are pulled tight, the result must be a high-frequency bond
between the cable shield and the drive chassis.
2198-E1004-ERS
2198-E1007-ERS
2198-E1015-ERS
2198-E1020-ERS
2198-E2030-ERS
2198-E2055-ERS
2198-E4055-ERS
2198-E2075-ERS
2198-E4075-ERS
2198-E2150-ERS
2198-E4150-ERS
2198-E4004-ERS
2198-E4007-ERS
2198-E4015-ERS
2198-E4020-ERS
2198-E4030-ERS

<!-- page 22 -->

## Motor Brake Connections

For wiring the brake circuit, see Motor Brake Circuit on page 68 for wiring and
configuration details. See Kinetix 5100 Drive/Rotary Motor Wiring Examples
on page 464 for motor brake wiring examples.
•
Kinetix TLP servo motors use 2090-CTPB-MxDx power/brake cables.
Servo drives with a power rating ≥ 5.5 kW require a separate brake cable.
See Kinetix TLP Motor Power/Brake Cable Compatibility on page 94 for
the drive catalog numbers that require a separate brake cable.
•
Kinetix MP motors and actuators use 2090-CPBM7DF power/brake
cables.
•
Kinetix TLY motors use 2090-CPBM6DF power/brake cables.
•
Kinetix TL motors use 2090-DANBT-18Sxx brake cables.
Wire the Motor Feedback
Connector
2090-CTFB-MxDD motor feedback cables are designed specifically for
Kinetix TLP motors. The drive-end connector plugs directly into the 15-pin
(MFB) feedback connector. The 2198-K51CK-DM15 feedback connector kit is
also available when making your own cables. See Wire the 2198-K51CK-D15M
Feedback Connector Kit on page 107 for pinouts and wiring.
2090-CFBM6DD and 2090-CFBM7DD motor feedback cables also provide a
drive-end connector that plugs directly into the 15-pin (MFB) feedback
connector. The 2198-K51CK-DM15 feedback connector kit can also be used with
2090-CFBM6DF and 2090-CFBM7DF flying-lead cables.
When using the 2198-K51CK-DM15 feedback connector kit, 2090-CFBM7DF
flying-lead cables require preparation to make sure the ground plate attaches
properly and conductors route smoothly to the connector terminals. All of the
current and legacy feedback cables listed below are compatible with the
2198-K51CK-D15M connector kit.

> **Table 58** — Kinetix TLP Motor Feedback Cable Compatibility

Servo Motor Cat. No.
 Feedback Cable Cat. No. (1)
(1)
Refer to the Kinetix Motion Accessories Specifications Technical Data, publication KNX-TD004, for cable specifications.
TLP-A046-xxx, TLP-A/B070-xxx, TLP-A/B090-xxx, TLP-A100-xxx
2090-CTFB-MADD-CFAxx (standard) or
2090-CTFB-MADD-CFFxx (continuous-flex)
TLP-A/B115-xxx, TLP-A/B145-xxx, TLP-A/B200-xxx, TLP-A/B235-xxx
2090-CTFB-MFDD-CFAxx (standard) or
2090-CTFB-MFDD-CFFxx (continuous-flex)

<!-- page 23 -->

Cable Preparation for Kinetix TLP Feedback Cables
For Kinetix TLP motors, 2090-CTFB-MxDD feedback cables (with battery box)
are available for applications with and without the need for battery backup.
•
For multi-turn feedback, use 2090-CTFB-MxDD cables with drive-end
connector plugs and wire the battery box (included with each
Kinetix TLP feedback cable) and install a customer-supplied battery.
Battery specifications are shown in Table 35 on page 73.
See Feedback Battery Box Installation Instructions, publication
2198-IN022, for more information.
•
For single-turn feedback, use 2090-CTFB-MxDD cables with drive-end
connector plugs, however, the battery box option is not required.
•
If you build your own cables, see Build Your Own Kinetix TLP Motor
Cables Installation Instructions, publication 2090-IN048, and make
flying-lead feedback connections to the 2198-K51CK-D15M connector kit.

> **Figure 65** — Battery Box Wired With Battery

> **Table 59** — Compatible Motors and Actuators

Single-turn or Multi-turn Absolute Encoders
Incremental Encoders
Servo Motor Cat. No.
Feedback Cable (1) Cat. No.
Servo Motor Cat. No.
Feedback Cable (1) Cat. No.
MPL-A/B15xxx…MPL-A2xxx-V/E
MPL-A/B3xxx…MPL-A5xxx-M/S
2090-CFBM7DF-CEAAxx
2090-CFBM7DD-CEAAxx
2090-CFBM7DF-CERAxx (standard) or
2090-CFBM7DF-CEAFxx
2090-CFBM7DD-CEAFxx
2090-CFBM7DF-CDAFxx (continuous-flex)
MPL-A/B15xxx…MPL-A/B2xxx-H
MPL-A/B3xxx…MPL-A/B45xxx-H
2090-XXNFMF-Sxx
(standard)
2090-CFBM7DF-CDAFxx
(continuous-flex)
MPM-A/Bxxxxx-M/S
–
MPF-A/Bxxxx-M/S
–
MPS-A/Bxxxx-M/S
–
MPAR-A/B1xxxx-V and MPAR-A/B2xxxx-V
(series B)
MPAR-A/B3xxxx-M
MPAI-A/BxxxxxM3
MPAS-A/Bxxxx-ALMx2C (direct drive)
MPAS-A/Bxxxx1-V05SxA (ballscrew)
MPAS-A/Bxxxx2-V20SxA (ballscrew)
LDAT-Sxxxxxx-xBx
LDC-Cxxxxxx-xH
LDL-xxxxxxx-xH
TLY-Axxxx-B
2090-CFBM6DF-CBAAxx (standard)
2090-CFBM6DD-CCAAxx (standard)
TLY-Axxxx-H
2090-CFBM6DF-CBAAxx (standard)
2090-CFBM6DD-CCAAxx (standard)
TL-Axxxx-B
2090-DANFCT-Sxx (standard)
–
–
(1)
Refer to the Kinetix Motion Accessories Specifications Technical Data, publication KNX-TD004, for cable specifications.

> **Table 60** — Legacy Motor Feedback Cables

## Motor Cable

Description
Cable Cat. No.
Standard
Encoder feedback, threaded
2090-XXNFMF-Sxx
2090-UXNFBMF-Sxx
Encoder feedback, bayonet
2090-UXNFBMP-Sxx
2090-XXNFMP-Sxx
Continuous-flex
Encoder feedback, bayonet
2090-XXTFMP-Sxx
Encoder feedback, threaded
2090-CFBM4DF-CDAFxx
Battery
Battery backup wires
inserted in terminals.
Battery Box
(cover removed)
Feedback
Cable

**Extracted table (page 23, #1):**

| Feedback Cable (1) Cat. No. | Servo Motor Cat. No. |
| --- | --- |
| 2090-CFBM7DF-CEAAxx 2090-CFBM7DD-CEAAxx 2090-CFBM7DF-CERAxx (standard) or 2090-CFBM7DF-CEAFxx 2090-CFBM7DD-CEAFxx 2090-CFBM7DF-CDAFxx (continuous-flex) | MPL-A/B15xxx…MPL-A/B2xxx-H MPL-A/B3xxx…MPL-A/B45xxx-H |
|  | – |
|  | – |
|  | – |
|  | MPAS-A/Bxxxx-ALMx2C (direct drive) |
|  | LDAT-Sxxxxxx-xBx |
|  | LDC-Cxxxxxx-xH LDL-xxxxxxx-xH |
| 2090-CFBM6DF-CBAAxx (standard) 2090-CFBM6DD-CCAAxx (standard) | TLY-Axxxx-H |
| 2090-DANFCT-Sxx (standard) | – |

<!-- page 24 -->

Cable Preparation for 2090-CFBM7Dx Feedback Cables
2090-CFBM7DD motor feedback cables, used with Kinetix MP motors and
actuators (with Hiperface encoders), also provide a drive-end connector that
plugs directly into the 15-pin Kinetix 5100 (MFB) feedback connector. Use the
2198-K51CK-D15M feedback connector kit with 2090-CFBM7DF flying-lead
cables.
Cable Preparation for Kinetix TL and TLY Feedback Cables
For Kinetix TLY motors, 2090-CFBM6Dx feedback cables are available for
applications with and without the need for battery backup.
•
For multi-turn encoders (TLY-Axxxx-B motors), use the
2198-K51CK-D15M feedback connector kit (with customer-supplied
battery) and 2090-CFBM6DF flying-lead cables.
•
For incremental encoders (TLY-Axxxx-H motors), use 2090-CFBM6DD
cables with drive-end connector and plug directly into the 15-pin (MFB)
feedback connector.
- If the 2090-CFBM6DF flying-lead cable is preferred, the
2198-K51CK-D15M connector kit (without battery) can also be used.
For Kinetix TL-Axxxx-B motors, use 2090-DANFCT-Sxx feedback cables. You
must remove the drive-end connector and prepare the leads for terminating at
the 2198-K51CK-D15M connector kit. Install a (customer-supplied) battery for
multi-turn encoder position backup.

> **Figure 66** — Feedback Connection for Kinetix TL Motors

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
2090-DANFCT-Sxx
Motor Feedback Cable
(drive-end connector removed)
2090-DANPT-16Sxx
Motor Power Cable
Kinetix TL (TL-Axxxx-B) Servo Motors
(high-resolution encoder)
Kinetix 5100 Servo Drive
(front view)
2198-K51CK-D15M
Feedback Connector Kit
(battery backup is optional)

<!-- page 25 -->

## Motor Feedback Cable Preparation

When using the 2198-K51CK-D15M feedback connector kit, you must prepare
the Kinetix 2090 flying-lead conductors with the proper strip length. The cable
shield requires a high-frequency bond with the ground pad.
Follow these steps to prepare feedback cables.
1.
Remove 127 mm (5.0 in.) of cable jacket and 120 mm (4.7 in.) of cable
shield.
2. Determine the length for each wire and trim as necessary.
3.
Remove 5.0 mm (0.2 in.) of insulation from the end of each wire.
IMPORTANT
This length of wire is needed for the longest wires terminated at
each 8-pin connector. However, most wires are trimmed shorter,
depending on the terminal they are assigned to.
7.0 (0.3)
5.0 (0.2)
127 (5.0)
120 (4.7)
Cable Jacket
Cable Shield
Dimensions are in mm (in.)

<!-- page 26 -->

Apply the Connector Kit Shield Clamp
Follow these steps to apply the connector kit shield clamp.
1.
Position the 12 mm (0.5 in.) of exposed cable shield over the ground pad
to achieve a high-frequency bond.
2. Place the shield clamp over the cable shield and install the clamp screws.
Apply 0.34 N•m (3.0 lb•in) torque to each screw.
3.
Route and insert each wire to its assigned terminal, apply 0.20 N•m
(1.8 lb•in) maximum torque to each screw.
Refer to the connector pinout as shown in Figure 67 on page 107.
4. Attach the tie-wrap (customer-supplied) through the slots and around
the cable shield for stress relief and to create a high-frequency bond
between shield and ground pad.
IMPORTANT
Cable preparation and positioning that provides a highfrequency bond between the shield braid and clamp is
required to optimize system performance.
Also, make sure that the cable is positioned where the cover
clamps onto the jacket for added stress relief.
Shield Clamp
Cable positioned where the cover
clamps onto the cable jacket.

<!-- page 27 -->

## Kinetix 2090 Feedback Cable Pinouts

The following tables provide motor connector pinouts and wire colors to the
2198-K51CK-D15M connector kit.

> **Table 62** — 2090-CTFB-MxDD-CFxxx Feedback Cables

> **Table 61** — 2090-CFBM7DF-CEAxxx Feedback Cables

Absolute,
High-resolution
Feedback
MPL-B15xxx and MPL-B2xxx-V/Ex4/7xAA
MPL-B3xxx…MPL-B9xxx-M/Sx7xAA
MPL-A5xxx-M/Sx7xAA
MPL-A15xxx and MPL-A2xxx-V/Ex4/7xAA
MPL-A3xxx-M/Sx7xAA
MPL-A4xxx-M/Sx7xAA
MPL-A45xxx-M/Sx7xAA
MPM-A115xxx…MPM-A130xxx-M/S
MPF/MPS-A3xx-M/S
MPF/MPS-A4xx-M/S
MPF/MPS-A45xx-M/S
Wire Color
2198-K51CK-D15M
Connector Kit Pin
Motor/Actuator Pin
MPM-A165xxx…MPM-A215xxx-M/S
MPM-Bxxxxx-M/S
MPF-Bxxx-M/S
MPF-A5xxx-M/S
MPS-Bxxx-M/S
MPAS-Bxxxxx-VxxSxA
MPAR-Bxxxx, MPAI-Bxxxx
MPAS-Axxxxx-VxxSxA
MPAR-Axxxx, MPAI-Axxxx

SIN+
SIN+
Black

SINSINWhite/Black

COS+
COS+
Red

COSCOSWhite/Red

DATA+
DATA+
Green

DATADATAWhite/Green

Reserved
EPWR_5V
Gray

ECOM
ECOM
White/Gray
6 (1)

EPWR_9V
Reserved
Orange

TS+
TS+
White/Orange

(1)
The ECOM and TS- connections are tied together and connect to the cable shield.
Motor Pin
TLP-Axxx-xxx and TLP-Bxxx-xxx
24-bit Absolute, Multi-turn/Single-turn
High-resolution
Wire Color
2198-K51CK-D15M
Connector Kit Pin
A
T+
White

B
T–
White/Red

C
BAT+
Red
Pin +
D
BAT–
Black
Pin –
L
Drain
–
R
ECOM
Blue

S
EPWR_5V
Brown

**Extracted table (page 27, #1):**

| MPL-B15xxx and MPL-B2xxx-V/Ex4/7xAA MPL-B3xxx…MPL-B9xxx-M/Sx7xAA MPL-A5xxx-M/Sx7xAA | MPL-A15xxx and MPL-A2xxx-V/Ex4/7xAA MPL-A3xxx-M/Sx7xAA MPL-A4xxx-M/Sx7xAA MPL-A45xxx-M/Sx7xAA MPM-A115xxx…MPM-A130xxx-M/S MPF/MPS-A3xx-M/S MPF/MPS-A4xx-M/S MPF/MPS-A45xx-M/S | Wire Color |
| --- | --- | --- |
| MPM-A165xxx…MPM-A215xxx-M/S MPM-Bxxxxx-M/S MPF-Bxxx-M/S MPF-A5xxx-M/S MPS-Bxxx-M/S |  |  |
| MPAS-Bxxxxx-VxxSxA MPAR-Bxxxx, MPAI-Bxxxx | MPAS-Axxxxx-VxxSxA MPAR-Axxxx, MPAI-Axxxx |  |
| SIN+ | SIN+ | Black |
| SIN- | SIN- | White/Black |
| COS+ | COS+ | Red |
| COS- | COS- | White/Red |
| DATA+ | DATA+ | Green |
| DATA- | DATA- | White/Green |
| Reserved | EPWR_5V | Gray |
| ECOM | ECOM | White/Gray |
| EPWR_9V | Reserved | Orange |
| TS+ | TS+ | White/Orange |

**Extracted table (page 27, #2):**

| TLP-Axxx-xxx and TLP-Bxxx-xxx 24-bit Absolute, Multi-turn/Single-turn High-resolution | Wire Color |
| --- | --- |
| T+ | White |
| T– | White/Red |
| BAT+ | Red |
| BAT– | Black |
| Drain | – |
| ECOM | Blue |
| EPWR_5V | Brown |

<!-- page 28 -->

> **Table 63** — 2090-XXNFMF-Sxx or 2090-CFBM7DF-CDAxxx Feedback Cables

> **Table 64** — 2090-CFBM6DF-CBAAxx Feedback Cables

> **Table 65** — 2090-CFBM6DF-CBAAxx Feedback Cables

Incremental
Feedback
MPL-A/B15xxx…MPL-A/B2xxx-Hx4/7xAA
Wire Color
2198-K51CK-D15M
Connector Kit Pin
Motor Pin

SIN+
Black

SINWhite/Black

COS+
Red

COSWhite/Red

DATA+
Green

DATAWhite/Green

EPWR_5V
Gray

ECOM
White/Gray
6 (1)
(1)
The ECOM and TS- connections are tied together and connect to the cable shield.

EPWR_9V
Orange

TS+
White/Orange

S1
White/Blue

S2
Yellow

S3
White/Yellow

## Motor Pin

TLY-Axxxx-H
Incremental Encoder Feedback
Wire Color
2198-K51CK-D15M
Connector Kit Pin

AM+
Black

AM–
White/Black

BM+
Red

BM–
White/Red

IM+
Green

IM–
White/Green

EPWR_5V
Gray

ECOM
White/Gray
6 (1)
(1)
The ECOM and TS- connections are tied together and connect to the cable shield.

S1
White/Blue

S2
Yellow

S3
White/Yellow

Drain
–
Motor Pin
TLY-Axxxx-B
17-bit Absolute, Multi-turn, High-resolution Feedback
Wire Color
2198-K51CK-D15M
Connector Kit Pin

DATA+
Green

DATA–
White/Green

EPWR_5V
Gray

ECOM and BATWhite/Gray
6 (1)
(1)
BAT- is tied to ECOM (pin 23) in the cable.

BAT+
Orange
BAT+

Drain
–

**Extracted table (page 28, #1):**

| MPL-A/B15xxx…MPL-A/B2xxx-Hx4/7xAA | Wire Color |
| --- | --- |
| SIN+ | Black |
| SIN- | White/Black |
| COS+ | Red |
| COS- | White/Red |
| DATA+ | Green |
| DATA- | White/Green |
| EPWR_5V | Gray |
| ECOM | White/Gray |
| EPWR_9V | Orange |
| TS+ | White/Orange |
| S1 | White/Blue |
| S2 | Yellow |
| S3 | White/Yellow |

**Extracted table (page 28, #2):**

| TLY-Axxxx-H Incremental Encoder Feedback | Wire Color |
| --- | --- |
| AM+ | Black |
| AM– | White/Black |
| BM+ | Red |
| BM– | White/Red |
| IM+ | Green |
| IM– | White/Green |
| EPWR_5V | Gray |
| ECOM | White/Gray |
| S1 | White/Blue |
| S2 | Yellow |
| S3 | White/Yellow |
| Drain | – |

**Extracted table (page 28, #3):**

| TLY-Axxxx-B 17-bit Absolute, Multi-turn, High-resolution Feedback | Wire Color |
| --- | --- |
| DATA+ | Green |
| DATA– | White/Green |
| EPWR_5V | Gray |
| ECOM and BAT- | White/Gray |
| BAT+ | Orange |
| Drain | – |

<!-- page 29 -->

> **Table 66** — 2090-DANFCT-Sxx Feedback Cables

> **Figure 67** — Wire the 2198-K51CK-D15M Feedback Connector Kit

## Motor Pin

TL-Axxxx-B
17-bit Absolute, Multi-turn, High-resolution Feedback
Wire Color
2198-K51CK-D15M
Connector Kit Pin

SD+
Brown

SD–
White/Brown 10

EPWR_5V
Gray

ECOM and BATWhite/Gray
6 (1)
(1)
BAT- is tied to ECOM (pin 8) in the cable.

BAT+
Orange
BAT+

Drain
–
Shield Clamp
Clamp Screws (2)
Tie Wrap is
recommended for
Stress Relief and
Wire Management
Exposed Shield Aligned
Under the Shield Clamp
8-pin
Connector (2x)
15-pin D-sub to
Motor Feedback (MFB)
Connector
Kinetix 2090
Feedback Cable
Mounting
Screws (2x)
1. Place exposed cable shield
in the channel.
2. Place the shield clamp over
the exposed shield.
3. Tighten screws, torque
0.35 N•m (3.097 lb•in).
Terminal
Signal
Wire Color

SIN+
AM+
Black

SIN–
AM–
White/Black

COS+
BM+
Red

COS–
BM–
White/Red

DATA+
IM+
Green

ECOM (1)
(1)
The ECOM and TS- connections are tied together and
connect to the cable shield.
White/Gray

EPWR_9V
Orange

S3
White/Yellow

DATA–
IM–
White/Green

TS+
White/Orange

S1
White/Blue

S2
Yellow

EPWR_5V
Gray
+
Battery +
N/A (2)
(2)
See cable pinouts for wire colors.
–
Battery –
N/A (2)
Drain
Shield

**Extracted table (page 29, #1):**

| TL-Axxxx-B in 17-bit Absolute, Multi-turn, High-resolution Feedback | Wire Color |
| --- | --- |
| SD+ | Brown |
| SD– | White/Brown |
| EPWR_5V | Gray |
| ECOM and BAT- | White/Gray |
| BAT+ | Orange |
| Drain | – |

<!-- page 30 -->

## External Passive-shunt

Resistor Connections
Passive shunt connections are made at the shunt connector on the bottom of
the drive or the terminal blocks on the front of the drive.
Follow these guidelines when wiring your 2198-Rxxx or 2097-Rx passive shunt:
•
Refer to External Passive Shunt Modules on page 45 for noise zone
considerations.
•
Refer to Passive Shunt Wiring Examples on page 463.
•
Refer to the installation instructions provided with your Bulletin 2198
shunt module, publication 2198-IN011.
All Kinetix 5100 servo drives have internal shunt IGBT. However, only
2198-E1004-ERS…2198-E2030-ERS (200V) drives and 2198-E4004-ERS…
2198-E4015-ERS (400V) drives have an internal shunt resistor. The DC+ to ISH
jumper connects the internal shunt resistor.

> **Figure 68** — 2198-E1004-ERS, 2198-E1007-ERS and 2198-E1015-ERS Servo Drives

> **Figure 69** — 2198-E1020-ERS, 2198-E2030-ERS, 2198-E2055-ERS, 2198-E2075-ERS, 2198-E2150-ERS,

and 2198-E4xxx-ERS Servo Drives
IMPORTANT
To improve system performance, run wires and cables in the wireways
as established in Chapter 2.
IMPORTANT
You must remove the internal shunt jumper in the shunt connector
(between DC+ and ISH) before connecting the Bulletin 2198 or 2097
passive shunt resistor wires.
DC+ ISH ESH
Kinetix 5100 Servo Drives
(2198-E1004-ERS drive is shown)
Bottom View
Shunt Connector
DC+
ESH
DC+
ISH
ESH
NET
5100
MOD
CHARGE

I/O
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
2198-E1020-ERS,
2198-E2030-ERS,
2198-E4004-ERS,
2198-E4007-ERS, and
2198-E4015-ERS
Kinetix 5100 Servo Drives
Front View
2198-E2055-ERS, 2198-E2075-ERS,
2198-E2150-ERS, 2198-E4020-ERS,
2198-E4030-ERS, 2198-E4055-ERS,
2198-E4075-ERS, and 2198-E4150-ERS
Kinetix 5100 Servo Drives
Front View
Kinetix 5100 Servo Drives
(2198-E1020-ERS drive is shown)
Front View
Shunt Connections

<!-- page 31 -->

See Passive Shunt Considerations on page 35 for shunts compatible with your
Kinetix 5100 servo drive.

> **Table 67** — Shunt Connector Specifications

Kinetix 5100 Drive
Cat. No.
Connects to
Terminals
Recommended Wire
Size
mm2 (AWG)
Strip Length
mm (in.)
Torque Value
N•m (lb•in)
2198-E1004-ERS
2198-E1007-ERS
2198-E1015-ERS
DC+
ISH
ESH
0.20…3.31
(24…12)
11 (0.4)
N/A (1)
(1)
This connector uses spring tension to hold wires in place.
2198-E1020-ERS
2198-E2030-ERS
0.20…5.26
(24…10)
13 (0.5)
N/A (1)
2198-E2055-ERS
DC+
ESH
0.82…8.36
(18…8)
1.8 (2)
(15.49)
(2)
Attach using a terminal crimp lug.
2198-E2075-ERS
11 (0.4)
1.6 (2)
(13.90)
2198-E2150-ERS
2.08…21.1
(14…4)
13 (0.5)
3.1 (2)
(27.44)
2198-E4004-ERS
2198-E4007-ERS
2198-E4015-ERS
DC+
ESH
0.20…5.26
(24…10)
13 (0.5)
N/A (1)
2198-E4020-ERS
2198-E4030-ERS
2198-E4055-ERS
0.82…8.36
(18…8)
1.6 (2)
(13.90)
2198-E4075-ERS
11 (0.4)
2198-E4150-ERS
2.08…21.1
(14…4)
13 (0.5)
3.1 (2)
(27.44)

**Extracted table (page 31, #1):**

| Connects to Terminals | Recommended Wire Size mm2 (AWG) | Strip Length mm (in.) |
| --- | --- | --- |
| DC+ ISH ESH | 0.20…3.31 (24…12) | 11 (0.4) |
|  | 0.20…5.26 (24…10) | 13 (0.5) |
| DC+ ESH | 0.82…8.36 (18…8) |  |
|  |  | 11 (0.4) |
|  | 2.08…21.1 (14…4) | 13 (0.5) |
| DC+ ESH | 0.20…5.26 (24…10) | 13 (0.5) |
|  | 0.82…8.36 (18…8) |  |
|  |  | 11 (0.4) |
|  | 2.08…21.1 (14…4) | 13 (0.5) |

<!-- page 32 -->

## Ethernet Cable Connections

This procedure assumes that you have your Logix 5000™ controller and
Kinetix 5100 drives mounted and are ready to connect the network cables.
The EtherNet/IP™ network is connected by using the PORT 1 and PORT 2
connectors of the drive. Refer to page 50 to locate the Ethernet connectors on
your drive module. Refer to Figure 70 to locate the connectors on your Logix
5000 controller.
Shielded Ethernet cable is required for EMC compliance and is available in
several standard lengths. Ethernet cable lengths that connect drive-to-drive,
drive-to-controller, or drive-to-switch must not exceed 100 m (328 ft). Refer to
the Kinetix Motion Accessories Specifications Technical Data, publication
KNX-TD004, for more information.

> **Figure 70** — ControlLogix and CompactLogix Ethernet Port Locations

The Logix 5000 controllers accept linear, ring (DLR), and star network
configurations. Refer to Typical Communication Configurations on page 21
for linear, ring, and star configuration examples.
1 (Front)
2 (Rear)
00:00:BC:2E:69:F6

LNK1LNK2 NET OK

OK
FORCE SD
RUN
Logix5585
LINK
NET
TM
SAFETY ON
0000
ControlLogix 5570 Controller with
Bulletin 1756 EtherNet/IP Communication Module
ControlLogix Ethernet Ports
The 1756-EN2T modules have only one port,
1756-EN2TR and 1756-EN3TR modules have two.
Bottom View
Front Views
CompactLogix 5370 Controller,
Compact GuardLogix® 5370 Controller
(CompactLogix 5370 controller is shown)
Port 1, Front
Port 2, Rear
ControlLogix 5580 and
GuardLogix 5580 Controller
1 GB Ethernet Port
Front View
CompactLogix 5380 Controller, or
Compact GuardLogix 5380 Controller
(CompactLogix 5380 controller is shown)
Front View

<!-- page 33 -->

You can add the drive to your Studio 5000 Logix Designer® application by
adding it to a configured EtherNet/IP™ module or controller under the I/O
configuration folder. After setting network parameters, you can view the drive
status information in Studio 5000® environment and use it in your Logix
Designer application.
Settings are stored in nonvolatile memory. You can change the IP address
through the keypad interface, Module Configuration dialog box in RSLinx®
software, by using KNX5100C software, or through the drive Internet Protocol
page of Module Properties in your Logix Designer application. Changes to the
IP address take effect after drive power is cycled.
If configure DHCP is turned ON in the Network Parameters, you must
configure the IP address of drive by using BOOTP-DHCP tool.
Use one of the following three methods to set the network parameters:
•
Panel display
•
KNX5100C software on page 114
•
BOOTP-DHCP tool on page 115
The drive is factory programmed to static IP address of 192.168.1.1 and Gateway
address of 192.168.1.254.
Topic
Page
Set Network Parameters by Using the Keypad Interface

Set Network Parameters by Using KNX5100C Software

Configure IP Address by Using BOOTP-DHCP Tool

IMPORTANT
Only standalone mode is supported for linear motors and induction
motors.
