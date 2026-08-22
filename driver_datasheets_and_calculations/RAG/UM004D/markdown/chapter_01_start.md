# Chapter 1: Start

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `02_Ch01_Start.pdf` (14 pages)_

<!-- page 1 -->

Use this chapter to become familiar with the Kinetix® 5100 drive system and
obtain an overview of installation configurations.
About the Kinetix 5100 Drive
System
The Kinetix 5100 EtherNet/IP™ indexing servo drives are designed to provide a
single drive solution for applications with output power requirements
between 0.4…15.0 kW (2.6…78 A rms).
Topic
Page
About the Kinetix 5100 Drive System

Typical Hardware Configuration

Motor and Auxiliary Feedback Configurations

Typical Communication Configurations

Typical Control Configurations

Catalog Number Explanation

Agency Compliance

> **Table 1** — Kinetix 5100 Drive System Overview

Kinetix 5100 System
Component
Cat. No.
Description
Kinetix 5100 Servo Drive
2198-Exxxx-ERS
Kinetix 5100 EtherNet/IP indexing drives with Safe Torque Off (STO) are available with 120V single-phase,
200…230V single-phase, 230V three-phase, and 480V three-phase (nom) input voltages.
Terminal block for I/O
connector
2198-TBIO
50-pin terminal block. Plugs into I/O connector for control interface connections.
Motor Feedback
Connector Kit
2198-K51CK-D15M
Motor feedback connector kit with 15-pin connector plug for compatible servo motors. Kit features battery
backup for Kinetix TLP, TL, and TLY multi-turn encoders.
Auxiliary Feedback
Connector Kit
2198-AUXKIT
Auxiliary feedback connector kit for master feedback and load feedback connections to the AUX connector.
Feedback Battery Box
2198-KTBT
The feedback battery box is used in applications where Kinetix TLP motor position data must be maintained in
the event of a power loss. The battery box is included with Kinetix 2090 cables for Kinetix TLP motors and is also
available as this replacement kit.
Logix PAC® Controller
Platforms
Bulletin 5069 and 1769
EtherNet/IP networking with CompactLogix™ 5370 and CompactLogix 5380 controllers with embedded dual-port.
CompactLogix 5480 controllers for the benefits of Logix control with Windows®-based computing.
1756-EN2T, 1756-EN2TR,
and 1756-EN3TR module
EtherNet/IP network communication modules for use with ControlLogix® 5570 and ControlLogix 5580 controllers.
Micro Controller Platforms
MicroLogix™ 1400 controllers provide communication ports, an isolated combination RS-232/485 communication port, an Ethernet port, and a
non-isolated RS-232 communication port.
Micro800™ controllers with embedded inputs/outputs can accommodate from two to five plug-in modules and up to four expansion I/O
modules.
Configuration Software
Studio 5000® Environment Studio 5000 Logix Designer® application (version 30 or later) is used to program, commission, and maintain
Logix 5000™ controllers.
Connected Components
Workbench software
Connected Components Workbench™ design and configuration software (CCW), version 10.0 or later, provides
support for programming, configuration of Micro800 controller, and integration with the HMI editor.
KNX5100C software
KNX5100C software, version 1.001 or later, provides configuration and tuning of Kinetix 5100 drives via the
mini-USB cable connection.
RSLogix 500® software
RSLogix 500 software is used to program MicroLogix 1100 and 1400 controllers.

<!-- page 2 -->

## Rotary Servo Motors

Kinetix TLP
Compatible rotary motors include Kinetix TLP (200V and 400V-class) servo motors.
Kinetix MP
Compatible rotary motors include Kinetix MPL, MPM, MPF, and MPS (200V and 400V-class) servo motors.
Kinetix TL and TLY
Compatible rotary motors include Kinetix TL and TLY (200V-class) servo motors.
Linear Actuators
Kinetix MP and
LDAT
Compatible linear actuators include 200V and 400V-class Kinetix MPAS and MPMA linear stages, Kinetix MPAR and
MPAI linear actuators, and LDAT-Series linear thrusters.
Linear Motors
Kinetix LDC and
LDL
Compatible motors include Kinetix LDC iron-core and Kinetix LDL ironless linear motors.
Induction Motors
N/A
Induction motors with closed-loop control are supported.
Cables
2090-CTFB-MxDx-xxxxx
Kinetix 2090 motor feedback cables for Kinetix TLP motors.
2090-CTPx-MxDx-xxxxx
Kinetix 2090 motor power/brake cables for Kinetix TLP motors.
2090-CFBM6Dx-CxAAxx
Motor feedback cables for Kinetix TLY servo motors.
2090-CPxM6DF-16AAxx
Motor power/brake cables for Kinetix TLY servo motors.
2090-DANFCT-Sxx
Motor feedback cables for Kinetix TL servo motors.
2090-DANPT-16Sxx
Motor power cables for Kinetix TL servo motors.
2090-DANBT-18Sxx
Motor brake cables for Kinetix TL servo motors.
2090-CFBM7DF-CEAxxx
Motor feedback cables for Kinetix MP servo motors with Hiperface encoders.
2090-CPxM7DF-xxAxxx
Motor power/brake cables for Kinetix MP servo motors.
2090-XXNFMF-Sxx
2090-CFBM7DF-CDAFxx
Standard and continuous-flex feedback cables that include additional conductors for use with incremental
encoders.
1585J-M8CBJM-x
1585J-M8UBJM-x
Ethernet cables are available in standard lengths. Shielded cable is required to meet EMC specifications.
2198-USBC
Interface cable with mini-USB connector for KNX5100C software configuration.
2198-USBF
Filter for mini-USB port to reduce the vulnerability to electrical noise.
AC Line Filters
2198-DBxxx-F
2198-DBRxxx-F
Bulletin 2198 three-phase AC line filters are required to meet CE and UK and are available for use in all
Kinetix 5100 drive systems.
24V DC Power Supply
1606-XLxxx
Bulletin 1606 24V DC power supply for digital input/output, Safe Torque Off (STO) circuitry, and motor brake
control.
External Shunt Resistors
2097-R6, and 2097-R7
Bulletin 2097 and 2198 external passive shunt resistors are available for when the internal shunt capability of the
drive is exceeded.
2198-R004, 2198-R031

> **Table 1** — Kinetix 5100 Drive System Overview (Continued)

Kinetix 5100 System
Component
Cat. No.
Description

<!-- page 3 -->

## Typical Hardware

Configuration
Typical Kinetix 5100 drive systems include single-phase and three-phase
standalone configurations.
In this example, three-phase input power is applied to the Kinetix 5100 drive.

> **Figure 1** — Kinetix 5100 Standalone Drive with Three-phase Input Power

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
L1
L2 L3
Kinetix 5100 Servo Drive
(2198-E1020-ERS drive is shown)
2198-DBxxx-F
or 2198-DBRxxx-F
AC Line Filter
(required for CE and UK)
Line
Disconnect
Device
Circuit
Protection
Three-phase
Input Power
Kinetix 2090
Motor Power Cable
2198-TBIO Terminal
Expansion Block
2198-Rxxx or 2097-Rx
Shunt Resistor
(optional equipment)
Ground Plate
for Motor Power
Ground Connection
Kinetix 2090
Motor Feedback Cable
Kinetix TLP
Servo Motor
Circuit
Protection

<!-- page 4 -->

Motor and Auxiliary
Feedback Configurations
Motor feedback connections are made at the 15-pin motor feedback (MFB)
connector. Auxiliary feedback connections are made by using the auxiliary
feedback (AUX) connector. These examples illustrate how you can use the
Bulletin 2198 connector kits for making these connections. To see motor power
and brake connections, refer to Chapter 4 on page 94.

> **Figure 2** — Feedback Configuration Example

P1
P2
DC–
L1
L2
L3
L1C
L2C
DC+
ESH
U
V
W
NET
MOD
CHARGE
5100
I/O
AUX
MFB
AUX
MFB
MBRK+
MBRKAUX
MFB
AUX
MFB
AUX
MFB
CAT. NO. LDC-M075500
SERIAL NO. XXXX X XXXX
SERIES A
www.ab.com
MADE IN USA
CAT. NO. LDC-M075500
SERIAL NO. XXXX X XXXX
SERIES A
www.ab.com
MADE IN USA
2198-AUXKIT Auxiliary Feedback Connector Kit
Accepts Digital AqB incremental encoder feedback (TTL).
Application uses:
•
Load feedback (PT Mode, dual loop)
•
Master feedback (PR Mode, used as E-CAM command source)
10-pin Auxiliary Feedback
(AUX) Connector
15-pin Motor Feedback
(MFB) Connector
2198-K51CK-D15M Motor Feedback Connector Kit
Accepts multiple encoder feedback types and provides battery-backup for multi-turn position data:
•
Hiperface high-resolution absolute multi-turn and single-turn encoders
– Kinetix MPL-A/Bxxx-S/M, MPM-A/Bxxx-S/M, MPF-A/Bxxx-S/M, MPS-A/Bxxx-S/M servo motors
– Kinetix MPL-A/Bxxx-E/V servo motors
•
Nikon (24-bit) high-resolution serial encoder
– Kinetix TLP-Axxx-xxx-D servo motors
•
Tamagawa (17-bit) high-resolution serial encoder
– Kinetix TL-AxxxP-B servo motors
– Kinetix TLY-AxxxP-B servo motors
Kinetix 5100 Servo Drive
(2198-E2055-ERS drive is shown)
2090-CFBM7Dx and 2090-CPxM7DF
Motor Feedback and Power Cables
Kinetix MP Motors and Actuators
(MPL-Bxxxx motor is shown)
2090-CTFB-MxDD and 2090-CTPx-MxDF
Motor Feedback and Power Cables
2090-CFBM6Dx and 2090-CPxM6DF
Motor Feedback and Power Cables
Kinetix TLP Motors
(TLP-A100 motor is shown)
Kinetix TL/TLY Motors
(TLY-A110 motor is shown)
2090-CTFB-MxDD Feedback Cable
Provides battery-backup for multi-turn position data:
•
Nikon (24-bit) high-resolution serial encoder
– Kinetix TLP-A/Bxxx-xxx-D servo motors
Battery
Box
•
Digital AqB (TTL) encoders with UVW (incremental)
– MPL-A/B15xxx-H, MPL-A/B2xxx-H, MPL-A/B3xxx-H,
MPL-A/B4xxx-H, MPL-A/B45xxx-H rotary motors
– Kinetix TLY-Axxxx-H servo motors
LDAT-Sxxxxxx-xBx
Linear Thrusters
Kinetix MPAS Linear Stages
(MPAS-B9xxx ballscrew linear stage is shown)
Kinetix MPAI Heavy-duty Electric Cylinders
(MPAI-B3xxxx heavy-duty electric cylinder is shown)
Induction Rotary Motors
•
Closed loop
•
Incremental encoder feedback
LDC-Series Linear Motors
(LDC-Cxxxxxxx linear motor shown)
LDL-Series Linear Motors
(LDL-xxxxxxxx linear motor shown)

<!-- page 5 -->

## Typical Communication

Configurations
The Kinetix 5100 drives support linear, ring, and star Ethernet topologies by
using ControlLogix, CompactLogix, MicroLogix, and Micro800 controllers.
These examples feature the CompactLogix 5370 programmable automation
controllers (catalog number 1769-LxxER, for example) with support for
Kinetix 5100 drives via Class 1 EtherNet/IP connection by using an Add-On
Profile (AOP) and Add-On Instructions or Explicit Messaging (using Class 3
EtherNet/IP messaging) over the EtherNet/IP network. Other Allen-Bradley®
controllers are also compatible with the Kinetix 5100 servo drives.
Refer to CompactLogix Controllers Specifications Technical Data, publication
1769-TD005, for more information on CompactLogix 5370 L1, L2, and L3
controllers.
Linear Topology
In this example, all devices are connected in linear topology. The Kinetix 5100
drives include dual-port connectivity, however, if any device becomes
disconnected, all devices downstream of that device lose communication.
Devices without dual-ports must include the 1783-ETAP module or be
connected at the end of the line.

> **Figure 3** — Kinetix 5100 Linear Communication Installation

1 (Front)
2 (Rear)
00:00:BC:2E:69:F6

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
0 2

1734-AENTR
Module
Status
Network
Activity
Network
Status
Point Bus
Status
System
Power
Field
Power
POINT I O
Link 1
Activity/
Status
Link 2
Activity/
Status
CompactLogix 5370 Controller
Studio 5000 Logix Designer
Application
 1585J-M8CBJM-x
Ethernet (shielded) Cable
CompactLogix Controller Programming Network (USB connection
1585J-M8CBJM-OM3
0.3 m (1.0 ft) Ethernet cable
for drive-to-drive connections.
Kinetix 5100 Servo Drive System
(2198-E1004-ERS drives are shown)
1734-AENTR POINT I/O™
EtherNet/IP Adapter
PanelView™ 5310
Display Terminal

<!-- page 6 -->

## Ring Topology

In this example, the devices are connected by using ring topology. If only one
device in the ring is disconnected, the rest of the devices continue to
communicate. For the ring topology to work correctly, a Device Level Ring
(DLR) supervisor is required (for example, the Bulletin 1783 ETAP device). DLR
is an ODVA standard. For more information, refer to the EtherNet/IP
Embedded Switch Technology Application Guide, publication ENET-AP005.
Devices without dual-ports, for example the display terminal, require a
1783-ETAP module to complete the network ring.

> **Figure 4** — Kinetix 5100 Ring Communication Installation

1 (Front)
2 (Rear)
00:00:BC:2E:69:F6

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
0 2

1734-AENTR
Module
Status
Network
Activity
Network
Status
Point Bus
Status
System
Power
Field
Power
POINT I O
Link 1
Activity/
Status
Link 2
Activity/
Status
PanelView 5310
Display Terminal
CompactLogix 5370 Controller
(Dual-Port is used - ring
Studio 5000 Logix Designer
Application
 1585J-M8CBJM-x
Ethernet (shielded) Cable
1585J-M8CBJM-OM3
0.3 m (1.0 ft) Ethernet cable
for drive-to-drive connections.
Kinetix 5100 Servo Drives
(2198-E1004-ERS drives are shown)
1734-AENTR POINT I/O
EtherNet/IP Adapter
CompactLogix Controller Programming Network (USB connection

<!-- page 7 -->

## Star Topology

In this example, the devices are connected by using star topology. Each device
is connected directly through a Stratix® 5700 switch.
Kinetix 5100 drives have dual-ports, so linear topology is maintained from the
switch port to the drive, but each drive uses a unique port on the Ethernet
switch. The loss of one device does not impact the operation of other devices.

> **Figure 5** — Kinetix 5100 Star Communication Installation

1 (Front)
2 (Rear)
00:00:BC:2E:69:F6

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
0 2

1734-AENTR
Module
Status
Network
Activity
Network
Status
Point Bus
Status
System
Power
Field
Power
POINT I O
Link 1
Activity/
Status
Link 2
Activity/
Status
CompactLogix Controller Programming Network (USB connection
 1783-BMS
Stratix 5700
Switch (1)
CompactLogix 5370 Controller
Studio 5000 Logix Designer
Application
PanelView 5310
Display Terminal
Kinetix 5100 Servo Drives
(2198-E1004-ERS drives are shown)
1734-AENTR POINT I/O
EtherNet/IP Adapter
(1) While a switch with PTP is shown in this example, the Kinetix 5100 drive does not require a switch with the PTP function.

<!-- page 8 -->

## Typical Control

Configurations
You can configure Kinetix 5100 servo drives by using various methods for
network control.
All Kinetix 5100 drive configurations require the use of KNX5100c software.
This software is required to configure the following:
•
Motor direction (rotation) and unit scaling
•
Motor and feedback selection (including loop types)
•
Digital I/O
•
Tuning
•
E-CAM profiles
•
PR configurations

<!-- page 9 -->

Logix Enabled Using a Class 1 EtherNet/IP Connection
You can use the Kinetix 5100 drive with a Logix 5000™ controller and
Studio 5000 software to deliver a simplified programming experience by using
a Class 1 Ethernet/IP connection (AOP) and supported Add-On-Instructions to
program and control the drive. In this architecture, the drive is configured by
using the KNX5100C software with a USB connection. It is important to note
that this implementation seems like Integrated Motion on Ethernet/IP (CIP)
motion. This connection is NOT CIP motion but does provide simple control
for your small motion application. More information on the AOP is found in
Chapter 8 and the Add-On Instruction library is found in Appendix C.

> **Figure 6** — Kinetix 5100 Drive System with PAC Controller and EtherNet/IP Network Control

IMPORTANT
The Kinetix 5100 drive does not support PTP or Integrated Motion on
EtherNet/IP (CIP) motion. The Kinetix 5100 drive is a Class 1 EtherNet/IP
device and uses a Requested Packet Interval (RPI) to exchange data
between the PAC and drive.
IMPORTANT
Class 1 and Class 3 EtherNet/IP Connections do not support induction
motors and linear motors.
LNK1 LNK2 NET
OK
EtherNet/IP

OK
FORCE SD
RUN
Logix5585
LINK
NET
TM
SAFETY ON
0000
DC INPUT
AC OUTPUT
DC INPUT
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

R
W
P
PanelView 5310
Display Terminal
Studio 5000
Logix Designer Application
Optional Controllers
ControlLogix 5570 Controllers or
GuardLogix® 5570 Safety Controllers
ControlLogix 5580 Controllers or
GuardLogix 5580 Safety Controllers
CompactLogix 5370 Controllers or
Compact GuardLogix 5370 Safety Controllers
CompactLogix 5380 and 5480 Controllers or
Compact GuardLogix 5380 Safety Controllers
1585J-M8CBJM-x (shielded) or
1585J-M8UBJM-x (high-flex shielded)
Ethernet Cable
Any Logix 5000 Controller
1783-US05T
Stratix 2000 Switch
Kinetix 5100 Servo Drives
(2198-E1020-ERS drives are shown)
EtherNet/IP Network
2198-USBC
Mini-USB Interface Cable
(with 2198-USBF filter)
KNX5100C Drive
Configuration Software

<!-- page 10 -->

Micro800 Using a Class 3 EtherNet/IP Connection
You can use the Kinetix 5100 drive with a Micro800 controller and Connected
Components Workbench software to provide a simplified programming
experience using Class 3 explicit messaging between the drive and controller.
Connected Components Workbench software is used to program the
controller and uses a User Defined Function Block (UDFB) to pass data (both
command and status) between the controller and drive. In this architecture the
drive is configured by using the KNX5100C software with a USB connection.
Examples that use explicit messaging are available from the sample code
website.
From https://www.rockwellautomation.com/, go to the Support tab and click
on Sample Code Library. Use a keyword search: Kinetix 5100.

> **Figure 7** — Kinetix 5100 Drive System with PLC Controller and Class 3 EtherNet/IP Explicit Messaging

Control
IMPORTANT
Class 1 and Class 3 EtherNet/IP Connections do not support induction
motors and linear motors.
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
1585J-M8CBJM-x (shielded) or
1585J-M8UBJM-x (high-flex shielded)
Ethernet Cable
Controller Options with
Explicit Messaging Capability
Kinetix 5100 Servo Drive
(2198-E1020-ERS drive is shown)
To Other EtherNet/IP Devices
Third-party Controller
with EtherNet/IP Network
2080-LCxx-xxxx
Micro830® Controller
•
Connected Components Workbench software for
programming Micro800 controllers
•
KNX5100C Drive Configuration software for
configuration and tuning of Kinetix 5100 drives
2198-USBC
Mini-USB Interface Cable
(with 2198-USBF filter)

<!-- page 11 -->

Pulse Train Output Control with Motion User Defined Function Block
The Kinetix 5100 drive can be used with a Micro800 controller + Connected
Components Workbench (CCW) software to provide a simplified
programming experience using Pulse Train Output (PTO) wiring between the
drive and controller. Connected Components Workbench software is used to
program the controller and uses the built in CCW Motion Library and User
Defined Function Block (UDFB) to control the drive. In this architecture the
drive is configured by using the KNX5100C software with a USB connection.
There is an optional PTO feedback (AQB) channel that can be used to provide
feedback from this drive. There is also one digital input used to enable the
drive (Servo On) and one digital output used for the Ready signal. Examples
using explicit messaging are available from the sample code website.
From https://www.rockwellautomation.com/, go to the Support tab and click
on Sample Code Library. Use a keyword search: Kinetix 5100.

> **Figure 8** — Kinetix 5100 Drive System with PLC Controller and PTO, Analog or Digital I/O Control

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
Controller Options with
Pulse Train Output, Analog or
Digital I/O (indexing)
Kinetix 5100 Servo Drive
(2198-E1020-ERS drive is shown)
2080-LCxx-xxxx
Micro830 Controller
Third-party Controller
with Pulse Train Output or
Analog or Digital I/O
2198-USBC
Mini-USB Interface Cable
(with 2198-USBF filter)
•
Connected Components Workbench software for
programming Micro800 controllers
•
KNX5100C Drive Configuration software for
configuration and tuning of Kinetix 5100 drives

<!-- page 12 -->

## Catalog Number

Explanation
Kinetix 5100 drive catalog numbers and descriptions are listed in these tables.

> **Table 2** — Kinetix 5100 EtherNet/IP Indexing Servo Drives

Cat. No.
Input Voltage
Continuous Output Power(1)
kW
Continuous Output Current
A (rms)
Peak Output Current
A (rms)
2198-E1004-ERS
95…132V rms single-phase
170…253V rms single-phase
170…253V rms three-phase
0.20
0.40
0.40
2.6
6.5
2198-E1007-ERS
0.375
0.75
0.75
5.1
15.4
2198-E1015-ERS
0.75
1.50
1.50
7.9
23.7
2198-E1020-ERS
1.00
2.00
2.00
13.4
40.6
2198-E2030-ERS
170…253V rms three-phase
3.00
17.9
55.9
2198-E2055-ERS
5.50
41.3
91.4
2198-E2075-ERS
7.50
49.0
127.5
2198-E2150-ERS
15.00
78.0
162.0
2198-E4004-ERS
342…528V rms three-phase
0.40
1.60
5.4
2198-E4007-ERS
0.75
3.19
8.0
2198-E4015-ERS
1.50
6.05
15.11
2198-E4020-ERS
2.00
7.42
20.78
2198-E4030-ERS
3.00
13.95
26.08
2198-E4055-ERS
5.50
24.80
37.65
2198-E4075-ERS
7.50
31.0
53.32
2198-E4150-ERS
15.00
41.26
70.14
(1)
Continuous Output Power is rated at the Kinetix TLP motor shaft output power.

> **Table 3** — Kinetix 5100 Servo Drive Accessories

Cat. No.
Drive Components
2198-TBIO
Terminal block for I/O connections
2097-R6, 2097-R7,
2198-R004, and 2198-R031
External passive-shunt resistors for use when additional shunt capability is needed.
2198-DBxxx-F
2198-DBRxxx-F
AC line filters (required to meet CE and UK)
2198-K51CK-D15M
Motor feedback connector kit
2198-AUXKIT
Auxiliary feedback connector kit
2198-KTBT
Feedback battery-box replacement kit
2198-USBC
Interface cable with mini-USB connector for KNX5100C software configuration.
2198-USBF
Filter for mini-USB port to reduce the vulnerability to electrical noise.

**Extracted table (page 12, #1):**

| Input Voltage | Continuous Output Power(1) kW | Continuous Output Current A (rms) |
| --- | --- | --- |
| 95…132V rms single-phase 170…253V rms single-phase 170…253V rms three-phase | 0.20 0.40 0.40 | 2.6 |
|  | 0.375 0.75 0.75 | 5.1 |
|  | 0.75 1.50 1.50 | 7.9 |
|  | 1.00 2.00 2.00 | 13.4 |
| 170…253V rms three-phase | 3.00 | 17.9 |
|  | 5.50 | 41.3 |
|  | 7.50 | 49.0 |
|  | 15.00 | 78.0 |
| 342…528V rms three-phase | 0.40 | 1.60 |
|  | 0.75 | 3.19 |
|  | 1.50 | 6.05 |
|  | 2.00 | 7.42 |
|  | 3.00 | 13.95 |
|  | 5.50 | 24.80 |
|  | 7.50 | 31.0 |
|  | 15.00 | 41.26 |

<!-- page 13 -->

## Agency Compliance

If this product is installed within the European Union and has the CE marking,
or within the United Kingdom and has the UKCA marking, the following
regulations apply.
For more information on electrical noise reduction, see the System Design for
Control of Electrical Noise Reference Manual, publication GMC-RM001.
To comply with EN/IEC 61800-3 (category C3) and EN/IEC 61800-5-2, these
requirements apply:
•
Install an AC line filter (catalog number 2198-Dxxx-F) as close to the drive
as possible.
•
Bond drive modules and line-filter grounding screws by using a braided
ground strap as shown in Figure 52 on page 85.
•
Use Kinetix 2090 motor-power cables or use connector kits and connect
the cable shields to the subpanel with clamp provided.
•
Use Kinetix 2090 motor-feedback cables or use connector kits and
properly connect the feedback cable shield.
•
Drive-to-motor cables must not exceed 50 m (164 ft), depending on AC
input power and feedback type. See Maximum Cable Length on page 95
for specifications.
•
Install the Kinetix 5100 system inside an enclosure. Run input power
wiring in conduit (grounded to the enclosure) outside of the enclosure.
Separate signal and power cables.
•
Separate signal and power cables. Segregate input power wiring and
motor power cables from control wiring and motor feedback cables. Use
shielded cable for power wiring and provide a grounded 360° clamp
termination.
See Appendix A on page 457 for interconnect diagrams, including input power
wiring and drive/motor interconnect diagrams.
ATTENTION: The drive and line filter must be grounded. Failure to do this
renders the filter ineffective and can cause damage to the filter. For ground
examples, see Ground the Drive System on page 85.

<!-- page 14 -->

Notes:
