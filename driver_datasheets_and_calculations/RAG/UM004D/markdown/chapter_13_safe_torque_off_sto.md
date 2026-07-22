# Chapter 13: Safe Torque Off (STO)

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `14_Ch13_Safe_Torque_Off.pdf` (13 pages)_

<!-- page 1 -->

Certification
The TÜV Rheinland group has approved 2198-Exxxx-ERS servo drives with
hardwired safe torque-off for use in safety-related applications up to
ISO 13849-1, Performance Level d (PL d) and Category 3, SIL CL 2 per
IEC 61508, IEC 61800-5-2, and IEC 62061, in which removing the motion
producing power is considered to be the safe state.
For product certifications currently available from Rockwell Automation, go to
website rok.auto/certifications.
Important Safety Considerations
The system user is responsible for the following:
•
Validation of any sensors or actuators connected to the system
•
Completing a machine-level risk assessment
•
Certification of the machine to the desired ISO 13849-1 performance level
or IEC 62061 SIL level
•
Project management and proof testing in accordance with
ISO 13849
Category 3 Requirements According to ISO 13849-1
Safety-related parts are designed with these attributes:
•
A single fault in any of these parts does not lead to the loss of the safety
function.
•
A single fault is detected whenever reasonably practicable.
•
Accumulation of undetected faults can lead to the loss of the safety
function and a failure to remove motion producing power from the
motor.
Stop Category Definition
Stop Category 0 as defined in IEC 60204 or Safe Torque Off (STO) as defined
by IEC 61800-5-2 is achieved with immediate removal of motion producing
power to the actuator.
Performance Level (PL) and Safety Integrity Level (SIL)
For safety-related control systems, Performance Level (PL), according to ISO
13849-1, and SIL levels, according to IEC 61508 and IEC 62061, include a rating
of the systems ability to perform its safety functions. All of the safety-related
components of the control system must be included in both a risk assessment
and the determination of the achieved levels.
IMPORTANT
In the event of a malfunction, the most likely stop category is
Stop Category 0. When designing the machine application, timing
and distance must be considered for a coast to stop. For more
information regarding stop categories, refer to IEC 60204-1.

<!-- page 2 -->

Refer to the ISO 13849-1, IEC 61508, and IEC 62061 standards for complete
information on requirements for PL and SIL determination.
Description of Operation
The Safe Torque Off (STO) feature provides a method, with sufficiently low
probability of failure, to force the power-transistor control signals to a disabled
state. When disabled, or any time power is removed from the safety enable
inputs, all of the drive output-power transistors are released from the ONstate. This results in a condition where the drive performs a Category 0 Stop.
Disabling the power transistor output does not provide mechanical isolation of
the electrical output that is required for some applications.
For hardwired control of the safe torque-off function, the appropriate wiring
must be connected to the Safety connector plug. Refer to Safe Torque Off
Specifications on page 418 for more information on the safety inputs.
Under normal operation, the safe torque-off inputs are energized. If an STO
fault is detected, then all of the output power transistors turn off. The safe
torque-off response time is less then 20 ms.
The ServoOutputStatus parameter represents various drive status values. Bit 0
(Servo Ready) is used to indicate the status of the Safe Torque Off inputs. This
bit can be monitored in KNX5100C software.

> **Figure 218** — ServoOutputStatus Parameter Setting

ATTENTION: Permanent magnet motors can, in the event of two
simultaneous faults in the IGBT circuit, result in a rotation of up to 180
electrical degrees.
ATTENTION: If either of the safety enable inputs are de-energized for more
than 1 second, or both inputs are in the OFF state simultaneously for more
than 10 ms, a fault condition results.

<!-- page 3 -->

> **Figure 219** — System Operation when Inputs are Meeting Timing Requirements

## STO-related Fault Codes

For information on how to clear faults, see Clear Faults on page 454.
Figure 220 demonstrates when the safe torque-off mismatch is detected and
fault E 501 (STO_A signal loss) or E 502 (STO_B signal loss) is posted.

> **Table 125** — ID107 (P0.046) ServoOutputStatus

Channel
Status of Output (1)
(1)
ServoOutputStatus is located in KNX5100C software>Parameter Editor>Status Monitor.
STO_A
ON
ON
OFF
OFF
STO_B
ON
OFF
ON
OFF
Status
Ready (2)
(2)
Kinetix 5100 drive is Ready and able to produce torque (current) to the motor. ServoOutputStatus/Servo Ready (Bit 0) = 1.
Torque off (3)
(3)
When STO_B is lost for more than 1 second, with STO_A high, the Kinetix 5100 drive faults (E 502) and has no motor torque
(current). ServoOutputStatus/Servo Ready (Bit 0) = 0.
Torque off (4)
(4)
When STO_A is lost for more than 1 second, with STO_B high, the Kinetix 5100 drive faults (E 501) and has no motor torque
(current). ServoOutputStatus/Servo Ready (Bit 0) = 0.
Torque off (5)
(5)
When both STO_A and STO_B are lost for more than 10 ms, the Kinetix 5100 drive faults (E 500) and has no motor torque
(current). ServoOutputStatus/Servo Ready (Bit 0) = 0.
24V DC
24V DC
0V DC
0V DC

< 10 ms

< 1 Second

## STO Fault Code

STO_A (S1)
STO_B (S2)
ServoOutputStatus/Servo Ready, Bit 0
Event
Description

One input is switched-off and second input is on.

First input is switched-on within 1 second.

Both inputs are switched-off.

Both inputs are in OFF state simultaneously within 10 ms.

Second input is switched-on within 1 second of event 4.
This manual links to Kinetix® 5100 Servo Drive Fault Codes Reference
Data, publication 2198-RD001, for fault codes and Kinetix 5100 Servo
Drive Parameters Reference Data, publication 2198-RD002, for
parameters. Download the spreadsheets now for offline access.

**Extracted table (page 3, #1):**

| ON | ON | OFF |
| --- | --- | --- |
| ON | OFF | ON |
| Ready (2) | Torque off (3) | Torque off (4) |

<!-- page 4 -->

> **Figure 220** — System Operation in the Event that the Safety Enable Inputs Mismatch

When the STO self-diagnostic (STO circuit and wiring, for example) fails, fault
E 503 is posted.
Figure 221 demonstrates when both inputs in the OFF state are detected and
fault E 500 (STO enabled) is posted.

> **Figure 221** — System Operation When Both Safety Enable Inputs are in OFF State Simultaneously

> **Figure 222** — Typical Fault Reset Sequence

24V DC
24V DC
0V DC
0V DC

1 Second

## STO Fault Code E 501

STO_A (S1)
STO_B (S2)
ServoOutputStatus/Servo Ready, Bit 0
24V DC
24V DC
0V DC
0V DC

10 ms

## STO Fault Code E 500

STO_A (S1)
STO_B (S2)
ServoOutputStatus/Servo Ready, Bit 0
ATTENTION: The safe torque-off fault is detected upon demand of the Safe
Torque Off (STO) function. After troubleshooting the STO function or
performing maintenance that might affect the STO function, the STO
function must be executed to verify correct operation.
24V DC
24V DC
0V DC
0V DC

10 ms

## STO Fault Code E 500

STO_A (S1)
STO_B (S2)
Fault Reset Command

<!-- page 5 -->

For Safety Status (SS), you can configure one parameter/attribute, ID252
(P2.093) STOFeedbackConfiguration and determine whether SS will latch, if
an STO fault occurs. If SS signal is latched when STO fault occurs, the status of
SS signal remains even when the fault has been cleared.

> **Figure 223** — Parameter Format Legend

Average Frequency of a
Dangerous Failure per Hour
Safety-related systems are classified as operating in a High-demand/
continuous mode. The SIL value for a High-demand/continuous mode safetyrelated system is directly related to the probability of a dangerous failure
occurring per hour (PFH).
PFH calculation is based on the equations from IEC 61508 and show worst-case
values. Table 127 provides data for a 20-year proof test interval and
demonstrates the worst-case effect of various configuration changes on the
data.
IMPORTANT
The STO fault (E 500) can be reset only if both inputs are in the ON
state. After the fault reset requirement is satisfied, a Fault Reset
(AOI: raC_xxx_K5100_MAFR instruction) command in the application
software or DI.ARST (physical input) must be issued to reset the
E 500 fault. You can reset faults E 501, E 502, and E 503 with power
cycle.

> **Table 126** — SS Signal Status/Behavior for STO Faults

## Servo Drive Status

SS Signal Status (1)
(1)
Open indicates no continuity between SS+ and SS– an open circuit. Close indicates continuity between SS+ and SS– is
a short circuit.
Parameter ID252 (P2.093)
XX1X
XX2X
SS signal behavior
No latch
Latch
No STO fault occurs
Open
STO fault occurs
E 500
Close
E 501
Open
E 502
Open
E 503
Open
ID252 (P2.093) = XX. 0 X
Reserved
Latch Status
1 = SS no latch
2 = SS latch
Reserved
IMPORTANT
Determination of safety parameters is based on the assumptions
that the system operates in High-demand mode and that the safety
function is requested at least once every three months.

> **Table 127** — PFH for 20-year Proof Test Interval

Attribute
Value
PFH (1e-9)
0.96
Proof test (years)

<!-- page 6 -->

## Safe Torque Off Connector

Data
The Kinetix 5100 drive ships with the (8-pin) wiring-plug header that connects
your safety circuit to the Kinetix 5100 drive Safe Torque Off (STO) connector.
The header includes jumper wires that by-pass the safety function for drives
that do not use the Safe Torque Off feature. Remove the jumper wires when the
Safe Torque Off feature is used.

> **Figure 224** — Pin Orientation for 8-pin Safe Torque-off Connector

Wire the Safe Torque Off
Circuit
This section provides guidelines for wiring your Kinetix 5100 Safe Torque Off
(STO) drive connections.
SB–
S1C
S2C
SS–
SB+
S1
S2
SS+

> **Table 128** — Kinetix 5100 Drive Safe Torque Off Connector Pinout

Description
 Signal
Used for safety jumper +
SB+
Used for safety jumper –
SB–
STO_A+
S1
STO_A–
S1C
STO_B+
S2
STO_B–
S2C
Safety status or feedback
SS+
Safety status or feedback
SS–
IMPORTANT
Use pins SB+ and SB– only for the by-pass jumpers to defeat the Safe
Torque Off function. When the Safe Torque-off function is in
operation, the 24V supply must come from an external source.
IMPORTANT
The National Electrical Code and local electrical codes take
precedence over the values and methods provided.
IMPORTANT
To improve system performance, run wires and cables in the
wireways as established in Establish Noise Zones beginning on
page 43.
IMPORTANT
Pins SB+ and SB– are used to disable the safe torque-off function.
When wiring to the STO connector, use an external 24V supply for the
external safety device that triggers the safe torque-off request. To
avoid jeopardizing system performance, do not use pin SB+ as a
power supply for the external safety device.

<!-- page 7 -->

## Safe Torque Off Wiring Requirements

The Safe Torque Off (STO) connector uses spring tension to secure the wire.
Depress the orange tab along side each pin to insert or release the wire. Wire
must be copper with 75 °C (167 °F) minimum rating.

> **Figure 225** — Safe Torque Off Terminal Plug

## Safe Torque Off Feature

The Safe Torque Off (STO) circuit, when used with suitable safety components,
provides protection according to ISO 13849-1 (PLd), Category 3 or according to
IEC 61508, IEC 61800-5-2, and IEC 62061 (SIL CL2). All components in the
system must be chosen and applied correctly to achieve the desired level of
operator safeguarding.
The Safe Torque Off circuit is designed to safely turn off all of the outputpower transistors. You can use the Safe Torque Off circuit in combination with
other safety devices to achieve Stop Category 0 and protection-against-restart
as specified in IEC 60204-1.
IMPORTANT
Stranded wires must terminate with ferrules to prevent short
circuits, per table D.4 of ISO 13849-2:2012.
SB–
S1C
S2C
SS–
SB+
S1
S2
SS+
Kinetix 5100 Servo Drive
Top View

> **Table 129** — Safe Torque Off Terminal Plug Wiring

Signal
Recommended Wire Size
mm2 (AWG)
Strip Length
mm (in.)
Torque Value
N•m (lb•in)
SB+
SBS1
S1C
S2
S2C
SS+
SS0.75 (18))
(stranded wire with ferrule)
1.5 (16)
(solid wire)
8.0 (0.31)
N/A (1)
(1)
This connector uses spring tension to hold the wires in place.
ATTENTION: This option is suitable only for performing mechanical work on
the drive system or affected area of a machine. It does not provide
electrical safety.
SHOCK HAZARD: In Safe Torque Off mode, hazardous voltages can still be
present at the drive. To avoid an electric shock hazard, disconnect power to
the system and verify that the voltage is zero before performing any work on
the drive.

<!-- page 8 -->

## Safe Torque Off Feature Bypass

The 2198-Exxxx-ERS drives do not operate without a safety circuit or safety
bypass wiring. For applications that do not require the Safe Torque Off (STO)
feature you must install jumper wires (included with the drive) to bypass the
safe torque-off circuitry.
Each 2198-Exxxx-ERS drive includes one 8-pin wiring plug for wiring to safety
devices. Jumper wires are installed by default to bypass the safety function, as
shown in Figure 226. With the jumper wires installed, the Safe Torque Off
feature is not used.

> **Figure 226** — Safe Torque Off Bypass Wiring

Cascade the Safe Torque Off Signal
The total number of drives in a single cascaded safety circuit is limited by the
current carrying capacity of the cascaded safety circuit. Refer to Table 130 for
current rating per channel, per drive.

> **Figure 227** — Cascaded Safe Torque Off Wiring

ATTENTION: Personnel responsible for the application of safety-related
programmable electronic systems (PES) shall be aware of the safety
requirements in the application of the system and shall be trained in using
the system.
SB+
S1
S2
SS+
SB–
S1C
S2C
SS–
24V DC
SB+
S1
S2
SS+
SB–
S1C
S2C
SS–
SB+
S1
S2
SS+
SB–
S1C
S2C
SS–
SB+
S1
S2
SS+
SB–
S1C
S2C
SS–
First Drive
Middle Drive
Last Drive
Dual-channel
Equivalent
Safety Device

<!-- page 9 -->

## Safe Torque Off

Specifications
To maintain the safety rating, Kinetix 5100 drives must be installed inside
protected control panels or cabinets appropriate for the environmental
conditions of the industrial location. The protection class of the panel or
cabinet must be IP54 or higher.

## Safe Torque Off Wiring

Diagrams
This section provides a typical wiring diagram for the Kinetix 5100 Safe Torque
Off (STO) feature with other Allen-Bradley® safety products.
For additional information regarding Allen-Bradley safety products, including
safety relays, light curtain, and gate interlock applications, see the Safety
Components, webpage https://www.rockwellautomation.com/en_NA/
products/safety-components/overview.page.
The drive is shown in a single-axis relay configuration for Stop Category 0 per
IEC-60204-1 Safety of Machinery Directive. This is an example, however, and
your application can differ based on the required overall machine performance
level requirements.

> **Table 130** — Safe Torque Off Signal Specifications

Attribute
Value
Safety inputs
(per channel)
Input ON voltage
11…30V DC
Input OFF voltage, max
5V DC
Input ON current, per input, max 7.34 mA
Input OFF current, max
(@ V in < 5V DC)
2.9 mA
Pulse rejection width
60 µs
Feedback output OFF current,
max
100 µA
Feedback output ON current,
max
40 mA
Feedback output OFF voltage,
max
30V DC
Feedback output ON voltage,
max
1.5 V @ 40 mA
External power supply
SELV/PELV
Input type
Optically isolated and reverse voltage protected
IMPORTANT
The Kinetix 5100 drive has been qualified and rated as a component
to meet ISO 13849-1 performance level d (PLd), category 3.
It is suggested to evaluate the entire machine performance level
required with a risk assessment and circuit analysis. Contact your
local distributor or Rockwell Automation Sales for more information.

<!-- page 10 -->

> **Figure 228** — Single-axis Relay Configuration (Stop Category 0)

(1)
Bypass Jumper is removed from SB+ and SB-.
Sinking output status is true (=1) when the drive displays E 500 status (SS+ and SS- are closed).
In this example, the drive is shown with two axes configuration in a relay
configuration for Stop Category 0 per IEC-60204-1 Safety of Machinery
Directive.

> **Figure 229** — Multiple-axis Relay Configuration (Stop Category 0)

(1)
Bypass Jumper is removed from SB+ and SB-.
Sinking output status is true (=1) when the drive displays E 500 status (SS+ and SS- are closed)
SS+/SS–
S1
S1C/S2C
S2
S21
S22
S34
A2

A1
S11
S52
S12

Kinetix 5100 Drive
STO (1)
Connector with
Wiring Header
External 24V COM
STO Demand
External +24V DC
Allen-Bradley
Monitoring Safety Relay
MSR127RP (440R-N23135)
IMPORTANT
Reset of the STO fault is required via digital input DI.ARST or
raC_xxx_5100_MAFR instruction.
SS+/SS–
S1
S1C/S2C
S2
S21
S22
S34
A2

A1
S11
S52
S12

SS+/SS–
S1
S1C/S2C
S2
Kinetix 5100 Drive
STO (1)
Connector with
Wiring Header
External 24V COM
STO Demand
External +24V DC
Allen-Bradley
Monitoring Safety Relay
MSR127RP (440R-N23135)
Kinetix 5100 Drive
STO (1)
Connector with
Wiring Header

**Extracted table (page 10, #1):**

| A1 | S11 | S52 | S12 | 13 | 23 | 33 | 41 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Allen-Bradley Monitoring Safety Relay SR127RP (440R-N23135) |  |  |  |  |  |  |  |
| S21 | S22 | S34 | A2 | 14 | 24 | 34 | 42 |

**Extracted table (page 10, #2):**

| SS | +/SS– |
| --- | --- |
| S1 |  |
| S1C | /S2C |
| S2 |  |

**Extracted table (page 10, #3):**

| A1 | S11 | S52 | S12 | 13 | 23 | 33 | 41 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Allen-Bradley Monitoring Safety Relay MSR127RP (440R-N23135) |  |  |  |  |  |  |  |
| S21 | S22 | S34 | A2 | 14 | 24 | 34 | 42 |

**Extracted table (page 10, #4):**

| SS | +/SS– |
| --- | --- |
| S1 |  |
| S1C | /S2C |
| S2 |  |

**Extracted table (page 10, #5):**

| SS | +/SS– |
| --- | --- |
| S1 |  |
| S1C | /S2C |
| S2 |  |

<!-- page 11 -->

.
In this example, the drive is shown in a single-axis relay configuration for Stop
Category 1 per IEC-60204-1 Safety of Machinery Directive.

> **Figure 230** — Single-axis Relay Configuration (Stop Category 1) - PR Operation Mode

(1)
Bypass Jumper is removed from SB+ and SB-.
Sinking output status is true (=1) when the drive displays E 500 status (SS+ and SS- are closed).
(2)
You can use the 'Servo on with Holding Brake' input as well, depending on the timing required and if your load uses a holding
brake. See Table 71 on page 161 for detailed descriptions.
The MotorStopMode parameter is used to determine the type of stop in the
drive. Upon removal of digital input Servo On, the drive behavior in Table 131
executes.

> **Figure 231** — Digital Input Servo On

> **Figure 232** — MotorStopMode Setting

IMPORTANT
Reset of the STO fault is required via digital input DI.ARST or
raC_Dvc_5100_MAFR instruction.
SS+/SS–
S1
S1C/S2C
S2
Servo On
DCOM

A1
S52
S11
S12
S21
S22
S33
S34

A2
X1
X2
X3
X4
Y39
Y40
Y2
Y1

## Allen-Bradley Monitoring Safety Relay

MSR138.1DP (440R-M23088)
External 24V COM
STO Demand
External +24V DC
Reset
I/O Connector with
2198-TBIO Expansion Block
Kinetix 5100 Drive
STO (1)
Connector with
Wiring Header
 (2)

> **Table 131** — MotorStopMode Settings in Drive Firmware

## ID675 (P1.032) MotorStopMode Setting

Drive Behavior
0000 (default)
Dynamic brake stop
0010
Disable and coast
0020
Dynamic brake stop first, when motor speed is slower than ID145
(P1.038), then coast stop
0030
Ramped decel

**Extracted table (page 11, #1):**

| A1 | S52 |  | S11 | S12 | S21 | S22 |  | S33 | S34 | 13 | 23 | 37 | 47 | 55 |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Allen-Bradley Monitoring Safety Relay MSR138.1DP (440R-M23088) |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
| A2 | X1 |  | X2 | X3 | X4 | Y39 | Y40 | Y2 | Y1 | 14 | 24 | 38 | 48 | 56 |  |

**Extracted table (page 11, #2):**

|  | SS | +/SS– |
| --- | --- | --- |
|  | S1 |  |
|  | S1 | C/S2C |
|  | S2 |  |

<!-- page 12 -->

The Safety Relay output (14) in Figure 230 is wired to a 24V DC input module in
the controller and evaluated in the logic. The Motion Operation Add-On
Instructions are used to stop and disable the motor before the Safety Relay
Dwell Time expires.

> **Figure 233** — Single Axis Timing Diagram: Category 1 - Using IO Operation Mode

## STO Active Alarm

(Sts = 1280)
raC_xxx_K5100_MAFR
Safe Torque Off Inputs
(38+48)
raC_xxx_K5100_MSO
_Drive01_CtrlSts_Active
STO Request
Safety Relay Output (14)
Application Logic
raC_xxx_K5100_MAS
_Drive01_CtrlSts.ZeroSpee
Application Logic
raC_xxx_K5100_MSF
Safety Relay Delay
Timing
Stopping
Safety Relay Dwell Time
Brake Engage Timing
Brake Release Timing

<!-- page 13 -->

Notes:
