# Chapter 3: Connector Data and Feature Descriptions

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `04_Ch03_Connector_Data.pdf` (29 pages)_

<!-- page 1 -->

## Kinetix 5100 Connector Data

Use these illustrations to identify the connectors and indicators for
Kinetix 5100 servo drives.

> **Figure 18** — Features and Indicators

(catalog numbers 2198-E1004-ERS, 2198-E1007-ERS, and 2198-E1015-ERS)

## L1 L2 L3

STO
MFB
L1C L2C P1 P2 DC-

I/0
AUX
5100
NET
MOD
CHARGE
U V W
DC+ ISH ESH
Kinetix 5100 Drive, Bottom View
(2198-E1004-ERS drive is shown)
Kinetix 5100 Drive, Front View
(2198-E1004-ERS drive is shown)
Kinetix 5100 Drive, Top View
(2198-E1004-ERS drive is shown)

> **Table 13** — Features and Indicators Description

Item
Description
Item
Description

Status display

Motor cable ground plate

Navigation push buttons

Safe torque-off (STO) connector

Module, Network, and Charge status indicators

Mains input power connector

Mini USB connector

• Control power input (L1C and L2C) connections
• Reserved (P1, P2, and negative DC-bus are not used)

Ethernet (PORT2) RJ45 connector

Motor feedback (MFB) connector

Ethernet (PORT1) RJ45 connector

Motor power output terminals

I/O signal connector

Shunt resistor terminals

Auxiliary feedback (AUX) connector

<!-- page 2 -->

> **Figure 19** — Features and Indicators (catalog numbers 2198-E1020-ERS, 2198-E2030-ERS,

2198-E4004-ERS, 2198-E4007-ERS, 2198-E4015-ERS)

> **Figure 20** — Features and Indicators (catalog numbers 2198-E4020-ERS, 2198-E4030-ERS)

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
STO
MFB

## Kinetix 5100 Drive, Bottom View

(2198-E1020-ERS drive is shown)
Kinetix 5100 Drive, Front View
(2198-E1020-ERS drive is shown)
Kinetix 5100 Drive, Top View
(2198-E1020-ERS drive is shown)
For feature descriptions see table on page 52.

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

STO

## Kinetix 5100 Drive, Bottom View

(2198-E4020-ERS drive is shown)
Kinetix 5100 Drive, Front View
(2198-E4020-ERS drive is shown)
Kinetix 5100 Drive, Top View
(2198-E4020-ERS drive is shown)
For feature descriptions see table on page 52.

<!-- page 3 -->

> **Figure 21** — Features and Indicators (catalog numbers 2198-E2055-ERS, 2198-E2075-ERS,

2198-E2150-ERS, 2198-E4055-ERS, 2198-E4075-ERS, and 2198-E4150-ERS)
Safe Torque-off Connector Pinout
The hardwired safe torque-off (STO) connector pinouts apply to all
Kinetix 5100 servo drives. For feature descriptions and wiring information,
refer to Chapter 13 beginning on page 415.

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
STO
MFB
Kinetix 5100 Drive, Bottom View
(2198-E2055-ERS drive is shown)
Kinetix 5100 Drive, Front View
(2198-E2055-ERS drive is shown)
Kinetix 5100 Drive, Top View
(2198-E2055-ERS drive is shown)

> **Table 14** — Features and Indicators Description

Item
Description
Item
Description

Status display

Safe torque-off (STO) connector

Navigation push buttons

Mains input power terminals

Module, Network, and Charge status indicators

Control power input terminals (1)

Mini USB connector

Motor feedback (MFB) connector

Ethernet (PORT2) RJ45 connector

Motor power output terminals

Ethernet (PORT1) RJ45 connector

Shunt resistor terminals

I/O signal connector

Reserved (P1, P2, and negative DC-bus are not used)

Auxiliary feedback (AUX) connector

Cooling fans

Motor cable ground plate

Protective cover
(1)
Control power terminals are labeled L1C/L2C for 2198-1xxx-ERS and 2198-2xxx-ERS (200V-class) drives and 24V+/24V- for 2198-4xxx-ERS (400V-class) drives.

<!-- page 4 -->

## Power Connector Pinouts

Catalog numbers 2198-E1004-ERS, 2198-E1007-ERS, and 2198-E1015-ERS have
connector plugs on the top and bottom of the drive for power connections.

> **Table 15** — AC Input Power Connector Pinouts

Signal
Description
L1
AC power in - L1 phase
L2
AC power in - L2 phase
L3
AC power in - L3 phase

> **Table 16** — Control AC Input Power Connector Pinout

Signal
Description
L1C
Control AC power in - L1C phase
L2C
Control AC power in - L2C phase
P1
Reserved (not used) (1)
(1)
P1 and P2 jumper is applied (default) at the factory. Do not remove jumper.
P2
DC–
Negative DC bus

> **Table 17** — Shunt Resistor Connector Pinout

Signal
Description
DC+
Positive DC bus
ISH
Internal shunt connection (1)
(applies to only 2198-E1004-ERS, 2198-E1007-ERS,
and 2198-E1015-ERS drives)
(1)
For internal shunt, keep jumper applied between DC+ and ISH (default). Remove jumper and
connect external shunt between DC+ and ESH.
ESH
External shunt connection (applies to all drives)

> **Table 18** — Motor-Power Connector Pinout

Signal
Description
U
Motor power out - U phase
V
Motor power out - V phase
W
Motor power out - W phase
L1 L2 L3
 L1C L2C P1 P2 DCESH
DC+ ISH
U V W

<!-- page 5 -->

Catalog numbers 2198-E1020-ERS, 2198-E2030-ERS, 2198-E2055-ERS,
2198-E2075-ERS, 2198-E2150-ERS, and 2198-E4xxx-ERS have power
connections on the I/O terminal block on the front of the drive.

> **Figure 22** — Power Pinouts on I/O Terminal Block

For connector pinout descriptions, see Table 13 and Table 14 beginning on
page 50.
The 2198-E2055-ERS, 2198-E2075-ERS, and 2198-E2150-ERS, 2198-E4020-ERS,
2198-E4030-ERS, 2198-E4055-ERS, 2198-E4075-ERS, and 2198-E4150-ERS
drives do not include an internal shunt resistor. However, an external shunt
resistor can be connected to the DC+ and ESH terminals.
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
24V+
24V–
I/O Terminal Block for 2198-E2055ERS, 2198-E2075-ERS, and 2198E2150-ERS, Servo Drives
I/O Terminal Block for
2198-E1020-ERS, and 2198-E2030-ERS
Servo Drives
I/O Terminal Block for
2198-E4004-ERS,
2198-E4007-ERS,
2198-E4015-ERS
Servo Drives(a)
I/O Terminal Block for
2198-E4020-ERS,
2198-E4030-ERS,
2198-E4055-ERS,
2198-E4075-ERS, and
2198-E4150-ERS
Servo Drives(a)
(a) P1 and P2 jumper is applied
(default) at the factory. Do not

> **Table 19** — Control Input Power Connector Pinout - (400V-Class Drives)

Signal
Description
24V+
Control 24V+ DC
24VControl 24V- DC common

> **Table 20** — Shunt Resistor Connector Pinout

Signal
Description
DC+
Positive DC bus
ISH
Internal shunt connection (1)
(applies to only 2198-E1020-ERS, 2198-E2030-ERS, 2198-E4004-ERS, 2198-E4007-ERS,
and 2198-E4015-ERS drives)
(1)
For internal shunt, keep jumper applied between DC+ and ISH (default). Remove jumper and connect external shunt between
DC+ and ESH.
ESH
External shunt connection (applies to all drives)

<!-- page 6 -->

> **Figure 23** — Pin Orientation for 50-pin SCSI I/O Connector

> **Table 21** — I/O Connector Pinout

## I/O Pin

Signal
Description
I/O Pin
Signal
Description

OUTPUT4+
Digital output 4+

OUTPUT4–
Digital output 4–

OUTPUT3–
Digital output 3–

OUTPUT5–
Digital output 5–

OUTPUT3+
Digital output 3+

OUTPUT5+
Digital output 5+

OUTPUT2–
Digital output 2–

INPUT9
Digital input 9 (high speed)

OUTPUT2+
Digital output 2+

INPUT8
Digital input 8

OUTPUT1–
Digital output 1–

INPUT7
Digital input 7

OUTPUT1+
Digital output1+

INPUT6
Digital input 6

INPUT4
Digital input 4

INPUT5
Digital input 5

INPUT1
Digital input 1

INPUT3
Digital input 3

INPUT2
Digital input 2

BPWR
External power input of BX+/BX– for single-end operation

DCOM
Common for digital inputs, connected to +24 or 0V DC

BX+
Pulse input B+/DIR+/CCW+

AGND
Analog input signal ground

BX–
Pulse input B–/DIR–/CCW–

AGND
Analog input signal ground

INPUT10
Digital input 10 (high speed)

—
Reserved (1)

APWR
External power input of AX+/AX– for single-end operation

AOUT2
Analog monitor output 2

OUTPUT6–
Digital output 6–

AOUT1
Analog monitor output 1

AX–
Pulse input A–/Step–/CW–

—
Reserved (1)

COMMAND2
Analog position or speed command input

COMMAND1
Analog torque input

AX+
Pulse input A+/Step+/CW+

AGND
Analog input signal ground

AGND
Analog input signal ground

—
Reserved (1)

—
Reserved (1)

AMOUT+
Buffered encoder output Ch A+

OUTPUT6+
Digital output 6+

AMOUT–
Buffered encoder output Ch A–

—
Reserved (1)

BMOUT–
Buffered encoder output Ch B–

OCZMOUT
Buffered Encoder Output Ch Z open collector

ZMOUT–
Buffered encoder output Ch Z–

—
Reserved (1)

BMOUT+
Buffered encoder output Ch B+

ZMOUT+
Buffered encoder output Ch Z+
Drain wire
(1)
The reserved pins are not present on the 2198-TBIO terminal expansion block.

<!-- page 7 -->

> **Figure 24** — Pin Orientation for 15-pin Motor Feedback (MFB) Connector

Auxiliary Feedback Connector Pinout

> **Figure 25** — Pin Orientation for Auxiliary Feedback (AUX) Connector

> **Table 22** — Motor Feedback (MFB) Connector Pinout

## MFB Pin

Signal
Description
MFB Pin
Signal
Description

SIN+
AM+
Sine differential input+
AM+ differential input+

–
Reserved

SIN–
AM–
Sine differential input–
AM– differential input–

DATA–
IM–
Data differential input –
Index pulse–

COS+
BM+
Cosine differential input+
BM+ differential input+

TS
Motor thermal switch (normally closed) (1)

COS–
BM–
Cosine differential input–
BM– differential input–

S1
Single-ended 5V Hall effect commutation

DATA+
IM+
Data differential input +
Index pulse+

S2
Single-ended 5V Hall effect commutation

ECOM
Common

EPWR_5V (2)
Encoder power (+5V)

EPWR_9V (2)
Encoder power (+9V)

–
Reserved

S3
Single-ended 5V Hall effect commutation
(1)
Not applicable unless motor has integrated thermal protection.
(2)
Determine which power supply your encoder requires and connect to only the specified supply. Do not make connections to both.
ATTENTION: The motor feedback will determine which encoder power
source is used. Be sure you use the correct power source for your encoder
to avoid equipment damage.
IMPORTANT
For the maximum length of the drive to motor power and feedback
cable, see Maximum Cable Length on page 95. System performance
was tested at these specifications and also applies when meeting CE
and UK requirements.
Pin 11
Pin 6
Pin 15
Pin 1
Pin 10
Pin 5
Pin
Signal
Description
Pin
Signal
Description

AM+
Channel A Differential Input +

IM–
Channel Index Differential Input -

AM–
Channel A Differential Input -

ECOM
Encoder Common

BM+
Channel B Differential Input +

EPWR5V
Encoder 5V Power Output

BM–
Channel B Differential Input -

Reserved
Reserved

IM+
Channel Index Differential Input +

Reserved
Reserved

View from rear of
connector kit.
Soldered Pins
Front of
Connector Kit

<!-- page 8 -->

Ethernet Communication Connector Pinout

> **Figure 26** — Pin Orientation for 8-pin Ethernet Communication Port

## Control Signal

Specifications
This section provides a description of the Kinetix 5100 drive digital I/O, analog
outputs, Ethernet communication, motor brake circuitry, and control power
current specifications.
Digital Inputs
The Kinetix 5100 drive supports ten customer-defined digital input (DI) points
to provide maximum flexibility. All inputs are configurable with KNX5100C
software.
Registration inputs can only be assigned to high-speed inputs as shown in
Table 23.
The default input configuration is disabled for all modes. Assignments can be
changed via KNX5100C software > Digital IO/Jog Control in the Function List.
The digital input functions are defined in Description of Digital Input
Functions on page 433. If the defined digital input function needs to change to
meet your application requirements, you can change the functions by using a
PR Write to Parameters program type for the function of INPUT1…INPUT10 by
using the corresponding parameters listed in Table 24.
Port 1 Pin
Signal
Description
Port 1 Pin
Signal
Description

+ TX
Transmit Port (+) Data Terminal

–
–

– TX
Transmit Port (–) Data Terminal

– RX
Receive Port (–) Data Terminal

+ RX
Receive Port (+) Data Terminal

–
–

–
–

–
–

> **Table 23** — Digital Input Assignments

## Digital Input

Function
Function
INPUT1
Digital input 1
User configurable, excluding registration
INPUT2
Digital input 2
INPUT3
Digital input 3
INPUT4
Digital input 4
INPUT5
Digital input 5
INPUT6
Digital input 6
INPUT7
Digital input 7
INPUT8
Digital input 8
INPUT9
Digital input 9 (high speed)
User configurable, including registration
INPUT10
Digital input 10 (high speed)

<!-- page 9 -->

Wiring and Signal Specifications
The digital inputs are optically isolated and sink up to 24V DC. Electrical
details are shown in Table 25. You can configure the inputs for PNP sourcing or
NPN sinking.

> **Figure 27** — Digital Input Circuitry

> **Table 24** — DigitaI Input Signal Parameters

Signal
Pin
Configuration
Parameter
Signal
Pin
Configuration
Parameter
INPUT1

ID195 (P2.010)
INPUT6

ID200 (P2.015)
INPUT2

ID196 (P2.011)
INPUT7

ID201 (P2.016)
INPUT3

ID197 (P2.012)
INPUT8

ID202 (P2.017)
INPUT4

ID198 (P2.013)
INPUT9

ID220 (P2.036)
INPUT5

ID199 (P2.014)
INPUT10

ID221 (P2.037)
24V DC
INPUTx
DCOM
4.7 kΩ,
approx.
24V DC
INPUTx
DCOM
4.7 kΩ,
approx.
Servo Drive
Servo Drive
NPN Transistor (Source mode)
PNP Transistor (Sink mode)

<!-- page 10 -->

## Digital Input Wiring

See Digital Inputs on page 57 for the default digital input assignments for
Kinetix 5100 drives.
In this example, Servo On is assigned to digital input 1 as a sinking type input.

> **Figure 28** — Digital Input Example

> **Table 25** — Digital Input Specifications

Attribute
Value
Digital input response (delay)
• Standard inputs: 1.25 ms, max
• High speed inputs: 3 µs
Digital inputs scan time
• Standard inputs: 500 µs, max
• High speed inputs: 1 µs
Type
Current sourcing and current sinking (IEC61131-2 Type 1)
Dedicated functions
• Standard inputs: INPUT1…INPUT8 and DCOM.
• High speed inputs (registration inputs): INPUT9, INPUT10, and
DCOM.
• When configured as Disabled, inputs can be used by
programs as a programming condition.
Only one function at a time is possible.
Input current (with 26.4V applied)
6 mA, max
ON state voltage
15…26.4V
OFF state voltage
–1.0…5.0V
Pulse reject filtering (all digital inputs)
0.5 µs
Propagation delay (registration functions)
3 µs
Registration accuracy
3 µs
Registration repeatability
1 µs
24V DC
Supply
Servo On (INPUT1)
DCOM
+24V DC
–24V COM

2198-Exxxx-ERS
Kinetix 5100 Servo Drive
I/O Connector with
2198-TBIO Expansion Block

<!-- page 11 -->

## Digital Outputs

The Kinetix 5100 drives support six customer-defined digital output (DO)
points to provide maximum flexibility. OUTPUT1…OUTPUT6 are available on
the 2198-TBIO connector. Outputs are optically isolated open-collector/emitter
and are fully isolated from the drive circuits. Each output,
OUTPUT1…OUTPUT6, is disabled for all modes by default.
The digital output functions are defined in Description of Digital Output
Functions on page 437.
If the defined digital output function needs to change to meet your application
requirements, you can use a PR Write to Parameters program type to change
the function of OUTPUT1…OUTPUT6 by using the corresponding parameters
listed in Table 26.

> **Table 26** — Digital Output Signal Parameters

Signal
Pin
Configuration
Parameter
Signal
Pin
Configuration
Parameter
OUTPUT1+

ID203
(P2.018)
OUTPUT4+

ID206
(P2.021)
OUTPUT1-

OUTPUT4-

OUTPUT2+

ID204
(P2.019)
OUTPUT5+

ID207
(P2.022)
OUTPUT2-

OUTPUT5-

OUTPUT3+

ID205
(P2.020)
OUTPUT6+

ID225
(P2.041)
OUTPUT3-

OUTPUT6-

<!-- page 12 -->

Wiring and Signal Specifications
The digital outputs are optically isolated and sink up to 24V DC. Electrical
details are shown in Table 27.

> **Figure 29** — Digital Output Circuitry

OUTPUTx–
OUTPUTx+
24V DC
R
OUTPUTx–
OUTPUTx+
24V DC
In this example, the drive applies the external
24V DC power supply to a resistive load.
In this example, the drive applies the external
24V DC power supply to an inductive load.
Servo Drive
Servo Drive

> **Table 27** — Digital Output Signal Specifications

Parameter
Description
Min
Max
ON state current
Current flow when the output transistor is ON
–
40 mA
OFF state current
Current flow when the output transistor is OFF
–
0.1 mA
ON state voltage
Voltage across the output transistor when ON
–
1.5V @ 40 mA
OFF state voltage
Voltage across the output transistor when OFF
–
30V
Scan time
Interval of the digital outputs status updating in
drive firmware
–
250 µs
Pass through delay
Signal propagation delay from the firmwareaccessible registers to the digital output
–
1.0 ms

**Extracted table (page 12, #1):**

| Description | Min |
| --- | --- |
| Current flow when the output transistor is ON | – |
| Current flow when the output transistor is OFF | – |
| Voltage across the output transistor when ON | – |
| Voltage across the output transistor when OFF | – |
| Interval of the digital outputs status updating in drive firmware | – |
| Signal propagation delay from the firmware- accessible registers to the digital output | – |

<!-- page 13 -->

## Digital Output Wiring

In this example, digital output 1 (pin 7+, pin 6–) is connected to an output relay
that changes a contact state used in a PLC or other circuit as shown.

> **Figure 30** — Digital Output Example

(1)
Customer-supplied diode or MOV suppression device.
The I/O connector provides up to six digital outputs. Digital outputs are opencollector type and are configurable with KNX5100C software.
An example brake circuit contains the following components:
•
Digital output 40 mA (max) continuous current.
•
Relay 700-HK36Z24 with DIN mount 700-HN121 or equivalent
•
Choose from these suppression devices:
- 1N4004 diodes or equivalent
- Bulletin 199-MSMV1 MOV or equivalent
See Digital Outputs on page 60 for the default digital output assignments for
Kinetix 5100 drives.

OUTPUT1+
OUTPUT1–
2198-Exxxx-ERS
Kinetix 5100 Servo Drive
I/O Connector with
2198-TBIO Expansion Block
Relay
(9)
page 45
To PLC or
Other Circuit
Customer Supplied
+24V DC
(1)
Choose a relay rated for 40 mA continuous current or less.

<!-- page 14 -->

## Analog Inputs

There are two analog inputs, COMMAND1 and COMMAND2, available on the
I/O connector. When the drive mode is configured for Speed or Torque, the
analog inputs are used for Torque and Speed commands.

> **Figure 31** — Analog COMMAND Input Configuration

## Pulse Inputs

There are pulse inputs available on the 2198-TBIO connector. They support
either single-ended or differential pulse signals. When using the single-ended
signals, they can be wired as current sinking (PNP) or sourcing (NPN) inputs.

> **Figure 32** — Pulse Input - Single-ended Configuration (current sourcing)

> **Table 28** — Analog Input Specifications

Parameter
Description
Analog inputs voltage
–10 V… +10 Vs
Analog inputs resolution
11 bits, min
Analog inputs scan time
0.0625 ms, max
Analog inputs impedance
12 kΩ typical, approx.
AGND
±10V
10 kΩ

COMMAND1-Torque
COMMAND2-Speed
Analog GND
Servo Drive
Controller
I/O Connector with
2198-TBIO Expansion Block
DC
24V
1 .5KΩ
51 Ω
51 Ω
51 Ω
1.5KΩ
Pulse input frequency (max):
200 kHz
51 Ω

Pulse input frequency (max):
200 kHz

## Pulse B+

BPWR
Pulse B–
APWR
Pulse A+
Pulse A–
Servo Drive
Controller
I/O Connector with
2198-TBIO Expansion Block

**Extracted table (page 14, #1):**

|  |  |  |  | C | OMMAND1-Torque |
| --- | --- | --- | --- | --- | --- |
|  |  |  | 42 13 | C | OMMAND2-Speed |
|  |  |  |  | A | nalog GND |

<!-- page 15 -->

> **Figure 33** — Pulse Input - Single-ended Configuration (current sinking)

In Differential mode, the pulse input (line driver) only accepts 2.8…3.6V DC
(5V DC nominal). Do not apply 24V power.

> **Figure 34** — Pulse Input (line driver) Configuration

51 Ω
51Ω
51 Ω
51Ω
24V DC
+
-
.5 kΩ
1.5kΩ
Pulse input frequency (max):
200 kHz
Pulse input frequency (max):
200 kHz

BPWR

## Pulse B–

APWR
Pulse A–
Servo Drive
Controller
I/O Connector with
2198-TBIO Expansion Block
51Ω
51 Ω
51Ω
51Ω
/SIGN
/PULSE
SIGN

PULSE

Pulse input frequency (max):
4 MHz
Pulse input frequency (max):
4 MHz
Pulse B+
Pulse B–
Pulse A+
Pulse A–
Servo Drive
Controller
I/O Connector with
2198-TBIO Expansion Block

**Extracted table (page 15, #1):**

|  | B |
| --- | --- |
| 37 P |  |
| 39 |  |
|  | A |

**Extracted table (page 15, #2):**

|  |  |  | /SIGN 37 |  | Pulse B+ |  |
| --- | --- | --- | --- | --- | --- | --- |
|  |  | PULSE 43 |  |  |  |  |
|  |  |  | /PULSE 41 |  | Pulse A+ 51Ω Pulse A– |  |

<!-- page 16 -->

## Analog Outputs

There are two analog outputs, AOUT1 and AOUT2, available on the I/O
connector. Assignments are changed via KNX5100C software > Function List >
Analog IO > Output Monitor.

> **Figure 35** — Analog Output Circuitry

## Buffered Encoder Outputs

Encoder output signals can be connected to the receiving device with line
receiver (differential) or opto-coupler isolated inputs. The encoder output
signals are flexible. The signals are scaled and programmed by using
KNX5100C software > Function List > Pulse Output.

> **Table 29** — Analog Output Specifications

Parameter
Description
Analog outputs voltage
–8V… +8V DC or -10V…+10VDC, user configurable
Analog outputs resolution
10 bits, min
Analog outputs current
1 mA, max
Analog outputs scan time
0.25 ms, max
AGND
24 kΩ
Output:
1 mA, max
8 kΩ
8V
Full-scale
V

AOUT1
AOUT2
Analog GND
Servo Drive
Controller
I/O Connector with
2198-TBIO Expansion Block

**Extracted table (page 16, #1):**

| AOUT | 1 | 15 13 |  |  |  |
| --- | --- | --- | --- | --- | --- |
| AOUT |  |  |  |  |  |
|  | 2 |  |  |  |  |
| Analog GN | D |  |  |  |  |

<!-- page 17 -->

> **Figure 36** — Encoder Output Position (line driver)

> **Figure 37** — Encoder Output Position (opto-isolator)

120 Ω
120 Ω
120 Ω

AMOUT+
AMOUT–
Output Current (max), 20 mA

BMOUT+
BMOUT–

ZMOUT+
ZMOUT–
Servo Drive
Receiving Device
I/O Connector with
2198-TBIO Expansion Block
I/O Connector with
2198-TBIO Expansion Block
I/O Connector with
2198-TBIO Expansion Block

AMOUT+
AMOUT–
Output Current (max), 20 mA

BMOUT+
BMOUT–

ZMOUT+
ZMOUT–
200 Ω
200 Ω
200 Ω
High Speed
Photo Coupler
High Speed
Photo Coupler
High Speed
Photo Coupler
Servo Drive
I/O Connector with
2198-TBIO Expansion Block
I/O Connector with
2198-TBIO Expansion Block
I/O Connector with
2198-TBIO Expansion Block
Receiving Device

<!-- page 18 -->

> **Figure 38** — Encoder OCA Output (open collector Z pulse output)

## Ethernet Communication Specifications

The PORT1 and PORT2 (RJ45) Ethernet connectors provide EtherNet/IP
communication.
24V

30V (max),
50 mA
OCZMOUT
Analog GND
AGND
Servo Drive
I/O Connector with
2198-TBIO Expansion Block

> **Table 30** — Ethernet Communication Specifications

Attribute
Value
Communication
The drive auto-negotiates Speed and Duplex modes. These
modes can be forced through the Logix Designer application.
100BASE-TX, full-duplex is recommended for maximum
performance.
Request Packet Interval (RPI)
2.0 ms, min (20 ms default)
Auto MDI/MDIX crossover detection/
correction
Yes
Cabling
CAT5e shielded, 100 m (328 ft), max

<!-- page 19 -->

## Motor Brake Circuit

The brake option is a motor mounted spring-set holding brake that releases
when voltage is applied to the brake coil in the motor. The customer-supplied
24V power supply drives the brake output through a relay.
Wire the Brake Control Circuit
One digital output can be used for motor brake control. In this example,
OUTPUT6 is used. Wire the brake control circuit according to the appropriate
interconnect diagram in Kinetix 5100 Drive/Rotary Motor Wiring Examples
beginning on page 464. An external customer-supplied 24V power supply is
required.

> **Figure 39** — Brake Control Circuit Example

(1)
Customer-supplied diode or MOV suppression device.
An example brake circuit contains the following components:
•
Digital output 40 mA (max) continuous current.
•
Relay 700-HK36Z24 with DIN mount 700-HN121 or equivalent
•
Suppression device examples include 1N4004 diode,
Bulletin 199-MSMV1 MOV, or equivalent
See Kinetix Rotary Motion Specifications Technical Data, publication
KNX-TD001, for coil current ratings and brake response times.

OUTPUT6+
OUTPUT6–
BR+
BR–
2198-Exxxx-ERS
Kinetix 5100 Servo Drive
I/O Connector with
2198-TBIO Expansion Block
Relay
Motor Brake
Connections
Customer Supplied
+24V DC
Servo Motor
(1)
(1)
Choose a relay rated for 40 mA continuous current or less.

<!-- page 20 -->

Configure the Brake Control Circuit
Follow these steps to configure brake control in KNX5100C software.
1.
Double-click Function List > Digital IO/Jog Control
2. Check Edit DIO configurations.
3.
From the Digital Output (DO) pull-down menu, choose Brake Control.
4. Verify that N.O. (normally open) is selected.
5.
Uncheck Edit DIO configurations.
6. Click Settings>General Setting and configure the brake response engage
and disengage delay times based on the motor selected.
For motor brake coil-current and response time specifications for all
Allen-Bradley® motor families, see Kinetix Rotary Motion Specifications
Technical Data, publication KNX-TD001.
Brake control is configurable in KNX5100C software. An active signal
releases the motor brake. Turn-on and turn-off delays are specified by
ID149 (P1.042) Disengage Delay Time and ID150 (P1.043) Engage Delay
Time parameter settings.
7.
In Brake Time Settings, enter ID149 (P1.042) Disengage Delay Time and
ID150 (P1.043) Engage Delay Time parameter values.
IMPORTANT
Holding brakes that are available on Allen-Bradley rotary
motors are designed to hold a motor shaft at 0 rpm for up to
the rated brake-holding torque, not to stop the rotation of the
motor shaft, or to be used as a safety device.
You must command the servo drive to 0 rpm and engage the
brake only after verifying that the motor shaft is at 0 rpm.

<!-- page 21 -->

8. Verify that the MotorStopMode ID675 (P1.032) parameter is set to 0000.
Refer to Parameter Editor screen General parameter group.
Motor Brake Control Operation
Brake control is automatic. Figure 40 shows the timing of the brake control in
two different scenarios. Below is a description of the brake control operation
shown in Figure 40:
•
Brake Disengage (physically release the brake)
When the Servo On condition is ON (digital input ‘Servo On’ activates or
Add-On Instruction command raC_xxx_k5100_MSO is issued), ID149
(P1.042) DisengageDelayTime begins timing. When this delay expires,
the brake output is set and motion can occur.
•
Brake Engage
This operation involves parameters ID145 (P1.038) ZeroSpeedWindow
rpm and ID150 (P1.043) EngageDelayTime.
ZeroSpeedWindow is a programmable value. When the motor speed
(rpm) is below the ZeroSpeedWindow value, the zero speed condition is
met.
When the Servo On condition is OFF (digital input 'Servo On' is removed,
Add-On Instruction command raC_xxx_k5100_MSF is issued, or the
drive faults), ID150 (P1.043) Engage Delay Time begins timing. The
ZeroSpeedWindow condition is actively evaluated. If the zero speed
condition occurs before the Engage Delay Time expires, the brake output
is OFF (scenario 2). If the zero speed condition is not met and the Engage
Delay Time expires, the brake output is OFF (scenario 1).
For vertical loads, MotorStopMode 0000 controls the motor to below the
ZeroSpeedWindow ID145 (P1.038) where the brake function executes (see
Figure 40).
IMPORTANT
For MPL-A/B15xxx and MPL-A/B2xxx motors when MotorStopMode is
set at 0000 or 0020 (dynamic brake is enabled), there is a risk that
these motors can demagnetize during the stop. For these motors, set
MotorStopMode at 0010 (disable and coast).
IMPORTANT
If the ZeroSpeedWindow and Brake Delay parameters are not
set correctly, the brake can set while the motor is in motion.

<!-- page 22 -->

> **Figure 40** — Brake Control Timing Diagram

For motor brake specifications to size the interposing relay, see Kinetix Rotary
Motion Specifications Technical Data, publication KNX-TD001. See the
interconnect diagram for your Kinetix 5100 drive/motor beginning on
page 464 for typical motor brake wiring.
More information on the Vertical Load Control setting (which populates
default brake settings) is found on page 161.
Control Power
Kinetix 5100 200V-class drives require 95…132V AC (120V nom) single-phase,
with 120V AC input power or 170…253V AC (200…230V nom) single-phase, with
200…230V AC input power.
ON
ON
OFF
OFF
OFF
OFF
Servo On condition
Brake Output (DO)
ZSPD ID145 (P1.038)
Motor Speed (rpm)
Disengage Delay Time ID149 (P1.042)
Engage Delay Time ID150 (P1.043)
ZSPD ID145 (P1.038)
Motor Speed (rpm)
Engage Delay
Time ID150 (P1.043)
Scenario 1
Scenario 2
Servo On Condition
Brake Output (DO)
ZSPD ID145 (P1.038)
Motor Speed (rpm)
ZSPD ID145 (P1.038)
Motor Speed (rpm)
Disengage Delay Time ID149 (P1.042)
Engage Delay Time ID150 (P1.043)
Engage Delay
Time ID150
(P1.043)

> **Table 31** — Control Power Specifications - 200V-class Drives

Kinetix 5100 (200V-class)
Drives Cat. No.
Input Current of Control Power
A rms at 120V rms, nom
Inrush Current of Control
Power, max
A 0-pk at 120V rms, nom
Input Current of Control Power
A rms at 230V rms, nom
Inrush current of
Control Power, max
A 0-pk at 230V rms, nom
2198-E1004-ERS
0.34
15.80
0.20
37.0
2198-E1007-ERS
0.38
18.20
0.22
37.40
2198-E1015-ERS
0.38
19.20
0.22
39.80
2198-E1020-ERS
0.63
19.20
0.35
32.40
2198-E2030-ERS
–
–
0.35
36.40
2198-E2055-ERS
–
–
0.46
32.80
2198-E2075-ERS
–
–
0.48
40.0
2198-E2150-ERS
–
–
0.92
37.0

**Extracted table (page 22, #1):**

| OFF |  |  | ON |  |  |  |  |  |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Disengage Delay Disengage Delay |  |  |  | e ID149 (P1.042) Engage Delay T e ID149 (P1.042) Engage Delay Scenario 1 |  |  |  |  |  |
|  | Disengage De Disengage De | lay lay | Tim Tim | e ID149 (P1.042) e ID149 (P1.042) | Engage Dela Engage Dela | y T y | ime Time | ID ID | 150 (P1.043) 150 (P1.043) |
|  |  |  |  | Scenario 2 |  |  |  |  |  |

**Extracted table (page 22, #2):**

| Input Current of Control Power A rms at 120V rms, nom | Inrush Current of Control Power, max A 0-pk at 120V rms, nom | Input Current of Control Power A rms at 230V rms, nom |
| --- | --- | --- |
| 0.34 | 15.80 | 0.20 |
| 0.38 | 18.20 | 0.22 |
| 0.38 | 19.20 | 0.22 |
| 0.63 | 19.20 | 0.35 |
| – | – | 0.35 |
| – | – | 0.46 |
| – | – | 0.48 |
| – | – | 0.92 |

<!-- page 23 -->

Kinetix 5100 400V-class drives require 21.6…26.4V DC (24v, nom) input control
power.
Feedback Specifications
The Kinetix 5100 drive uses the MFB connector for various types of motor
feedback. The AUX connector uses TTL incremental feedback only.
Use the 2198-K51CK-D15M feedback connector kit for terminating feedback
conductors when building your own cables.

> **Table 32** — Control Power Specifications - 400V-class Drives

Cat. No.
Maximum Input Current of
Control Power
A rms at 24V DC
Inrush current of Control
Power
A at 24V DC
2198-E4004-ERS
1.27
4.14
2198-E4007-ERS
2198-E4015-ERS
2198-E4020-ERS
1.40
4.97
2198-E4030-ERS
1.77
4.97
2198-E4055-ERS
2.03
3.24
2198-E4075-ERS
2198-E4150-ERS
4.43
3.40

> **Table 33** — Feedback General Specifications

Attribute
Motor Feedback
Auxiliary Feedback
Feedback device support
• Nikon (24-bit) serial (Kinetix TLP motors)
• Hiperface
• Tamagawa (17-bit) serial (Kinetix TL/TLY motors)
• Digital AqB with or without UVW, incremental
Digital AqB incremental
Power supply (EPWR5V)
5.09…5.41V, 300 mA, max
Power supply (EPWR9V)
8.3…9.9V, 150 mA, max
Motor thermostat
Single-ended input:
• Under 500 Ω = No Fault
• Over 10 kΩ = Fault

<!-- page 24 -->

Motor Feedback Supported by Using the MFB Connector
The Kinetix 5100 drive accepts motor feedback signals from Hiperface, Nikon,
Tamagawa, and TTL incremental encoders on the MFB connector.
The selected motor determines if the motor thermostat connections (MTR_TS)
are used.

> **Table 34** — Feedback Signals by Device Type

Pin
Hiperface
(all compatible motors
and actuators)
Nikon
(Kinetix TLP)
Tamagawa
(Kinetix TL/TLY-B)
Digital AqB with UVW
(all compatible motors
and actuators)
Incremental
(Kinetix TLY-H)
Generic TTL
Incremental
Generic Sine/Cosine

MTR_SIN+
–
–
MTR_AM+
MTR_AM+
MTR_AM+
MTR_SIN+

MTR_SIN–
–
–
MTR_AM–
MTR_AM–
MTR_AM–
MTR_SIN–

MTR_COS+
–
–
MTR_BM+
MTR_BM+
MTR_BM+
MTR_COS+

MTR_COS–
–
–
MTR_BM–
MTR_BM–
MTR_BM–
MTR_COS–

MTR_DATA+
MTR_T+
MTR_DATA+ (TLY-B)
MTR_SD+ (TL-B)
MTR_IM+
MTR_IM+
MTR_IM+
MTR_IM+

MTR_ECOM
MTR_ECOM
MTR_ECOM
MTR_ECOM
MTR_ECOM
MTR_ECOM
MTR_ECOM

MTR_EPWR9V (1)
–
–
–
–
–
–

–
–
–
MTR_S3
MTR_S3
MTR_S3
MTR_S3

–
–
–
–
–
–
–

MTR_DATA–
MTR_T–
MTR_DATA– (TLY-B)
MTR_SD– (TL-B)
MTR_IM–
MTR_IM–
MTR_IM–
MTR_IM–

MTR_TS
–
–
–
–
MTR_TS
MTR_TS

–
–
–
MTR_S1
MTR_S1
MTR_S1
MTR_S1

–
–
–
MTR_S2
MTR_S2
MTR_S2
MTR_S2

MTR_EPWR5V (1)
MTR_EPWR5V
MTR_EPWR5V
MTR_EPWR5V
MTR_EPWR5V
MTR_EPWR5V
MTR_EPWR5V

–
–
–
–
–
–
–
(1)
Determine which power supply your encoder requires and connect to that supply only. Do not make connections to both supplies.
ATTENTION: The motor feedback determines which encoder power source
is used. Be sure you use the correct power source for your encoder to avoid
equipment damage.

> **Table 35** — Hiperface Encoder Specifications

Attribute
Value
Protocol
Hiperface
Memory support
Encoders programmed with Allen-Bradley motor data
Hiperface data communication
RS-485, 9600 communication, 8 data bits, no parity
Sine/Cosine interpolation
2048 counts/sine period
Input frequency (AM/BM)
250 kHz, max
Input voltage (AM/BM)
0.6...1.2V, p-p, which is measured at the drive inputs
Line loss detection (AM/BM)
Average (sin2 + cos2) > constant

> **Table 36** — Nikon Encoder Specifications

Attribute
Value
Communication protocol
Proprietary format
Encoder nonvolatile memory usage
Programmed with Kinetix TLP motor data as Allen-Bradley memory
format
Differential input voltage
1.0…7.0V
Data communication
8 Mbps, 21 data bits with ECC, no parity
Battery type
3.6V, ER14252 or equivalent, 1/2AA size

**Extracted table (page 24, #1):**

| Hiperface (all compatible motors and actuators) | Nikon (Kinetix TLP) | Tamagawa (Kinetix TL/TLY-B) | Digital AqB with UVW (all compatible motors and actuators) | Incremental (Kinetix TLY-H) | Generic TTL Incremental |
| --- | --- | --- | --- | --- | --- |
| MTR_SIN+ | – | – | MTR_AM+ | MTR_AM+ | MTR_AM+ |
| MTR_SIN– | – | – | MTR_AM– | MTR_AM– | MTR_AM– |
| MTR_COS+ | – | – | MTR_BM+ | MTR_BM+ | MTR_BM+ |
| MTR_COS– | – | – | MTR_BM– | MTR_BM– | MTR_BM– |
| MTR_DATA+ | MTR_T+ | MTR_DATA+ (TLY-B) MTR_SD+ (TL-B) | MTR_IM+ | MTR_IM+ | MTR_IM+ |
| MTR_ECOM | MTR_ECOM | MTR_ECOM | MTR_ECOM | MTR_ECOM | MTR_ECOM |
| MTR_EPWR9V (1) | – | – | – | – | – |
| – | – | – | MTR_S3 | MTR_S3 | MTR_S3 |
| – | – | – | – | – | – |
| MTR_DATA– | MTR_T– | MTR_DATA– (TLY-B) MTR_SD– (TL-B) | MTR_IM– | MTR_IM– | MTR_IM– |
| MTR_TS | – | – | – | – | MTR_TS |
| – | – | – | MTR_S1 | MTR_S1 | MTR_S1 |
| – | – | – | MTR_S2 | MTR_S2 | MTR_S2 |
| MTR_EPWR5V (1) | MTR_EPWR5V | MTR_EPWR5V | MTR_EPWR5V | MTR_EPWR5V | MTR_EPWR5V |
| – | – | – | – | – | – |

<!-- page 25 -->

## Auxiliary Feedback Specifications

The Kinetix 5100 drives support TTL incremental feedback devices on the
10-pin auxiliary feedback connector (AUX). See Table 38 on page 74 for Digital
AqB encoder feedback specifications.

> **Table 40** — Auxiliary Feedback Signals by Device Type

> **Table 37** — Tamagawa Serial Specifications

Attribute
Value
Encoder nonvolatile memory usage
Programmed with TL-Axxxx-B and TLY-Axxxx-B motor data as
Allen-Bradley memory format.
Differential input voltage
1.0…7.0V
Data communication
2.5 Mbps, 8 data bits, no parity
Battery
3.6V, ER14252 or equivalent, 1/2AA size

> **Table 38** — Generic TTL Encoder Feedback Specifications

Attribute
Value
TTL incremental encoder support
5V, differential A quad B
Quadrature interpolation
4 counts / square wave period
Differential input voltage
(MTR_AM, MTR_BM, and MTR_IM)
5V DC, differential line driver (DLD) output compatible
DC current draw
(MTR_AM, MTR_BM, and MTR_IM)
30 mA, max
Input signal frequency
(MTR_AM, MTR_BM, and MTR_IM)
5.0 MHz, max
Edge separation
(MTR_AM and MTR_BM)
42 ns min, between any two edges
Commutation verification
Commutation angle verification performed at the first
Hall signal transition and periodically verifies thereafter
Hall inputs
(MTR_S1, MTR_S2, and MTR_S3)
Single-ended, TTL, open collector, or none

> **Table 39** — Generic Sine/Cosine Incremental Specifications

Attribute
Value
Input frequency
(MTR_SIN and MTR_COS)
250 kHz, max
Differential input voltage
(MTR_SIN and MTR_COS)
0.6…1.2V, peak to peak
Commutation verification
Commutation angle verification performed at the first Hall signal transition
and periodically verifies thereafter.
Hall inputs
(MTR_S1, MTR_S2, and MTR_S3)
Single-ended, TTL, open collector, or none.
Pin
Digital AqB Incremental

AUX_AM+

AUX_AM-

AUX_BM+

AUX_BM-

AUX_IM+

AUX_IM-

AUX_ECOM

AUX_EPWR5V

Reserved

Reserved

<!-- page 26 -->

## Encoder Phasing Definitions

For TTL encoders, the drive position increases when A leads B. Clockwise
motor rotation is assumed, when looking at the motor shaft.

> **Figure 41** — TTL Encoder Phasing

For Sin/Cos encoders, Hiperface for example, the drive position increases
when Cosine (B) leads Sine (A). Clockwise motor rotation is assumed, when
looking at the motor shaft.

> **Figure 42** — Sine/Cosine Encoder Phasing

The drive MFB connector uses Hall signals to initialize the commutation angle
for permanent magnet motor commutation.
A
/A
90°
90°
90°
90°
360°
B
/B
Z
/Z
B
A
IMPORTANT
The Sine/Cosine encoder signal phasing is different than the TTL
encoder signal phasing.
IMPORTANT
When using absolute feedback devices (for example, Hiperface) the
drive simulates a marker signal because these devices don't have a
marker signal required for the home-to-marker sequence to
complete.

<!-- page 27 -->

> **Figure 43** — Hall Encoder Phasing

## Absolute Position Feature

The absolute position feature tracks the position of the motor, within the
multi-turn retention limits, while the drive is powered off. The absolute
position feature is available with only multi-turn encoders.

> **Figure 44** — Absolute Position Limits (measured in turns or revolutions)

VUN
VWN
VVN
S1
S2
S3

> **Table 41** — Absolute Position Retention Limits

## Encoder Type

Cat. No.
Designator
Motor Cat. No.
Linear Cat. No.
Retention Limits
Turns (rotary)
mm (linear)
Hiperface
-M
MPL-A/Bxxxxx-M
MPM-A/Bxxxxx-M
MPF-A/Bxxxxx-M
MPS-A/Bxxxxx-M
MPAR-A/B3xxxx-M
MPAI-A/BxxxxxM
2048 (±1024)
–
-V
MPL-A/Bxxxxx-V
MPAS-A/Bxxxx1-V05, MPAS-A/Bxxxx2-V20
MPAR-A/B1xxxx-V, MPAR-A/B2xxxx-V
MPAI-A/BxxxxxV
4096 (±2048)
–
Nikon (24-bit) serial with battery
backup
-D
TLP-A/Bxxxx-D
–
65,536 (±32,768)
–
Tamagawa (17-bit) serial with battery
backup
-B
TL-Axxxx-B
TLY-Axxxx-B
–
+2048
-2048
+1024
-1024
-16,384
-8192

-32,768
-4096
+16,384
+8192
+32,768
+4096
Position at Power Down
4096 Turns
65,536 Revolutions
2048 Turns

**Extracted table (page 27, #1):**

| Cat. No. Designator | Motor Cat. No. | Linear Cat. No. |  |
| --- | --- | --- | --- |
|  |  |  | Turns (rotary) |
| -M | MPL-A/Bxxxxx-M MPM-A/Bxxxxx-M MPF-A/Bxxxxx-M MPS-A/Bxxxxx-M | MPAR-A/B3xxxx-M MPAI-A/BxxxxxM | 2048 (±1024) |
| -V | MPL-A/Bxxxxx-V | MPAS-A/Bxxxx1-V05, MPAS-A/Bxxxx2-V20 MPAR-A/B1xxxx-V, MPAR-A/B2xxxx-V MPAI-A/BxxxxxV | 4096 (±2048) |
| -D | TLP-A/Bxxxx-D | – | 65,536 (±32,768) |
| -B | TL-Axxxx-B TLY-Axxxx-B | – |  |

<!-- page 28 -->

## Safe Torque Off Feature

Kinetix 5100 servo drives have Safe Torque Off (STO) capability and can safely
remove inverter power when the STO signals are removed, resulting in Stop
Category 0 behavior.
2198-Exxxx-ERS (hardwired) servo drives support parallel input connections
for cascading additional drives. For applications that do not require the STO
safety capability, you must install jumper wires to bypass the safe torque-off
feature.
Refer to Safe Torque Off Feature on page 416 for the STO connector pinout,
installation, and wiring information.
Operation Modes
The Kinetix 5100 servo drive supports three basic modes of operation: Position,
Speed, and Torque. You can switch between these modes by using Dual or
Multi mode selections. IO Mode uses a Class 1 Ethernet/IP connection with a
Logix Controller.

> **Table 42** — Single Mode

Mode
Mode Abbreviation
Code
Description
Position mode
(I/O terminal block input)
PT

This mode is sometimes referred to as Pulse Train Output or Step and Direction. The
servo drive receives the Position command and commands the motor to run to the target
position. The Position command is communicated through the I/O terminal block and the
signal type is pulse. You can configure this mode for pulse-pulse following, which is a
form of gearing.
Position mode
(register input)
PR

The servo drive receives the Position command and commands the motor to run to the
target position. Position commands are issued from the program registers (99 sets in
total). You can select the register number with binary-weighted Digital Input (DI) signals
or through communication.
Speed mode
S

The servo drive receives the Speed command and commands the motor to run at the
target speed. The Speed command is issued from the internal registers (3 sets in total) or
by analog voltage (-10V…+10V) that is communicated through the I/O terminal block. You
can select the command with binary-weighted DI signals.
Speed mode
(no analog input)
Sz

The servo drive receives the Speed command and commands the motor to run at the
target speed. The Speed command can only be issued from the internal registers (3 sets
in total). You can select the command with binary-weighted DI signals.
Torque mode
T

The servo drive receives the Torque command and commands the motor to run with the
target torque. The Torque commands can be issued from the internal registers (3 sets in
total) and by analog voltage (-10V…+10V) that is communicated through the I/O terminal
block. You can select the command with binary-weighted DI signals.
Torque mode
 (no analog input)
Tz

The servo drive receives the Torque command and commands the motor to run with the
target torque. The Torque command can only be issued from the internal registers (3 sets
in total). You can select the command with binary-weighted DI signals.
IO mode
IO
OC
The servo drive receives commands from the Logix controller through the EtherNet/IP
network connection. Commands are issued through the Add-On Instruction in the Logix
Designer application.

> **Table 43** — Dual Mode

Mode
Mode Abbreviation
Code
Description
Position mode PT (I/O terminal block input) and Speed mode
PT-S

Switches PT and S mode with DI signals.
Position mode PT (I/O terminal block input) and Torque mode
PT-T

Switches PT and T mode with DI signals.
Position mode PR (register input) and Speed mode
PR-S

Switches PR and S mode with DI signals.
Position mode PR (register input) and Torque mode
PR-T

Switches PR and T mode with DI signals.
Speed mode and Torque mode
S-T
0A
Switches S and T mode with DI signals.
–
–
0B
Reserved
Position mode PT (I/O terminal block input) and Position mode PR (register
input)
PT-PR
0D
Switches PT and PR Mode with DI signals.

**Extracted table (page 28, #1):**

| Mode Abbreviation | Code |
| --- | --- |
| PT | 00 |
| PR | 01 |
| S | 02 |
| Sz | 04 |
| T | 03 |
| Tz | 05 |
| IO | OC |

**Extracted table (page 28, #2):**

| Mode Abbreviation | Code |
| --- | --- |
| PT-S | 06 |
| PT-T | 07 |
| PR-S | 08 |
| PR-T | 09 |
| S-T | 0A |
| – | 0B |
| PT-PR | 0D |

<!-- page 29 -->

The Multi Mode is a combination that uses the Dual Mode and a Single Mode
of operation.

> **Table 44** — Multi Mode

Mode
Mode Abbreviation
Code
Description
Position mode PT (I/O terminal block input), Position mode PR (register
input), and Speed mode
PT-PR-S
0E
Switches PT, PR, and S mode with DI signals.
Position mode PT (I/O terminal block input), Position mode PR (register
input), and Torque mode
PT-PR-T
0F
Switches PT, PR, and T mode with DI signals.

**Extracted table (page 29, #1):**

| Mode Abbreviation | Code |
| --- | --- |
| PT-PR-S | 0E |
| PT-PR-T | 0F |

