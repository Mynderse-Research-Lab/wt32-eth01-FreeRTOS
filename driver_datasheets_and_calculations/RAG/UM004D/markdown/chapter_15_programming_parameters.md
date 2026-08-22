# Chapter 15: Programming Parameters

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `16_Ch15_Programming_Parameters.pdf` (18 pages)_

<!-- page 1 -->

Organization of Parameters
Parameter Groups
Parameters are listed across 26 functional groups. Some parameters have dual
functions and appear in multiple groups. The primary groups are listed in the
parameter spreadsheet in the Kinetix 5100 Servo Drive Parameter Data and
Fault Codes. The additional groups to which the parameters belong are listed
in the Additional Groups column of that spreadsheet and are identified in the
table.
Topic
Page
Organization of Parameters

Description of Digital Input Functions

Description of Digital Output Functions

Description of System Variable Monitoring

Description of Parameter Monitoring

Use a MSG Instruction to Set Parameters

This manual links to Kinetix® 5100 Servo Drive Fault Codes Reference
Data, publication 2198-RD001, for fault codes and Kinetix 5100 Servo
Drive Parameters Reference Data, publication 2198-RD002, for
parameters. Download the spreadsheets now for offline access.

<!-- page 2 -->

## Numeric/Decimal Parameters

These parameters are integer based, with no decimal or negative sense.
Parameters in Hexadecimal Format
The drive has some parameters that are 16-bits or 32-bits in size and are
represented in hexadecimal format. The settings for these parameters are
defined either by the value of each bit or by the value of each hexadecimal digit,
represented by D, C, B, A (high) and U, Z, Y, X (low).
For example, ID230 (P2.047) is a 16-bit parameter that translates to one
hexadecimal number with four digits. This settings for this parameter are
defined both by the actual value of the X hexadecimal digit and by the values of
the individual bits in the Y and Z digits.
The setting for Auto resonance suppression is defined by the value of X as
follows:
•
X = 0: Disable auto resonance suppression
•
X = 1: Enable auto resonance suppression

> **Table 133** — Parameters and the Functional Groups

## Primary Group

Number
Group Name
Additional
Group Number Group Name

Motor

System

Drive

BRK_RegResistor

General

Pulse Setting

Status Monitor

Gear Ratio

Control

Filter

I/O

Limit

Communication

Position Loop

Diagnosis

Velocity Loop

Motion_1

Current Loop

Motion_2

Tuning

Speed Setting

Event Setting

Homing

E-CAM

Capture

Compare
ID230 (P2.047) Auto resonance suppression mode setting
X
Auto resonance suppression function
Z
Fixed resonance suppression parameter
Y
Fixed resonance suppression parameter
U
Reserved

**Extracted table (page 2, #1):**

| Group Name | Additional Group Number |
| --- | --- |
| Motor | 11 |
| Drive | 12 |
| General | 13 |
| Status Monitor | 14 |
| Control | 15 |
| I/O | 16 |
| Communication | 17 |
| Diagnosis | 18 |
| Motion_1 | 19 |
| Motion_2 | 20 |
|  | 21 |
|  | 22 |
|  | 23 |
|  | 24 |
|  | 25 |
|  | 26 |

<!-- page 3 -->

The fixed resonance suppression parameter is defined by the value of the bits
that make up the Y digit, as follows:
Description of Digital Input
Functions
The Kinetix 5100 drive provides 10 physical digital inputs and three virtual
digital inputs. These digital inputs are primarily configured in KNX5100C
software from Function List>Settings>Digital IO/Jog Control.
These inputs can be forced (when the drive is online and Enable is checked) to
be On or Off.
In the Digital Input dialog box, Status represents the logical level of the input
that is based on the use of N.O. and N.C. configurations. The Status (in the
KNX5100C software window) is NOT necessarily the actual voltage level on the
terminals (0V DC = OFF, 24V DC = ON) but rather the logical level of the digital
input. When N.O. is used, the relationship between input voltage and Status
matches (Status = ON = 24V DC, OFF = 0V DC). When N.C. is used, this
relationship is reversed. See Digital I/O and Jog Function in KNX5100C
Software on page 184 for more information.
The available digital input functions are listed in Table 135.
Y(1)
(1)
Bit 0…3 of digit Y correspond to Bits 4…7 of the 16-bit parameter.
Function
Description
Bit 0
Notch 1 auto / manual setting
0: Auto resonance suppression
1: Manually set the first set of resonance suppression
Bit 1
Notch 2 auto / manual setting
0: Auto resonance suppression
1: Manually set the second set of resonance suppression
Bit 2
Notch 3 auto / manual setting
0: Auto resonance suppression
1: Manually set the third set of resonance suppression
Bit 3
Notch 4 auto / manual setting
0: Auto resonance suppression
1: Manually set the fourth set of resonance suppression

> **Table 134** — Relevant Parameters

Parameter
Name
ID195 (P2.010)
DI1Configuration
ID196 (P2.011)
DI2Configuration
ID197 (P2.012)
DI3Configuration
ID198 (P2.013)
DI4Configuration
ID199 (P2.014)
DI5Configuration
ID200 (P2.015)
DI6Configuration
ID201 (P2.016)
DI7Configuration
ID202 (P2.017)
DI8Configuration
ID220 (P2.036)
DI9Configuration
ID221 (P2.037)
DI10Configuration
ID222 (P2.038)
VirtualDI11Configuration
ID223 (P2.039)
VirtualDI12Configuration
ID224 (P2.040)
VirtualDI13Configuration

<!-- page 4 -->

> **Table 135** — Digital Inputs

Setting
DI Description
Trigger Method
Control Mode
0x01
Servo On
When this DI is on, servo is activated (Servo On). When this DI is off, the servo is deactivated the default
deceleration profile is dynamic brake type (similar to a current decel profile). Always configure this DI as N.O.
Level triggered
All except I/O
0x02
Alarm/Fault Reset
When the DI transitions on, any faults are cleared. If the fault is still active, this reset does not clear the fault.
Rising-edge triggered All
0x03
Gain Switching
In speed and position mode, when this DI is on and GainSwitchingSelection ID212 (P2.027) X= 1 and the Gain
Switching Method = 0, the original gains are multiplied by Position /Velocity Gain Changing Rate ID186/ID190 (P2.001/
P2.005).
Level triggered
PT, PR, S
0x04
Pulse Clear
This input can be configured for Edge/Level trigger by changing PulseClearMode ID233 (P2.050). When this input is
on, any accumulated position error is cleared (set to zero).
Rising-edge triggered,
level triggered
PT
0x05
Zero Speed Threshold (ZClamp)
When the speed is below the setting of ZeroSpeedWindow ID145 (P1.038), the motor stops moving when this DI is on.
See Zero Speed Threshold Function on page 249 for additional information.
Level triggered
S, I/O (Speed)
0x06
Reverse Direction of Input Command
In speed mode, the input command reversed when this DI is on.
Level triggered
S, Sz, T, I/O
(Speed, Gear
Mode )
0x08
Command Triggered
In PR Mode, after selecting the PR command (POS0…POS6), this DI is the signal to carry out the execution of the
selected PR. Once the rising edge transition occurs, the selected PR will execute. This DI needs to transition off to
on for execution.
Rising-edge triggered PR
0x09
Torque Limit
When this DI is on and VelocityTorqueLimitAction ID118 (P1.002 Y) is enabled, the selected torque limit is applied.
Level triggered
PT, PR, S, I/O
(Position, Index,
Gear, Speed)
0x0C
Latch Function of Analog Position Command
While this DI is on, the motor is held at its current position, even if there is a change in command. When this DI is
off, the motor completes the command that occurred while the motor was stationary.
Level triggered
PT, I/O (Gear)
Speed
command
Setting value of
ID145 (P1.038)(
zero speed)
ZCLAMP
input signal
Motor Speed
Setting value
of ID145 (P1.038)
(zero speed)

**Extracted table (page 4, #1):**

| DI Description | Trigger Method |
| --- | --- |
| Servo On When this DI is on, servo is activated (Servo On). When this DI is off, the servo is deactivated the default deceleration profile is dynamic brake type (similar to a current decel profile). Always configure this DI as N.O. | Level triggered |
| Alarm/Fault Reset When the DI transitions on, any faults are cleared. If the fault is still active, this reset does not clear the fault. | Rising-edge triggered |
| Gain Switching In speed and position mode, when this DI is on and GainSwitchingSelection ID212 (P2.027) X= 1 and the Gain Switching Method = 0, the original gains are multiplied by Position /Velocity Gain Changing Rate ID186/ID190 (P2.001/ P2.005). | Level triggered |
| Pulse Clear This input can be configured for Edge/Level trigger by changing PulseClearMode ID233 (P2.050). When this input is on, any accumulated position error is cleared (set to zero). | Rising-edge triggered, level triggered |
| Zero Speed Threshold (ZClamp) When the speed is below the setting of ZeroSpeedWindow ID145 (P1.038), the motor stops moving when this DI is on. Speed command Setting value of ID145 (P1.038)( zero speed) ZCLAMP input signal Motor Speed Setting value of ID145 (P1.038) (zero speed) See Zero Speed Threshold Function on page249 for additional information. | Level triggered |
| Reverse Direction of Input Command In speed mode, the input command reversed when this DI is on. | Level triggered |
| Command Triggered In PR Mode, after selecting the PR command (POS0…POS6), this DI is the signal to carry out the execution of the selected PR. Once the rising edge transition occurs, the selected PR will execute. This DI needs to transition off to on for execution. | Rising-edge triggered |
| Torque Limit When this DI is on and VelocityTorqueLimitAction ID118 (P1.002 Y) is enabled, the selected torque limit is applied. | Level triggered |
| Latch Function of Analog Position Command While this DI is on, the motor is held at its current position, even if there is a change in command. When this DI is off, the motor completes the command that occurred while the motor was stationary. | Level triggered |

<!-- page 5 -->

0x10
Speed Limit
In torque mode, the motor speed will be limited when this DI is on, and the limited speed command is the internal
register or analog voltage command.
Level triggered
T, I/O (Torque)
0x11
Register Position Command Selection 1…99 Bit 0.
See Digital Input (DI) Trigger on page 354 for more information.
Level triggered
PR
0x12
Register Position Command Selection 1…99 Bit 1.
See section Digital Input (DI) Trigger on page 354 for more information.
Level triggered
PR
0x13
Register Position Command Selection 1…99 Bit 2.
See section Digital Input (DI) Trigger on page 354 for more information.
Level triggered
PR
0x1A
Register Position Command Selection 1…99 Bit 3.
See section Digital Input (DI) Trigger on page 354 for more information.
Level triggered
PR
0x1B
Register Position Command Selection 1…99 Bit 4.
See section Digital Input (DI) Trigger on page 354 for more information.
Level triggered
PR
0x1C
Register Position Command Selection 1…99 Bit 5.
See section Digital Input (DI) Trigger on page 354 for more information.
Level triggered
PR
0x1E
Register Position Command Selection 1…99 Bit 6.
See section Digital Input (DI) Trigger on page 354 for more information.
Level triggered
PR
0x1F
Set Up or Clear Absolute System Coordinates
When the DI.ABSC signal is on, the number of turns data stored in absolute encoder will be cleared. But this DI is only
valid when the DI.ABSE signal is on.
Rising-edge triggered All
0x14
Register Speed Command Selection 1…4 Bit 0.
See Configure and Select the Preset Speeds on page 246 for more information.
Level triggered
S
T, I/O (Torque)
for speed limit
function
0x15
Register Speed Command Selection 1…4 Bit 1.
See Configure and Select the Preset Speeds on page 246 for more information.
Level triggered
S
T, I/O (Torque)
for speed limit
function
0x16
Register Torque Command Selection 1…4 Bit 0.
See Configure and Select the Preset Speeds on page 246 for more information.
Level triggered
T
PR, PT, S, I/O
(Position, Index,
Gear, Speed) for
torque limit
function
0x17
Register Torque Command Selection 1…4 Bit 1.
See Configure and Select the Preset Speeds on page 246 for more information.
Level triggered
T
PR, PT, S, I/O
(Position, Index,
Gear, Speed) for
torque limit
function

> **Table 135** — Digital Inputs (Continued)

Setting
DI Description
Trigger Method
Control Mode

**Extracted table (page 5, #1):**

| DI Description | Trigger Method |
| --- | --- |
| Speed Limit In torque mode, the motor speed will be limited when this DI is on, and the limited speed command is the internal register or analog voltage command. | Level triggered |
| Register Position Command Selection 1…99 Bit 0. See Digital Input (DI) Trigger on page354 for more information. | Level triggered |
| Register Position Command Selection 1…99 Bit 1. See section Digital Input (DI) Trigger on page354 for more information. | Level triggered |
| Register Position Command Selection 1…99 Bit 2. See section Digital Input (DI) Trigger on page354 for more information. | Level triggered |
| Register Position Command Selection 1…99 Bit 3. See section Digital Input (DI) Trigger on page354 for more information. | Level triggered |
| Register Position Command Selection 1…99 Bit 4. See section Digital Input (DI) Trigger on page354 for more information. | Level triggered |
| Register Position Command Selection 1…99 Bit 5. See section Digital Input (DI) Trigger on page354 for more information. | Level triggered |
| Register Position Command Selection 1…99 Bit 6. See section Digital Input (DI) Trigger on page354 for more information. | Level triggered |
| Set Up or Clear Absolute System Coordinates When the DI.ABSC signal is on, the number of turns data stored in absolute encoder will be cleared. But this DI is only valid when the DI.ABSE signal is on. | Rising-edge triggered |
| Register Speed Command Selection 1…4 Bit 0. See Configure and Select the Preset Speeds on page246 for more information. | Level triggered |
| Register Speed Command Selection 1…4 Bit 1. See Configure and Select the Preset Speeds on page246 for more information. | Level triggered |
| Register Torque Command Selection 1…4 Bit 0. See Configure and Select the Preset Speeds on page246 for more information. | Level triggered |
| Register Torque Command Selection 1…4 Bit 1. See Configure and Select the Preset Speeds on page246 for more information. | Level triggered |

<!-- page 6 -->

0x18
Position/Speed Modes Selection
When the operation mode is dual mode (PR or PT / S) and this DI is off, the operation mode is positioning (PT or PR).
When this DI is on, the operation mode is Speed control. In multi-mode, this input is used with PT/PR Mode Selection
DI to give multi-mode selections.
Level triggered
Dual Mode
0x19
Torque/Speed Modes Switching
When the operation mode is dual mode (S/T) and this DI is off, the operation mode is speed control. When this DI is
on, the operation mode is torque control.
Level triggered
Dual Mode
0x20
Torque/Position Mode
When the operation mode is dual mode (any mode using Torque and Position, PT or PR) and this DI is off, the
operation mode is torque control. When this DI is on, the operation mode uses the Position control operation mode.
In multi-mode, this input is used with PT/PR Mode Selection DI to give multi-mode selections.
Level triggered
Dual Mode
0x21
Emergency Stop
When this DI is on, the drive decelerates and disables the motor. MotorStopMode ID 675 (P1.032) is used to define the
deceleration profile. If a holding brake is used, this stop uses the configured brake timing. This stop issues E013
when its complete.
IMPORTANT: This stop type has no safety implications. It is not a safety rated input.
Level triggered
All
0x22
Reverse Limit Switch (NL) - This sensor indicates the most negative point of travel for the axis. When this DI is on
(typically configured as N.C. - and Status displays Off), this setting means that there is no active Overtravel
condition. When this DI is off, the A015 alarm is issued (Status shows On) and motion is allowed only in the forward
direction (to move away from the limit), once the limit transitions back to on, the alarm is cleared automatically.
Level triggered
All
0x23
Forward Limit Switch (PL) - This sensor indicates the most positive point of travel for the axis. When this DI is on
(typically configured as N.C. - and Status displays Off), this setting means that there is no active Overtravel
condition. When this DI is off, the A014 alarm is issued (Status shows On) and motion is allowed only in the reverse
direction (to move away from the limit), once the limit transitions back to on, the alarm is cleared automatically.
Level triggered
All
0x24
Homing Switch (ORG) -
This DI represents the Homing Switch when the configurable Homing Method uses a switch.
See Setting Homing Mode ID297 (P5.004) - PR Mode on page 298 for additional information.
Rising- and fallingedge triggered
PR, I/O (Index)
0x27
Enable Homing
This input executes the configured Homing Method.
See Setting Homing Mode ID297 (P5.004) - PR Mode on page 298 for additional information.
Rising-edge triggered PR
0x2B
PT/PR Modes Selection
When the operation mode is dual mode (PT/PR) and this DI is off, the operation mode is positioning (PT). When this
DI is on, the operation mode is positioning (PR). In multi-mode, this input is used with PR /S/T Mode Selection DI to
give multi-mode selections.
Level triggered
Dual Mode
0x35
Electronic Cam Phase Alignment
When this DI is on, and the E-CAM is enabled ECamControlConfiguration ID248 (P2.076 Bit0 = 1) the Alignment index
is executed ECamAlignmentTargetPosition ID247 (P2.075).
Rising-edge triggered PR, I/O (Position,
Index )
0x36
E-CAM Engaging Control
This DI is used when the E-CAM is configured as DI-CAM enabled. When this DI is on, the E-CAM function, as
configured, is executed. Once the E-CAM is executing, this DI can be off until the E-CAM is complete.
See E-CAM on page 384.
Rising- and fallingedge triggered
PR, I/O (Position,
Index )
0x37
Motor JOGs in the Forward Direction
When this DI is on, the motor jogs in a forward direction.
Level triggered
All except I/O
0x38
Motor JOGs in the Reverse Direction
When this DI is on, the motor jogs in a reverse direction.
Level triggered
All except I/O
0x39
Event Trigger Command 1
When this DI is triggered, the configured Event (EV1) PR is executed
(KNX5100C software, Function List>Motion Control>PR Mode Editor>General Parameter Setting).
Note that this input can be configured to execute events on a rising or falling edge trigger.
Rising- and fallingedge triggered
PR, I/O (Index)
0x3A
Event Trigger Command 2
When this DI is triggered, the configured Event (EV2) PR is executed
(KNX5100C software, Function List>Motion Control>PR Mode Editor>General Parameter Setting).
Note that this input can be configured to execute events on a rising or falling edge trigger.
Rising- and fallingedge triggered
PR, I/O (Index)
0x3B
Event Trigger Command 3
When this DI is triggered, the configured Event (EV3) PR is executed
(KNX5100C software, Function List>Motion Control>PR Mode Editor>General Parameter Setting).
Note that this input can be configured to execute events on a rising or falling edge trigger.
Rising- and fallingedge triggered
PR, I/O (Index)
0x3C
Event Trigger Command 4
When this DI is triggered, the configured Event (EV4) PR is executed
(KNX5100C software, Function List>Motion Control>PR Mode Editor>General Parameter Setting).
Note that this input can be configured to execute events on a rising or falling edge trigger.
Rising- and fallingedge triggered
PR, I/O (Index)
0x43
E-Gear Ratio (Numerator) Selection 0
See Configure Electronic Gear (E-Gear) Ratio on page 169 for additional information.
Level triggered
PR, PT
0x44
E-Gear Ratio (Numerator) Selection 1
See Configure Electronic Gear (E-Gear) Ratio on page 169 for additional information.
Level triggered
PR, PT

> **Table 135** — Digital Inputs (Continued)

Setting
DI Description
Trigger Method
Control Mode

**Extracted table (page 6, #1):**

| DI Description | Trigger Method |
| --- | --- |
| Position/Speed Modes Selection When the operation mode is dual mode (PR or PT / S) and this DI is off, the operation mode is positioning (PT or PR). When this DI is on, the operation mode is Speed control. In multi-mode, this input is used with PT/PR Mode Selection DI to give multi-mode selections. | Level triggered |
| Torque/Speed Modes Switching When the operation mode is dual mode (S/T) and this DI is off, the operation mode is speed control. When this DI is on, the operation mode is torque control. | Level triggered |
| Torque/Position Mode When the operation mode is dual mode (any mode using Torque and Position, PT or PR) and this DI is off, the operation mode is torque control. When this DI is on, the operation mode uses the Position control operation mode. In multi-mode, this input is used with PT/PR Mode Selection DI to give multi-mode selections. | Level triggered |
| Emergency Stop When this DI is on, the drive decelerates and disables the motor. MotorStopMode ID 675 (P1.032) is used to define the deceleration profile. If a holding brake is used, this stop uses the configured brake timing. This stop issues E013 when its complete. IMPORTANT: This stop type has no safety implications. It is not a safety rated input. | Level triggered |
| Reverse Limit Switch (NL) - This sensor indicates the most negative point of travel for the axis. When this DI is on (typically configured as N.C. - and Status displays Off), this setting means that there is no active Overtravel condition. When this DI is off, the A015 alarm is issued (Status shows On) and motion is allowed only in the forward direction (to move away from the limit), once the limit transitions back to on, the alarm is cleared automatically. | Level triggered |
| Forward Limit Switch (PL) - This sensor indicates the most positive point of travel for the axis. When this DI is on (typically configured as N.C. - and Status displays Off), this setting means that there is no active Overtravel condition. When this DI is off, the A014 alarm is issued (Status shows On) and motion is allowed only in the reverse direction (to move away from the limit), once the limit transitions back to on, the alarm is cleared automatically. | Level triggered |
| Homing Switch (ORG) - This DI represents the Homing Switch when the configurable Homing Method uses a switch. See Setting Homing Mode ID297 (P5.004) - PR Mode on page298 for additional information. | Rising- and falling- edge triggered |
| Enable Homing This input executes the configured Homing Method. See Setting Homing Mode ID297 (P5.004) - PR Mode on page298 for additional information. | Rising-edge triggered |
| PT/PR Modes Selection When the operation mode is dual mode (PT/PR) and this DI is off, the operation mode is positioning (PT). When this DI is on, the operation mode is positioning (PR). In multi-mode, this input is used with PR /S/T Mode Selection DI to give multi-mode selections. | Level triggered |
| Electronic Cam Phase Alignment When this DI is on, and the E-CAM is enabled ECamControlConfiguration ID248 (P2.076 Bit0 = 1) the Alignment index is executed ECamAlignmentTargetPosition ID247 (P2.075). | Rising-edge triggered |
| E-CAM Engaging Control This DI is used when the E-CAM is configured as DI-CAM enabled. When this DI is on, the E-CAM function, as configured, is executed. Once the E-CAM is executing, this DI can be off until the E-CAM is complete. See E-CAM on page384. | Rising- and falling- edge triggered |
| Motor JOGs in the Forward Direction When this DI is on, the motor jogs in a forward direction. | Level triggered |
| Motor JOGs in the Reverse Direction When this DI is on, the motor jogs in a reverse direction. | Level triggered |
| Event Trigger Command 1 When this DI is triggered, the configured Event (EV1) PR is executed (KNX5100C software, Function List>Motion Control>PR Mode Editor>General Parameter Setting). Note that this input can be configured to execute events on a rising or falling edge trigger. | Rising- and falling- edge triggered |
| Event Trigger Command 2 When this DI is triggered, the configured Event (EV2) PR is executed (KNX5100C software, Function List>Motion Control>PR Mode Editor>General Parameter Setting). Note that this input can be configured to execute events on a rising or falling edge trigger. | Rising- and falling- edge triggered |
| Event Trigger Command 3 When this DI is triggered, the configured Event (EV3) PR is executed (KNX5100C software, Function List>Motion Control>PR Mode Editor>General Parameter Setting). Note that this input can be configured to execute events on a rising or falling edge trigger. | Rising- and falling- edge triggered |
| Event Trigger Command 4 When this DI is triggered, the configured Event (EV4) PR is executed (KNX5100C software, Function List>Motion Control>PR Mode Editor>General Parameter Setting). Note that this input can be configured to execute events on a rising or falling edge trigger. | Rising- and falling- edge triggered |
| E-Gear Ratio (Numerator) Selection 0 See Configure Electronic Gear (E-Gear) Ratio on page169 for additional information. | Level triggered |
| E-Gear Ratio (Numerator) Selection 1 See Configure Electronic Gear (E-Gear) Ratio on page169 for additional information. | Level triggered |

<!-- page 7 -->

Description of Digital Output
Functions
The Kinetix 5100 drive provides six physical digital outputs. These digital
outputs are primarily configured in KNX5100C software from
Function List>Settings>Digital IO/Jog Control.
You can force these outputs (when the drive is online and the Enable DO
Override is checked) to be On or Off.
In the Digital Input dialog box, Status represents the logical level of the output.
The available digital output functions are listed in the following table.
0x45
Disable External pulse
When using PT Operation mode and this DI is on, the drive stops responding to commands using external pulses.
The motor does not move while this DI is on. This function only works when configured with DI8.
Level triggered
PT
0x46
Stop
When this DI is on, the drive decelerates the motor. AutoProtectionDecelTime ID 296 (P5.003) is used to define the
deceleration profile. This DI only stops PR command types (including positioning and constant velocity; Jog). This
command does not stop Jog commands (outside of a PR command) or E-CAM commands.
Rising-edge triggered,
level triggered
PR
0x47
Profile Quick Stop
When this DI is on, the drive decelerates and disables the motor. AutoProtectionDecelTime ID 296 (P5.003) is used to
define the deceleration profile. If a holding brake is used, this stop uses any configured brake timing. This stop
issues alarm: A35F when its complete.
Rising-edge triggered PT, PR, T, S
0x48
Servo On with holding brake
Use this DI when a holding brake is used. When this DI is on, the drive is activated (Servo On). When this DI is off, the
drive decelerates and disables the motor. AutoProtectionDecelTime ID 296 (P5.003) STP is used to define the
deceleration profile. This DI is used with Vertical Load Control and this DI setting uses any configured brake timing.
Always configure this DI as N.O.
Level triggered
All except I/O

> **Table 135** — Digital Inputs (Continued)

Setting
DI Description
Trigger Method
Control Mode

> **Table 136** — Relevant Parameters

Parameter
Name
ID203 (P2.018)
DO1Configuration
ID204 (P2.019)
DO2Configuration
ID205 (P2.020)
DO3Configuration
ID206 (P2.021)
DO4Configuration
ID207 (P2.022)
DO5Configuration
ID225 (P2.041)
DO6Configuration

> **Table 137** — Digital Outputs

Setting
DO Description
Triggering Method
Control Mode
0x01
Servo Ready
This DO is on when both the control and main power is applied to the drive and the drive is not faulted.
Level triggered
All
0x02
Servo On
This DO is on when the servo is activated (enabled) and the drive is not faulted.
Level triggered
All
0x03
Motor is at zero speed
This DO is on whenever the motor is within the ZeroSpeedWindow ID145 (P1.038).
Level triggered
All

**Extracted table (page 7, #1):**

| DI Description | Trigger Method |
| --- | --- |
| Disable External pulse When using PT Operation mode and this DI is on, the drive stops responding to commands using external pulses. The motor does not move while this DI is on. This function only works when configured with DI8. | Level triggered |
| Stop When this DI is on, the drive decelerates the motor. AutoProtectionDecelTime ID 296 (P5.003) is used to define the deceleration profile. This DI only stops PR command types (including positioning and constant velocity; Jog). This command does not stop Jog commands (outside of a PR command) or E-CAM commands. | Rising-edge triggered, level triggered |
| Profile Quick Stop When this DI is on, the drive decelerates and disables the motor. AutoProtectionDecelTime ID 296 (P5.003) is used to define the deceleration profile. If a holding brake is used, this stop uses any configured brake timing. This stop issues alarm: A35F when its complete. | Rising-edge triggered |
| Servo On with holding brake Use this DI when a holding brake is used. When this DI is on, the drive is activated (Servo On). When this DI is off, the drive decelerates and disables the motor. AutoProtectionDecelTime ID 296 (P5.003) STP is used to define the deceleration profile. This DI is used with Vertical Load Control and this DI setting uses any configured brake timing. Always configure this DI as N.O. | Level triggered |

**Extracted table (page 7, #2):**

| DO Description | Triggering Method |
| --- | --- |
| Servo Ready This DO is on when both the control and main power is applied to the drive and the drive is not faulted. | Level triggered |
| Servo On This DO is on when the servo is activated (enabled) and the drive is not faulted. | Level triggered |
| Motor is at zero speed This DO is on whenever the motor is within the ZeroSpeedWindow ID145 (P1.038). | Level triggered |

<!-- page 8 -->

0x04
Motor reaches the target speed
This DO is on whenever the motor reaches UpToSpeedLimit ID146 (P1.039).
Level triggered
All
0x05
Motor reaches target position
This DO is on whenever the motor position is within InPositionWindow ID159 (P1.054).
Level triggered
PT, PR, I/O
(Position, Gear)
0x06
Torque Limit Activated
This DO is on whenever the drive is in a torque limited condition.
Level triggered
All (Except for T
and Tz)
0x07
Servo Alarm
This DO is on whenever an alarm or fault condition is active. This DO does not turn on when: forward or reverse
limits are active, communication error, undervoltage, and fan error.
Level triggered
All
0x08
Brake Control
This DO is on whenever an alarm or fault condition is active. This DO does not turn on when: forward or reverse
limits are active, communication error, undervoltage, and fan error.
See Motor Brake Circuit on page 68.
Level triggered
All
0x09
Homing Completed
This DO is on when homing is successfully completed on the axis. When Motor Feedback>Startup Method is:
• Incremental: This DO is off when control/main power is cycled.
• Absolute: This DO is on when control/main power is cycled.
• If the position cycle counts overflow occurs, this DO is off.
Level triggered
PR
0x0B
At Home Position
This DO is on when:
• Homing is complete
• The Command Position is equal to the Home Position
• The difference between Feedback Position and the Home Position is within InPositionWindow ID 159 (P1.054)
Level triggered
PR
0x0D
Absolute Type System Error
This DO is on when a fault occurs while the Absolute Homing is in process.
Level triggered
All
0x0E
Indexing Coordinate is defined
The indexing coordinate is defined when homing is completed. This DO is on when homing is complete.
—
PR

> **Table 137** — Digital Outputs (Continued)

Setting
DO Description
Triggering Method
Control Mode
OFF
OFF
Motor Speed
ON
ON
SON
M_BRK

**Extracted table (page 8, #1):**

| DO Description | Triggering Method |
| --- | --- |
| Motor reaches the target speed This DO is on whenever the motor reaches UpToSpeedLimit ID146 (P1.039). | Level triggered |
| Motor reaches target position This DO is on whenever the motor position is within InPositionWindow ID159 (P1.054). | Level triggered |
| Torque Limit Activated This DO is on whenever the drive is in a torque limited condition. | Level triggered |
| Servo Alarm This DO is on whenever an alarm or fault condition is active. This DO does not turn on when: forward or reverse limits are active, communication error, undervoltage, and fan error. | Level triggered |
| Brake Control This DO is on whenever an alarm or fault condition is active. This DO does not turn on when: forward or reverse limits are active, communication error, undervoltage, and fan error. ON OFF OFF SON ON M_BRK Motor Speed See Motor Brake Circuit on page68. | Level triggered |
| Homing Completed This DO is on when homing is successfully completed on the axis. When Motor Feedback>Startup Method is: • Incremental: This DO is off when control/main power is cycled. • Absolute: This DO is on when control/main power is cycled. • If the position cycle counts overflow occurs, this DO is off. | Level triggered |
| At Home Position This DO is on when: • Homing is complete • The Command Position is equal to the Home Position • The difference between Feedback Position and the Home Position is within InPositionWindow ID 159 (P1.054) | Level triggered |
| Absolute Type System Error This DO is on when a fault occurs while the Absolute Homing is in process. | Level triggered |
| Indexing Coordinate is defined The indexing coordinate is defined when homing is completed. This DO is on when homing is complete. | — |

<!-- page 9 -->

0x10
Early Warning for Overload
When the output average load (Load) is > 100%, this load can be applied for a period (Operating time) before the
drive faults (tOL).
The OverloadWarningUserThreshold ID 161 (P1.056) value (max 120%) is used with this Operating time, so that:
tOLW = (OverloadWarningUserThreshold x Operating time)
During any excess load condition, this DO is on when tOLW is exceeded but before the Operating time (tOL) is
reached. When the Operating time is met, Servo Alarm E006-Motor Overload is on and the DO Servo Alarm is on.
For example: When the output average load of the servo drive is 200% and the Operating time exceeds 8 seconds,
the overload fault E 006 Motor overload occurs.
When OverloadWarningUserThreshold = 60%, tOLW = 60% x 8sec = 4.8 seconds. When this output average load of
200% exceeds 4.8 sec, this DO is on. When this load of 200% exceeds 8 seconds, the E006 Motor overload occurs
and the DO Servo Alarm is on.
Level triggered
All
0x11
Warning
This DO is on for the exceptions of the Alarm DO: forward/reverse limit, communication error, undervoltage, or fan
error.
Level triggered
All
0x12
Position Command Overflows
Position command / feedback exceeds limit range.
Level triggered
PT, PR
0x13
Reverse Software Limit (NL)
This DO is on when the Reverse software limit is active.
Level triggered
PR
0x14
Forward Software Limit (PL)
This DO is on when the Forward software limit is active.
Level triggered
PR
0x15
PR Command Completed
This DO is off when a PR is executing. When the PR is completed, this DO is on. This DO only indicates that the
command is complete but not necessarily that the motor is in the target position, it could be still reaching its target
position.
Level triggered
PR
0x16
CAP Procedure Completed
Capture procedure is completed.
Level triggered
All
0x17
PR Procedure Completed
This DO is on when DO PR command completed and DO Motor reaches the target position are on. This DO can remain
on after being triggered, this setting is in ToSpeedAction ID155 (P1.048).
Level triggered
PR, I/O (Index
when the PR
command type
is not Speed)
0x18
Master position of the E-CAM is in the Setting Area
This DO is on when the E-CAM is active and the master position falls between:
ECamDOCamArea1RisingEdgeAngle ID378 (P5.090) and
ECamDOCamArea1FallingEdgeAngle ID379 (P5.091).
Level triggered
PR, I/O (Position,
Index)
0x19
Speed reaches the Target Speed
When you use a speed command, this DO is on when the motor speed is within the SpeedWindow ID250 (P2.079) of
the Speed command.
Level triggered
S, Sz, I/O (Speed)
0x1A
Master position of the E-CAM is in the Setting Area 2
This DO is on when the E-CAM is active and the master position falls between:
ECamDOCamArea2RisingEdgeAngle ID249 (P2.078) and
ECamDOCamArea2FallingEdgeAngle ID250 (P2.079).
Level triggered
PR
0x1D
Second CAP procedure completed
The second capture procedure is completed.
Level triggered
All
0x2C
P0.009 'ON' between ID113 (P0.054) and ID114 (P0.055)
This DO is on after SystemVariableMonitorFilterTime ID112 (P0.053) elapses and: SystemVariableMonitorLowerLimit
ID113 (P0.054) ≤ SystemVariableMonitor1Value ID663 (P0.009) ≤ SystemVariableMonitorUpperLimit ID114 (P0.055)
Level triggered
All
0x30
Output Bit 00 of ID283 (P4.006)
This DO is on when Bit 00 of DOStatus ID283 (P4.006) is on.
Level triggered
PR
0x31
Output Bit 01 of ID283 (P4.006)
This DO is on when Bit 01 of DOStatus ID283 (P4.006) is on.
Level triggered
PR

> **Table 137** — Digital Outputs (Continued)

Setting
DO Description
Triggering Method
Control Mode
Load
Operating

**Extracted table (page 9, #1):**

| DO Description | Triggering Method |
| --- | --- |
| Early Warning for Overload Load Operating When the output average load (Load) is > 100%, this load can be applied for a period (Operating time) before the drive faults (tOL). The OverloadWarningUserThreshold ID 161 (P1.056) value (max 120%) is used with this Operating time, so that: tOLW = (OverloadWarningUserThreshold x Operating time) During any excess load condition, this DO is on when tOLW is exceeded but before the Operating time (tOL) is reached. When the Operating time is met, Servo Alarm E006-Motor Overload is on and the DO Servo Alarm is on. For example: When the output average load of the servo drive is 200% and the Operating time exceeds 8 seconds, the overload fault E 006 Motor overload occurs. When OverloadWarningUserThreshold = 60%, tOLW = 60% x 8sec = 4.8 seconds. When this output average load of 200% exceeds 4.8 sec, this DO is on. When this load of 200% exceeds 8 seconds, the E006 Motor overload occurs and the DO Servo Alarm is on. | Level triggered |
| Warning This DO is on for the exceptions of the Alarm DO: forward/reverse limit, communication error, undervoltage, or fan error. | Level triggered |
| Position Command Overflows Position command / feedback exceeds limit range. | Level triggered |
| Reverse Software Limit (NL) This DO is on when the Reverse software limit is active. | Level triggered |
| Forward Software Limit (PL) This DO is on when the Forward software limit is active. | Level triggered |
| PR Command Completed This DO is off when a PR is executing. When the PR is completed, this DO is on. This DO only indicates that the command is complete but not necessarily that the motor is in the target position, it could be still reaching its target position. | Level triggered |
| CAP Procedure Completed Capture procedure is completed. | Level triggered |
| PR Procedure Completed This DO is on when DO PR command completed and DO Motor reaches the target position are on. This DO can remain on after being triggered, this setting is in ToSpeedAction ID155 (P1.048). | Level triggered |
| Master position of the E-CAM is in the Setting Area This DO is on when the E-CAM is active and the master position falls between: ECamDOCamArea1RisingEdgeAngle ID378 (P5.090) and ECamDOCamArea1FallingEdgeAngle ID379 (P5.091). | Level triggered |
| Speed reaches the Target Speed When you use a speed command, this DO is on when the motor speed is within the SpeedWindow ID250 (P2.079) of the Speed command. | Level triggered |
| Master position of the E-CAM is in the Setting Area 2 This DO is on when the E-CAM is active and the master position falls between: ECamDOCamArea2RisingEdgeAngle ID249 (P2.078) and ECamDOCamArea2FallingEdgeAngle ID250 (P2.079). | Level triggered |
| Second CAP procedure completed The second capture procedure is completed. | Level triggered |
| P0.009 'ON' between ID113 (P0.054) and ID114 (P0.055) This DO is on after SystemVariableMonitorFilterTime ID112 (P0.053) elapses and: SystemVariableMonitorLowerLimit ID113 (P0.054) ≤ SystemVariableMonitor1Value ID663 (P0.009) ≤ SystemVariableMonitorUpperLimit ID114 (P0.055) | Level triggered |
| Output Bit 00 of ID283 (P4.006) This DO is on when Bit 00 of DOStatus ID283 (P4.006) is on. | Level triggered |
| Output Bit 01 of ID283 (P4.006) This DO is on when Bit 01 of DOStatus ID283 (P4.006) is on. | Level triggered |

<!-- page 10 -->

Description of System
Variable Monitoring
You can use the five available SystemVariableMonitorSelection values to store
the values of the selected parameters, see Figure 236. These selections can be
used in the scope tracing or when using Statements in the PR commands.

> **Figure 236** — System Variable Monitoring

There are two ways to monitor the system variables. You can monitor via the
panel display or you can monitor via the system variable monitoring
parameters.
Panel Display
When the panel is in Real Time Data Display mode, press the UP / DOWN keys
to select the variable to be monitored. See Chapter 6, Use the Keypad Interface.
System Variable Monitoring Parameters
The following parameters are used to support system variable monitoring.
0x32
Output Bit 02 of ID283 (P4.006)
This DO is on when Bit 02 of DOStatus ID283 (P4.006) is on.
Level triggered
PR
0x33
Output Bit 03 of ID283 (P4.006)
This DO is on when Bit 03 of DOStatus ID283 (P4.006) is on.
Level triggered
PR
0x34
Output Bit 04 of ID283 (P4.006)
This DO is on when Bit 04 of DOStatus ID283 (P4.006) is on.
Level triggered
PR
0x35
Output Bit 05 of ID283 (P4.006)
This DO is on when Bit 05 of DOStatus ID283 (P4.006) is on.
Level triggered
PR

> **Table 137** — Digital Outputs (Continued)

Setting
DO Description
Triggering Method
Control Mode

**Extracted table (page 10, #1):**

| DO Description | Triggering Method |
| --- | --- |
| Output Bit 02 of ID283 (P4.006) This DO is on when Bit 02 of DOStatus ID283 (P4.006) is on. | Level triggered |
| Output Bit 03 of ID283 (P4.006) This DO is on when Bit 03 of DOStatus ID283 (P4.006) is on. | Level triggered |
| Output Bit 04 of ID283 (P4.006) This DO is on when Bit 04 of DOStatus ID283 (P4.006) is on. | Level triggered |
| Output Bit 05 of ID283 (P4.006) This DO is on when Bit 05 of DOStatus ID283 (P4.006) is on. | Level triggered |

<!-- page 11 -->

## System Variables List

The property code of each system variable is described as follows:
Monitoring variables are described in the following table according to the code
sequence:

> **Table 138** — System Variable Monitoring Parameters

Parameter
Name
Description
ID663 (P0.009)
SystemVariableMonitor1Value
The value to be monitored can be set by using
ID668 (P0.017) SystemVariableMonitor1Selection.
Set ID102 (P0.002) LEDMonitorSelection to 23 to display
the value of this parameter on the panel display.
ID664 (P0.010)
SystemVariableMonitor2Value
The value to be monitored can be set by ID 669 (P0.018).
Set ID102 (P0.002) to 24 to display the value of this
parameter on the panel display.
ID665 (P0.011)
SystemVariableMonitor3Value
The value to be monitored can be set by ID670 (P0.019).
Set ID102 (P0.002) to 25 to display the value of this
parameter on the panel display.
ID666 (P0.012)
SystemVariableMonitor4Value
The value to be monitored can be set by ID671 (P0.020).
Set ID102 (P0.002) to 26 to display the value of this
parameter on the panel display.
ID667 (P0.013)
SystemVariableMonitor5Value
The value to be monitored can be set by ID672 (P0.021).
Set ID102 (P0.002) to 27 to display the value of this
parameter on the panel display.
ID668 (P0.017)
SystemVariableMonitor1Selection
Use the pull-down menu in the parameter editor to
choose the parameter to map.
ID669 (P0.018)
SystemVariableMonitor2Selection
Use the pull-down menu in the parameter editor to
choose the parameter to map.
ID670 (P0.019)
SystemVariableMonitor3Selection
Use the pull-down menu in the parameter editor to
choose the parameter to map.
ID671 (P0.020)
SystemVariableMonitor4Selection
Use the pull-down menu in the parameter editor to
choose the parameter to map.
ID672 (P0.021)
SystemVariableMonitor5Selection
Use the pull-down menu in the parameter editor to
choose the parameter to map.
Property
Description
B
BASE: Basic variables, can be selected via the UP / DOWN keys on the panel.
Dec, Hex
Display format on panel. Dec indicated Decimal, Hex indicates Hexadecimal.
Currently all the system variables are displayed in the Dec format.

> **Table 139** — System Variables Code

Code
Variable name
Property
Description
User Unit(1)
000 (00h)
Feedback position
(PUU)
B, DEC
Current feedback position of the motor encoder.
PUU
001 (01h)
Position command
(PUU)
B, DEC
Current coordinate of the position command.
PT Mode: Number of pulse commands received by the drive.
PR Mode: Absolute coordinates of the position command.
PUU
002 (02h)
Position deviation
(PUU)
B, DEC
Error between the Command position and Feedback position.
PUU
003 (03h)
Feedback position
(count)
B, DEC
Current feedback position of the motor encoder.
count
004 (04h)
Position command
(count)
B, DEC
Value of the Position command.
This value is after the E-Gear ratio conversion.
count
005 (05h)
Position deviation
(count)
B, DEC
Error between the Command position and Feedback position.
count
006 (06h)
Pulse command
frequency
B, DEC
Frequency of the pulse command received by the drive.
Applicable to PT, PR and I/O (Gear) modes
KHz
007 (07h)
Speed feedback
B, DEC
Motor speed. This is the speed after applying the low-pass filter.
0.1 rpm

**Extracted table (page 11, #1):**

| Variable name | Property | Description |
| --- | --- | --- |
| Feedback position (PUU) | B, DEC | Current feedback position of the motor encoder. |
| Position command (PUU) | B, DEC | Current coordinate of the position command. PT Mode: Number of pulse commands received by the drive. PR Mode: Absolute coordinates of the position command. |
| Position deviation (PUU) | B, DEC | Error between the Command position and Feedback position. |
| Feedback position (count) | B, DEC | Current feedback position of the motor encoder. |
| Position command (count) | B, DEC | Value of the Position command. This value is after the E-Gear ratio conversion. |
| Position deviation (count) | B, DEC | Error between the Command position and Feedback position. |
| Pulse command frequency | B, DEC | Frequency of the pulse command received by the drive. Applicable to PT, PR and I/O (Gear) modes |
| Speed feedback | B, DEC | Motor speed. This is the speed after applying the low-pass filter. |

<!-- page 12 -->

008 (08h)
Speed command
(analog)
B, DEC
Speed command derived from the analog speed terminals.
0.01 Volt
009 (09h)
Speed command
(integrated)
B, DEC
Integrated Speed command. Source includes analog, register, or position loop.
0.1 rpm
010 (0Ah)
Torque command
(analog)
B, DEC
Torque command derived from the analog torque terminals.
0.01 Volt
011 (0Bh)
Torque command
(integrated)
B, DEC
Integrated Torque command. Source includes analog, register, or speed loop.
percentage (%)
012 (0Ch)
 Average load rate
B, DEC
Average load rate (moving average every 20 ms) from the servo drive.
percentage (%)
013 (0Dh)
 Peak load rate
B , DEC
This can be used to monitor a motor overload condition.
percentage (%)
014 (0Eh)
 DC Bus voltage
 B, DEC
 Rectified capacitor voltage.
Volt
015 (0Fh)
Load inertia ratio
B, DEC
Ratio of the load inertia to the motor inertia.
0.1 times
016 (10h)
 IGBT temperature
B, DEC
Temperature of IGBT.
°C
017 (11h)
Resonance frequency
B, DEC
Resonance frequency of the system consists of two groups of frequencies: F1 and F2. The low word
is frequency F2, when the high word is frequency F1.
Hz
018 (12h)
Z phase offset
B, DEC
Offset value between motor position and Z phase; range: -4999…+5000 (-180 degrees to 180
degrees).
Where it overlaps with Z phase, the value is 0; the greater the value, the greater the offset.
180/5000 degree
019 (13h)
Mapping parameter
content #1
B, DEC
Returns the value of P0.025, which is mapped by P0.035.
—
020 (14h)
Mapping parameter
content #2
B, DEC
Returns the value of P0.026, which is mapped by P0.036.
—
021 (15h)
Mapping parameter
content #3
B, DEC
Returns the value of P0.027, which is mapped by P0.037.
—
022 (16h)
Mapping parameter
content #4
B, DEC
Returns the value of P0.028, which is mapped by P0.038.
—
023 (17h)
Mapping monitoring
variable #1
B, DEC
Returns the value of P0.009, which is mapped by P0.017.
—
024 (18h)
Mapping monitoring
variable #2
B, DEC
Returns the value of P0.020, which is mapped by P0.018.
—
025 (19h)
Mapping monitoring
variable #3
B, DEC
Returns the value of P0.011, which is mapped by P0.019.
—
026 (1Ah)
Mapping monitoring
variable #4
B, DEC
Returns the value of P0.012, which is mapped by P0.020.
—
027 (1Bh)
Z phase offset (PUU
Unit)
B, DEC
Offset value between motor position and Z phase (panel only)
PUU
028 (1Ch)
Alarm code
B, DEC
The Error Code (Reserved for future release).
—
029 (1Dh)
Auxiliary encoder
feedback
DEC
Position feedback from the auxiliary encoder.
PUU
030 (1Eh)
Position error (PUU)
DEC
Error from the position command and feedback position of the auxiliary encoder.
PUU
031 (1Fh)
Main / auxiliary
encoder position
deviation (PUU)
DEC
Error between the feedback position of the main encoder and auxiliary encoder.
PUU
035 (23h)
Indexing coordinate
command
DEC
Current command for the indexing coordinates.
PUU
037 (25h)
Compare data of
COMPARE
DEC
The actual Compare data is the Compare data plus a specified value: CMP_DATA = DATA_ARRAY[*] +
P1.023 + P1.024.
PUU
039 (27h)
DI status (integrated)
DEC
Integrated DI status of the drive. Each bit corresponds to one DI channel. Source includes Hardware
channel / ID281 (P4.007), which is determined by ID268 (P3.006).
—
040 (28h)
DO status (hardware)
DEC
Actual status from the DO hardware. Each bit corresponds to one DO channel.
—
041 (29h)
Status of the drive
DEC
Returns ID280 (P4.006) Refer to the description of this parameter.
—
043 (2Bh)
Latest capture data
DEC
The latest data captured by CAP hardware. CAP can continuously capture multiple points.
PUU
048 (30h)
Auxiliary encoder CNT
DEC
Pulse counts from the auxiliary encoder.
count
049 (31h)
Pulse command CNT
DEC
Pulse counts from the pulse command (I/O Terminal block input).
count
050 (32h)
Speed command
(integrated)
DEC
Integrated Speed command. Source includes analog, register, or position loop.
 0.1 rpm
051 (33h)
Speed feedback
(immediate)
DEC
Actual motor speed.
 0.1 rpm

> **Table 139** — System Variables Code (Continued)

Code
Variable name
Property
Description
User Unit(1)

**Extracted table (page 12, #1):**

| Variable name | Property | Description |
| --- | --- | --- |
| Speed command (analog) | B, DEC | Speed command derived from the analog speed terminals. |
| Speed command (integrated) | B, DEC | Integrated Speed command. Source includes analog, register, or position loop. |
| Torque command (analog) | B, DEC | Torque command derived from the analog torque terminals. |
| Torque command (integrated) | B, DEC | Integrated Torque command. Source includes analog, register, or speed loop. |
| Average load rate | B, DEC | Average load rate (moving average every 20 ms) from the servo drive. |
| Peak load rate | B , DEC | This can be used to monitor a motor overload condition. |
| DC Bus voltage | B, DEC | Rectified capacitor voltage. |
| Load inertia ratio | B, DEC | Ratio of the load inertia to the motor inertia. |
| IGBT temperature | B, DEC | Temperature of IGBT. |
| Resonance frequency | B, DEC | Resonance frequency of the system consists of two groups of frequencies: F1 and F2. The low word is frequency F2, when the high word is frequency F1. |
| Z phase offset | B, DEC | Offset value between motor position and Z phase; range: -4999…+5000 (-180 degrees to 180 degrees). Where it overlaps with Z phase, the value is 0; the greater the value, the greater the offset. |
| Mapping parameter content #1 | B, DEC | Returns the value of P0.025, which is mapped by P0.035. |
| Mapping parameter content #2 | B, DEC | Returns the value of P0.026, which is mapped by P0.036. |
| Mapping parameter content #3 | B, DEC | Returns the value of P0.027, which is mapped by P0.037. |
| Mapping parameter content #4 | B, DEC | Returns the value of P0.028, which is mapped by P0.038. |
| Mapping monitoring variable #1 | B, DEC | Returns the value of P0.009, which is mapped by P0.017. |
| Mapping monitoring variable #2 | B, DEC | Returns the value of P0.020, which is mapped by P0.018. |
| Mapping monitoring variable #3 | B, DEC | Returns the value of P0.011, which is mapped by P0.019. |
| Mapping monitoring variable #4 | B, DEC | Returns the value of P0.012, which is mapped by P0.020. |
| Z phase offset (PUU Unit) | B, DEC | Offset value between motor position and Z phase (panel only) |
| Alarm code | B, DEC | The Error Code (Reserved for future release). |
| Auxiliary encoder feedback | DEC | Position feedback from the auxiliary encoder. |
| Position error (PUU) | DEC | Error from the position command and feedback position of the auxiliary encoder. |
| Main / auxiliary encoder position deviation (PUU) | DEC | Error between the feedback position of the main encoder and auxiliary encoder. |
| Indexing coordinate command | DEC | Current command for the indexing coordinates. |
| Compare data of COMPARE | DEC | The actual Compare data is the Compare data plus a specified value: CMP_DATA = DATA_ARRAY[*] + P1.023 + P1.024. |
| DI status (integrated) | DEC | Integrated DI status of the drive. Each bit corresponds to one DI channel. Source includes Hardware channel / ID281 (P4.007), which is determined by ID268 (P3.006). |
| DO status (hardware) | DEC | Actual status from the DO hardware. Each bit corresponds to one DO channel. |
| Status of the drive | DEC | Returns ID280 (P4.006) Refer to the description of this parameter. |
| Latest capture data | DEC | The latest data captured by CAP hardware. CAP can continuously capture multiple points. |
| Auxiliary encoder CNT | DEC | Pulse counts from the auxiliary encoder. |
| Pulse command CNT | DEC | Pulse counts from the pulse command (I/O Terminal block input). |
| Speed command (integrated) | DEC | Integrated Speed command. Source includes analog, register, or position loop. |
| Speed feedback (immediate) | DEC | Actual motor speed. |

<!-- page 13 -->

053 (35h)
Torque command
(integrated)
DEC
Integrated Torque command. Source includes analog, register, or speed loop.
 0.1%
054 (36h)
Torque feedback
DEC
Actual motor torque.
0.1%
055 (37h)
Current feedback
DEC
Actual motor current.
0.01 A (ampere)
056 (38h)
DC Bus voltage
DEC
Rectified capacitor voltage.
0.1 Volt
057(39h)
ECAM engage status
DEC
0-stop, 1-engaged, 2-pre-engaged
—
059 (3Bh)
Pulse from E-CAM
master axis
(accumulative)
DEC
Accumulative pulse number of the E-CAM master axis. Same as ID374 (P5.086).
Same as the
master axis pulse
060 (3Ch)
Pulse from E-CAM
master axis
(incremental)
DEC
Incremental pulse number of the E-CAM master axis. The increment per ms.
Same as the
master axis pulse
061 (3Dh)
Pulse from E-CAM
master axis (lead pulse) DEC
The lead pulse of the E-CAM master axis which determines the engagement condition.When
disengaged: lead pulse = ID375 (P5.087) or ID380 (P5.092); when the value is 0, E-CAM engages.When
engaged: lead pulse = ID377 (P5.089); when the value is 0, it disengages.
Same as the
master axis pulse
062 (3Eh)
Position of E-CAM
master axis
DEC
Position of the E-CAM which corresponds to the master axis pulse, and can be used to find the phase
of the E-CAM. when the incremental pulse number of the master axis is P, E-CAM rotates M cycles,
where ID371 (P5.083) = M, ID372(P5.084) = P.
Same as the
master axis pulse
063 (3Fh)
Position of E-CAM slave
axis
DEC
Position of the E-CAM slave axis and can be found from the E-CAM table.Unit: unit used in the E-CAM
table.
PUU
064 (40h)
Endpoint register of PR
command
DEC
In PR Mode, the endpoint of the Position command (Cmd_E).
PUU
065 (41h)
Output register of PR
command
DEC
In PR Mode, the accumulative output of the Position command.
PUU
067 (43h)
PR target speed
DEC
Target speed specified in the PR command.
0.1 RPM or PPS
(pulse per
second)
068 (44h)
S-curve (input)
DEC
Input data of the S-curve filter.Effective in PR Mode, E-CAM, and register Speed command.
PUU
069 (45h)
S-curve (output)
DEC
Output data of the S-curve filter.Effective in PR Mode, E-CAM, and register Speed command.
PUU
072 (48h)
Speed command
(analog)
DEC
Speed command from the analog channel.
0.1 rpm
085 (55h)
E-CAM alignment
deviation percentage
DEC
The alignment error rate after filtering. 10 indicates 1% and the angle conversion is 360° × 1% = 3.6°. 0.1%
091 (5Bh)
Indexing coordinate
feedback
DEC
Immediate feedback position of the indexing coordinates.
PUU
096 (60h)
Drive firmware version DEC
Includes 2 versions: DSP and CPLD.
• Low word returns the DSP version number
• High word returns the CPLD version number
—
111 (6Fh)
Error code of the servo
drive
DEC
Error code from the servo drive: control loop of the servo only, not including the motion controller.
—
112 (70h)
Encoder
communication error
rate
DEC
When this value continues to increase, it indicates that there is communication interference. In an
interference-free environment, this value should not increase.
—
113 (71h)
Overload (E006)
protection counter
DEC
Displays the motor load during operation. When the value reaches 100%, E006 occurs.
—
114 (72h)
Encoder temperature
DEC
Monitor the encoder temperature.
°C
115 (73h)
Encoder type
DEC
Displays the encoder type.
—
116 (74h)
Deviation between
position and Z phase of
auxiliary encoder
(pulse)
DEC
Distance between the current feedback position of the auxiliary encoder and the Z phase position of
the auxiliary encoder.
count
117 (75h)
Hall sensor phase
sequence and Z pulse
data from auxiliary
encoder feedback
DEC
Use the bit to determine the UVW phase sequence of the Hall sensor and Z pulse from auxiliary
encoder feedback. Bit 0: Z pulse, Bit 1: U phase, Bit 2: V phase, Bit 3: W phase.
118 (76h)
Hall sensor phase
sequence and Z pulse
data from main
encoder feedback
DEC
Use the bit to determine the UVW phase sequence of the Hall sensor and Z pulse from main encoder
feedback. Bit 0: Z pulse, Bit 1: U phase, Bit 2: V phase, Bit 3: W phase.
123 (7Bh)
Value returned when
monitoring by panel
—
Monitoring value displayed when returned to the monitoring panel.
—

> **Table 139** — System Variables Code (Continued)

Code
Variable name
Property
Description
User Unit(1)

**Extracted table (page 13, #1):**

| Variable name | Property | Description |
| --- | --- | --- |
| Torque command (integrated) | DEC | Integrated Torque command. Source includes analog, register, or speed loop. |
| Torque feedback | DEC | Actual motor torque. |
| Current feedback | DEC | Actual motor current. |
| DC Bus voltage | DEC | Rectified capacitor voltage. |
| ECAM engage status | DEC | 0-stop, 1-engaged, 2-pre-engaged |
| Pulse from E-CAM master axis (accumulative) | DEC | Accumulative pulse number of the E-CAM master axis. Same as ID374 (P5.086). |
| Pulse from E-CAM master axis (incremental) | DEC | Incremental pulse number of the E-CAM master axis. The increment per ms. |
| Pulse from E-CAM master axis (lead pulse) | DEC | The lead pulse of the E-CAM master axis which determines the engagement condition.When disengaged: lead pulse = ID375 (P5.087) or ID380 (P5.092); when the value is 0, E-CAM engages.When engaged: lead pulse = ID377 (P5.089); when the value is 0, it disengages. |
| Position of E-CAM master axis | DEC | Position of the E-CAM which corresponds to the master axis pulse, and can be used to find the phase of the E-CAM. when the incremental pulse number of the master axis is P, E-CAM rotates M cycles, where ID371 (P5.083) = M, ID372(P5.084) = P. |
| Position of E-CAM slave axis | DEC | Position of the E-CAM slave axis and can be found from the E-CAM table.Unit: unit used in the E-CAM table. |
| Endpoint register of PR command | DEC | In PR Mode, the endpoint of the Position command (Cmd_E). |
| Output register of PR command | DEC | In PR Mode, the accumulative output of the Position command. |
| PR target speed | DEC | Target speed specified in the PR command. |
| S-curve (input) | DEC | Input data of the S-curve filter.Effective in PR Mode, E-CAM, and register Speed command. |
| S-curve (output) | DEC | Output data of the S-curve filter.Effective in PR Mode, E-CAM, and register Speed command. |
| Speed command (analog) | DEC | Speed command from the analog channel. |
| E-CAM alignment deviation percentage | DEC | The alignment error rate after filtering. 10 indicates 1% and the angle conversion is 360° × 1% = 3.6°. |
| Indexing coordinate feedback | DEC | Immediate feedback position of the indexing coordinates. |
| Drive firmware version | DEC | Includes 2 versions: DSP and CPLD. • Low word returns the DSP version number • High word returns the CPLD version number |
| Error code of the servo drive | DEC | Error code from the servo drive: control loop of the servo only, not including the motion controller. |
| Encoder communication error rate | DEC | When this value continues to increase, it indicates that there is communication interference. In an interference-free environment, this value should not increase. |
| Overload (E006) protection counter | DEC | Displays the motor load during operation. When the value reaches 100%, E006 occurs. |
| Encoder temperature | DEC | Monitor the encoder temperature. |
| Encoder type | DEC | Displays the encoder type. |
| Deviation between position and Z phase of auxiliary encoder (pulse) | DEC | Distance between the current feedback position of the auxiliary encoder and the Z phase position of the auxiliary encoder. |
| Hall sensor phase sequence and Z pulse data from auxiliary encoder feedback | DEC | Use the bit to determine the UVW phase sequence of the Hall sensor and Z pulse from auxiliary encoder feedback. Bit 0: Z pulse, Bit 1: U phase, Bit 2: V phase, Bit 3: W phase. |
| Hall sensor phase sequence and Z pulse data from main encoder feedback | DEC | Use the bit to determine the UVW phase sequence of the Hall sensor and Z pulse from main encoder feedback. Bit 0: Z pulse, Bit 1: U phase, Bit 2: V phase, Bit 3: W phase. |
| Value returned when monitoring by panel | — | Monitoring value displayed when returned to the monitoring panel. |

<!-- page 14 -->

(1)
PUU is Position of User Unit; count is encoder unit.

<!-- page 15 -->

Description of Parameter
Monitoring
These parameter values are used in IO operation mode to pass parameter
values from the drive to the Logix controller as part of the input assembly.
These parameters are configured in KNX5100C software from
Function List > Parameter Editor > Status Monitor > ID060…ID064. These
parameters cannot be modified if there is an active Ethernet/IP network
connection between the drive and controller. These configurations must be
made before the connection is established, or while the connection is
inhibited.
Parameters can be monitored by using ID55(P0.025)…ID59(P0.029), which
contents are specified by ID60(P0.035)…ID64(P0.039).
Example 1: When ID60 (P0.035) is 1, then the value of ID55 (P0.025) is equal to
ID1 (PM.000). When you monitor ID55 (P0.025) in Logix, the value of ID60
(P0.035) (mapped to ID1) is passed at the drive update.
Example 2: When ID60(P0.035) is 4, then the value of ID55 (P0.025) is equal to
ID4 (PM.029). When you monitor ID55 (P0.025) in Logix, the value of ID60
(P0.035) (mapped to ID4) is passed at the drive update.

> **Table 140** — Relevant Parameters

Parameter
Name
Description
ID55 (P0.025)
ParameterMonitor1Value
Parameter Monitor 1 Value
You can use ID60 (P0.035) to specify the mapping
parameter instance ID number. The content of the
parameter that is specified by ID60 (P0.035) is
shown in ID55 (P0.025).
ID56 (P0.026)
ParameterMonitor2Value
Parameter Monitor 2 Value
You can use ID61 (P0.036) to specify the mapping
parameter instance ID number. The content of the
parameter that is specified by ID61 (P0.036) is
shown in ID56 (P0.026).
ID57 (P0.027)
ParameterMonitor3Value
Parameter Monitor 3 Value
You can use ID62 (P0.037) to specify the mapping
parameter instance ID number. The content of the
parameter that is specified by ID62 (P0.037) is
shown in ID56 (P0.027).
ID58 (P0.028)
ParameterMonitor4Value
Parameter Monitor 4 Value
You can use ID63 (P0.038) to specify the mapping
parameter instance ID number. The content of the
parameter that is specified by ID63 (P0.038) is
shown in ID57 (P0.028).
ID59 (P0.029)
ParameterMonitor5Value
Parameter Monitor 5 Value
You can use ID64 (P0.039) to specify the mapping
parameter instance ID number. The content of the
parameter that is specified by ID64 (P0.039) is
shown in ID57 (P0.028).
ID60 (P0.035)
ParameterMonitor1Selection
The content of the parameter that is specified by
ID60 (P0.035) is shown in ID55 (P0.025).
ID61 (P0.036)
ParameterMonitor2Selection
The content of the parameter that is specified by
ID61 (P0.036) is shown in ID56 (P0.026).
ID62 (P0.037)
ParameterMonitor3Selection
The content of the parameter that is specified by
ID62 (P0.037) is shown in ID57 (P0.027).
ID63 (P0.038)
ParameterMonitor4Selection
The content of the parameter that is specified by
ID63 (P0.038) is shown in ID58 (P0.028).
ID64 (P0.039)
ParameterMonitor5Selection
The content of the parameter that is specified by
ID64 (P0.039) is shown in ID59 (P0.029).

<!-- page 16 -->

This example describes the passing of the DC Bus voltage parameter, which is
only represented as a SystemVariable, to the ParameterMonitor variable so it
can be monitored within the Logix tag structure.

> **Figure 237** — Parameter Monitoring

1.
Navigate to Function List>Settings>Monitoring Status.
By default, setting (14) is the DC Bus Voltage value.
2. Find an available SystemVariableMonitorxSelection placeholder; enter
the numerical value of the DC Bus Voltage (14).
3.
This present DC Bus Voltage appears in the
SystemVariableMonitorxValue when this is downloaded to the drive.
4. Enter the ID of the SystemVariableMonitorxValue you setup in Step 3
(663).
The SystemVariables are not accessible in the Logix tag subsystem. The
ParameterMonitor values are accessible.
5.
When there is an active connection wtih Logix, you can monitor the
Input Assembly (or Device Object Add-On Instruction) and see the value
of the ParameterxMonitorValue.

<!-- page 17 -->

Use a MSG Instruction to Set
Parameters
When the drive is not using IO operation mode, Class 3 EtherNet/IP™
messaging is allowed and some Parameter IDs can be read/written. Below is
an example of a write operation that is performed in the Logix Designer
application. Alternately, you can use a MSG instruction to set parameters by
following these steps.
1.
Create a Parameter Write MSG instruction in the ladder logic program.
2. Use the parameter ID as the instance.
3.
Select or create a Source Element, and specify the length of it.
4. Configure the communication path.

<!-- page 18 -->

Notes:
