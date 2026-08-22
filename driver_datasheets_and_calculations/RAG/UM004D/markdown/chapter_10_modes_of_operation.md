# Chapter 10: Modes of Operation

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `11_Ch10_Modes_of_Operation.pdf` (52 pages)_

<!-- page 1 -->

Select Operation Mode and
Direction Control
You can change the Direction Control and Operation Mode by using
KNX5100C software and by changing parameters, either programmatically or
by using the Parameter Editor.
Change using the Parameter Editor by using KNX5100C Software or
Programmatically
Changing the Operation Mode with Programming/Parameter Editor
1.
Disable the drive (Servo power is off).
2. Set ID117 (P1.001) and refer to YX: Control Mode Setting for the mode
selection.
3.
After setting the parameter, cycle power to the servo drive.
The following tables show how to set the ID117 (P1.001) Control Mode
parameter.
Settings:
Dual mode(1)
PT-S
Switches PT and S mode with DI signals.
PT-T
Switches PT and T mode with DI signals.
PR-S
Switches PR and S mode with DI signals.
PR-T
Switches PR and T mode with DI signals.
S-T
Switches S and T mode with DI signals.
–
Reserved
PT-PR
Switches PT and PR mode with DI signals.
Multi-mode(1)
PT-PR-S
Switches PT, PR, and S mode with DI signals.
PT-PR-T
Switches PT, PR, and T mode with DI signals.
(1)
When these modes are used, the changes are immediate, which can result in unintended motion.

> **Table 86** — Kinetix 5100 Drive Control Modes (Continued)

## Control Mode

Short Name
Description
YX =
Control Mode Setting Z =
Directional Control
U =
DIO Setting Value Control
Where:
See below
0 = Forward direction
0 = Same value
1 = Reverse direction
1 = Resets to default value

**Extracted table (page 1, #1):**

| YX = Control Mode Setting | Z = Directional Control |
| --- | --- |
| See below | 0 = Forward direction |
|  | 1 = Reverse direction |

<!-- page 2 -->

•
YX: Control Mode Setting
•
Z: Direction Control
•
U: DIO Setting Value Control
Setting No.
Description

When switching modes, DIO settings ID195…ID207 (P2.010…P2.022) remain the same
value.

When switching modes, DIO settings ID195…ID207 (P2.010…P2.022) and ID220...ID225
(P2.036...P2.041)are reset to the default of each mode.
Forward direction
Reverse direction

<!-- page 3 -->

Change Operation Mode by using KNX5100C Software
From Function List > Settings, use the pull-down menu to select your new
drive operating mode or select the mode from the top status bar next to the
Offline/Online status.
Change the Motor Rotation Polarity by using KNX5100C Software
From Function List > Settings > General Setting, choose the direction that
represents your desired direction of rotation.
Position Control
Three input modes for position control are available on the drive: External
pulse or analog input (PT Mode), internal register (PR Mode) and IO Mode.
In PT Mode, the servo drive is able to receive either analog (+/- 10V analog
input) used to position or pulse commands that represent position step and
motor direction. The drive can handle an input pulse rate up to 4 MHz.
In PR Mode (Position Register) the drive's indexing and program capabilities
are used. The drive provides 99 command registers and these position registers
are executed in one of two ways:
•
Using standalone PR operation mode.
- Pre-program the program registers (up to 99 individual registers)
- Enable the drive
- Use DIPOS0…DIPOS6 signals on the I/O connector to represent the
binary weighted PR number to execute
- Execute the PR commands using the DI Command Triggered
- You can directly set the register values via communication

<!-- page 4 -->

•
Using IO operation mode.
The position command can also come from the Logix controller when the
operating mode ID117 (P1.001) is set to IO Mode (0xC). There are several
drive commands that you can execute in this mode. These commands
include:
- Jogging
- Indexing
- Gearing
Details are found in IO Mode on page 271.
PT Mode (Position Command with I/O Terminal Block Input)
You can configure PT Mode by using KNX5100C software or by directly
changing the drive parameters. There are three pulse types and each type has
positive/negative logic, which can be set in ID116 (P1.000).
The following tables explain how to set the ID116 (P1.000) External Pulse Type
parameter.
Settings:
X =Command Source
Z =Logic Type
UY =Filter Width
Where:
0 = I/O, AB phase pulse (4x)
1 = I/O, Clockwise and counterclockwise pulse
2 = I/O, Pulse + direction
3 = Reserved
4 = AUX, AB phase pulse (4X)
5 = AUX, clockwise and counterclockwise pulse
6 = AUX, pulse + direction
0 = Positive logic
1 = Negative logic
See UY: Filter Width Setting on
page 239

<!-- page 5 -->

•
Z: Logic Type
Digital circuits use 0 and 1 to represent the voltage level of low and high
respectively. Using positive logic, 1 represents high voltage and 0 represents
low voltage; using negative logic, 1 represents low voltage and 0 represents
high voltage.
For example:
MHz
kHz
kHz
MHz
Positive logic
Negative logic
Logic
type
Logic
type
Pulse
type
Pulse
type
Pulse input
Pulse phase lead
Pulse phase lag
Forward
Reverse
Pulse input
Reverse
Forward
Sign = low
Sign = high
Pulse Specification
Min. Allowed Time Width
Max. Input
Frequency
Max. Input Frequency
Pulse Specification
Differential signal
Differential signal
Open-collector
Open-collector
Voltage
Forward current

<!-- page 6 -->

•
UY: Filter Width Setting
As the pulse frequency increases, the pulse width becomes smaller. When the
pulse width becomes smaller than the filter width setting, those pulses are
filtered out as noise. Thus, the filter width must be set smaller than the actual
pulse width. Set the filter width to be four times smaller than the actual pulse
width.
For example:
Y Setting Value
U = 0
Unit: µs (kHz)
U = 1
Unit: µs (kHz)

No filter function
No filter function

2 (250)
0.2 (2500)

3 (166)
0.3 (1666)

4 (125)
0.4 (1250)

5 (100)
0.5 (1000)

6 (83)
0.6 (833)

7 (71)
0.7 (714)

8 (62)
0.8 (625)

9 (55)
0.9 (555)

10 (50)
1 (500)
A
11 (45)
1.1 (454)
B
12 (41)
1.2 (416)
C
13 (38)
1.3 (384)
D
14 (35
1.4 (357)
E
15 (33)
1.5 (333)
If
Then
U is set to 1 and Y is set to 1 (filter width is 0.2 µs
[2500 kHz])
and
The high and low duty width of the command pulse are
both larger than 0.8 µs (625 kHz) (filter width is four times
of 0.2 µs [2500 kHz])
The pulse command is not filtered out.
When the high or low duty width of the pulse is smaller than the filter width, then the pulse command is
filtered out.
If this first pulse width is shorter than 0.8 µs (625 kHz), it
can be filtered, and thus two input pulses are regarded as
one pulse. If this pulse width is shorter than 0.2 µs (2500
kHz), it is filtered.
If this low level pulse width is shorter than 0.8 µs
(625 kHz), it can be filtered, and thus two input
pulses are regarded as one pulse. If this low level
pulse width is shorter than 0.2 µs (2500 kHz), it is
filtered.
If 125 ns (4 MHz) input pulse is used, set the filter setting value Y to 0 as no filter function.
IMPORTANT
When the signal is the high-speed pulse specification of 125 ns
(4 MHz) and the setting value of the filter is 0, then the pulse is not
filtered.

<!-- page 7 -->

## Pulse Command Input Inhibitor (INHP)

In PT Mode, when a DI is configured for Disable External Pulse (DI.INHP) and
the signal is on, the drive ignores incoming pulses and the motor stops motion.
Using this Pulse Command Inhibit feature requires you to configure DI8 as the
Disable External Pulse DI. A different Digital Input cannot be used for this
feature.
For more information on the INHP function, see the Description of Digital
Input Functions on page 433.
Analog Input
The position using analog input mode is active when the drive is in PT Mode
and the Command Source is set for Analog Input ID167 (P1.064 X=1). The
source for the analog position command comes from two terminals of the 50pin I/O connector: 42 (V_REF) and 44 (GND).

> **Figure 97** — Analog Input

DI.INHP
ON
OFF
ON
Pulse
Command

<!-- page 8 -->

The decryption of ID167 (P1.064) is shown in the segment display image that
corresponds to the table:
Settings for ID168 (P1.065):
The analog position command smoothing constant is only valid for analog
position commands. This value is the amount of smoothing that is used
essentially for speed control when the motor is moving towards the analog
setpoint. The smaller value (0) is a very aggressive speed that is used to move to
the analog setpoint. The largest value (1000) is a very slow speed that is used to
move to the analog setpoint.

> **Table 87** — Relevant Parameters

Parameter
Name
ID167 (P1.064)
AnalogToPositionStartupConfiguration
ID168 (P1.065)
AnalogToPositionSmoothTime
ID169 (P1.066)
AnalogToPositionMaxRotationNum
Bit X:
Bit Y:
Bit U/Z:
0 = The analog position
function is not used
(disabled).
0 = When the drive transitions to ON, the current motor position is
considered 0V; the motor now moves to the corresponding motor position
based on the voltage level at V_REF/GND. The amount of movement is
defined by the volts/rev setting shown below and the speed of the
movement uses the Smoothing Constant described below.
Reserved
1 = The analog position
function is used (enabled).
1 = When the drive transitions to ON, the current motor position is
considered 0V; the motor now moves to the corresponding motor position
based on the voltage level at V_REF/GND. The amount of movement is
defined by the volts/rev setting shown below and the speed of the
movement uses the Smoothing Constant described below.
IMPORTANT
Choose the smoothing constant carefully because aggressive speed
settings can potentially damage your equipment. By using a smoothing
constant of 0 or 1, you are creating, essentially, a step input to
maximum speed as defined by the Volts/(Max. Rotation Number). A
typical value for the smoothing constant is 200…400.

<!-- page 9 -->

Settings for ID169 (P1.066):
This parameter is the maximum number of revolutions used when the
maximum analog input voltage (+10V) is reached. Analog position command is
determined as follows:
For example:
If the parameter ID169 (P1.066) is set to 30 from the panel and the external
voltage is input 5V, then the analog position command is as follows:
Latch Function of Analog Position Command (DI code:0x0C)
When a Digital Input (DI) is configured for Latch function of analog Position
command, and this DI is ON, the motor position is held at the current position
when the DI was triggered. While this DI is ON, the motor does not move, even
if there is a change of analog input command. When this DI is OFF, the motor
completes any change to the position command while the DI was ON.
x ID168 (P1.066) setting = Analog position command revolutions
Input voltage value

x 30 = 15 revolutions
5V

Motor
position
when DI is
triggered
Voltage
when DI is
triggered
When DI is on,
analog input
command
changed the
amount; motor
does not move
When DI is off, motor
operates to the
corresponding position
inputted by analog
Motor position
(Tum)
Analog input
command (V)

<!-- page 10 -->

Clear Function of Analog Position Command (DI code:0x0D)
When a Digital Input (DI) is configured for Clear function of analog Position
command, and this DI is ON, the motor position is held at the current position
when the DI transitions ON.
While this DI is ON, the motor does not move, even if there is a change of
analog input command. When the motor transitions to OFF, the current motor
position is redefined to the position that corresponds with the analog input
voltage on VREF/GND.
PR Mode (Position Command with Internal Register Input)
PR Mode is typically used to control your application when standalone
operation is required. This mode contains the indexing functionality and
program control that you can customize for your application.
The drive supports the following motion command types, which are described
in detail in Chapter 11, Motion Control in PR Mode:
•
Homing
•
Speed
•
Position (indexing)
•
Jump
•
Parameter write
•
Arithmetic operation
The drive provides the following methods to initiate a PR:
•
Digital input (DI)
•
Event-triggered
•
PRCmdTrigger Digital Input ID300 (P5.007) with PR selection using
binary weighted Digital Inputs
•
Capture (high-speed position capturing)
•
Compare-triggered (high-speed position comparing)
•
E-CAM
Motor
position
when DI is
triggered
Voltage when DI is
triggered
When DI is on, all
input command is
cleard
When DI is off, motor is still,
but its coordinate is
redefined
Motor position
(Tum)
Analog input
command (V)
Analog input
command (V)

<!-- page 11 -->

You can choose the most suitable triggering method according to the
applications and requirements. For detailed descriptions of the methods, see
Trigger Methods for PR Commands on page 354.
Control Structure of Position Mode
The basic control structure is as follows:
For better control, the pulse signals are processed by the position command
processing unit. Structure is shown in the following diagram.
Position
command
processing unit
Position control
unit
Position command
Speed loop
Current loop
Motor
Command Source
ID397…ID524
(P6.000…P7.027)
Notch Filter
ID135…ID136
(P1.025…P1.026)
Command Selection
ID117 (P1.001)
1. P1.001
2. P1.064
1. P1.001
2. I/O mode par
1. P1.001
2. I/O mode par
1. P1.001
2. I/O mode par
Moving Filter
ID170 (P1.068)
Accel/Decel Time
ID312…ID327
(P5.020…P5.035)
Low-pass Filter
ID124 (P1.008)
Notch Filter
ID137…ID138
(P1.027…P1.028)
Pulse Type
Selection
ID116 (P1.000)
Counter
Position Command Processing Unit
High
Speed
Pulse Signal
Inhibit(1)
Inhibit(1)
I/O Controller
POS0…POS6
CRTG
Low
Speed
Delay Time
ID332…ID347
(P5.040…P5.055)
CyclicMove Type
(for example, YX
of ID399 [P6.002])
Acceleration
Reference
Starting Index
I/O Mode - Index
Position Reference
I/O Mode - Position
Deceleration
Reference
Acceleration
Reference
Position Reference
I/O Mode - Home
I/O Mode - Gear
Home
Return
Speed
Speed
Reference
Homing
Method
Deceleration
Reference
Target Speed
ID352…ID367
(P5.060…P5.075)
GNUM0, GNUM1
GearRatioFollowerCounts N1 - ID151 (P1.044)
GearRatioFollowerCounts N2 - ID236 (P2.060)
GearRatioFollowerCounts N3 - ID237 (P2.061)
GearRatioFollowerCounts N4 - ID238 (P2.062)
GearRatioMasterCounts - ID152 (P1.045)
Numerators
Denominator
S-curve Filter
ID143 (P1.036)
Analog Input
Scaling
ID169 (P1.066)
Analog
Command Filter
ID168 (P1.065)
PTO
AUX
(1)
For more information, see Pulse Command Input Inhibitor (INHP) on page 240.

<!-- page 12 -->

## Position Mode Timing (PR Mode)

In PR Mode, the program (PR) is selected by using DI (binary weighted
POS0…POS6) and triggers the program (PR) using the Command Triggered
(CTRG) Digital Input. These timing diagrams show the timing used to execute
programs in PR Mode. SON represents Servo ON Digital Input.
The following apply to the internal digital output (DO) timing diagram:
• Cmd_OK is on when PR command is completed.
• TPOS is on when the error is smaller than value set by the ID159
(P1.054) In Position Window parameter.
• MC_OK is on when Cmd_OK and TPOS are both on.
When the deviation between the target position and the actual motor position
is smaller than the setting range of ID159 (P1.054) in PR Mode, then the
DO.TPOS signal is on.

> **Table 88** — Relevant Parameter

Parameter
Name
ID159 (P1.054)
InPositionWindow
Internal
register
(PR#0…99)
Motion
Curve
(Speed)
External
DI
Internal
DO
Set the response time with ID194(P2.009)

<!-- page 13 -->

## Speed Mode

This mode is used when the operation mode is set for Speed Control (S) or Zero
speed/Internal speed register mode (Sz).
Speed commands can come from the analog input terminals (COMMAND2Speed, 42, Analog GND, 13). The analog voltage (+/-10V) represents a bidirectional velocity signal and is configured with the Analog I/O page in
KNX5100C software.
Speed commands can come from preset speed registers with digital I/O to
select speeds, the binary weighted DIs represent which preset speeds are
selected. Use these presets with both the Speed (S) and Speed Register (Sz)
mode.
When using IO mode, the raC_xxx_K5100_MAJ Add-On Instruction is used to
provide a constant speed to the motor.
When preset speeds are used back-to-back, there can be a problem with speed
discontinuity. You can use the S-curve smoothing time filter to smooth the
transition between preset speeds. The S-curve filter is explained on page 175
and page 256.
Configure and Select the Preset Speeds
An example of how to set speed presets is shown on page 264.
The speed command origin depends on the Operation Mode setting. Speed
Control mode, Sz mode, and IO mode can all generate speed commands.
When Speed Control mode is used, an analog voltage is used to generate the
speed command. Presets are also available in Speed Control mode. When the
binary weighted speed preset of 0 is used for the preselected speed, the analog
speed terminals are used for the speed command. When Sz mode is used, the
analog speed terminals are ignored.
In the KNX5100C software:
1.
From Function List > Digital I/O/Jog Control, edit the DIO configuration
to add the speed command selection bits (bits 0 and 1).
2. From Function List > Analog I/O, use the pull-down menus for SPD0 and
SPD1 to associate the Digital I/O with the appropriate binary weighting.

<!-- page 14 -->

3.
Enter the Preset Speeds to use for your application.
Notice that you can change the Preset Speeds by using the ID125, ID126
and ID127 (P1.009, P1.010, and P1.011) respectively.
This table shows the binary weighting representation:
Scaling the Analog Command (Speed Mode)
The motor speed command is determined by the analog voltage difference
between V_REF and VGND. Use parameters AnalogToVelocityScale ID147
(P1.040) and AnalogToVelocityScale2 ID679 (P1.081) to adjust the slope of speed
and its range.
Speed Command
SPD0
SPD1
Analog Input Speed

Speed Register 1

Speed Register 2

Speed Register 3

> **Table 89** — Relevant Parameters

Parameter
Name
ID147 (P1.040)
AnalogToVelocityScale
ID679 (P1.081)
AnalogToVelocityScale2
IMPORTANT: Use a digital input 0x0F (shown below) to switch between ID147 (P1.040) and ID679
(P1.081).
Slope is set by ID147
(P1.040)/ID679 (P1.081)
Slope is set by ID147
(P1.040)/ID679 (P1.081)
Analog voltage input (V)

<!-- page 15 -->

Control Structure of Speed Mode
The following diagram illustrates the basic control structure.
The speed command processing unit is to select the command source,
including the parameter ID147 (P1.040) scaling setting for rotation speed and
S-curve setting for smoothing the speed. The speed control unit manages the
gain parameters of the servo drive and calculates the current command for
servo motor.
Motor
Current Loop
Speed
Control
Speed Command
Speed
Command
Processing
Resonance
Suppression
Speed Estimator
Torque Limit
Speed
Command
Processing
Speed Command
Speed Estimator
Speed
Control
Resonance
Suppression
Torque Limit
Current Loop
Motor
Preset Speed
Command
ID125…ID127
(P1.009…P1.011)
Smoothing Time
of S-curve
ID143 (P1.036)
Control Mode
ID117 (P1.001)
Speed Command
Low Pass Filter
Time Constant
ID122 (P1.006)
Velocity Command
Moving Filter
Time Constant
ID164 (P1.059)
Analog to
Speed Scale
ID147 (P1.040)
A/D
Deceleration
Reference
Smoothing Time
of S-curve
ID143 (P1.036)
Acceleration
Reference
Speed
Reference
I/O Mode - Speed
= I/O Assembly Output Parameter
Analog Signal
Feedback Pulse

<!-- page 16 -->

## Speed Mode Timing

In the Speed mode timing diagram, the following applies.
•
‘Off’ signifies the contact is open while ‘On’ signifies the contact is closed.
•
When the drive is in Sz operation mode, the speed command is disabled,
therefore S1=0.
When the drive is in S Speed Control mode, the speed command S1 is
represented as the analog voltage input.
•
SON represents the Servo On digital input and is on when the drive is
enabled.
•
When the drive is enabled (SON=on), the command is selected according
to the state of digital inputs SPD0 and SPD1.
Zero Speed Threshold Function
The Zero Speed Threshold function is enabled when the following conditions
are met:
•
Condition 1: Operation Mode is Speed Control (Operating Mode = S or
Sz)
•
Condition 2: a digital input configured as Zero Speed Threshold (0x05)
and this input is on.
•
Condition 3: Motor speed is lower than the value of the parameter
ZeroSpeedWindow ID145 (P1.038)
•
The analog input is used in this example.
Analog Input ID167 (P1.064 X = 1)

> **Table 90** — Relevant Parameter

Parameter
Name
ID145 (P1.038)
ZeroSpeedWindow
Internal
register
External analog
voltage or zero
External I/O

<!-- page 17 -->

The Zero Speed Threshold (sometimes called ZClamp) feature uses the analog
speed command without acceleration/deceleration to determine if any motor
speed limiting should be performed. The motor speed is limited at zero speed
when the Zero Speed Threshold conditions are true.
The Zero Speed Threshold (sometimes called ZClamp) feature uses the preset
speed commands without acceleration/deceleration to determine if any motor
speed limiting should be performed. The motor speed is limited at zero speed
when the Zero Speed Threshold conditions are true.
Zero Speed
Window ID145
Analog speed command
Motor speed
(Before ZCLAMP is established)
Motor speed
(After ZCLAMP is established)
Motor speed
(After ZCLAMP is established)
Analog speed command
Zero Speed
Window ID145
Motor speed
(Before ZCLAMP is established)

<!-- page 18 -->

These two examples show the Zero Speed Threshold using the different speed
command conditions.
•
1: Command source is analog voltage.
The Zero Speed Threshold feature uses the analog speed command
without acceleration/deceleration to determine if this function is
enabled. When the Zero Speed Threshold conditions are met, the motor
speed decelerates to 0 rpm by S-curve deceleration. If not, the motor
follows the analog speed command through S-curve.
•
1: Command source is register.
The Zero Speed Threshold feature uses the register speed command with
acceleration/deceleration to determine if this function should be
enabled. When the Zero Speed Threshold conditions are met, the motor
speed is set to 0 rpm.
For more information on the Zero Speed Threshold feature, see the
Description of Digital Input Functions on page 433.
Motor speed
(After ZCLAMP is
Analog speed command
Zero Speed
Window ID145
Motor speed
(Before ZCLAMP is established)
Motor speed
(Before ZCLAMP is established)
Analog speed command
Zero Speed
Window ID145
(P1.038)
Motor speed
(After ZCLAMP is
Motor speed
(Before ZCLAMP is established)
Register speed command
Zero Speed
Window ID145
(P1.038)
Motor speed
(After ZCLAMP is established)

<!-- page 19 -->

## Torque Mode

Torque commands can come from the analog input terminals(COMMAND1Torque, 18, Analog GND, 13). The analog voltage (+/-10V) represents a bidirectional torque signal and is configured with the Analog I/O page in
KNX5100C software.
Torque commands can come from preset torque registers with digital I/O to
select different torque values, the binary weighted DIs represent which preset
torques are selected. These presets can be used with both the T (Torque
Control) and Tz operating mode.
When using IO Mode, the raC_xxx_K5100_MAT Add-On Instruction is used to
provide a constant torque to the motor.
Selection of Torque Command
The torque command origin depends on the Operation Mode setting. Torque
Control mode, Tz mode, and I/O mode can all generate torque commands.
When Torque Control mode is used, an analog voltage is used to generate the
torque command. Presets are also available in Torque Control mode. When the
binary weighted torque preset of 0 is used for the preselected torque, the
analog torque terminals are used for the torque command. When Tz mode is
used, the analog torque terminals are ignored.
Configure and Select the Preset Torques
From the KNX5100C software, you can configure the following.
1.
From Function List>Digital I/O/Jog Control, edit the DIO configuration
to add the torque command selection bits (bits 0 and 1)
2. From Function List>Analog I/O, use the pull-down menu for TCM0/
TCM1 to associate the Digital I/O with the appropriate binary weighting.
3.
Enter the Preset Torques to use for your application.
Notice that you can change the Preset Torques by using the ID128, ID129,
and ID130 (P1.012, P1.013, and P1.014) respectively.

<!-- page 20 -->

This table shows the binary weighting representation:
Control Structure of Torque Mode
The following diagram shows the basic control structure of torque mode:
The torque command unit is to specify the torque command source, including
the parameter Analog Voltage Scaling ID148 (P1.041) and S-curve setting. The
torque control unit manages the gain parameters of the servo drive and
calculates the current for servo motor in time; this can only set by commands.
The structure of a torque command unit is as follows.
The upper path is the command from the preset torque register, while the
middle path is the external analog command. The command is selected
according to the status of the DI.TCM0 and DI.TCM1 signals, and with the
Operation Mode set to T or Tz.
The lower path is used when the operation mode is IO mode. The intention is
to use the raC_xxx_K5100_MAT add-on instruction.
Torque Command
TCM0
TCM1
Analog Input Torque

Torque Register 1

Torque Register 2

Torque Register 3

> **Table 91** — Relevant Parameters

Parameter
Name
ID117 (P1.001)
ControlMode
ID123 (P1.007)
TorqueCmdLowPassFilterTime
ID148 (P1.041)
AnalogToTorqueScale
Motor
Current Loop
Speed
Control
Speed
Command
Speed
Command
Processing
Resonance
Suppression
Current Sensor
Torque Limit
Speed
Command
Processing
Speed Command
Current Sensor
Speed
Control
Resonance
Suppressio
Torque Limit
Current Loop
Motor
Analog Voltage
Scaling
ID148 (P1.041)
A/D
Register
ID116 …ID118
(P1.012…P1.014)
Command
Selection
ID117 (P1.001)
Low-pass
Filter
ID123 (P1.007)
I/O Connector DI.TCM0, DI.TCM1 signal
Torque
Ramp Time
Torque
Reference
I/O Mode - Torque
Analog Signal
= I/O Assembly Output Parameter

<!-- page 21 -->

Scaling of Analog Command (Torque Mode)
The motor torque command is controlled by the analog voltage difference
between the T_REF and GND analog signals. The torque slope and its range
can be adjusted by the Analog to Torque Scale parameter.
Motor torque command is based on the following equation:
If the ID148 (P1.041) parameter is set at its default setting of 100 and the
external analog input voltage is 10V, the torque command is 100% of the rated
torque.
If the ID148 (P1.041) parameter is set to 300 and the external analog input
voltage is 10V, the torque command is 300% of the rated torque.
If the ID148 (P1.041) parameter is set at its default setting of 100 and the
external analog input voltage is 5V, the torque command is 50% of the rated
torque.
In Speed, PT and PR Modes set the torque limit corresponding to 10V (max.
voltage) for analog torque limit.

> **Table 92** — Relevant Parameter

Parameter
Name
ID148 (P1.041)
AnalogToTorqueScale
Torque control command =
= Unit %
External analog input voltage x ID148 (P1.041) setting value

Torque control command =
= 100%
10V x 100

Torque control command =
= 300%
10V x 300

Torque control command =
= 50%
5V x 100

Slope is set by ID148
(P1.041)
Analog voltage input (V)
Torque
command

<!-- page 22 -->

## Torque Mode Timing

•
Off signifies the contact is open while on signifies the contact is closed.
•
When it is in Tz mode, the torque command T1 equals 0; when it is in T
mode, the torque command T1 is the external analog voltage input.
•
In the servo-on (SON) state, the command is selected according to the
state of DI.TCM0 and DI.TCM1 inputs.
Register
External analog
voltage or zero
External
DI/O

<!-- page 23 -->

Filter
The Position, Speed, and Torque modes use different filters to remove
unwanted resonance from the drive (these filter types are available for use in
different drive modes). To configure filters, see Configure Filter on page 174
and Configure Notch Filter on page 176.
Position Mode
These filters are used in Position mode.
S-curve Filter (Position Mode)
S-curve filter smooths the motion command in position mode. With this filter,
speed/acceleration can be continuous and jerk is reduced, and a smoother
mechanical operation can be achieved. If the load inertia increases, the
operation of the motor will be influenced by friction and inertia when it starts
or stops rotating. Setting a larger acceleration/deceleration constant of Scurve (TSL) and acceleration/deceleration time (numbers 0…15) in
ID312…ID327 (P5.020…P5.035) can increase the smoothness of operation.
When the position command source is pulse, its speed and angular
acceleration are continuous, thus, S-curve filter is not a must.
Low Pass Filter (Position Mode)
Low pass filter for commands is typically used to filter out unwanted highfrequency response or noise so that the speed becomes smoother.

> **Table 93** — Relevant Parameters

Parameter
Name
ID124 (P1.008)
PositionCmdLowPassFilterTime
IMPORTANT
The filter functions are disabled when the parameter values are set
to 0.
Target
Position
Position Filter

<!-- page 24 -->

## Speed Mode

These filters are used in Speed mode.
S-curve Filter (Speed Mode)
During acceleration or deceleration, the S-curve filter applies the three-stage
acceleration curve and tailors a smoother motion trajectory. It is used to avoid
jerk (the differentiation of acceleration), resonance as well as noise caused by
abrupt speed variation. You can use the parameter ID141 (P1.034), Acceleration
Time Constant of S-Curve Velocity Profile (TACC), to adjust the slope changed
by acceleration; the parameter ID142 (P1.035), Deceleration Time Constant of
S-curve Velocity Profile (TDEC), to adjust the slope changed by deceleration,
and the parameter ID143 (P1.036), Smoothing Time of S-curve (TSL), to
improve the status of motor activation and stop. The drive can calculate the
total time for executing the command.
T (ms) signifies the operation time and S (rpm) signifies the absolute speed
command, which is the absolute value of initial speed minus end speed.
IMPORTANT
These three parameters can be set individually and even when the
parameter ID143 (P1.036), Smoothing Time of S-curve, is set to 0
(disabled), the S-curve still has acceleration/deceleration of a
trapezoidal profile.
Error Compensation Function
When ID143
(P1.036) = 0
When ID143
(P1.036) = 1
When ID143
(P1.036) >1
Smoothing function of S-curve
Disable
Disable
Enable
Following error compensation function
Disable
Enable
Determine by ID 241
(P2.068.X)(1)
(1)
For ID241 (P2.068) following error compensation, 0: Disable or 1: Enable.
Speed
Acceleration
Deceleration
Rated Speed
Torque
Time
(ms)
Time
(ms)
ID141 (P1.034)
ID143
(P1.036)/2
ID143
(P1.036)/2
ID143
(P1.036)/2
ID143
(P1.036)/2

**Extracted table (page 24, #1):**

| When ID143 (P1.036) = 0 | When ID143 (P1.036) = 1 |
| --- | --- |
| Disable | Disable |
| Disable | Enable |

<!-- page 25 -->

## Analog Speed Command Filter

Analog speed command filter provided by the drive helps to smooth motion to
the motor when the analog input signal (speed) changes rapidly.
The above diagram is the curve of speed command and motor torque when
analog speed command filter is applied. In the diagram above, the slopes of
speed command in acceleration and deceleration are different. You can adjust
the time setting by using parameters SCurveAccelTime ID141 (P1.034),
SCurveDecelTime ID142 (P1.035), and SCurveSmoothTime ID143 (P1.036) as
required for your cycle profile.
Low Pass Filter (Speed Mode)
Parameter ID122 (P1.006) filters out unwanted high-frequency resonances or
noise so that the speed becomes smoother.
VelocityCmdLowPassFilterTime ID122 (P1.006) is a low-pass filter, while
VelocityCmdMovingFilterTime ID164 (P1.059) is a moving filter. The Moving
filter applies smoothing at the beginning and end of the Acceleration cycle, the
Low Pass applies smoothing at the end of the cycle.
If the Operation Mode contains a position loop, use the
VelocityCmdLowPassFilterTIme ID122 (P1.006). If the Operation mode is
Speed Control (or Sz), then use VelocityCmdMovingFilterTime ID164 (P1.059).

> **Table 94** — Relevant Parameters

Parameter
Name
ID122 (P1.006)
VelocityCmdLowPassFilterTime
ID164 (P1.059)
VelocityCmdMovingFilterTime
Speed
Motor Torque
Time (sec)
Target Speed
Speed Filter

<!-- page 26 -->

## Resonance Suppression (Notch Filter, Speed Mode)

When resonance occurs, it can be naturally occurring resonance present on the
mechanism or can be as a result of increasing the control loop gains beyond
the limits of the mechanism. Mitigating these two factors can improve the
situation. In addition, parameter Low-pass Filter ID210 (P2.023) and 5 notch
filters (see Table 95 on page 259) are provided to suppress the resonance if the
control parameters remain unchanged.
The drive provides two types of resonance suppression, one is a notch filter
and the other is a low pass filter. See the following diagrams for the results of
suppression by each type.
As shown in the previous two examples, if the value of parameter
ResonanceSuppressionLowPassFilterTime ID210 (P2.025) is increased from 0,
the bandwidth (BW) becomes smaller. Although it solves the problem of
resonance, it also reduces the response bandwidth and phase margin, so either
the control loop gains have to be lowered, or the system can become unstable.
If the resonance frequency is known, you can mitigate the suppression by
using the Notch filter, which is better than the low pass filter in this case. If the
resonance frequency drifts along with time or other causes and the drifting
amount is too great, using a notch filter is not recommended.

> **Table 95** — Relevant Parameters

Parameter
Name
ID208 (P2.023)
NotchFilter1Frequency
ID209 (P2.024)
NotchFilter1Depth
ID226 (P2.043)
NotchFilter2Frequency
ID227 (P2.044)
NotchFilter2Depth
ID228 (P2.045)
NotchFilter3Frequency
ID229 (P2.046)
NotchFilter3Depth
ID254 (P2.095)
NotchFilter1QValue
The system open-loop gain with resonance:
(1) Cutoff frequency of low pass filter = 1000 / ID210 (P2.025) Hz
NOTE: BW = Bandwidth
Notch filter
Low pass filter
Resonance point
suppressed by the
Notch filter
Resonance point
suppressed by the
low pass filter
Point of resonance
Point of resonance
Gain
Gain
Gain
Gain
Gain
Gain
Gain
Resonance
Frequency
Resonance
Frequency
Resonance
Frequency
Resonance
Frequency
 Frequency
 Frequency
 Frequency
 Frequency
 Frequency
 Frequency
BW
BW
BW
BW
Attenuation rate (-3 dB)
Notch filter

<!-- page 27 -->

The following figure shows the system open-loop gain with resonance
suppression.
When the value of parameter ResonanceSuppressionLowPassFilterTime ID210
(P2.025) is increased from 0, the system bandwidth becomes smaller. Although
it solves the problem of resonance frequency, the system bandwidth and phase
margin are reduced.
If the resonance frequency is given, the notch filter can mitigate the resonance
directly. Frequency of the notch filter is 50…5000 Hz and the suppression
attenuation is 0…40 dB. If the resonance frequency does not fall within these
values, using the low pass filter to reduce the resonance is suggested.
Auto-resonance Suppression Mode
Settings:
•
X: Auto-resonance Suppression Function
0 = Disable - After this function is disabled, the existing suppression
values remain with their last value.
1 = Enable - When this setting is true, the drive determines the following:
- If the servo is stable and resonance is suppressed and no other
resonance is present; then the servo saves the resonance suppression
data and changes Auto Suppression function = 0. If there is still a
resonance or the drive is unstable, set this value back to 1 and the drive
executes this process again.

> **Table 96** — Relevant Parameter

Parameter
Name
ID230 (P2.047)
ResonanceSuppressionConfig
Gain
 Frequency
Gain
 Frequency
BW

<!-- page 28 -->

•
ZY: Fixed Resonance Suppression Parameter
In auto resonance suppression, you can set the notch filters that require
manual resonance suppression.
•
U = Reserved

> **Figure 98** — Auto-resonance Suppression Mode

For example:
If the user sets the parameter ResonanceSuppression Config ID230 (P2.047) to
0x0021, which is Notch Filter 2 enabled; shown in Figure 98 with the auto
resonance suppression enabled, the servo searches for the resonance and
suppresses it. When Y is set to 0010 (decimal 2), you can manually set the
second set of resonance suppression. Thus, if the servo finds two resonant
frequencies, then the servo writes data of the first point to the first set of
resonance suppression parameters. Then, data of the second point is written
to the third set of resonance suppression parameters. Therefore, the first and
second resonant frequencies are attenuated.
Bit
Function
Description

Notch 1 auto/manual setting
• 0: Auto resonance suppression
• 1: Manually set the first set of resonance suppression

Notch 2 auto/manual setting
• 0: Auto resonance suppression
• 1: Manually set the second set of resonance suppression

Notch 3 auto/manual setting
• 0: Auto resonance suppression
• 1: Manually set the third set of resonance suppression

Notch 4 auto/manual setting
• 0: Auto resonance suppression
• 1: Manually set the fourth set of resonance suppression

Notch 5 auto/manual setting
• 0: Auto resonance suppression
• 1: Manually set the fifth set of resonance suppression

<!-- page 29 -->

## Auto-resonance Detection Level

The smaller this parameter value is, the more sensitive the drive is to
resonance. When the value of parameter ID231 (P2.048) is bigger, then the
resonance sensitivity is lower.
Torque Mode Low Pass Filter
Low pass filter for commands is typically used to filter out unwanted
highfrequency response or noise so that the torque command becomes
smoother.

> **Table 97** — Relevant Parameter

Parameter
Name
ID231 (P2.048)
ResonanceDiagnosticLevel

> **Table 98** — Relevant Parameter

Parameter
Name
ID123 (P1.007)
TorqueCmdLowPassFilterTime
 Target
Torque
TorqueCmdLowPassFilterTime
 t

<!-- page 30 -->

Speed and Torque Limit
Functions
The Kinetix 5100 drive can apply speed and torque limits depending on the
Operation Mode of the drive.
Speed Limits
The Speed Limits are used to limit the maximum motor speed for the
application (Max. Speed Limit) and to provide a limited speed in Torque mode
•
Operation Mode: Position Mode (PR/PT) and Speed Mode (S/Sz)
Max. Speed Limit ID160 (P1.055)
•
Operation Mode: Torque (T/Tz)
The Speed can be limited while in Torque mode. The
VelocityTorqueLimitAction ID118 (P1.002) is used to enable the limit and
the preset registers contain the speed limited values.
You can change the speed limit:
• One time - The Speed Limit remains active indefinitely. First, choose
Enable with ID118 (P1.002.X). This speed change does not require a
digital input (SPDLM=None) and only requires enabling X.1 in
VelocityTorqueLimitAction ID118 (P1.002). The preset speed is chosen
with Digital Inputs SPD0/1.
• More than once - The Speed Limit is changed by using a Digital Input.
(DI.Speed Limit/SPDLM) - First, choose Disable with ID118 (P1.002.X).
This limiting is flexible so that the speed limit can be changed while
the torque limit is active. The preset speeds are selected using the
binary weighted value of Digital Inputs SPD0/1. The speed limit is
applied when DI.Speed Limit/SPDLM is ON.
The overall speed limit is still active. That means Max. Speed Limit ID160
(P1.055) is still observed.

<!-- page 31 -->

Apply a Speed Limit
The maximum speed in each mode is determined by the internal parameter
ID160 (P1.055). This bi-directional speed limit is applied at the end of the
profile generation, so it limits speed regardless of mode or command source.
Speed limit is applicable only in torque mode (T) for controlling the motor
maximum speed. If using external analog voltage in torque mode, DI signals
are available and can be set to SPD0…SPD1 for motor speed limit selection
(internal parameters). You can calibrate the analog input max value to motor
rpm max speed. When parameter ID118 (P1.002) is set to 1, the speed limit
function is enabled. See the following timing diagram.
To set the speed limit, see the following diagram.

> **Table 99** — Relevant Parameters

Parameter
Name
ID118 (P1.002)
VelocityTorqueLimitAction
ID125 (P1.009)
PresetVelocityCmd_Limit_1
ID126 (P1.010)
PresetVelocityCmd_Limit_2
ID127 (P1.011)
PresetVelocityCmd_Limit_3
ID160 (P1.055)
MaximumSpeed
Select by I/O Connector DI.SPD0, DI.SPD1 signal
Vref
(analog
command)
(0)
PresetVelocityCmd_Limit_1
ID125 (P1.009)
PresetVelocityCmd_Limit_2
ID126 (P1.010)
Speed Limit
Command
PresetVelocityCmd_Limit_3
ID127 (P1.011)

<!-- page 32 -->

## Torque Limits

The Torque Limits are used to limit the maximum motor torque for the
application.
•
Operation Mode: Position Mode (PR/PT) and Speed Mode (S/Sz)
In these modes, you can use the speed and torque limiting to control the
motor.
You can change the speed limit (used with torque limiting). The torque
configurations are in the following images:
• One time - In this case, the Speed Limit is used with Torque Limiting.
First, choose Enable with ID118 (P1.002.Y). This torque limit does not
require a digital input (TRQLM=None) and only requires enabling Y.1
in VelocityTorqueLimitAction ID118 (P1.002). The preset torque is
chosen with Digital Inputs TCM0/1.
• More than once - By using a Digital Input. (DI.Torque Limit/TRQLM) -
First, choose Disable with ID118 (P1.002.Y). This torque limiting is
flexible so that the torque limit can be changed. The preset torques are
selected using the binary weighted value of Digital Inputs TCM0/1. The
torque limit is applied when DI.Torque Limit/TRQLM is ON.

<!-- page 33 -->

•
Operation Mode: Torque Mode (T/Tz)
The torque limit can be changed using the preset torque registers and the
TCM0/1 Digital Inputs. Once the motor is enabled, the torque limit is
active. You can use the speed limit (described above) to control the speed
limit while in torque mode. When the digital inputs for TCM0/1 are
changed, the torque limits are changed dynamically.
This table shows the binary weighting representation:
Speed Command
TCM0
TCM1
Analog Input Torque

Torque Register 1

Torque Register 2

Torque Register 3

<!-- page 34 -->

Apply the Torque Limit
The issuing method of torque limit command and torque command are
identical. The command source can be external analog voltage, which used
with Max Torque Command ID148 (P1.041) or internal parameters
ID128…ID130 (P1.012…P1.014).
Torque limit can be used in position mode (PT, PR) or speed mode (S). It is
used for limiting the motor torque output. When the command in position
mode is issued by external pulse or the command in speed mode is issued by
external analog voltage, DI signals are available and can be set to TCM0…TCM1
to determine the torque limit command (internal parameters). If there are not
enough DI signals available, you can limit the torque by using the analog
voltage command with Max Torque Command ID148 (P1.041). When the
parameter ID118 (P1.002) is set to 1, the different torque limiting presets are
used. When ID118 (P1.002) is set to 0, then Max Torque Command ID148
(P1.041) value is used to limit torque. See the following timing diagram.
To set the torque limit, see the following diagram.

> **Table 100** — Relevant Parameters

Parameter
Name
ID118 (P1.002)
VelocityTorqueLimitAction
ID128 (P1.012)
PresetTorqueCmd_Limit_1
ID128 (P1.013)
PresetTorqueCmd_Limit_2
ID130 (P1.014)
PresetTorqueCmd_Limit_3
Select by I/O Connector DI.TCM0, DI.TCM1 signal
Vref
(analog
command)
(0)
PresetTorqueCmd_Limit_1
ID128 (P1.012)
PresetTorqueCmd_Limit_2
ID129 (P1.013)
Torque Limit
Command
PresetTorqueCmd_Limit_3
ID130 (P1.014)

<!-- page 35 -->

Enable/Disable Limits by using VelocityTorqueLimitAction
The VelocityTorqueLimitAction configuration is set by KNX5100C software.
Choose Enable in P1.002.Y to enable or disable the Torque Limit. Once the
torque limit is enabled, the torque limit function can be changed:
•
One time - This torque change does not require a digital input and just
requires enabling Y.1 in VelocityTorqueLimitAction (ID118, P1.002).
•
More than once - By using a Digital Input. (DI.Torque Limit) This is
flexible so that the torque limit can be modified by changing the torque
presets and toggling the digital input.
Choose Enable in P1.002.X to enable or disable the Speed Limit. Once the
speed limit is enabled, the speed limit function can be changed:
•
One time - This speed change does not require a digital input and just
requires enabling X.1 in VelocityTorqueLimitAction (ID118, P1.002).
•
More than once - This is flexible so that the speed limit can be modified
by changing the speed presets and toggling the digital input.

> **Table 101** — Relevant Parameters

Parameter
Name
ID118 (P1.002)
VelocityTorqueLimitAction

<!-- page 36 -->

Dual and Multi-modes
Eight dual/multiple modes are provided for operation in addition to the single
modes. The Dual and Multi-mode functions are chosen by using the Operation
Mode setting in KNX5100C setting (Control Mode, ID117, P1.001), and then
using Digital Inputs that contain all the mode combinations.
Speed/Position Dual Mode
The timing chart shows the behavior of the dual mode operation when
switching from Speed mode into Position mode.
•
CTRG (Digital Input = CmdTriggered)
•
S-P (Digital Input = Position/Speed modes selection,
0 = Position, 1 = Speed as shown)
•
POS0…POS6 indicates a valid binary-weighted PR is selected
•
SPD0/1 indicate that the preset speed registers are valid
Here is the typical configuration for this operation:
1.
Choose the PR/S Operation Mode.
2. Set your Digital I/O (in this example, we are using PR1 and SPD0).
3.
Set Position/Speed Mode Selection - this switches between the two
modes dynamically.
4. Command Triggered DI (CTRG) is selected.
In Speed Mode, when the transition to Position mode occurs, we require the
CTRG (CmdTriggered) input to transition Off to On to begin the selected PR.
Mode
Short Name
Setting Code Description
Dual
PT-S

PT and S can be switched by using the DI signal, S_P.
PT-T

PT and T can be switched by using the DI signal, T_P.
PR-S

PR and S can be switched by using the DI signal, S_P.
PR-T

PR and T can be switched by using the DI signal, T_P.
S-T
0A
S and T can be switched by using the DI signal, S_T.
PT-PR
0D
PT and PR can be switched by using the DI signal, PT_PR.
Multi-mode(1)
(1)
Multiple (multi-) modes are a combination of a dual mode and a single mode.
PT-PR-S
0E
PT, PR, and S can be switched by using the DI signal, S_P and
PT_PR.
PT-PR-T
0F
PT, PR, and T can be switched by using the DI signal, T_P and
PT_PR.
IMPORTANT
When dual/multi modes are used, the mode changing is immediate,
which can result in unintended motion.

**Extracted table (page 36, #1):**

| Short Name | Setting Code |
| --- | --- |
| PT-S | 06 |
| PT-T | 07 |
| PR-S | 08 |
| PR-T | 09 |
| S-T | 0A |
| PT-PR | 0D |
| PT-PR-S | 0E |
| PT-PR-T | 0F |

<!-- page 37 -->

In Position Mode, when the transition to Speed mode occurs and the selected
speed preset is valid, the motor begins executing the selected preset speed.
For more information, see Position Control on page 236 and Speed Mode on
page 246.
Speed/Torque Dual Mode
The timing chart below shows the behavior of the dual mode operation when
switching from Speed mode into Torque Mode.
•
S-T (Digital Input = Torque/Speed modes selection, 0=Torque, 1=Speed
as shown
•
TCM0/TCM1 indicate that the preset torque register values are valid
•
SPD0/1 indicate that the preset speed registers are valid
IMPORTANT
To avoid large speed changes when switching modes, be sure to
set the S-curve Smoothing constant to the time your application
requires to transition to the worst case preset speed. When
modes are changed dynamically, the change to the control loops
are immediate. Care must be used so speed/torque limits do not
exceed application requirements.
IMPORTANT
To avoid large speed changes when switching modes, be sure to set the
S-curve Smoothing constant to the time your application requires to
transition to the worst case preset speed. When modes are changed
dynamically, the change to the control loops are immediate. Care must
be used so speed/torque limits do not exceed application
requirements.

<!-- page 38 -->

## Torque/Position Dual Mode

The timing chart below shows the behavior of the dual mode operation when
switching from Torque mode into Position Mode.
•
CTRG (Digital Input = CmdTriggered)
•
T-P (Digital Input = Torque/Position modes selection,
0 = Torque, 1 = Position)
•
POS0…POS6 indicates a valid binary-weighted PR is selected
•
TCM0/1 indicate that the preset torque registers are valid
For more information, see Position Control on page 236 and Torque Mode on
page 252.
IO Mode
When the Kinetix 5100 Operating Mode is configured as IO Mode, the
operation and status of the drive comes from a Logix controller capable of a
Class 1 Ethernet/IP connection. An example of this operation is a
CompactLogix controller (for example, a 1769-L18). This controller uses
Studio 5000 Logix Designer® application for programming with a Kinetix 5100
drive pre-defined Add-On Profile to exchange data between the drive and
controller.
When the Kinetix 5100 is using IO Mode, Class 3 explicit messaging cannot be
used for that particular drive.
IMPORTANT
When modes are changed dynamically, the change to the control
loops are immediate. Care must be used so speed/torque limits
do not exceed application requirements.
IMPORTANT
 The induction and linear motors are not supported in IO Mode.
IMPORTANT
Although the Kinetix 5100 drive uses Motion Add-On Instructions to
program and an Add-On Profile (AOP) that looks similar to the Integrated
Motion on Ethernet/IP (CIP) drives, the Kinetix 5100 drive does not
function the same way. The Kinetix 5100 drive is a standard I/O device
on an Ethernet/IP network. It does not operate in the Motion Group and
does not use the motion group for synchronization. The Kinetix 5100
drive operates as a Class 1 I/O device on an Ethernet/IP network. This is
not a CIP Motion drive.

<!-- page 39 -->

The Input and Output assembly are shown for your convenience. While you
can directly manipulate these assemblies, they rely on your logic to operate
correctly, including timing, pre-existing drive conditions, and so on. It is
typical to use the pre-defined Motion Operation Add-On Instructions to
perform motion operations. These instructions contain interlocks and
condition checking to facilitate the programming effort. See Appendix C for
instruction details.

> **Table 102** — Kinetix 5100 Output Assembly Data (Instance 104)

Byte
Bit 7
Bit 6
Bit 5
Bit 4
Bit 3
Bit 2
Bit 1
Bit 0

Operating Mode

Start
Motion
Fault Reset
Stop
Motion
Servo Off
Servo On

Homing Method

Speed Reference (DINT)

Accel Reference (DINT)

Decel Reference (DINT)

Position Reference (DINT)

Home Return Speed (DINT)

Non-cyclic Move Type

Cyclic Move Type

Travel Mode

Captured
Position
Select
Position
Command
Overlap
Position
Command
Override

Torque Reference (DINT)

Torque Ramp Time (DINT)

Starting Index

<!-- page 40 -->

> **Table 103** — Kinetix 5100 Output Assembly Data (Instance 106)

Instance
Byte
Bit7
Bit6
Bit5
Bit4
Bit3
Bit2
Bit1
Bit0

OperatingMode (SINT)

Start
Motion
Fault
Reset
Stop
Motion
ServoOff
ServoOn

HomingMethod (SINT)

SpeedReference (DINT)

AccelReference (DINT)

DecelReference (DINT)

PositionReference (DINT)

HomeReturnSpeed (DINT)

NonCyclicMoveType (SINT)

CyclicMoveType (SINT)

TravelMode (SINT)

Captured
Position
Select
Position
Command
Overlap
Position
Command
Override

TorqueReference (DINT)

TorqueRampTime (DINT)

StartingIndex (SINT)

CamMasterReference (SINT) (Future)

CamExecutionSchedule (SINT) (Future)

CamExecutionMode (SINT) (Future)

CamStop
Mode
(Future)

**Extracted table (page 40, #1):**

| Byte | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 |  |  |  |  |  |  |  |  |
| 1 |  |  |  | Start Motion | Fault Reset | Stop Motion | ServoOff |  |
| 2 |  |  |  |  |  |  |  |  |
| 3 |  |  |  |  |  |  |  |  |
| 4 |  |  |  |  |  |  |  |  |
| 5 |  |  |  |  |  |  |  |  |
| 6 |  |  |  |  |  |  |  |  |
| 7 |  |  |  |  |  |  |  |  |
| 8 |  |  |  |  |  |  |  |  |
| 9 |  |  |  |  |  |  |  |  |
| 10 |  |  |  |  |  |  |  |  |
| 11 |  |  |  |  |  |  |  |  |
| 12 |  |  |  |  |  |  |  |  |
| 13 |  |  |  |  |  |  |  |  |
| 14 |  |  |  |  |  |  |  |  |
| 15 |  |  |  |  |  |  |  |  |
| 16 |  |  |  |  |  |  |  |  |
| 17 |  |  |  |  |  |  |  |  |
| 18 |  |  |  |  |  |  |  |  |
| 19 |  |  |  |  |  |  |  |  |
| 20 |  |  |  |  |  |  |  |  |
| 21 |  |  |  |  |  |  |  |  |
| 22 |  |  |  |  |  |  |  |  |
| 23 |  |  |  |  |  |  |  |  |
| 24 |  |  |  |  |  |  |  |  |
| 25 |  |  |  |  |  |  |  |  |
| 26 |  |  |  |  |  |  |  |  |
| 27 |  |  |  |  |  | Captured Position Select | Position Command Overlap |  |
| 28 |  |  |  |  |  |  |  |  |
| 29 |  |  |  |  |  |  |  |  |
| 30 |  |  |  |  |  |  |  |  |
| 31 |  |  |  |  |  |  |  |  |
| 32 |  |  |  |  |  |  |  |  |
| 33 |  |  |  |  |  |  |  |  |
| 34 |  |  |  |  |  |  |  |  |
| 35 |  |  |  |  |  |  |  |  |
| 36 |  |  |  |  |  |  |  |  |
| 37 |  |  |  |  |  |  |  |  |
| 38 |  |  |  |  |  |  |  |  |
| 39 |  |  |  |  |  |  |  |  |
| 40 |  |  |  |  |  |  |  |  |
| 41 |  |  |  |  |  |  |  |  |
| 42 |  |  |  |  |  |  |  |  |
| 43 |  |  |  |  | CamStop Mode (Future) |  |  |  |

<!-- page 41 -->

CamSlaveScaling (DINT) (Future)

Reserved

CamLockPosition (DINT) (Future)

CamMasterLockPosition (DINT) (Future)

CamMasterLeadingCounts (DINT) (Future)

CamMasterUnlockCounts (DINT) (Future)

CamMasterCyclicLeadingCounts (DINT) (Future)

Reserved

GearRatioSlaveCounts (DINT)

GearRatioMasterCounts (DINT)

> **Table 103** — Kinetix 5100 Output Assembly Data (Instance 106) (Continued)

Instance
Byte
Bit7
Bit6
Bit5
Bit4
Bit3
Bit2
Bit1
Bit0

**Extracted table (page 41, #1):**

| Byte | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 44 |  |  |  |  |  |  |  |
| 45 |  |  |  |  |  |  |  |
| 46 |  |  |  |  |  |  |  |
| 47 |  |  |  |  |  |  |  |
| 48 | Reserved |  |  |  |  |  |  |
| 49 |  |  |  |  |  |  |  |
| 50 |  |  |  |  |  |  |  |
| 51 |  |  |  |  |  |  |  |
| 52 |  |  |  |  |  |  |  |
| 53 |  |  |  |  |  |  |  |
| 54 |  |  |  |  |  |  |  |
| 55 |  |  |  |  |  |  |  |
| 56 |  |  |  |  |  |  |  |
| 57 |  |  |  |  |  |  |  |
| 58 |  |  |  |  |  |  |  |
| 59 |  |  |  |  |  |  |  |
| 60 |  |  |  |  |  |  |  |
| 61 |  |  |  |  |  |  |  |
| 62 |  |  |  |  |  |  |  |
| 63 |  |  |  |  |  |  |  |
| 64 |  |  |  |  |  |  |  |
| 65 |  |  |  |  |  |  |  |
| 66 |  |  |  |  |  |  |  |
| 67 |  |  |  |  |  |  |  |
| 68 |  |  |  |  |  |  |  |
| 69 |  |  |  |  |  |  |  |
| 70 |  |  |  |  |  |  |  |
| 71 |  |  |  |  |  |  |  |
| 72 | Reserved |  |  |  |  |  |  |
| 73 |  |  |  |  |  |  |  |
| 74 |  |  |  |  |  |  |  |
| 75 |  |  |  |  |  |  |  |
| 76 |  |  |  |  |  |  |  |
| 77 |  |  |  |  |  |  |  |
| 78 |  |  |  |  |  |  |  |
| 79 |  |  |  |  |  |  |  |
| 80 |  |  |  |  |  |  |  |
| 81 |  |  |  |  |  |  |  |
| 82 |  |  |  |  |  |  |  |
| 83 |  |  |  |  |  |  |  |

<!-- page 42 -->

> **Table 104** — Kinetix 5100 Drive Output Assembly Data Description

Name
Data
Type
Description
Semantics of Values
Operating
mode (output) SINT
This enumerated value indicates the drive's
internal mode setting. The drive can operate in
different sub-modes while in IO Mode.
• -128…-1: Reserved
• 0: Mode not specified
• 1: Position mode
• 2: Speed mode
• 3: Home mode
• 4: Torque mode
• 5: Gear mode
• 6: Index mode
• 7: ECAM mode
• 8…127: Reserved
Servo on
BOOL
A 0-to-1 transition enables the motor.
—
Servo off
A 0-to-1 transition disables the motor.
—
Stop motion
A 0-to-1 transition stops motion on the motor.
—
Fault reset
A 0-to-1 transition clears an active drive fault.
—
Start motion
A 0-to-1 transition means the motion command
is issued from the external controller.
—
Homing
method
SINT
Homing method.
See Table 112 on page 301.
Speed
reference
DINT
The commanded speed for the motor.
• Units are 0.1 RPM
• -80000…+80000
• 1…20000 (home mode)
Acceleration
reference
The commanded acceleration rate for the
motor.
Units are 0.1 RPM/sec
Deceleration
reference
The commanded deceleration rate for the
motor.
Units are 0.1 RPM/sec
Position
reference
The commanded position used for indexing.
User units as defined by the scaling
relationship from the E-Gear ratio in
KNX5100C software.
Home return
speed
The return speed when home mode is the
operating mode.
1…5000 units are 0.1 RPM (rotary
motors)
Non-cyclic
move type
SINT
Enumerated value used to determine the
noncyclic move type.
• -128…-1: Reserved
• 0: Absolute
• 1: Relative
• 2: Incremental
• 3: High-speed capture
• 4…127: Reserved
Cyclic move
type
Enumerated value used to determine the cyclic
move type.
• -128…-1: Reserved
• 0: Rotary positive
• 1: Rotary negative
• 2: Rotary shortest path
• 3…127: Reserved
Travel mode
Enumerated value used to determine the travel
constraints of the axis.
• -128…+1: Reserved
• 2: Non-cyclic move
• 3…9: Reserved
• 10: Cyclic move
• 11…127: Reserved
Position
command
override
BOOL
When executing a motion command, the next
movement can override the previous movement.
• 0: Does not override previous
movement
• 1: Can override previous
movement
Position
command
overlap
The end of the current movement can be
overlapped by the next movement.
• 0: Does not overlap the next
movement
• 1: Overlaps the next movement
Captured
position select BOOL
Selects between the high speed digital inputs
used to capture position feedback.
Vendor specific.
0: DI9 is selected
1: DI10 is selected

**Extracted table (page 42, #1):**

| Data Type | Description |
| --- | --- |
| SINT | This enumerated value indicates the drive's internal mode setting. The drive can operate in different sub-modes while in IO Mode. |
| BOOL | A 0-to-1 transition enables the motor. |
|  | A 0-to-1 transition disables the motor. |
|  | A 0-to-1 transition stops motion on the motor. |
|  | A 0-to-1 transition clears an active drive fault. |
|  | A 0-to-1 transition means the motion command is issued from the external controller. |
| SINT | Homing method. |
| DINT | The commanded speed for the motor. |
|  | The commanded acceleration rate for the motor. |
|  | The commanded deceleration rate for the motor. |
|  | The commanded position used for indexing. |
|  | The return speed when home mode is the operating mode. |
| SINT | Enumerated value used to determine the noncyclic move type. |
|  | Enumerated value used to determine the cyclic move type. |
|  | Enumerated value used to determine the travel constraints of the axis. |
| BOOL | When executing a motion command, the next movement can override the previous movement. |
|  | The end of the current movement can be overlapped by the next movement. |
| BOOL | Selects between the high speed digital inputs used to capture position feedback. |

<!-- page 43 -->

Torque
reference
DINT
Represents the output torque level when the
Operation Mode is Torque Mode (3). This value is
percent of motor rated torque.
-4000…4000 enumeration is 0.1x
Torque ramp
time
Represents the time to reach the torque
reference. This units are ms.
1…65500
Starting index SINT
The first index (position register) that the drive
should execute.
• -128…-1: Reserved
• 0: PR 0
• 1…99: PR1…PR99
• 100…127: Vendor specific

> **Table 105** — Kinetix 5100 Input Assembly Data (Instance 154)

Byte
Bit 7
Bit 6
Bit 5
Bit 4
Bit 3
Bit 2
Bit 1
Bit 0

Diagnostic
Active
Connection
Faulted
Run Mode

Diagnostic sequence count

Pad bytes for LINT alignment

Uncertain
Fault

At
Reference
Stopped
Homed
Status
Command in
Progress
Ready
Active
Warning
Present

Reserved

Operating Mode

Active Index

Reserved

Motor Type

Actual Speed (DINT)

Fault Code (UINT)

Warning Code (UINT)

Actual Position (DINT)

Actual Torque (DINT)

Parameter Monitor 1 Value (DINT)

> **Table 104** — Kinetix 5100 Drive Output Assembly Data Description (Continued)

Name
Data
Type
Description
Semantics of Values

**Extracted table (page 43, #1):**

| Data Type | Description |
| --- | --- |
| DINT | Represents the output torque level when the Operation Mode is Torque Mode (3). This value is percent of motor rated torque. |
|  | Represents the time to reach the torque reference. This units are ms. |
| SINT | The first index (position register) that the drive should execute. |

**Extracted table (page 43, #2):**

| Pad bytes for LINT alignment |  |  |  |  |  |  |  |
| --- | --- | --- | --- | --- | --- | --- | --- |
|  |  |  |  |  | Uncertain | Fault |  |
| At Reference | Stopped | Homed Status | Command in Progress | Ready | Active | Warning Present |  |

<!-- page 44 -->

Parameter Monitor 2 Value (DINT)

Parameter Monitor 3 Value (DINT)

Parameter Monitor 4 Value (DINT)

Parameter Monitor 5 Value (DINT)

> **Table 106** — Kinetix 5100 Drive Input Assembly Data Description

Name
Data
Type
Description
Semantics of Values
Run mode
BOOL
Indicates whether the drive is in run mode
• 0: Drive is idle
• 1: Drive is in run mode
Connection
faulted
Indicates whether the connection is faulted
• 0: Connection is not faulted
• 1: Connection is faulted
Diagnostic
active
Indicates whether the diagnostic is active
• 0: Diagnostic is not active
• 1: Diagnostic is active
Diagnostic
sequence
count
SINT
The sequence count for the diagnostic
—
Fault
BOOL
Indicates whether the drive is in a faulted
state
• 0: No Fault
• 1: Faulted
Uncertain
Indicates whether the data validity is
questionable
• 0: Data is valid
• 1: Data validity is questionable
Warning
present
Indicates whether the drive is in a warning
state
• 0: No warnings
• 1: Drive is in a warning state
Active
BOOL
Indicates whether the motor is enabled
• 0: Motor is not enabled
• 1: Motor is enabled
Ready
Indicates whether the motor is ready to be
enabled
• 0: Motor is not ready
• 1: Motor is ready
Command in
progress
Indicates whether the drive received the
command from the controller
Indicates the new command has been
received by the K5100 drive. It toggles
between 0 and 1 after a new command
has been received by the K5100 drive.
When this bit toggles it stays at the
toggled state until a new command is
received.
Homed status
Indicates whether the drive completed the
home operation
1: Drive completed the home operation
Stopped
Indicates whether the motor is stopped
1: Motor is stopped
At reference
Motor actual at reference (position, speed,
torque) based on mode
1: Motor actual at reference (position,
speed, torque) based on mode

> **Table 105** — Kinetix 5100 Input Assembly Data (Instance 154) (Continued)

Byte
Bit 7
Bit 6
Bit 5
Bit 4
Bit 3
Bit 2
Bit 1
Bit 0

**Extracted table (page 44, #1):**

| Data Type | Description |
| --- | --- |
| BOOL | Indicates whether the drive is in run mode |
|  | Indicates whether the connection is faulted |
|  | Indicates whether the diagnostic is active |
| SINT | The sequence count for the diagnostic |
| BOOL | Indicates whether the drive is in a faulted state |
|  | Indicates whether the data validity is questionable |
|  | Indicates whether the drive is in a warning state |
| BOOL | Indicates whether the motor is enabled |
|  | Indicates whether the motor is ready to be enabled |
|  | Indicates whether the drive received the command from the controller |
|  | Indicates whether the drive completed the home operation |
|  | Indicates whether the motor is stopped |
|  | Motor actual at reference (position, speed, torque) based on mode |

<!-- page 45 -->

Operating
mode (input)
SINT
Indicates the drive Operation Mode
Its value may be:
• -128…+1: Reserved
• 0: Mode not specified
• 1: Position mode
• 2: Speed mode
• 3: Home mode
• 4: Torque mode
• 5: Gear mode
• 6: Index mode
• 7…127: Reserved
Active index
Indicates the currently executing index (PR)
Currently executing index:
• -128…+1: Reserved
• 0: PR 0: Homing
• 1…99: PR 1…PR 99
• 100…127: Reserved
Motor type
Indicates which type of motor is connected to
the drive
• 0: No motor connected
• 1: Rotary motor connected
• 2: Linear motor connected
Actual speed
DINT
Motor actual velocity
The value is RPM
Fault code
UINT
Fault code
See View Status and Faults on page 450
Warning code
Warning code
Actual
position
DINT
Motor Actual Position
PUU (counts or user units)
Actual torque
Actual Motor Torque
% motor rated torque
Parameter
monitor 1
value
Parameter monitor selection 1
0 - no parameter is selected
0x0001…0xFFFF - returned value
mapped from KNX5100C Function
List>Parameter Editor>StatusMonitor
ID060
Parameter
monitor 2
value
Parameter monitor selection 2
0 - no parameter is selected
0x0001…0xFFFF - returned value
mapped from KNX5100C Function
List>Parameter Editor>StatusMonitor
ID061
Parameter
monitor 3
value
Parameter monitor selection 3
0 - no parameter is selected
0x0001…0xFFFF - returned value
mapped from KNX5100C Function
List>Parameter Editor>StatusMonitor
ID062
Parameter
monitor 4
value
Parameter monitor selection 4
0 - no parameter is selected
0x0001…0xFFFF - returned value
mapped from KNX5100C Function
List>Parameter Editor>StatusMonitor
ID063
Parameter
monitor 5
value
Parameter monitor selection 5
0 - no parameter is selected
0x0001…0xFFFF - returned value
mapped from KNX5100C Function
List>Parameter Editor>StatusMonitor
ID064

> **Table 106** — Kinetix 5100 Drive Input Assembly Data Description (Continued)

Name
Data
Type
Description
Semantics of Values

**Extracted table (page 45, #1):**

| Data Type | Description |
| --- | --- |
| SINT | Indicates the drive Operation Mode |
|  | Indicates the currently executing index (PR) |
|  | Indicates which type of motor is connected to the drive |
| DINT | Motor actual velocity |
| UINT | Fault code |
|  | Warning code |
| DINT | Motor Actual Position |
|  | Actual Motor Torque |
|  | Parameter monitor selection 1 |
|  | Parameter monitor selection 2 |
|  | Parameter monitor selection 3 |
|  | Parameter monitor selection 4 |
|  | Parameter monitor selection 5 |

<!-- page 46 -->

## IO Mode - Position

When the IO Mode is used and the drive internal sub-mode is Position Mode
(1), the drive command is an index or constant speed operation. Typically, the
Motion Add-On Instruction are used to perform the index
(raC_xxx_K5100_MAM) or Jog (raC_xxx_K5100_MAJ) operation. The control
structure of position mode is as follows.
See Appendix C, Use Add-On Instructions on page 489, for more information.
IO Mode - Home
When the IO Mode is used and the drive internal sub-mode is Home Mode (3),
the drive command is a homing operation. Typically, the Motion Add-On
Instruction are used to perform the home operation (raC_xxx_K5100_MAH).
The control structure of home mode is as follows.
See Appendix C, Use Add-On Instructions on page 489, for more information.
See Homing on page 298 for more information.
GNUM0, GNUM1
GearRatioFollowerCounts N1 - ID151 (P1.044)
GearRatioFollowerCounts N2 - ID236 (P2.060)
GearRatioFollowerCounts N3 - ID237 (P2.061)
GearRatioFollowerCounts N4 - ID238 (P2.062)
GearRatioMasterCounts - ID152 (P1.045)
O.PositionReference
Position
Command
Generator
1. ID117 (P1.001)
2. O. Operation
 Mode Setting
Other
Position
Commands
Numerators
Denominator
Filters
ID124 (P1.008)
ID134…ID138 (P1.024…P1.028)
ID143 (P1.036)
ID170 (P1.068)
O.SpeedReference
O.Acceleration
O.Deceleration
O.Selection
O.CyclicMoveType
O.MoveMethod
O.CaptureSource
O.Overlap
O.Interrupt
NOTE: ‘O.xxx’ are the output parameters of the I/O assembly.
Graphic to be recreated in
Adobe Illustrator and to
include ID #s.
GNUM0, GNUM1
GearRatioFollowerCounts N1 - ID151 (P1.044)
GearRatioFollowerCounts N2 - ID236 (P2.060)
GearRatioFollowerCounts N3 - ID237 (P2.061)
GearRatioFollowerCounts N4 - ID238 (P2.062)
GearRatioMasterCounts - ID152 (P1.045)
O.HomingMethod
Position
Command
Generator
1. ID117 (P1.001)
2. O. Operation
 Mode Setting
Other
Position
Commands
Numerators
Denominator
Filters
ID124 (P1.008)
ID134…ID138 (P1.024…P1.028)
ID143 (P1.036)
ID170 (P1.068)
O.PositionReference
O.SpeedReference
O.HomeReturnSpeed
O.Acceleration
O.Deceleration
NOTE: ‘O.xxx’ are the output parameters of the I/O assembly.

<!-- page 47 -->

## IO Mode - Gear

When the IO Mode is used and the drive internal sub-mode is Gear Mode (5),
the drive command is a fixed pulse-pulse operation (Gearing). This mode uses
a pulse-pulse following between this drive (slave) and a master that delivers
pulses via the PT terminals or AUX feedback input. Typically, there is
additional configuration in the KNX5100C software and then the Motion
Add-On Instruction is used to perform the gearing operation
(raC_xxx_K5100_MAG).
The control structure of gear mode is as follows.
Command Source
ID397…ID524
(P6.000…P7.027)
Notch Filter
ID135…ID136
(P1.025…P1.026)
Command Selection
ID117 (P1.001)
1. P1.001
2. P1.064
1. P1.001
2. I/O mode par
1. P1.001
2. I/O mode par
1. P1.001
2. I/O mode par
Moving Filter
ID170 (P1.068)
Accel/Decel Time
ID312…ID327
(P5.020…P5.035)
Low-pass Filter
ID124 (P1.008)
Notch Filter
ID137…ID138
(P1.027…P1.028)
Pulse Type
Selection
ID116 (P1.000)
Counter
Position Command Processing Unit
High
Speed
Pulse Signal
Inhibit(1)
Inhibit(1)
I/O Controller
POS0…POS6
CRTG
Low
Speed
Delay Time
ID332…ID347
(P5.040…P5.055)
CyclicMove Type
(for example, YX
of ID399 [P6.002])
Acceleration
Reference
Starting Index
I/O Mode - Index
Position Reference
I/O Mode - Position
Deceleration
Reference
Acceleration
Reference
Position Reference
I/O Mode - Home
I/O Mode - Gear
Home
Return
Speed
Speed
Reference
Homing
Method
Deceleration
Reference
Target Speed
ID352…ID367
(P5.060…P5.075)
GNUM0, GNUM1
GearRatioFollowerCounts N1 - ID151 (P1.044)
GearRatioFollowerCounts N2 - ID236 (P2.060)
GearRatioFollowerCounts N3 - ID237 (P2.061)
GearRatioFollowerCounts N4 - ID238 (P2.062)
GearRatioMasterCounts - ID152 (P1.045)
Numerators
Denominator
S-curve Filter
ID143 (P1.036)
Analog Input
Scaling
ID169 (P1.066)
Analog
Command Filter
ID168 (P1.065)
PTO
AUX
(1)
For more information, see Pulse Command Input Inhibitor (INHP) on page 240.

<!-- page 48 -->

## IO Mode - Index

When the IO Mode is used and the drive internal sub-mode is Index Mode (6),
the drive uses a PR (Position Register) that you specify and is stored in the
drive to execute. Typically, the Motion Add-On Instruction is used to perform
the PR, or index selection (raC_xxx_K5100_MAI). For more about PR Mode, see
Chapter 11, Motion Control in PR Mode.
The control structure of the index mode is as follows.
IO Mode - Speed
When the IO Mode is used and the drive internal sub-mode is Speed Mode (2),
the drive executes a constant speed profile. Typically, the Motion
Add-On Instruction is used to perform the constant speed profile
(raC_xxx_K5100_MAJ).
The control structure of speed mode is as follows.
IO Mode - Torque
When the IO Mode is used and the drive internal sub-mode is Torque Mode (4),
the drive outputs a constant torque. Typically, the Motion Add-On Instruction
is used to perform the constant torque output (raC_xxx_K5100_MAT).
The control structure of torque mode is as follows.
GNUM0, GNUM1
GearRatioFollowerCounts N1 - ID151 (P1.044)
GearRatioFollowerCounts N2 - ID236 (P2.060)
GearRatioFollowerCounts N3 - ID237 (P2.061)
GearRatioFollowerCounts N4 - ID238 (P2.062)
GearRatioMasterCounts - ID152 (P1.045)
Target Speed
ID352…ID367
(P5.060…P5.075)
1. ID117 (P1.001)
2. O. Operation
 Mode Setting
Other
Position
Commands
Numerators
Denominator
Filters
ID124 (P1.008)
ID134…ID138
(P1.024…P1.028)
ID143 (P1.036)
ID170 (P1.068)
Delay Time
ID332…ID347
(P5.040…P5.055)
Accel/Decel Time
ID312…ID327
(P5.020…P5.035)
Command Source
ID397…ID596
(P6.000…P7.099)
O.StartingIndex
NOTE: ‘O.xxx’ is the output parameter of the I/O assembly.
Low-pass Filter
ID122 (P1.006)
1. ID117 (P1.001)
2. O. Operation
 Mode Setting
Other
Speed Commands
S-curve Filter
ID143 (P1.036)
Speed
Command
Generator
O.SpeedReference
O.Acceleration
O.Deceleration
NOTE: ‘O.xxx’ are the output parameters of the I/O assembly.
Low-pass Filter
ID122 (P1.006)
1. ID117 (P1.001)
2. O. Operation
 Mode Setting
Other
Torque Commands
Torque
Command
Generator
O.TorqueReference
O.TorqueSlope
NOTE: ‘O.xxx’ are the output parameters of the I/O assembly.

<!-- page 49 -->

Analog Outputs and
Monitoring
There are two analog outputs available on the I/O Connector pins 15(AOUT2),
16(AOUT1), and 19(AGND). These outputs can be scaled based on the output
voltage and value used. You can use the MON1 and MON2 analog outputs at
the same time.
To configure the Analog Output, follow these steps.
1.
From KNX5100C software, choose Function List > Drive > Settings >
Analog I/O > Output Monitor.
2. Choose the output voltage used for the Analog Output.
3.
Choose the Polarity of the Analog Output.
4. From the pull-down menu, assign a drive parameter to the analog
output.
If required, you can scale the drive parameter to provide better output
granularity.
5.
Use the Mon Calculator to scale the drive parameter with the analog
output scale.
When you enter the Requirement Mapping to XX Volts, click the
Calculate to record the scaling into ID120 (P1.004).
You must disable the drive and download any changes to the drive.
For example:
By using the steps above, this example shows the selection of Motor RPM with
Analog Output1 (MON1) to specify a motor speed of 1000 rpm to correspond to
an analog output of 8V for a motor with a maximum speed of 5000 rpm, use
the following equation:

> **Table 107** — Relevant Parameters

Parameter
Name
ID103 (P0.003)
AOMonitorSelection
ID119 (P1.003)
EncoderOutputPolarity
ID120 (P1.004)
AnalogOutput1Scale
ID121 (P1.005)
AnalogOutput2Scale
ID290 (P4.020)
AnalogOuput1Offset
ID291 (P4.021)
AnalogOuput2Offset
ID120 (1.004) =
x 100%
x 100% = 20%
Required speed
Max speed
= 1000 rpm
5000 rpm

<!-- page 50 -->

To acquire the corresponding voltage output for the current motor speed, use
the following equations:
Voltage Drift
If analog voltage drift occurs, the voltage level defined as zero volts is different
from the measured zero volts point. To compensate for this offset,
AnalogOutput1Offset DOF1 ID290 (P4.020) and AnalogOutput2Offset DOF2
ID291 (P4.021) can be used to calibrate the offset voltage output.

> **Table 108** — Relevant Parameters

Parameter
Name
ID290 (P4.020)
AnalogOuput1Offset
ID291 (P4.021)
AnalogOuput2Offset
P1.004

MON1 = 8V x
x 100%
= 2.4V
Current speed
Max speed x
=

8V x
x 100%
300 rpm
5000 rpm x
P1.004

MON1 =
For a motor current speed of 300 rpm:
For a motor current speed of 900 rpm:
8V x
x 100%
= 7.2V
Current speed
Max speed x
=

8V x
x 100%
900 rpm
5000 rpm x

<!-- page 51 -->

Notes:

<!-- page 52 -->

This chapter provides information about how to use the PR (Position Register)
Operation Mode. In this mode, commands are executed based on the internal
registers (called PRs) of the servo drive. Various commands are available,
including Homing, Speed, Position, Jump, Write, Index Position, and
Arithmetic operation. This chapter contains detailed description of each
command.
Topic
Page
Detailed Operation in PR Mode

Homing

Constant Speed Control

Position Control Command

Jump Command

Write Command

Index Position Command

Arithmetic Operations Commands

Use the PR Mode Editor in KNX5100C Software

Display of PR Procedure in KNX5100C Software

Trigger Method for PR Commands

PR Execution Process

This manual links to Kinetix® 5100 Servo Drive Fault Codes
Reference Data, publication 2198-RD001, for fault codes and Kinetix
5100 Servo Drive Parameters Reference Data, publication 2198RD002, for parameters. Download the spreadsheets now for offline
access.
