# Chapter 11: Motion Control - PR Mode

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `12_Ch11_Motion_Control_PR_Mode.pdf` (85 pages)_

<!-- page 1 -->

Detailed Operation in PR
Mode
In PR Mode, the internal registers (PR) of the Kinetix 5100 drive generate
commands. The drive provides 100 PR (Position Registers) that allow different
commands to be executed by the drive. The figure below shows the location of
the key functions of PR Mode in the KNX5100C software.

> **Figure 99** — PR Mode Operation

## Parameter Editor

The Home and PR values (Settings and Data) can be accessed by using
parameters ID397…ID596 (P6.000…P7.099). These are grouped in pairs as
shown in Figure 100.
1.
Open the Parameter Editor; unction List>Settings>Parameter Editor.
2. Choose the Motion tab.
This tab shows the locations for the individual PR settings.
3.
Change the parameters by using explicit read/write Ethernet/IP
messaging with an external controller.
Item
Description

Operation Mode – set for PR Mode (Position Mode – Register Input).

PR Mode Editor - choose Function List>Settings>Motion Control>PR Mode Editor.

PR Mode Setting menu - the individual PR (Registers) – PR#01…PR#99 are shown.

PR#01 - the PR# dialog box shows the settings for the PR type that is chosen.
PR#00 is used as the Homing Configuration, so it cannot be used for any other purpose.

Parameter Editor - choose Function List>Settings>Parameter Editor to view or as an alternate method
to set the move data.

Within the Parameter Editor, choose the Motion tab, here the PR Settings and PR Data is shown. Now,
an external controller can use explicit messaging to read/write this data.

<!-- page 2 -->

> **Figure 100** — Setting Parameters for Each PR Command

PR#0
PR#1
PR#2
ID397 (P6.000)
ID399 (P6.002)
ID401 (P6.004)
ID398 (P6.001)
ID400 (P6.003)
ID402 (P6.005)
PR#50
PR#99
ID497 (P7.000)
ID498 (P7.001)
ID595 (P7.098)
ID596 (P7.099)
PR Command
Command Setting
Command Data

**Extracted table (page 2, #1):**

| ID397 (P6.000) | ID398 (P6.001) |
| --- | --- |
| ID399 (P6.002) | ID400 (P6.003) |
| ID401 (P6.004) | ID402 (P6.005) |

<!-- page 3 -->

## PR Mode Definitions

When an individual PR is selected (Figure 99, step ) and the dialog box appears
(Figure 99, step ) in KNX5100C software. The TYPE selection chooses which
OPTION values that are visible.
Some of the Options shown are only visible when the TYPE is selected.
PR Command Setting
If you need to read/write the PR Command (including indexing) Setting
values, the decoding of these hex values is shown below.
You can use an external controller capable of Class 3 Ethernet/IP messaging to
perform explicit read/writes for changing the index settings, which involves
changes to the control setting. This parameter ID399 (P6.002) for PR1 contains
a High and Low word that can be changed from Hex into Decimal. This section
shows how to ‘decode’ the settings so you can change any indexing value.

> **Table 109** — PR Mode Definitions - Part 1

Type
Description
[0] NA
—
[1] Constant speed control
See Constant Speed Control on page 324
[2] Point-to-Point Command
See Position Control Command on page 326
[3] Point-to-Point Command
(Proceed to the next command when completed)
—
[7] Jump to the specified command
See Jump Command on page 329
[8] Write to Parameters or Data Array
See Write Command on page 331
[0xA] Index Position Control
See Index Position Command on page 333
[0xB] Statement
See Arithmetic Operation Commands on page 339

> **Table 110** — PR Mode Definitions - Part 2

OPTIONS
SEMANTICS
DESCRIPTION
Constant Speed Control
Point to Point
Command
Point to Point (proceed
to next)
Jump to specified
command
Write to parameters/
data array
Index Position Control
Statement
Interrupt previous PR
See Command
Interrupts Execution on
page 363
0=NO, 1=YES
When this PR is executed, it will
interrupt (stop) any currently
executing PR
X
X
X
X
X
X
X
Overlap Next PR
See Overlap Command
Execution on page 369
0=NO, 1=YES
Will overlap the next PR with
the currently executing PR
X
X
X
AUTO
0=NO, 1=YES
Automatically load the next PR
cmd when the current PR cmd
completes
X
X
X
UNIT
0=0.1 RPM, 1= PPS (PUU/sec)
X
Speed factor
0=0.1RPM, 1=0.01RPM
X
CMD: Position command
types
00: Absolute Position
01: REL Relative Position
02: INC Incremental Position
03: CAP High Speed Position Capturing
X
Direction
0=forward, 1=reverse, 2=shortest direction
X
ROM
0=NO, 1=YES
Write to EERPOM when
uploading a parameter
X

**Extracted table (page 3, #1):**

| SEMANTICS | DESCRIPTION | lortnoC deepS tnatsnoC | tnioP dnammoC ot tnioP | deecorp( tnioP ot )txen tnioP ot | deificeps dnammoc ot pmuJ | /sretemarap yarra ot etirW atad | lortnoC noitisoP xednI |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0=NO, 1=YES | When this PR is executed, it will interrupt (stop) any currently executing PR | X | X | X | X | X | X |
| 0=NO, 1=YES | Will overlap the next PR with the currently executing PR |  | X | X |  |  | X |
| 0=NO, 1=YES | Automatically load the next PR cmd when the current PR cmd completes | X |  |  |  | X | X |
| 0=0.1 RPM, 1= PPS (PUU/sec) |  | X |  |  |  |  |  |
| 0=0.1RPM, 1=0.01RPM |  |  |  |  |  |  | X |
| 00: Absolute Position 01: REL Relative Position 02: INC Incremental Position 03: CAP High Speed Position Capturing |  |  | X |  |  |  |  |
| 0=forward, 1=reverse, 2=shortest direction |  |  |  |  |  |  | X |
| 0=NO, 1=YES | Write to EERPOM when uploading a parameter |  |  |  |  | X |  |

<!-- page 4 -->

Settings:
Format of this parameter: (High word h) DCBA: (Low word L) UZYX
The High/Low word values correspond to the following individual selections
shown by using the actual Index selection in KNX5100C software. Notice that
the drive uses pull-down selections for different index dynamics like speed,
acceleration, and deceleration. Figure 101 shows a Point-To Point Command
(indexing) and how these decoded values relate to the KNX5100C software.
We show how to decode the Control Setting so we can modify ID399 (P6.002)
by using an explicit write. This approach to reading/writing index values
works well for a single index approach. It also implies that you use all the
possible Speed/Accel/Decel/Delay that are pre-loaded in the drive and then the
control setting selects your dynamics by using the pull-down value.

> **Figure 101** — Point-to-Point Index Setting in KNX5100C Software

A
SPD, Target speed index
X
TYPE, Command type
B
DLY, Delay time index
Y
OPT, Option
C
AUTO
Z
ACC, Acceleration time index
D
Reserved
U
DEC, Deceleration time index
High word
Low word
C
D
B A
Y
Z
U
X

**Extracted table (page 4, #1):**

| SPD, Target speed index | X |
| --- | --- |
| DLY, Delay time index | Y |
| AUTO | Z |
| Reserved | U |

<!-- page 5 -->

Definitions of the words are as follows:
•
YX: option; command type - the X, Y rows line up to show what options
are available for the Command Type
- DIR sets the rotation direction (Bit3, Bit2).
00: Forward
01: Backward
10: Shortest distance
11: Reserved
- ROM lets the drive write parameters to both RAM and EEPROM at the
same time. This function can only write parameters.
- INS: executing this PR command interrupts the previous PR
command.
- OVLP: allow overlapping of the next PR command. Overlapping is not
allowed in Speed mode. When overlapping Position mode, DLY has no
function.
- AUTO: once current PR command is finished, automatically load the
next command.
- CMD is the position command selection (Bit3, Bit2).
00: ABS, Absolute Position, CMD = DATA
01: REL, Relative Position, CMD = Current Position + DATA
10: INC, Incremental Position, CMD = Previous CMD + DATA
11: CAP, High Speed Position Capturing, CMD = Captured Position +
DATA
- UNIT is the speed unit selection: 0 signifies 0.1 rpm and 1 signifies
pulse per second (PPS).
Y: OPT, Option
X: TYPE, Command Type
Bit 3
Bit 2
Bit 1
Bit 0
-
UNIT
AUTO
INS
1: Constant speed control
CMD
OVLP
INS
2: Point-to-point command
3: Point-to-point command (Proceed to the next command when
completed)
-
-
-
INS
7: Jump to the specified PR command
-
ROM
AUTO
INS
8: Write to parameter or Data Array
DIR
OVLP
INS
A: Index position control
-
-
-
-
B: Statement / arithmetic operation

**Extracted table (page 5, #1):**

| Bit 3 | Bit 2 | Bit 1 | Bit 0 |
| --- | --- | --- | --- |
|  | UNIT | AUTO | INS |
|  |  | OVLP | INS |
|  | - | - | INS |
|  | ROM | AUTO | INS |
|  |  | OVLP | INS |
|  | - | - | - |

<!-- page 6 -->

•
UZ: DEC, deceleration time; ACC, acceleration time
Accel/Decel are used from the same pull-down menu as follows:
U: DEC, Deceleration Time
Z: ACC, Acceleration Time Corresponding Parameter
Default Value (ms)

ID312 (P5.020)

ID313 (P5.021)

ID314 (P5.022)

ID315 (P5.023)

ID316 (P5.024)

ID317 (P5.025)

ID318 (P5.026)
1000

ID319 (P5.027)
1200

ID320 (P5.028)
1500

ID321 (P5.029)
2000

ID322 (P5.030)
2500

ID323 (P5.031)
3000

ID324 (P5.032)
5000

ID325 (P5.033)
8000

ID326 (P5.034)

ID327 (P5.035)

**Extracted table (page 6, #1):**

| Z: ACC, Acceleration Time | Corresponding Parameter |
| --- | --- |
| 0 | ID312 (P5.020) |
| 1 | ID313 (P5.021) |
| 2 | ID314 (P5.022) |
| 3 | ID315 (P5.023) |
| 4 | ID316 (P5.024) |
| 5 | ID317 (P5.025) |
| 6 | ID318 (P5.026) |
| 7 | ID319 (P5.027) |
| 8 | ID320 (P5.028) |
| 9 | ID321 (P5.029) |
| 10 | ID322 (P5.030) |
| 11 | ID323 (P5.031) |
| 12 | ID324 (P5.032) |
| 13 | ID325 (P5.033) |
| 14 | ID326 (P5.034) |
| 15 | ID327 (P5.035) |

<!-- page 7 -->

•
A: SPD, target speed index
•
B: DLY, delay time index
•
C: AUTO: once current PR command is finished, automatically load the
next command. This function is only enabled when X = A indexing
position control.
Description of each bit:
A
Corresponding parameter
Default value (ms)

ID352 (P5.060)

ID353 (P5.061)

ID354 (P5.062)

ID355 (P5.063)

ID356 (P5.064)

ID357 (P5.065)

ID358 (P5.066)

ID359 (P5.067)

ID360 (P5.068)
1000

ID361 (P5.069)
1300

ID362 (P5.070)
1500

ID363 (P5.071)
1800

ID364 (P5.072)
2000

ID365 (P5.073)
2300

ID366 (P5.074)
2500

ID367 (P5.075)
3000
B
Corresponding parameter
Default value (ms)

ID332 (P5.040)

ID333 (P5.041)

ID334 (P5.042)

ID335 (P5.043)

ID336 (P5.044)

ID337 (P5.045)

ID338 (P5.046)
1000

ID339 (P5.047)
1500

ID340 (P5.048)
2000

ID341 (P5.049)
2500

ID342 (P5.050)
3000

ID343 (P5.051)
3500

ID344 (P5.052)
4000

ID345 (P5.053)
4500

ID346 (P5.054)
5000

ID347 (P5.055)
5500
Bit 2
AUTO
0: disable auto function
1: once current PR command is finished, automatically load the next
command
Bit 0, Bit 1
Reserved
-

<!-- page 8 -->

If we chose our values from Figure 101 on page 289:
This becomes: 0 0 4 8 E E 4 3 which is 4,779,587 Decimal, this value is what we
write to ID399 (P6.002). This value is the PRCmdxSetting.
PR Command Data Setting
The PRCmdXData parameters define either the target position of the related
PR command or the target PR command for the jump command.
Shared PR Parameters
The drive dynamics and delays are shared by PR programs. The Speed,
Acceleration, Deceleration, and Delays that are chosen by pull-down menus
use a common value that is made in the PR Mode Editor>Speed and Time
Setting menu.

> **Figure 102** — Speed and Time Setting

For example, if multiple PR commands apply the target speed setting of
PresetVelocity0 ID352 (P5.060), and that preset rpm value is changed, that PR
command target speed is also changed. The Accel/Decel times that are used for
a Position Command are calculated based on reaching 3000 rpm. So, changing
this preset velocity also affects the Accel and Decel times for any profile that
uses the same Preset Velocity.

> **Table 111** — High/Low Word Values

## High Word

Value
Low Word
Value
A
8 (1000 RPM)
X
3 (Point to Point Command-Proceed to next PR)
B
4 (500 ms delay)
Y
0 1 0 0 (REL, OVLP, INS) – this is 4 in decimal
C
0 (Do not load next command)
Z
E (Accel of 50) – this is 14 in decimal
D

U
E (Decel of 50) – this is 14 in decimal
HEX
0 0 4 8
 HEX
E E 4 3
ATTENTION: Damage to machine can occur. Changing a setting that is used
by multiple PR commands will affect all PR commands using that setting.

<!-- page 9 -->

> **Figure 103** — Shared Parameter Data of PR Commands

See Speed and Time Settings on page 343 for information about configuring
the speed and time settings for these shared parameters.
Monitoring Variables in PR Mode
The Kinetix 5100 drive provides different options to monitor the drive
indexing operation in PR Mode. The Monitoring Status, Scope function, or
most accurate is the Digital Inputs/Outputs. You can use these options to
create timing information that can help understand how the drive indexing is
operating.
PR Mode provides the following four monitoring variables for servo command
and feedback.
•
Command position (PUU): The target position of the motion command
generated per scan cycle during servo operation (updated every 1 ms),
simplified as Cmd_O (Command Operation).
•
PR command end register: The target position of the PR command,
simplified as Cmd_E (Command End). When a command is triggered,
the servo drive calculates the target position and then updates the PR
command end register.
•
Feedback position (PUU): The feedback position (coordinates) for the
motor, simplified as Fb_PUU (Feedback PUU).
•
Position error (PUU): The deviation between the command position
(PUU) and the feedback position (PUU), simplified as Err_PUU (Error
PUU).
EXAMPLE
If acceleration time is set to 50 ms, this means when the target
speed of motion command is 3000 rpm, then the required duration
is 50 ms.
If the target speed of the motion command is 1500 rpm, then the
acceleration time is 25 ms.

## PR Command Setting

ACC:1
DEC:4
DLY:2
SPD:5
Acceleration / Deceleration Time
(ACC/DEC)

ID312 (P5.020)

ID313 (P5.021)

ID314 (P5.022)

ID315 (P5.023)

ID316 (P5.024)

ID317 (P5.025)

ID318 (P5.026)
1000
…
…

ID326 (P5.034)

ID327(P5.035)

ID332 (P5.040)

ID333 (P5.041)

ID334 (P5.042)

ID335 (P5.043)

ID336 (P5.044)

ID337 (P5.045)

ID338 (P5.046)
1000
…
…

ID346 (P5.054)
5000

ID347(P5.055)
5500
Delay Time (DLY)
Target Speed (SPD)

ID352 (P5.060)
20.0

ID353 (P5.061)
50.0

ID354 (P5.062)
100.0

ID355 (P5.063)
200.0

ID356 (P5.064)
300.0

ID357 (P5.065)
500.0

ID358 (P5.066)
600.0
…
…

ID366 (P5.074)
2500.0

ID367(P5.075)
3000.0

<!-- page 10 -->

How these four monitoring variables work is shown in Figure 104. After the
drive sets a Position command, the drive sets the position of Cmd_E once the
target position data is acquired. The motor moves to the target position based
on the PR command setting. Cmd_O calculates the command position data in
each fixed cycle (1 ms) and sends it to the servo drive, where it is treated as a
dynamic command. Fb_PUU is the motor feedback position and Err_PUU is
the deviation of Cmd_O minus Fb_PUU.

> **Figure 104** — Timing Diagram of PR Mode Monitoring Variables

The detailed command behavior of each stage is illustrated in Figure 105.
Cmd_E is the endpoint specified by the command; this value is determined
once the PR command is triggered. Fb_PUU is the feedback position, which is
the motor actual position. For example, Cmd_O is the target of this command
section and Err_PUU is the deviation between target position and feedback
position.
Err_PUU
Before
command
issued
Fb_PUU
Cmd_O
Cmd_E
Err_PUU
Fb_PUU
Cmd_O
Cmd_E
Err_PUU
Fb_PUU
Cmd_O
Cmd_E
Err_PUU
Fb_PUU
Cmd_O
Cmd_E
Err_PUU
Fb_PUU
Cmd_O
Cmd_E
Command in
execution
Command
completed
Motor
positioned
After
command
issued

<!-- page 11 -->

> **Figure 105** — Monitoring Variables’ Status When Executing a Command IN PR Mode

Use digital input (DI) and digital output (DO) signals to monitor PR
commands as shown in Figure 106. When the motion command is triggered by
DI.CTRG [0x08], the servo drive operates based on the command from the
internal register. Once the execution is completed, DO.Cmd_OK [0x15] turns
on. And when the motor is within its target position window, which is set by
ID159 (P1.054) InPositionWindow, the DO.TPOS [0x05] is on. Once the PR
position command completes and the motor reaches the target position, both
DO signals are on and the servo drives outputs the MC_OK [0x17] signal to
signify that this PR command is completed.

> **Figure 106** — Operation of DI and DO Signals in PR Mode

Cmd_O
Err_PUU
Command
Fb_PUU
Position (PUU)
Time
Cmd_E

<!-- page 12 -->

> **Figure 107** — Operation of DI and DO Signals in PR Mode(including delay time)

See Description of System Variable Monitoring on page 440 for more
information.

<!-- page 13 -->

Homing
The Kinetix 5100 drive provides flexible homing sequences. The drive provides
11 homing methods in PR Mode. These methods include the use of a home
sensor, travel limit switch, motor marker pulse, or collision point (torque limit)
and come with sub-selections such as whether to refer to marker pulse and
travel limit signal as the trigger. Homing method is specified by HomingMode
ID297 (P5.004) and the homing definition is determined by HomingSetting
ID397 (P6.000). Simple homing configuration is done by using KNX5100C
software. The Homing Settings are accessed using the PR Mode Editor.
In addition to the physical items used with a homing method, an assigned
Digital Input (DI.Enable Homing) is used to perform homing. An alternative
to assigning this Digital input is to execute PR00. This PR is dedicated to
performing the homing operation.
Setting Homing Mode ID297 (P5.004) - PR Mode
This setting is used with PR mode or by monitoring the Parameter Editor.
Settings:
Definition of each setting value:
X: Homing Method
Y: Z Pulse Setting
Z: Limit setting
U: Reserved
0: homing in forward direction and define PL as
homing origin
0: return to Z pulse
1: go forward to Z
pulse
2: do not look for Z
pulse
—
—
1: homing in reverse direction and define NL as
homing origin
2: homing in forward direction, ORG: OFFON as
homing origin
When encounter
limit:
0: show error
1: reverse direction
3: homing in reverse direction, ORG: OFFON as
homing origin
4: look for Z pulse in forward direction and define it as
homing origin
—
5: look for Z pulse in reverse direction and define it as
homing origin
6: homing in forward direction, ORG: ONOFF as
homing origin
0: return to Z pulse
1: go forward to Z
pulse
2: do not look for Z
pulse
7: homing in reverse direction, ORG: ONOFF as
homing origin
8: define current position as the origin
—
—
9: look for the collision point in forward direction and
define it as the origin
0: return to Z pulse
2: do not look for Z
pulse
A: look for the collision point in reverse direction and
define it as the origin
IMPORTANT
The Homing Method values shown are for the PR Operation mode. The
homing operations are the same as in IO Mode, however, the Homing
Method values are different. See raC_xxx_K5100_MAH on page 520.
Y
Z
U
X

**Extracted table (page 13, #1):**

| Y: Z Pulse Setting | Z: Limit setting |
| --- | --- |
| 0: return to Z pulse 1: go forward to Z pulse 2: do not look for Z pulse | — |
|  | When encounter limit: 0: show error 1: reverse direction |
| — |  |
| 0: return to Z pulse 1: go forward to Z pulse 2: do not look for Z pulse |  |
| — | — |
| 0: return to Z pulse 2: do not look for Z pulse |  |

<!-- page 14 -->

## Configuring Homing Setting ID397 (P6.000) - PR Mode

Settings:
•
A: DEC2: Choose the deceleration time (0…F) selected from the pulldown menu in KNX5100C software.
Function List>PR Mode Editor>Speed and Time Setting>Accel/Decel
Time.
This time corresponds to AC00 ID312 (P5.020)…AC15 ID327 (P5.035). This
deceleration is the second homing speed that is shared with the Motor
Stops Deceleration setting, which is part of Deceleration Time for AutoProtection ID296 (P5.003).
•
B: DLY: Choose the delay time (0…F) selected from the pull-down menu
in KNX5100C software.
Function List>PR Mode Editor>Speed and TIme Setting>Delay Time.
This time corresponds to DLY00 ID312 (P5.020)…DLY15 ID327 (P5.035).
This delay is used to delay the start of the homing sequence.
•
D: BOOT: (0=Disable, 1=Enable). When enabled, this setting executes the
Homing sequence after the drive is powered on and enabled (BOOT+
ServoON).
•
YX: CMD: command type
0x0: Stop: stops the motion once the Homing is completed.
0x1…0x63: once the Homing sequence is completed, execute the specified
PR command (PR#01…PR#99).
•
Z: ACC: Choose the acceleration time (0…F) selected from the pull-down
menu in KNX5100C software.
Function List>PR Mode Editor>Speed and Time Setting>Accel/Decel
Time.
These times correspond to AC00 parameter ID312 (P5.020)…AC15
parameter ID327 (P5.035).
•
U: DEC1: Choose the deceleration time (0…F) selected from the pulldown menu in KNX5100C software.
Function List>PR Mode Editor>Speed and Time Setting>Accel/Decel
Time.
This time corresponds with AC00 ID312 (P5.020)…AC15 ID327 (P5.035).
This deceleration is the first deceleration used in the homing sequence.
A
DEC2: deceleration time selection of second homing YX
CMD: command type
B
DLY: select 0…F for delay time
Z
ACC: select 0…F for acceleration time
C
—
U
DEC1: deceleration time selection of first homing
D
BOOT
-
-
IMPORTANT
After the Home position is set the motor has to decelerate to a stop.
The motor end position depends on the homing speed and deceleration
rate used for homing.
If you require to move back to the Home position, change the CMD to a
PR that performs an absolute Point-to-Point Index to move the motor
back to the Home Position (Origin Definition).
The Home position does not have to be zero.
Homing speed is limited to 200 rpm in rotary motors. Linear motors do
not have this restriction.
B
C
D
A
YX
Z
U

**Extracted table (page 14, #1):**

| DEC2: deceleration time selection of second homing | YX |
| --- | --- |
| DLY: select 0…F for delay time | Z |
| — | U |
| BOOT | - |

<!-- page 15 -->

Homing Speed and Position
This section describes how the Homing Positon and Homing Speed are used in
the Kinetix 5100 drive.
Home Position
Parameter ID398 (P6.001) HomePosition defines the Origin Position
(sometimes called the Origin Definition). The range is
-2147483648…2147483647.
Home Speed
The homing sequence is shown in Figure 108. The homing procedure uses two
speeds, a high speed and a low speed. Homing starts with Acceleration to High
Speed Homing (1st speed). Once the ORG switch is detected, the motor is
decelerated (using the 1st Deceleration Time). It will use the Low Speed
Homing (2nd speed) to complete the homing sequence by moving to a marker
pulse. Then uses the second Deceleration time to stop the motor.

> **Figure 108** — Speed Settings for Homing

Operation of Homing Types
This section describes the homing methods supported by the drive. They can
be categorized into six types.
•
Homing to Positive Limit (PL)
•
Homing to Negative Limit (NL)
•
Homing to Forward ORG OFF to ON (Rising Edge condition)
•
Homing to Reverse ORG OFF to ON (Rising Edge condition)
•
Homing to Forward ORG ON to OFF (Falling Edge condition)
•
Homing to Reverse ORG ON to OFF (Falling Edge condition)
These Homing sequences are used in PR and IO modes, although the method
for executing the homing is different (KNX5100C software/Logix Designer
application), their behavior is the same. When the marker pulse is used, the
Home Position physical location is more accurate than using an input signal
only (Home Switch/ORG for example).
The homing method value is used when you want to change the homing type
with IO mode (Class 1 Ethernet/IP); the raC_xxx_K5100_MAH Add-On
Instruction uses these methods with Set_HomingMethod value.

<!-- page 16 -->

Within the Homing section, we reference these acronyms:
•
PL - Positive Limit
•
NL - Negative Limit
•
ORG - Home Switch

> **Table 112** — Homing Method Values - IO Mode

Value
Description

Homing in forward direction and regard PL as homing origin. Return to Z pulse.

Homing in forward direction and regard PL as homing origin. Go forward to Z pulse.

Homing in forward direction and regard PL as homing origin. Do not look for Z pulse.

Homing in reverse direction and regard NL as homing origin. Return to Z pulse.

Homing in reverse direction and regard NL as homing origin. Go forward to Z pulse.

Homing in reverse direction and regard NL as homing origin. Do not look for Z pulse.

Homing in forward direction, ORG: OFF
ON as homing origin. Return to Z pulse. Shows error when
encounter limit.

Homing in forward direction, ORG: OFF
ON as homing origin. Return to Z pulse. Reverse direction when
encounter limit.

Homing in forward direction, ORG: OFF
ON as homing origin. Go forward to Z pulse. Shows error when
encounter limit.

Homing in forward direction, ORG: OFF
ON as homing origin. Go forward to Z pulse. Reverse direction
when encounter limit.

Homing in forward direction, ORG: OFF
ON as homing origin. Do not look for Z pulse. Shows error when
encounter limit.

Homing in forward direction, ORG: OFF
ON as homing origin. Do not look for Z pulse. Reverse direction
when encounter limit.

Homing in reverse direction, ORG: OFF
ON as homing origin. Return to Z pulse. Shows error when
encounter limit.

Homing in reverse direction, ORG: OFF
ON as homing origin. Return to Z pulse. Reverse direction when
encounter limit.

Homing in reverse direction, ORG: OFF
ON as homing origin. Go forward to Z pulse. Shows error when
encounter limit.

Homing in reverse direction, ORG: OFF
ON as homing origin. Go forward to Z pulse. Reverse direction
when encounter limit.

Homing in reverse direction, ORG: OFF
ON as homing origin. Do not look for Z pulse. Shows error when
encounter limit.

Homing in reverse direction, ORG: OFF
ON as homing origin. Do not look for Z pulse. Reverse direction
when encounter limit.

Look for Z pulse in forward direction and regard it as homing origin. Shows error when encounter limit.

Look for Z pulse in forward direction and regard it as homing origin. Reverse direction when encounter
limit.

Look for Z pulse in reverse direction and regard it as homing origin. Shows error when encounter limit.

Look for Z pulse in reverse direction and regard it as homing origin. Reverse direction when encounter
limit.

<!-- page 17 -->

Homing in forward direction, ORG: ON
OFF as homing origin. Return to Z pulse. Shows error when
encounter limit.

Homing in forward direction, ORG: ON
OFF as homing origin. Return to Z pulse. Reverse direction when
encounter limit.

Homing in forward direction, ORG: ON
OFF as homing origin. Go forward to Z pulse. Shows error when
encounter limit.

Homing in forward direction, ORG: ON
OFF as homing origin. Go forward to Z pulse. Reverse direction
when encounter limit.

Homing in forward direction, ORG: ON
OFF as homing origin. Do not look for Z pulse. Shows error when
encounter limit.

Homing in forward direction, ORG: ON
OFF as homing origin. Do not look for Z pulse. Reverse direction
when encounter limit.

Homing in reverse direction, ORG: ON
OFF as homing origin. Return to Z pulse. Shows error when
encounter limit.

Homing in reverse direction, ORG: ON
OFF as homing origin. Return to Z pulse. Reverse direction when
encounter limit.

Homing in reverse direction, ORG: ON
OFF as homing origin. Go forward to Z pulse. Shows error when
encounter limit.

Homing in reverse direction, ORG: ON
OFF as homing origin. Go forward to Z pulse. Reverse direction
when encounter limit.

Homing in reverse direction, ORG: ON
OFF as homing origin. Do not look for Z pulse. Shows error when
encounter limit.

Homing in reverse direction, ORG: ON
OFF as homing origin. Do not look for Z pulse. Reverse direction
when encounter limit.

Define current position as the origin.

Look for the collision point in forward direction and regard it as the origin. Return to Z pulse. Shows error
when encounter negative limit.

Look for the collision point in forward direction and regard it as the origin. Do not look for Z pulse.

Look for the collision point in reverse direction and regard it as the origin. Return to Z pulse. Shows error
when encounter positive limit.

Look for the collision point in reverse direction and regard it as the origin. Do not look for Z pulse.
IMPORTANT
The Homing Method values shown are for the IO Operation mode. The
homing operations are the same as in PR Mode, however, the Homing
Method values are different.

> **Table 112** — Homing Method Values - IO Mode (Continued)

Value
Description

<!-- page 18 -->

Homing to Positive Limit
This homing method uses the positive or negative limit (the limit is also called
a travel or overtravel), see Figure 109 through Figure 111 for examples. When
the limit is detected, you can choose to look for the marker pulse (Z) and set the
Home Position when the marker pulse is detected. If no marker pulse is used,
the Home Position is set when the limit is detected. Changing the starting
position does not change the homing operation.

> **Figure 109** — Homing to Positive Limit - Homing Method 0

## Homing Method - IO Mode

Homing Method - Description

Homing in forward direction and regard PL as homing origin, Return to Z pulse.
• Return is called 'Move backward' in KNX5100C software
• High Speed Homing (HS) (1st speed setting) is /10 - so 1000 = 100 rpm
• Low Speed (LS) Homing (2nd speed setting) is /10 - so 200 = 20 rpm
• Home Position is set at Green dot
NL
PL
HS
LS
LS
Negative Limit (NL)
Positive Limit (PL)
Homing Method: 0
Start Point 1
Start Point 2
Limit Signal
Z-Pulse
Start Point
Positive Limit OFF to ON
Positive Limit ON to OFF
Z-Pulse ON
End Point
Home Position Set
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 18, #1):**

|  |  |  |  |  |  | LS |
| --- | --- | --- | --- | --- | --- | --- |
|  |  |  |  |  |  | LS |
|  |  | NL PL |  |  |  |  |

<!-- page 19 -->

> **Figure 110** — Homing to Positive Limit - Homing Method 1

## Homing Method - IO Mode

Homing Method - Description

Homing in forward direction and regard PL as homing origin, Go forward to Z pulse.
• High Speed Homing (1st speed setting) is /10 - so 1000 = 100 rpm
• Low Speed Homing (2nd speed setting) is /10 - so 200 = 20 rpm
• Home Position is set at Green dot
NL
PL
HS
LS
LS
LS
Start Point
Positive Limit OFF to ON
Positive Limit ON to OFF
Z-Pulse ON
End Point
Home Position Set
Negative Limit (NL)
Positive Limit (PL)
Homing Method: 1
Start Point 1
Start Point 2
Limit Signal
Z-Pulse
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 19, #1):**

|  |  |  |  |  |  | LS |
| --- | --- | --- | --- | --- | --- | --- |
|  |  |  |  |  |  | LS |
|  |  | NL PL |  |  |  |  |

<!-- page 20 -->

> **Figure 111** — Homing to Positive Limit - Homing Method 2

## Homing Method - IO Mode

Homing Method - Description

Homing in forward direction and regard PL as homing origin, Do not look for Z pulse.
• High Speed Homing (1st speed setting) is /10 - so 1000 = 100 rpm
• Low Speed Homing (2nd speed setting) is /10 - so 200 = 20 rpm
• Home Position is set at Green dot
NL
PL
HS
LS
LS
Start Point
Positive Limit OFF to ON
Positive Limit ON to OFF
Z-Pulse ON
End Point
Home Position Set
Negative Limit (NL)
Positive Limit (PL)
Homing Method: 2
Start Point 1
Start Point 2
Limit Signal
Z-Pulse
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 20, #1):**

|  |  |  |  |  |  | LS |  |
| --- | --- | --- | --- | --- | --- | --- | --- |
|  |  |  |  |  |  | LS |  |
|  |  | NL PL |  |  |  |  |  |

<!-- page 21 -->

Homing to Negative Limit
This homing method uses the negative limit (the limit is also called a travel or
overtravel), see Figure 112 through Figure 114 for examples. When the limit is
detected, you can choose to look for the marker pulse (Z) and set the Home
Position when the marker pulse is detected. If no marker pulse is used, the
Home Position is set when the limit is detected. Changing the starting position
does not change the homing operation.

> **Figure 112** — Homing to Negative Limit - Homing Method 3

## Homing Method - IO Mode

Homing Method - Description

Homing in reverse direction and regard NL as homing origin, Return to Z pulse.
Home Position is set at Green dot.
NL
PL
HS
LS
LS
Homing Method: 3
Negative Limit (NL)
Positive Limit (PL)
Start Point 1
Start Point 2
Start Point
Negative Limit OFF to ON
Negative Limit ON to OFF
Z-Pulse ON
End Point
Home Position Set
Limit Signal
Z-Pulse
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

<!-- page 22 -->

> **Figure 113** — Homing to Negative Limit - Homing Method 4

## Homing Method - IO Mode

Homing Method - Description

Homing in reverse direction and regard NL as homing origin, Go forward to Z
pulse. Home Position is set at Green dot.
NL
PL
HS
LS
LS
LS
Homing Method: 4
Negative Limit (NL)
Positive Limit (PL)
Start Point 1
Start Point 2
Start Point
Negative Limit OFF to ON
Negative Limit ON to OFF
Z-Pulse ON
End Point
Home Position Set
Limit Signal
Z-Pulse
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 22, #1):**

|  | LS LS |  | HS |  |  |  |  |
| --- | --- | --- | --- | --- | --- | --- | --- |
|  | LS |  |  |  |  |  |  |
|  |  | NL PL |  |  |  |  |  |

<!-- page 23 -->

> **Figure 114** — Homing to Negative Limit - Homing Method 5

## Homing Method - IO

Mode
Homing Method - Description

Homing in reverse direction and regard NL as homing origin, Return to Z pulse. Home
Position is set at Green dot.

Homing in reverse direction and regard NL as homing origin, Go forward to Z pulse.
Home Position is set at Green dot.

Homing in reverse direction and regard NL as homing origin, Do not look for Z pulse.
Home Position is set at Green dot.
NL
PL
LS
HS
LS
Homing Method: 5
Negative Limit (NL)
Positive Limit (PL)
Start Point 1
Start Point 2
Start Point
Negative Limit OFF to ON
Negative Limit ON to OFF
Z-Pulse ON
End Point
Home Position Set
Limit Signal
Z-Pulse
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

<!-- page 24 -->

Homing to Forward ORG OFF to ON (Rising Edge condition)
You can use the home sensor (ORG) to set the Home Position. You can use the
ORG with (or without) the marker (Z) pulse to set the Home Position.
See Figure 115…Figure 117 for description.

> **Figure 115** — Homing to Forward ORG OFF to ON (Rising Edge condition) - Homing Method 6 and 7

## Homing Method - IO Mode

Homing Method - Description

Homing in forward direction, ORG: OFF to ON as homing origin, Return to Z pulse,
Shows error when encounter limit. Home Position is set at Green dot.

Homing in forward direction, ORG: OFF to ON as homing origin, Return to Z pulse,
Reverse direction when encounter limit. Home Position is set at Green dot.
NL
PL
ORG
HS
LS
HS
HS
LS
LS
Start Point
ORG OFF to ON
ORG ON to OFF
Z-Pulse ON
End Point
Home Position Set
ORG Signal
Z-Pulse
Homing Method: 6 and 7
Limit Signal
Start Point 3
Start Point 2
Start Point 1
Negative Limit (NL)
Positive Limit (PL)
ORG
ORG = Home Limit
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 24, #1):**

|  |  |  | LS |  | HS |
| --- | --- | --- | --- | --- | --- |
|  |  |  |  | LS | HS |
|  |  |  | LS |  |  |
| ORG |  |  |  |  |  |

<!-- page 25 -->

> **Figure 116** — Homing to Forward ORG OFF to ON (Rising Edge condition) - Homing Method 8 and 9

## Homing Method - IO Mode

Homing Method - Description

Homing in forward direction, ORG: OFF to ON as homing origin, Go forward to Z
pulse, Shows error when encounter limit. Home Position is set at Green dot.

Homing in forward direction, ORG: OFF to ON as homing origin, Go forward to Z
pulse, Reverse direction when encounter limit. Home Position is set at Green dot.
NL
PL
ORG
HS
LS
HS
HS
LS
LS
LS
Start Point
ORG (Home Input) OFF to ON
ORG (Home Input) ON to OFF
Z-Pulse ON
End Point
Home Position Set
ORG Signal
Z-Pulse
Homing Method: 8 and 9
Limit Signal
Start Point 3
Start Point 2
Start Point 1
Negative Limit (NL)
Positive Limit (PL)
ORG
ORG = Home Limit
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 25, #1):**

|  |  |  | LS |  |  |
| --- | --- | --- | --- | --- | --- |
|  |  |  | LS |  | HS |
|  |  |  |  |  | HS |
|  |  |  | LS |  |  |
|  |  |  | LS |  |  |
| ORG |  |  |  |  |  |

<!-- page 26 -->

> **Figure 117** — Homing to Forward ORG OFF to ON (Rising Edge condition) - Homing Method 10 and 11

## Homing Method - IO Mode

Homing Method - Description

Homing in forward direction, ORG: OFF to ON as homing origin, Do not look for Z
pulse, Shows error when encounter limit. Home Position is set at Green dot.

Homing in forward direction, ORG: OFF to ON as homing origin, Do not look for Z
pulse, Reverse direction when encounter limit. Home Position is set at Green dot.
NL
PL
ORG
HS
LS
HS
HS
LS
LS
ORG Signal
Z-Pulse
Homing Method: 10 and 11
Limit Signal
Start Point 3
Start Point 2
Start Point 1
Start Point
ORG (Home Input) OFF to ON
ORG (Home Input) ON to OFF
Z-Pulse ON
End Point
Home Position Set
Negative Limit (NL)
Positive Limit (PL)
ORG
ORG = Home Limit
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 26, #1):**

|  |  |  | LS |  | HS |
| --- | --- | --- | --- | --- | --- |
|  |  |  |  | LS | HS |
|  |  |  | LS |  |  |
| ORG |  |  |  |  |  |

<!-- page 27 -->

Homing to Reverse ORG OFF to ON (Rising Edge condition)
Figure 118…Figure 120 show the different homing methods that use the ORG
signal with optional marker (Z) pulse. Notice that in some cases the low speed
(LS) moves the motor forward or backward. When the ORG signal is used
alone, a second transition of this signal must occur for the homing sequence to
complete (End).

> **Figure 118** — Homing to Reverse ORG OFF to ON (Rising Edge condition) - Homing Method 12 and 13

## Homing Method - IO Mode

Homing Method - Description

Homing in reverse direction, ORG: OFF to ON as homing origin, Return to Z pulse,
Shows error when encounter limit. Home Position is set at Green dot.

Homing in reverse direction, ORG: OFF to ON as homing origin, Return to Z pulse,
Reverse direction when encounter limit. Home Position is set at Green dot.
NL
PL
ORG
HS
LS
HS
HS
LS
LS
Start Point
ORG (Home Input) OFF to ON
ORG (Home Input) ON to OFF
Z-Pulse ON
End Point
Home Position Set
Homing Method: 12 and 13
Negative Limit (NL)
Positive Limit (PL)
ORG
Start Point 1
Start Point 2
Start Point 3
ORG Signal
Z-Pulse
Limit Signal
ORG = Home Limit
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 27, #1):**

|  |  |  | LS |  |  |
| --- | --- | --- | --- | --- | --- |
| HS |  |  | LS |  | HS |
| HS |  |  | LS |  |  |
|  |  |  |  | ORG |  |

<!-- page 28 -->

> **Figure 119** — Homing to Reverse ORG OFF to ON (Rising Edge condition)- Homing Method 14 and 15

## Homing Method - IO Mode

Homing Method - Description

Homing in reverse direction, ORG: OFF to ON as homing origin, Go forward to Z
pulse, Shows error when encounter limit. Home Position is set at Green dot.

Homing in reverse direction, ORG: OFF to ON as homing origin, Go forward to Z
pulse, Reverse direction when encounter limit. Home Position is set at Green dot.
NL
PL
ORG
HS
LS
HS
HS
LS
LS
LS
Start Point
ORG (Home Input) OFF to ON
ORG (Home Input) ON to OFF
Z-Pulse ON
End Point
Home Position Set
Homing Method: 14 and 15
Negative Limit (NL)
Positive Limit (PL)
Start Point 1
Start Point 2
Start Point 3
ORG Signal
Z-Pulse
Limit Signal
ORG
ORG = Home Limit
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 28, #1):**

| HS |  |  | LS |  | HS |
| --- | --- | --- | --- | --- | --- |
| HS |  |  | LS LS |  |  |
|  |  |  | LS |  |  |
|  |  |  |  | ORG |  |

<!-- page 29 -->

> **Figure 120** — Homing to Reverse ORG OFF to ON (Rising Edge condition) - Homing Method 16 and 17

## Homing Method - IO Mode

Homing Method - Description

Homing in reverse direction, ORG: OFF to ON as homing origin, Do not look for Z
pulse, Shows error when encounter limit. Home Position is set at Green dot.

Homing in reverse direction, ORG: OFF to ON as homing origin, Do not look for Z
pulse, Reverse direction when encounter limit. Home Position is set at Green dot.
NL
PL
ORG
HS
LS
HS
LS
HS
LS
Start Point
ORG (Home Input) OFF to ON
ORG (Home Input) ON to OFF
Z-Pulse ON
End Point
Home Position Set
Homing Method: 16 and 17
Negative Limit (NL)
Positive Limit (PL)
Start Point 1
Start Point 2
Start Point 3
ORG Signal
Z-Pulse
Limit Signal
ORG
ORG = Home Limit
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 29, #1):**

|  |  |  | LS |  |  |
| --- | --- | --- | --- | --- | --- |
| HS |  |  | LS |  | HS |
| HS |  |  | LS |  |  |

<!-- page 30 -->

Referencing the Z Pulse
This homing method uses the motor marker pulse (Z) to set the
HomingPosition. The marker pulse is on the motor encoder and occurs once
per motor rotation.

> **Figure 121** — Z Pulse as Reference Point

## Homing Method - IO Mode

Homing Method - Description

Look for Z pulse in forward direction and regard it as homing origin, Shows error
when encounter limit.

Look for Z pulse in forward direction and regard it as homing origin, Reverse
direction when encounter limit.

Look for Z pulse in reverse direction and regard it as homing origin, Shows error
when encounter limit.

Look for Z pulse in reverse direction and regard it as homing origin, Reverse
direction when encounter limit.
V
t
V
t
HS = High Speed (1st Speed)
Homing Method: 18
and 19
Home Position Set
Homing Method: 20
and 21
Z pulse
Limit signal
Start point
End
Axis (Load)
Limit
Motor
V = Velocity
t = Time
End
HS
HS
Start point
Home Position Set

<!-- page 31 -->

Homing to Forward ORG ON to OFF (Falling Edge condition)
You can use the home sensor (ORG) to set the Home Position. You can use the
ORG with the marker (Z) pulse to set the Home Position.
See Figure 122…Figure 124 for description.

> **Figure 122** — Homing to Forward ORG ON to OFF (Falling Edge condition) - Homing Method 22 and 23

## Homing Method - IO Mode

Homing Method - Description

Homing in forward direction, ORG: ON to OFF as homing origin, Return to Z pulse,
Shows error when encounter limit. Home Position is set at Green dot.

Homing in forward direction, ORG: ON to OFF as homing origin, Return to Z pulse,
Reverse direction when encounter limit. Home Position is set at Green dot.
NL
PL
ORG
HS
LS
LS
HS
HS
LS
LS
LS
LS
Homing Method: 22 and 23
Negative Limit (NL)
Positive Limit (PL)
ORG
Start Point 1
Start Point 2
Start Point 3
Start Point
ORG (Home Input) OFF to ON
ORG (Home Input) ON to OFF
Z-Pulse ON
End Point
Home Position Set
ORG Signal
Z-Pulse
Limit Signal
ORG = Home Limit
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 31, #1):**

|  | LS |  |  |  |
| --- | --- | --- | --- | --- |
|  |  | LS |  | HS |
|  |  | LS |  |  |
|  |  | LS |  | HS |
|  |  | LS |  |  |
|  |  | LS |  |  |
| ORG |  |  |  |  |

<!-- page 32 -->

> **Figure 123** — Homing to Forward ORG ON to OFF (Falling Edge condition) - Homing Method 24 and 25

## Homing Method - IO Mode

Homing Method - Description

Homing in forward direction, ORG: ON to OFF as homing origin, Go forward to Z
pulse, Shows error when encounter limit. Home Position is set at Green dot.

Homing in forward direction, ORG: ON to OFF as homing origin, Go forward to Z
pulse, Reverse direction when encounter limit. Home Position is set at Green dot.
NL
PL
ORG
HS
LS
HS
HS
LS
LS
Homing Method: 24 and 25
Negative Limit (NL)
Positive Limit (PL)
ORG
Start Point 1
Start Point 2
Start Point 3
Start Point
ORG (Home Input) OFF to ON
ORG (Home Input) ON to OFF
Z-Pulse ON
End Point
Home Position Set
ORG Signal
Z-Pulse
Limit Signal
ORG = Home Limit
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 32, #1):**

|  | LS |  |  |  |
| --- | --- | --- | --- | --- |
|  |  | LS |  | HS |
|  |  | LS |  | HS |
| ORG |  |  |  |  |

<!-- page 33 -->

> **Figure 124** — Homing to Forward ORG ON to OFF (Falling Edge condition)- Homing Method 26 and 27

## Homing Method - IO Mode

Homing Method - Description

Homing in forward direction, ORG: ON to OFF as homing origin, Do not look for Z
pulse, Shows error when encounter limit. Home Position is set at Green dot.

Homing in forward direction, ORG: ON to OFF as homing origin, Do not look for Z
pulse, Reverse direction when encounter limit. Home Position is set at Green dot.
NL
PL
ORG
HS
LS
HS
HS
LS
LS
Homing Method: 26 and 27
Negative Limit (NL)
Positive Limit (PL)
ORG
Start Point 1
Start Point 2
Start Point 3
Start Point
ORG (Home Input) OFF to ON
ORG (Home Input) ON to OFF
Z-Pulse ON
End Point
Home Position Set
ORG Signal
Z-Pulse
Limit Signal
ORG = Home Limit
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 33, #1):**

|  | LS |  |  |  |
| --- | --- | --- | --- | --- |
|  |  | LS |  | HS |
|  |  | LS |  | HS |
| ORG |  |  |  |  |

<!-- page 34 -->

Homing to Reverse ORG ON to OFF (Falling Edge condition)
Figure 125…Figure 127 shows the different homing methods that use the ORG
signal (using the falling edge, or ON->OFF transition) with optional marker (Z)
pulse. Notice that in some cases the low speed (LS) moves the motor forward or
backward. When the ORG signal is used alone, a second transition of this
signal must occur for the homing sequence to complete (End).

> **Figure 125** — Homing to Reverse ORG ON to OFF (Falling Edge condition) - Homing Method 28 and 29

## Homing Method - IO Mode

Homing Method - Description

Homing in reverse direction, ORG: ON to OFF as homing origin, Return to Z pulse,
Shows error when encounter limit. Home Position is set at Green dot.

Homing in reverse direction, ORG: ON to OFF as homing origin, Return to Z pulse,
Reverse direction when encounter limit. Home Position is set at Green dot.
Homing Method: 28 and 29
Negative Limit (NL)
Positive Limit (PL)
ORG
Start Point 1
Start Point 2
Start Point 3
Start Point
ORG (Home Input) OFF to ON
ORG (Home Input) ON to OFF
Z-Pulse ON
End Point
Home Position Set
ORG Signal
Z-Pulse
Limit Signal
ORG = Home Limit
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

<!-- page 35 -->

> **Figure 126** — Homing to Reverse ORG ON to OFF (Falling Edge condition) - Homing Method 30 and 31

## Homing Method - IO Mode

Homing Method - Description

Homing in reverse direction, ORG: ON to OFF as homing origin, Go forward to Z
pulse, Shows error when encounter limit. Home Position is set at Green dot.

Homing in reverse direction, ORG: ON to OFF as homing origin, Go forward to Z
pulse, Reverse direction when encounter limit. Home Position is set at Green dot.
LS
HS
HS
HS
LS
LS
NL
PL
ORG
Homing Method: 30 and 31
Negative Limit (NL)
Positive Limit (PL)
ORG
Start Point 1
Start Point 2
Start Point 3
Start Point
ORG (Home Input) OFF to ON
ORG (Home Input) ON to OFF
Z-Pulse ON
End Point
Home Position Set
ORG Signal
Z-Pulse
Limit Signal
ORG = Home Limit
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 35, #1):**

| HS |  |  | LS | HS |
| --- | --- | --- | --- | --- |
| HS |  | LS |  |  |
|  |  |  | LS |  |
|  |  |  |  | ORG |

<!-- page 36 -->

> **Figure 127** — Homing to Reverse ORG ON to OFF (Falling Edge condition) - Homing Method 32 and 33

## Homing Method - IO Mode

Homing Method - Description

Homing in reverse direction, ORG: ON to OFF as homing origin, Do not look for Z
pulse, Shows error when encounter limit. Home Position is set at Green dot.

Homing in reverse direction, ORG: ON to OFF as homing origin, Do not look for Z
pulse, Reverse direction when encounter limit. Home Position is set at Green dot.
HS
LS
HS
HS
LS
LS
NL
PL
ORG
Homing Method: 32 and 33
Negative Limit (NL)
Positive Limit (PL)
ORG
Start Point 1
Start Point 2
Start Point 3
Start Point
ORG (Home Input) OFF to ON
ORG (Home Input) ON to OFF
Z-Pulse ON
End Point
Home Position Set
ORG Signal
Z-Pulse
Limit Signal
ORG = Home Limit
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)

**Extracted table (page 36, #1):**

| HS |  |  | LS | HS |
| --- | --- | --- | --- | --- |
| HS |  | LS |  |  |
|  |  |  | LS |  |
|  |  |  |  | ORG |

<!-- page 37 -->

Referencing the Present Position as the Origin
This homing method sets the present physical location of the motor as the
HomePosition ID 397 (P6.001). The motor does not move using this homing
method and the motor must be enabled. When the Operation Mode is IO, this
method is used with the raC_xxx_K5100_MAH and is Homing Method 34.

> **Figure 128** — Referencing Current Position as the Origin

The Digital Input (DI.Enable Homing) must be used to perform this homing.
Referencing the Torque Limit
This homing method references a user defined motor torque parameter
HomingTorqueCollisionTorqueLimit ID177 (P1.087) as a 'hard stop'. When the
motor is homing and the actual motor torque equals the
HomingTorqueCollisionTorqueLimit; and this condition is true for the
HomingCollisionTorqueLimitTime ID178 (P1.088), the optional marker
homing is executed and the homing is complete.
Be sure that the torque used for this homing method is within any user defined
torque limit or the homing does not complete.
Homing Method - IO
Mode
Homing Method - Description

Define current position as the origin.
Motor
End
DI.SHOM [0x27] Return to Homing Origin
DI. Enable homing

<!-- page 38 -->

> **Figure 129** — Torque Limit as Reference Point - Forward Direction

> **Figure 130** — Torque Limit as Reference Point - Reverse Direction

Homing Method - IO Mode Homing Method - Description

Look for the collision point in forward direction and regard it as the origin. Return to Z
pulse.

Look for the collision point in forward direction and regard it as the origin. Do not look
for Z pulse.
Homing Method - IO Mode
Homing Method - Description

Look for the collision point in reverse direction and regard it as the origin. Return to Z
pulse.

Look for the collision point in reverse direction and regard it as the origin. Do not look
for Z pulse.
V
t
V
t
Homing Method: 35
Homing Method: 36
Start point
Home
Position
Set
End
HS
LS
Start point
HS
End
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)
V = Velocity
t = Time
{
HomingCollisionTorqueLimitTime ID178m
ID178
(P1.088)
HomingTorqueCollisionTorqueLimit ID177
Protector (hard stop)
Axis (Load)
Motor
V
t
V
t
Homing Method: 37
Homing Method: 38
Start point
Home
Position
Set
End
HS
LS
Start point
HS
End
HS = High Speed (1st Speed)
LS = Low Speed (2nd Speed)
V = Velocity
t = Time
{
HomingCollisionTorqueLimitTime ID178m
Protector (hard stop)
Axis (Load)
Motor

**Extracted table (page 38, #1):**

| L |  |  |
| --- | --- | --- |
|  | S |  |
| En | d |  |

<!-- page 39 -->

## Constant Speed Control

When the Kinetix 5100 drive is in PR Operation Mode, you can use the
Constant Speed control function. The parameters available for PR Mode with
speed control are acceleration/deceleration time, delay time, and target speed.

> **Figure 131** — Parameter for PR Mode Speed Control

The constant speed control command type is a simple command. See PR Mode
Definitions on page 288 to see the different options available with this
command.

> **Figure 132** — PR Mode Speed Screen

The constant speed control is useful when it is triggered by using an event input
to interrupt a currently executing PR. Use the interrupt to change the speed of a
currently executing constant speed command.
Target speed
Acceleration time
Time
Speed
Feedback speed
Command
Delay time
Target speed
Deceleration time
Time
Speed
Delay time
Feedback speed
Command
Speed
Target Speed
Feedback Speed
Feedback Speed
Target Speed
Delay Time
Time
Acceleration Time
Time
Speed
Command
Command
Delay Time
Deceleration Time

<!-- page 40 -->

These settings are the functions of each bit when a speed command is applied
by using the PRCmdXSetting parameter ID339…ID595 (P6.002…P7.098).
Settings:
Format of this parameter: (High word h) DCBA: (Low word L) UZYX
•
Y OPT: option
See PR Mode Setting on page 346, where you can configure the PR Mode for
speed and control in KNX5100C Software.
A
Reserved
X
TYPE, Command type - Set to 1
B
DLY, Delay time index
Y
OPT, Option
C
Reserved
Z
ACC, Acceleration time index
D
Reserved
U
DEC, Deceleration time index
Data Content
Target speed [0.1 rpm / PPS]
BIT

## Command Type

-
UNIT
AUTO
INS
High word
Low word
C
D
B A
Y
Z
U
X
High Word
Low Word

**Extracted table (page 40, #1):**

| Reserved | X |
| --- | --- |
| DLY, Delay time index | Y |
| Reserved | Z |
| Reserved | U |

<!-- page 41 -->

## Position Control Command

When the Kinetix 5100 drive is in PR Operation Mode, you can use the
Positioning function. There are two command types:
•
Point-to-Point command
•
Point-to-Point command (Proceed to the next command when
completed).
The only difference between the two is the function at the end of the
command: stop or continue to the next consecutive PR. Continue means
the next sequential PR is executed.
See PR Command Setting on page 288 on for details on the PR commands.

> **Figure 133** — PR Mode Position Interface of the Configuration Software

> **Figure 134** — Parameters for PR Mode Position Settings

Target speed
Acceleration time
Time
Speed
Delay time
Deceleration time
Position
command
Speed
Target Speed
Delay Time
Time
Acceleration Time
Position
Command
Deceleration Time

**Extracted table (page 41, #1):**

|  |  | Position Position Command command |  |
| --- | --- | --- | --- |
| Target speed |  |  |  |
|  | cceleration Time Acceleration tim |  |  |

<!-- page 42 -->

These settings are the functions of each bit when a position command is
applied.
Settings:
Format of this parameter: (High word h) DCBA: (Low word L) UZYX
•
Y OPT: option
See Use the PR Mode Editor in KNX5100C Software on page 342, where you
can configure the PR Mode for one of two PR Mode position command types:
•
Mode 2 = Point-to-Point Command
•
Mode 3 = Point-to-Point Command (Proceed to next command when
complete)
Position Command Types
There are four types of position commands for the PR Mode. These same
position commands can be used for IO Mode, although their names are
different. The raC_xxx_K5100_MAM Set_MoveType is shown in brackets. You
can choose the position command according to the application requirements.
The functions of each type are described in the examples below. The condition
in these examples is that a position command is still being executed and
another type of command is inserted. To see how the position commands are
combined, see Figure 135.
•
Absolute position command (ABS, raC_xxx_K5100_MAM Type 0 =
Absolute): when executed, the target position value equals the absolute
command value. In Figure 135, an ABS command with the value of 60000
PUU is inserted in the previous PR command with setting target position
of 60000 PUU on the coordinate axis.
A
SPD, Target speed index
X
TYPE, Command type - Set to 2 or

B
DLY, Delay time index
Y
OPT, Option
C
Reserved
Z
ACC, Acceleration time index
D
Reserved
U
DEC, Deceleration time index
Data Content
Target position [PUU]
BIT

Description
Command type
CMD
OVLP
INS
-
Data Content

-
-
ABS (absolute positioning)

REL (relative positioning)

INC (incremental positioning)

CAP (high-speed position capturing)
High word
Low word
C
D
B A
Y
Z
U
X
High Word
Low Word

**Extracted table (page 42, #1):**

| SPD, Target speed index | X |
| --- | --- |
| DLY, Delay time index | Y |
| Reserved | Z |
| Reserved | U |

**Extracted table (page 42, #2):**

| 3 | 2 | 1 | 0 |
| --- | --- | --- | --- |
| CMD |  | OVLP | INS |
| 0 | 0 | - | - |
| 0 | 1 |  |  |
| 1 | 0 |  |  |
| 1 | 1 |  |  |

<!-- page 43 -->

•
Relative position command (REL, raC_xxx_K5100_MAM Type 7 =
Relative): when executed, the target position value is the motor's current
position value plus the position command value. In the figure, a REL
command with the value on 60000 PUU is inserted in the previous PR
command. The target position is the motor's current position (20000
PUU) plus the relative position command (60000 PUU), which equals
80000 PUU in the coordinate system. The target position specified by the
original command is omitted.
•
Incremental command (INC, raC_xxx_K5100_MAM Type 1 =
Incremental): when executed, the target position is the previous target
position value plus the current position command value. In the example
below, an INC command with the value of 60000 PUU is inserted in the
previous PR command. The target position is the previous target position
value 30000 PUU plus the relative position command 60000 PUU, which
equals 90000 PUU. The previous destination specified by the previous
command is combined to define the new one.
•
High-speed position capturing command
(CAP, raC_xxx_K5100_MAM Type 8 = Capture): when executed, the
target position is the last position acquired by the Capture function plus
the position command value. In the following example, a high-speed
capturing command with the value of 60000 PUU is inserted in the
previous PR command. The target position value is the captured position
value of 10000 PUU plus the relative command of 60000 PUU, which
equals 70000 PUU. The target position specified by the original
command is omitted.

> **Figure 135** — Four Types of Position Command

10000 20000 30000 40000 50000 60000 70000 80000 90000
100000
10000 20000 30000 40000 50000 60000 70000 80000 90000
100000
60000
Absolute command
(ABS)
60000 PUU
Motor’s current position
(Fb_PUU)
Target
position
Motor’s current
position
(Fb_PUU)
Relative command
(REL)
60000 PUU
Target
position

10000 20000 30000 40000 50000 60000 70000 80000 90000
100000
Motor’s current
position
(Fb_PUU)
Target
position
Endpoint of
previous command
(Cmd_E)
60000
Incremental
command (INC)
60000 PUU

10000 20000 30000 40000 50000 60000 70000 80000 90000
100000
Motor’s current
position
(Fb_PUU)
Target
position
Capturing
position
60000
High-speed position
capturing (CAP)
60000 PUU
Motor
current position
(Fb_PUU)
Target
position
Endpoint of
previous command
(Cmd_E)
Motor current
position
(Fb_PUU)
Motor current
position
(Fb_PUU)
Target
position
Target
position
Target
position
Motor current
position
(Fb_PUU)
Capturing
position
Incremental
command (INC)
60000 PUU
Relative command
(REL)
60000 PUU
Absolute command
(ABS)
60000 PUU
High-speed position
capturing (CAP)
60000 PUU

**Extracted table (page 43, #1):**

|  |  | 000 |
| --- | --- | --- |
|  | Motor current Moptoors’sit ciounrrent p position (F(Fbb__PPUUUU)) | Endpoint of Endpoint of rperveivoiouuss ccoommmmaannd ((CCmmdd__EE)) |
| Incremental Incremental command (INC) command (INC) 60000 PUU 60000 PUU |  |  |

<!-- page 44 -->

## Jump Command

The drive provides a jump command in PR Mode. It can call any PR command
or form PR commands into a loop, as shown in Figure 136. You can specify the
PR command number to be jumped to by using PR Mode setting screen in the
configuration software. Among the options, Interrupt Previous is available,
this interrupts the currently executing motion command. DLY is the delay
time determined by shared PR parameters ID332…ID347 (P5.040…P5.055).
Once a jump command is issued, the servo drive will start counting the delay
time and execute the jump once the delay time expires.

> **Figure 136** — Jump Command in PR Mode

> **Figure 137** — Using PR Mode Jump Command

PR #01
PR #07
PR #08
PR #09
PR #10
Jump
Jump

<!-- page 45 -->

These settings are the functions of each bit when a jump command is applied
using the ID399…ID595 (P6.002…P7.098) PRCmdXSetting.
Settings:
Format of this parameter: (High word h) DCBA: (Low word L) UZYX
•
Y: OPT: option
See Use the PR Mode Editor in KNX5100C Software on page 342, where you
can configure the PR Mode as Jump to specified command.
A
Reserved
X
TYPE, Command type - Set to 7
B
DLY, Delay time index
Y
OPT, Option
C
Reserved
Z
Reserved
D
Reserved
U
Reserved
Data Content
Jump to target PR command(0…99)
BIT

Command Type -
-
-
INS
High word
Low word
C
D
B A
Y
Z
U
X
High Word
Low Word

**Extracted table (page 45, #1):**

| Reserved | X |
| --- | --- |
| DLY, Delay time index | Y |
| Reserved | Z |
| Reserved | U |

<!-- page 46 -->

## Write Command

The write command in PR Mode can write constants, parameters, data arrays,
and monitoring variables to the specified parameters or data arrays. Users can
write the parameter by using the PR Mode setting screen of the configuration
software. This command can interrupt a currently executing PR and can load
the next PR when the write is completed. The ROM option lets the drive write
parameters to both RAM and EEPROM at the same time. However, frequent
usage will shorten the life of EEPROM. DLY is the delay time selected by shared
PR parameters ID332…ID347 (P5.040…P5.055). Once a jump command is
issued, the servo drive will start calculating the delay time. The table below
shows the bit function when a write command is used.

> **Figure 138** — Using PR Write Command

## Writing Target

Data Source
Parameter
Constant
Data array
Parameter
-
Data array
-
Monitoring variables

<!-- page 47 -->

These settings are the functions of each bit when a write command is applied
using the PRCmdXSetting ID399…ID595 (P6.002…P7.098).
Settings:
Format of this parameter: (High word h) DCBA: (Low word L) UZYX
•
Y: OPT: option
•
C: SOUR_DEST: data source and data format to be written.
•
Z,U,A: DESTINATION: destination
•
SOURCE: data source setting
See Use the PR Mode Editor in KNX5100C Software on page 342 to configure
the PR Mode as Write to Parameters or Data Array.
A
DESTINATION
X
TYPE, Command type - Set to 8
B
DLY, Delay time index
Y
OPT, Option
C
SOUR_DEST
Z
DESTINATION
D

U
DESTINATION
Data Content
SOURCE
BIT

Command type
-
ROM
AUTO
INS
BIT

Description
Command type
SOUR
-
DEST
Data source
Writing target
Data content

Constant
Parameter

Parameter
Parameter

Data array
Parameter

Monitoring variable
Parameter

Constant
Data array

Parameter
Data array

Data array
Data array

Monitoring variable
Data array
A
U
Z
Target: Parameter
Parameter group
Parameter number
Target: Data array
Data array number
D
C
B
A
U
Z
Y
X
Data source: Constant
Constant data
Data source: Parameter
-
Parameter
group
Parameter
number
Data source: Data array
-
Data array number
Data source: Monitoring variable -
Monitoring
variable number
High word
Low word
C
D
B A
Y
Z
U
X
High Word
Low Word

**Extracted table (page 47, #1):**

| DESTINATION | X |
| --- | --- |
| DLY, Delay time index | Y |
| SOUR_DEST | Z |
| 0 | U |

**Extracted table (page 47, #2):**

| 3 | 2 | 1 | 0 |  |
| --- | --- | --- | --- | --- |
| SOUR |  | - | DEST | Data source |
| 0 0 1 1 0 0 1 1 | 0 | 0 | 0 | Constant |
|  | 1 |  | 0 | Parameter |
|  | 0 |  | 0 | Data array |
|  | 1 |  | 0 | Monitoring variable |
|  | 0 |  | 1 | Constant |
|  | 1 |  | 1 | Parameter |
|  | 0 |  | 1 | Data array |
|  | 1 |  | 1 | Monitoring variable |

**Extracted table (page 47, #3):**

| - | Parameter group |
| --- | --- |
| - |  |
| - |  |

<!-- page 48 -->

## Index Position Command

At this time, the Kinetix 5100 drive does not have a native rotary mode
operation. This Index Position Command is a feature that lets you execute
absolute indexing commands that persist through the natural unwind of the
feedback device (typically, 2.147 billion counts).
An example is a conveyor that constantly indexes forward, once the natural
unwind of the encoder occurs, the absolute position will continue to position
using the index coordinate system. Since there is no unwind operation, the
feedback counts register does not reflect the index coordinate position (PUU).

> **Figure 139** — PR Mode Indexing Coordinates

ID234 (P2.052) IndexingCoordinatesScale sets the spacing of the indexing
coordinates, indexing command position, and indexing feedback position. If
the value is too small, it can cause errors in the indexing coordinates.

> **Table 113** — Relevant Parameters

Parameter
Name
ID8 (PM.032)
MotorMaxSpeed
ID151 (P1.044)
GearRatioslaveCountsN1
ID152 (P1.045)
GearRatioMasterCounts
ID234 (P2.052)
IndexingCoordinatesScale
ID600 (PM.004)
EncoderResolution
IMPORTANT
Prior to using the Index Position function, homing must be completed.

Indexing
coordinates
Total index moving distance
Cmd No.
Total index
moving
distance
Motor’s
running
direction
Indexing
coordinates

Total index moving
distance ID234 (P2.052) -1
Motor’s feedback
position
Motor’s running
direction
Motor’s index position

<!-- page 49 -->

We have created an Index Coordinates Setting Wizard to guide you in
selecting the correct values for your Index Position command.

> **Figure 140** — PR Mode Index Position Screen

Hex Settings for Index Coordinate System
These settings are the functions of each bit when a index command is applied
using the ID339…ID595 (P6.002…P7.098) PRCmdXSetting.
Settings:
Format of this parameter: (High word h) DCBA: (Low word L) UZYX
A
SPD, Target speed index
X
TYPE, Command type - Set to 0xA
B
DLY, Delay time index
Y
OPT, Option
C
OPT2
Z
ACC, Acceleration time index
D
Reserved
U
DEC, Deceleration time index
Data Content
Index Position command [PUU](0 –
P2.052-1)
High word
Low word
C
D
B A
Y
Z
U
X
High Word
Low Word

**Extracted table (page 49, #1):**

| SPD, Target speed index | X |
| --- | --- |
| DLY, Delay time index | Y |
| OPT2 | Z |
| Reserved | U |

<!-- page 50 -->

•
Y: OPT: option
•
C: OPT2: Option 2
See Index Coordinates Settings Wizard on page 335 for Index position control
and use the Index Coordinates Setting Wizard.
Index Coordinates Settings Wizard
The wizard simplifies the entries of the index coordinates by pre-populating
the PR data that is specified. The wizard uses your entries and creates one (or
multiple) PR# entries to represent the information entered in the wizard. Click
Index Coordinates Setting Wizard in the PR screen of the configuration
software to launch the wizard.

> **Figure 141** — Indexing Coordinates Setting Wizard

BIT

Description
Command type
DIR
OVLP
INS
-
Data content

-
-
Forward (always move forward)

Reverse (always move in the reverse
direction)

Shortest distance

-
BIT

Command type
-
AUTO
-
S_LOW

**Extracted table (page 50, #1):**

| 3 | 2 | 1 | 0 |
| --- | --- | --- | --- |
| DIR |  | OVLP | INS |
| 0 | 0 | - | - |
| 0 | 1 |  |  |
| 1 | 0 |  |  |
| 1 | 1 |  |  |

<!-- page 51 -->

> **Figure 142** — Indexing Coordinates Setting Wizard in PR Mode

<!-- page 52 -->

As shown in Figure 143, the start PR command is set to 1 and command
number is set to 8 (which means that the wizard creates 8 sequential PRs) and
total moving distance is 80000 PUU (the wizard creates 8 PRs with an
equidistant 10000 PUU each distance). Click OK for the wizard to
automatically pre-populate the PR values (Figure 144). You can modify the
values with your application requirements if needed.
These index types are used with the raC_xxx_K5100_MAM, Move Types 2,3,4.

> **Figure 143** — PR Mode Index Position Example

> **Figure 144** — PR Mode Setting

See Index Position Command on page 333 for information on the Index
Coordinates function. If you choose a command type of [0xA]: Index Position
Control, you can configure the OPTIONS and the Speed and Time Setting on
the PR Mode tab.

> **Figure 145** — PR Mode Editor for Index Position Command

Indexing
Coordinate
Target
position
Current
position
Total index
moving
distance
Motor’s
running
direction
Indexing
coordinates

Target
position
Current
position

.
10000 PUU
20000 PUU
30000 PUU
40000 PUU
50000 PUU
60000 PUU
70000 PUU
0 PUU
.
Forward Direction (Always forward).
raC_MAM; Move Type=3
Reverse Direction (Always reverse).
raC_MAM; Move Type=4
and
Shortest Distance. raC_MAM; Move Type=2
Forward Direction
(Always forward)
Reverse Direction
(Always reverse)
and
Shortest Distance

<!-- page 53 -->

## Index Position Command Operation

In the scope tracing in Figure 146, the blue pen is motor rpm and the pink pen
is motor feedback position.

> **Figure 146** — Index Position Command Operation

When executing the indexes (in this example, PR#01…PR#-08) the motor must
be homed before executing the indexes. The indexes move forward for 8
separate indexes, when it is complete, you can restart the sequence by setting
Command Triggered DI signal. This Scope trace shows two sequences by using
the Index Position instructions. In summary, the benefit to using this system
is that the absolute positioning can occur with repetitive indexes, without
having to re-calculate absolute targets. This system provides a 'quasi' rotary
mode of operation.
IMPORTANT
The indexing coordinate system may not reflect actual motor counts,
because its absolute range is persistent through the natural rollover of
the motor feedback. For this reason, it is not considered a true rotary
unwind.

<!-- page 54 -->

## Arithmetic Operation

Commands
In PR Mode, the drive provides arithmetic and simple logic commands,
including addition, subtraction, multiplication, division, OR, AND, MOD, and
logic conditions. Available operands that can be written or read include user
variable, parameter, data array, monitoring variable, and constant types.
Arithmetic operations must be set via the Arithmetic Operations dialog box in
KNX5100C software. See Index Position Command Operation on page 338. To
avoid error occurrences, do not use the front panel or a Message instruction to
do the setting.

> **Figure 147** — PR Arithmetic Operations Screen

## Arithmetic Operations

When you configure a PR command type as [0xB] Statement, you must use the
Arithmetic Operations dialog box in KNX5100C software to define the
commands, including addition, subtraction, multiplication, division, and logic
conditions. The configuration dialog box has three sections: Expressions,
Procedure, and Statement.
Type
Definition
User[*]
User Variable
(0…63)
64 total user variables stored in the drive - you can use these variables, but they
are internal to the drive only. These variables are 32 bit registers.
Px.xxx
Parameter Entry
Use the pull-down menu to select the parameter.
IMPORTANT: Not all parameters support a write using the statement command - it
is more likely to have success using a Write to Parameter Command dedicated
operation. However, depending on the parameter and the drive mode including
state, not all parameters can be modified.
Arr[*]
User Data Array
There are 800 user Data Array values that can be modified. These include E-CAM
points.

<!-- page 55 -->

When in PR Mode, the Statement type allows you to define an expression, then
evaluate the logical output of the PR once the Expression executes and the
Procedure statement is evaluated. The Statement type only executes once and
must have the Servo On signal true. The Statement type only operates within
the PR programming environment. For example, if you initiate a Jog Forward
command by using the Digital Input, the Statement PRs do not execute in
parallel. The Statement type (like other PRs) executes once the initial PR is
selected (by using Digital IO) and the Command Triggered Digital Input
transitions ON.
Statement Type Characteristics
•
Statements are executed once (the statement does not continuously
evaluate; unless a JMP statement is used)
•
Statements only operate while the drive is enabled

> **Figure 148** — PR Arithmetic Operations Screen

## Data Format

Value
Data Format
Value

<!-- page 56 -->

## Expressions Section

This section supports addition, subtraction, multiplication, division, AND, OR,
and MOD operation as well as logical operations for multiple data. Table 114
shows the supported operators and calculation data with data format in DEC
and HEX.
Procedure Section
This section uses the IF statement to evaluate whether the user-defined
condition is fulfilled. If it is true, jump to the specified PR command pull-down
setting is used. If it is false, jump to the other specified PR command pulldown setting is used. If you click Next PR in Quick Setting, the software
automatically inputs the condition and then jumps to the next PR command. If
you leave this section blank, then the PR procedure stops once the basic
operation is done. See Table 115 for data formats and operators.

> **Table 115** — Field Description for the Procedure Setting Section

> **Table 114** — Description of Each Field in the Expressions Section

Data to be Written
=
Calculation Data
Operator
Calculation Data
User variable (User[0…63])
User variable
(User[0…63])
Addition (+)
Subtraction (-)
Multiplication (*)
Division (/)
Obtain remainder (%)
And (&)
Or (|)
User variable (User[0…63])
Constant (Constant)
Parameter (PX.XXX)
Data array (Arr[0…799])
Constant
(Constant)
Data array (Arr[0…799])
Parameter (PX.XXX)
Data array (Arr[0…799])
Monitoring variable
(Mon[*])
Data format
Operator
Data format
User variable
(User[0…63])
Greater than (>)
Greater than or equal
to(≥)
Less than (<)
Less than or equal to (≤)
Equal to (=)
Not equal to (≠)
User variable
(User[0…63])
Constant
(Constant)
Data array
(Arr[0…799])
Constant
(Constant)
Parameter
(PX.XXX)
Data array
(Arr[0…799])
Monitoring variable
(Mon[*])

**Extracted table (page 56, #1):**

| = | Calculation Data | Operator |
| --- | --- | --- |
|  | User variable (User[0…63]) | Addition (+) Subtraction (-) Multiplication (*) Division (/) Obtain remainder (%) And (&) Or (|) |
|  | Constant (Constant) |  |
|  | Data array (Arr[0…799]) |  |
|  | Parameter (PX.XXX) |  |
|  | Monitoring variable (Mon[*]) |  |

<!-- page 57 -->

## Statements Tab Section

This section includes existing statements and memory capacity. Statements
save the data from the expression and procedure sections. Data in the
expression and procedure sections of the same statement always remain
identical and can be shared by multiple PR commands. If data in those two
sections are different, then the data is saved to another statement. The time
required to execute the statement is shown in the Spend Time field. Total
Capacity shows the servo drive memory capacity. Basic operations cannot be
performed if there is no memory space available.
The Statements tab is shown in Figure 149. The upper section displays all the
statements and the lower section displays the operations in each statement
and the values.

> **Figure 149** — PR Procedure

Use the PR Mode Editor in
KNX5100C Software
The PR Mode Editor is accessed by KNX5100C software:
Function List>Motion Control>Parameter Editor

> **Figure 150** — PR Mode Editor

<!-- page 58 -->

Speed and Time Settings
You can use KNX5100C software to configure the shared PR parameters for
these Speed and Time Settings:
•
Accel/Decel Time (shown in Figure 151)
•
Delay Time
•
Internal Target Speed
There are 16 accel/decel times and 16 internal (preset) speeds. These values can
be modified by using KNX5100C software from Function List>Parameter
Editor>Motion or from Function List>PR Mode Editor>Speed and Time
Setting. You can modify these parameters by using explicit writes via Class 3
Messages, or by using KNX5100C software.

> **Figure 151** — Speed/Time Setting Tab

<!-- page 59 -->

## General Parameter Settings

You can set the general parameters, such as electronic gear ratio, software
limit, Deceleration time, and Event On/Off. See Configure Electronic Gear (EGear) Ratio on page 169, Configure Limits on page 177, and Event Trigger on
page 356 for more information.

> **Figure 152** — General Parameter Settings

<!-- page 60 -->

## Homing Setting

From the Homing Setting, you can configure the Homing method, speed
settings, and Homing Definition parameters. See Homing on page 298 for
more information on the Homing Mode and Speed Settings. The Homing
Definition settings are shown:
•
Command: This pull-down menu selects what the next operation is when
the Homing is successfully completed.
•
Acceleration/Deceleration/Delay: The pull-down menu selections for
accel/decel/delay times that are used with the Homing Method timing
diagrams are chosen in the Homing Definition section.
•
Home: You can enable the Homing Operation on power-up. When this
option is enabled, the drive power is cycled and the motor is enabled, the
homing sequence begins.

> **Figure 153** — Homing Tab

The homing operation is also PR#00. This is not changeable and is
selectable in PR Mode when choosing a PR to execute.

<!-- page 61 -->

## PR Mode Setting

From the PR Mode Setting, you can configure the PR commands.

> **Figure 154** — PR Mode Tab

The Type pull-down menu lets you select the PR command type and
corresponds to the X value in the PRCmdXSetting parameters, for example
parameter PRCmd1Setting ID399 (P6.002).
X:TYPE, Command Type
1: Constant speed control
2: Point-to-point command
3: Point-to-point command (Proceed to the next command when completed)
7: Jump to the specified PR command
8: Write to parameter or Data Array.
A: Index position control
B: Statement / arithmetic operation

<!-- page 62 -->

Each command type, except for (B) Arithmetic/Statement, lets you configure
the Options and the Speed and Time Setting for the PR command. The options
differ depending on the command type. These settings correspond to the Y, A,
U, A, B, and C values of the PRCmdXSetting parameters, for example
parameter PRCmd1Setting ID399 (P6.002).
See PR Mode Definitions on page 288 for more information.
•
Jump to the specified command (7) - This command type lets you create
looping program sequences where you can jump to specific PR# values.
See Jump Command on page 329 for more information.
•
Write to Parameter or Data Array (8) - This command type lets you write
a specific parameter or data array value. See Write Command on page 331
for more information.

> **Figure 155** — PR Command Settings

<!-- page 63 -->

Display of PR Procedure in
KNX5100C Software
This section describes how the software displays information related to each of
the seven types of PR Mode commands. This display is on the Chart tab of the
PR Mode Editor in KNX5100C software. To help you understand how PR
procedure works, the configuration software presents the execution order and
calling sequence of all PR procedures.
Parts of the PR Display
The PR display includes five parts: number, execution property, command
type, next PR command, and command data.

> **Figure 156** — PR Display

1.
Number: the PR number, ranging from PR#0…PR#99 (100 sets of PR
commands).
2. Execution property: (B) Execute homing when power on; (O) Command
overlap; (R) write data to EEPROM; (I) command interrupt.
3.
Command type: there are six types of PR commands: homing, speed,
position, writing, jumping, and arithmetic operations. The color
displayed in this section depends on the command type.
4. Next PR command: if followed by a PR command, the arrow points to the
specified PR command.
5.
Command information: displays the details of this PR command. The
color depends on the information types.

PR#1 (I)(O)
 Position
DLY = [0] 0 ms
100000 PUU
INC
200 rpm

<!-- page 64 -->

## Homing PR Display

In the display of homing methods, PR#0 always signifies the homing
procedure, which is identified or indicated as ‘Homing’. See Figure 157.

> **Figure 157** — Homing Methods Display

1.
Command execution type: to execute homing when the drive is in Servo
On state, it displays (B); if homing is not required, then no information is
displayed.
2. Method selection: homing methods and Z pulse setting are shown in the
table below. Characters with red text (see Figure 157) indicate which
'method' is used to set the Home Position. For example, if a Z pulse is
used, the Z is used to set the Home Position, so the Z is in red text. If
there is no marker pulse used and the ORG transition is used to set the
Home Position then ORG is in red text.
• F signifies running forward
• R signifies running in reverse
• ORG signifies using the Home Origin DI
• CUR signifies using the present position as home
• BUMP represents the collision point (Home to Torque)
3.
Offset: origin offset, ID398 (P6.001)
4. Command: next PR command to be executed after homing
5.
Homing at high speed: first homing speed, ID298 (P5.005)
HomingSpeed.

PR#0 (B)
Homing
0:PLZ
Offset=0
PR#1
Speed1=100
Speed2=20

Homing methods
Displayed text
(using a marker pulse, Z)
Displayed text, not using a
marker pulse (Z)
X = 0: homing in forward direction with PL as the homing origin
0: PLZ
0: PL
X = 1: homing in reverse direction with NL as the homing origin
1: NLZ
1: NL
X = 2: homing in forward direction with ORG (when it switches from off to on state) as the homing
origin
2: F_ORGZ
2: F_ORG
X = 3: homing in reverse direction with ORG (when it switches from off to on state) as the homing
origin
3: R_ORGZ
3: R_ORG
X = 4: look for the Z pulse in forward direction with it as the homing origin
4: F_Z
X = 5: look for the Z pulse in reverse direction with it as the homing origin
5: R_Z
X = 6: homing in forward direction with ORG (when it switches from on to off state) as the homing
origin
6: F_ORGZ
6: F_ORG
X = 7: homing in reverse direction with ORG (when it switches from on to off state) as the homing
origin
7: R_ORGZ
7: R_ORG
X = 8: use the current point as the origin
8: CUR
X = 9: look for collision point in forward direction and use it as the origin
9: F_BUMPZ
9: F_BUMP
X = A: look for collision point in reverse direction and use it as the origin
A: R_BUMPZ
A: R_BUMP

<!-- page 65 -->

6. Homing at low speed: second homing speed, ID299 (P5.006)
HomingCreepSpeed.
Speed Command PR Display
You can use the Speed command in any PR command (PR#1…PR#99). It is
identified or indicated as "Speed". See Figure 158.

> **Figure 158** — Speed Command Display

1.
Command execution type: a Speed command can interrupt (INS) the
previous PR command. If the Interrupt function is enabled, it displays
(I); if not, no information is displayed.
2. Delay time (DLY): determined by shared PR parameters. It is defined by a
command from the controller; the servo drive starts counting the delay
time once it reaches the target speed.
3.
Target speed: the set target speed.
4. Acceleration time (ACC): determined by shared PR parameters; length of
time to reach the 3000 rpm speed from stopped.
5.
Deceleration time (DEC): determined by shared PR parameters; length of
time to decelerate from 3000 rpm speed to stopped.

PR#1 (I)
Speed
DLY=[0] 0 ms
100 rpm
Acc=[0] 6.67 ms
Dec=[0] 6.67 ms

<!-- page 66 -->

## Position Command PR Display

You can use the Position command in any PR command (PR#1…PR#99). It is
marked as ‘Position’, and includes the options to ‘Stop once position control
completed’ and ‘Load the next command once position control completed’. The
only difference is that ‘Load the next command once position control
completed’ shows an arrow pointing to the next PR. See Figure 159 and the PR
command to the right.

> **Figure 159** — Position Command Display

1.
Command execution type: a Position command can interrupt (INS) the
previous PR command. If the Interrupt function is enabled, it displays
(I); if not, no information is displayed. The Position command can
overlap (OVLP) the next PR command. If delay time is set to 0 when this
function is enabled, it displays (O). If the Overlap function is not used, no
information is displayed.
2. Delay time (DLY): determined by shared PR parameters. It is defined by a
command from the controller. The servo drive starts counting the delay
time once it reaches the target position.
3.
Target position: the set target position.
4. Position command type: ‘ABS’ means an absolute positioning command;
‘REL’ means relative positioning; "INC" means incremental positioning;
‘CAP’ means high speed position capture.
5.
Target speed: determined by shared PR parameters.
6. Acceleration time (ACC): determined by shared PR parameters; the
length of time to reach the 3000 rpm speed from stopped.
7.
Deceleration time (DEC): determined by shared PR parameters; the
length of time to decelerate from 3000 rpm speed to stopped.

PR#1 (I)(O)
 Position
DLY=[0] 0 ms
100000 PUU
ABS
200rpm
Acc=[0] 6.67 ms
Dec=[0] 6.67 ms

PR#1 (I)(O)
 Position
DLY=[0] 0 ms
100000 PUU
ABS
200rpm
Acc=[0] 6.67 ms
Dec=[0] 6.67 ms

<!-- page 67 -->

## Jump Command PR Display

You can use the Jump command in any PR command (PR#1…PR#99). It is
identified or indicated as ‘Jump’ and followed by an arrow pointing to the next
PR command. See Figure 160.

> **Figure 160** — Jump Command Display

1.
Command execution type: the Jump command can interrupt (INS) the
previous PR command. If the Interrupt function is enabled, it displays
(I); if not, no information is displayed.
2. Delay time (DLY): determined by shared PR parameters.
3.
Target PR number: the target PR number.
Write Command PR Display
You can use the Write command in any PR command (PR#1 - PR#99). It is
identified or indicated as ‘Write’. See Figure 161.

> **Figure 161** — Write Command Display

1.
Command execution type: a write command can interrupt (INS) the
previous PR command. If the Interrupt function is enabled, it displays
(I); if not, no information is displayed. You can determine whether to
write the data to EEPROM. If writing data to EEPROM is required, it
shows (R); if not, no information is displayed.
2. Delay time (DLY): determined by shared PR parameters.
3.
Writing target and data source: the corresponding target and data
sources are shown in the table below. Note that constants can be written
in DEC or HEX format.
Writing Target
Data Source
Parameter (PX.XXX)
Constant
Data array (Arr[#])
Parameter (PX.XXX)
-
Data array (Arr[#])
-
Monitoring variable (Mon[#])

PR#1 (I)
Jump
DLY=[0] 0 ms
PR#2

PR#1 (I)(R)
Write
DLY=[0] 0 ms
P1.001=1

<!-- page 68 -->

## Index Position Command PR Display

You can use the Indexing Position command in any PR command
(PR#1…PR#99). The number of PR commands is determined by the index
number. It is identified or indicated as "Index Position". See Figure 162.

> **Figure 162** — Indexing Position Command Display

1.
Indexing Position command section: the number of the index position. It
shows the total moving distance at the top using double arrows to show
that the motor can run reciprocally between each target position in each
PR command.
2. Command execution type: a position command can interrupt (INS) the
previous PR command. If the Interrupt function is enabled, it displays
(I). If not, no information is displayed. The Position command can
overlap (OVLP) the next PR command. If delay time is set to 0 when this
function is enabled, it displays (O). If the Overlap function is not used, no
information is displayed.
3.
Delay time (DLY): determined by shared PR parameters. It is defined by a
command from the controller. The servo drive starts counting the delay
time once it reaches the target position.
4. Position command: the numerator is the position of this PR command;
the denominator is the total moving distance of this indexing Position
command, which is set by ID234 (P2.052) IndexingCoordinatesScale.
5.
Rotation direction (Dir): available options are Forward (always move
forward), Reverse (always move in reverse direction), and the shortest
distance.
6. Target speed: determined by shared PR parameters.
7.
Acceleration time (ACC): determined by shared PR parameters; the
length of time to reach the 3000 rpm speed from stopped. See Shared PR
Parameters on page 293.
8. Deceleration time (DEC): determined by shared PR parameters; the
length of time to decelerate from 3000 rpm speed to stopped. See Shared
PR Parameters on page 293.
PR#2 (I)(O)
Index Position
DLY=[0] 0 ms
33333/100000 PUU
Dir=[0] Forward
Speed=200 rpm
Acc=[0] 6.67 ms
Dec=[0] 6.67 ms
Index Position List [100000]
PR#1 (I)(O)
Index Position
DLY=[0] 0 ms
0/100000 PUU
Dir=[0] Forward
Speed=200 rpm
Acc=[0] 6.67 ms
Dec=[0] 6.67 ms
PR#3 (I)(O)
Index Position
DLY=[0] 0 ms
66667/100000 PUU
Dir=[0] Forward
Speed=200 rpm
Acc=[0] 6.67 ms
Dec=[0] 6.67 ms

<!-- page 69 -->

## Arithmetic Operation PR Display

You can use arithmetic operations and statements in any PR command
(PR#1…PR#99). It is identified or indicated as ‘Statement’. When the condition
is fulfilled, an arrow pointing to the next PR command appears with a solid
line. If the condition is unfulfilled, an arrow pointing to the next PR appears
with a dotted line. You can choose to execute the next PR command and stop
once the execution is completed. See Figure 163.

> **Figure 163** — Arithmetic Operation Display

1.
Command execution type: an arithmetic operation command can
interrupt (INS) the previous PR command. If the Interrupt function is
enabled, it displays (I); if not, no information is displayed.
2. Statement number: displays the statement number used in the PR
command.
3.
Execution time (Exe.Time): the time required to execute the arithmetic
operation.
Trigger Methods for PR
Commands
This section describes the four triggering methods for PR commands.
Digital Input (DI) Trigger
When you use PR Operation Mode, you can choose the PR command to be
executed by using Digital Inputs with a binary weighted equivalent values
(Register Position command Selection - Bit 0…Bit 6). Use DI Command
Triggered [0x08] to initiate the selected PR command.
See Description of Digital Input Functions on page 433 for more information.
This can also be set in the Digital I/O and Jog Function dialog box in
KNX5100C software, as shown in Figure 164.
PR#1 (I)
Statement
S0
Exe.Time=3.89μs

True
False
PR#2

PR#1 (I)
Statement
S0
Exe.Time=3.89μs

True
False

PR#1 (I)
Statement
S0
Exe.Time=3.89μs

<!-- page 70 -->

> **Figure 164** — I/O Setting Screen

Once the DI combination is set, toggle DI - Command Triggered to execute the
PR selection (shown in Figure 164).
In addition, there are two sets of DI for special functions, DI. Enable homing
[0x27] (shown in Figure 164) return to homing origin and DI. Stop [0x46]
(shown in Figure 164), used to stop the motor. When DI Enable Homing is
triggered, the homing operation executes. When Stop is executed, any
currently executing PR and motor movement is stopped.
See Digital I/O and Jog Function in KNX5100C Software on page 184 for more
information on configuring the Digital I/O.

> **Table 116** — Use DI to Generate the Binary Weighted PR Command to be Triggered

Position
Command
DI
DI
DI
DI
DI
DI
DI
Parameter

Homing

HomeSetting ID397 (P6.000
HomePosition ID398 (P6.001)
PR#1

PRCmd1Setting ID399 (P6.002)
PRCmdData ID400 (P6.003)
…
PR#49

PRCmd49Setting ID495 (P6.098)
PRCmd49Data ID496 (P6.099)
PR#50

PRCmd50Setting ID497 (P7.000)
PRCmd50Data ID498 (P7.001)
…
PR#99

PRCmd99Setting ID595 (P7.098)
PRCmd99Data ID596 (P7.099)

**Extracted table (page 70, #1):**

| DI | DI | DI | DI | DI | DI | DI |
| --- | --- | --- | --- | --- | --- | --- |
| 6 | 5 | 4 | 3 | 2 | 1 | 0 |
| 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 0 | 0 | 0 | 0 | 0 | 0 | 1 |
| 0 | 1 | 1 | 0 | 0 | 0 | 1 |
| 0 | 1 | 1 | 0 | 0 | 1 | 0 |
| 1 | 1 | 0 | 0 | 0 | 1 | 1 |

<!-- page 71 -->

## Event Trigger

There are four event trigger commands that use Digital Inputs (DI.Event
Trigger Command 1…4) that can be set to execute a PR#. Valid PR numbers are
from PR#51…PR#63. The association for the PR to execute is configured in
KNX5100C software (Function List>Motion Control>PR Mode Editor>General
Parameter Setting). The edge transition of DI.Event Trigger Command
executes the associated PR#. This method is a way of using the 'interrupt'
condition in the Positioning mode.

> **Figure 165** — I/O Setting Screen

Notice the Events can be triggered with the ON transition or the OFF
transition. Different PR#s can be triggered for different conditions.

> **Figure 166** — Event Setting for Event Trigger

<!-- page 72 -->

## Use PR Command Trigger ID300 (P5.007)

This method is executed when an Ethernet/IP explicit write operations are
performed with the drive, and you use PRCmdTrigger parameter ID300
(P5.007), you can specify which PR# is executed when you set the Command
Triggered DI signal.
•
If you write 0 to the PRCmdTrigger register, the servo drive executes
homing.
•
If you write 1…99 to PRCmdTrigger register, the servo drive executes the
specified PR command (1…99).
•
If you write 1000, the servo drive stops executing PR commands and
stops motor movement, which is the same as using the DI.Stop.
•
Values 100…999 are not valid.
Using ID300 (P5.007) is useful to monitor the status of the PR# operation
within the drive from an external controller.
Use IO Mode and Add-On Instruction
When the drive is configured for IO Operating mode, a pre-entered PR# can be
executed by using raC_xxx_K5100_MAI (Motion Axis Index) Add-OnInstruction.
Because the Motion Axis Index Add-On Instruction uses a previously
configured PR, you cannot have an active Ethernet/IP network connection
when you are configuring the PR commands. The simplest way to configure
your PR commands is to use the KNX5100C software.
Once the PR Commands are configured, they cannot be changed when the I/O
connection is established. You can use this method when one of the predefined Motion Operation Add-On-Instructions cannot meet your
requirements. For example, you can use this method to trigger a PR that writes
a parameter to the drive.
Explicit Write Data Value
PRCmdTrigger ID300 (P5.007)
Action Taken by the Drive

Servo executes the configured Homing Mode
1…99
Executes the specified PR configured in the drive
1000
The drive terminates the executing PR command and
stops the motor movement
100…999
Invalid
Explicit Read Execution Point
Drive/Motor Execution
Returned Value in ID300 (P5.007)
During the beginning of the
command - before motion starts
Drive is processing the command,
motor has not started movement
PR# that is specified in the drive.
While the command is complete and
the motor is moving but not reached
its target position
Motor is moving, but has not reached
the target position
PR# that is specified + 10,000
While the command is complete and
the motor has reached its target
position
Motor is finished moving and
reached the target position
PR# that is specified + 20,000

<!-- page 73 -->

## PR Execution Process

The drive updates the command status every 1 ms. Figure 167 illustrates the PR
procedure execution flow and how the servo drive deals with PR commands.
Once a PR procedure is triggered, it goes through three internal processing
units, which are PR queue, PR executor, and motion command generator.

> **Figure 167** — PR Arrangement Procedure of the Drive

## Trigger Mechanism Priority

The priority for the triggering methods is the following:
•
(1) DI trigger (DI.Command Triggered)
•
(2) Explicit Write, PRCmdTrigger parameter ID300 (P5.007)
•
(3) DI.Event Trigger 1…4
•
(4) Explicit Write, EventRisingEdgePRNumber parameter ID386
(P5.098), EventFallingEdgePRNumber parameter ID387 (P5.099)
 A PR is executed as long as a trigger signal is received. When two different
trigger methods are used for one PR procedure within the same ms,
commands with higher priority are executed first. If multiple trigger
commands are generated at the same time (within 1 ms), the last command is
not sent to the PR queue.

High
Low
DI.CTRG
P5.007
Event(↑)
Event(↓)
PR queue
• Every 1 ms, issues the waiting lead PR to PR executor no matter whether the
executor has completed the commands or not.
• New lead PR replaces the PR in the executor.
PR executor
• Sends the motion commands to the generator (Speed Cmd, Position Cmd).
• Completes jump, write, and arithmetic operations commands.
• Within 1 ms, it completes at least 20 consecutive commands with interruption functions
and without delay (If arithmetic operations which cannot be interrupted are
included, duration is determined by the operation time and will be the last
command in 1 ms).
Motion command generator
• Integrates multiple commands (Sequence, interrupt, or overlap commands).
• Output the integrated motion commands.
Time
Speed
Trigger Mechanism Priority
ID300 (P5.007) = PRCmdTrigger
Trigger Mechanism Priority
High
Low
•
Every 1 ms, issues the waiting lead PR to PR executor regardless of if the executor has completed
the commands or not.
•
New lead PR replaces the PR in the executor.
•
Sends the motion commands to the generator (Speed Cmd, Position Cmd).
•
Completes jump, write, and arithmetic operations commands.
•
Within 1 ms, it completes at least 20 consecutive commands with interruption functions and without
delay
(if arithmetic operations that cannot be interrupted are included, duration is determined by the
operation time and is the last command in 1 ms).
•
Integrates multiple commands (Sequence, interrupt, or overlap commands).
•
Output the integrated motion commands.
Speed
Time
PR queue
PR executor
Motion command generator

<!-- page 74 -->

## PR Queue

The triggered PR command is the lead PR. The PR and its container Group are
organized and prepared to be sent to the PR Executor. In each ms, regardless
of another PR being queued, the servo drive sends the lead PR and its PR group
to the PR executor. Therefore, as long as a PR command is triggered, the PR
queue collects the command and sends the command to the executor.
PR Executor
Once the PR executor receives the lead PR, the PR group in execution is
replaced immediately. If a PR group includes motion commands, such as
speed commands and position commands, the PR executor sends them to the
Motion Command Generator. If the lead PR contains write or jump
commands, they are completed immediately when the PR executor processes
the lead PR. These commands do not enter the Motion Command Generator.
Any arithmetic based operations (Statement commands) are executed
immediately when entering PR executor. Depending on the expression, the
execution times vary and these commands cannot be interrupted until they are
completed.
The PR executor can consecutively complete a minimum of 20 PR commands
with interrupt commands (INS) and without a delay time setting within 1 ms.
If there is a PR command that hasn't been completed within 1 ms and a new PR
group has been sent to the executor by the queue, the new PR group then
replaces the previous one.
Motion Command Generator
Motion commands include speed and position types. The PR executor sends
this command type to the Motion Command Generator. This generator has a
buffer for creating the motion profile. This generator includes the capability to
modify the existing cycle profile (overlap, interrupt, and Proceed to next).
Motion commands can be executed as soon as they enter the generator. If
other motion commands (that are merge capable) enter the generator, it is
integrated with the existing command in the generator. This integration is
based on the PR settings.
Sequence Command Execution
Commands that you can configure are position and speed commands. A
sequence command is a motion command without an overlap or interrupt
function. When you use position commands, the Delay Time begins timing
when the target position is reached. When you use speed commands, the Delay
Time begins timing after the target speed is reached.

<!-- page 75 -->

## Consecutive Position Commands

When the PR executor receives two position commands consecutively, and
they are not set with interrupt or overlap functions, the PR executor sends this
lead PR to the motion command generator. The profile generator creates the
cycle profile from this lead PR command. After the lead PR completes, if no
delay time is set, the PR executor sends the second PR (which now becomes the
lead PR) to the motion generator and the cycle profile is made for the second
position command.
If the first position command uses DLY (Delay Time), the PR executor starts
the DLY timing when the motor reaches the target position. When the DLY
expires, the second position command is executed as described earlier and
shown in Figure 168.

> **Figure 168** — Position Command

PR#1
 Position
DLY=[0] 0 ms
10000 PUU
ABS
200 rpm
PR#2
 Position
DLY=[0] 0 ms
10000 PUU
ABS
500 rpm
PR
executor
(1 ms command cycle)
Motion
command
generator
Time
Speed
(1 ms command cycle)

PR#1
 Position
DLY=[1] 100 ms
10000 PUU
ABS
200 rpm
PR#2
 Position
DLY=[0] 0 ms
10000 PUU
ABS
500 rpm
PR
executor
Motion
command
generator
7LPH
6SHHG
100ms
(1 ms command cycle)
(1 ms command cycle)
Command with Delay
Command without Delay
PR
executor
Command without Delay
PR
executor
Motion
command
generator
Motion
command
generator
Command with Delay

<!-- page 76 -->

## Consecutive Speed Commands

When the PR executor receives two speed commands consecutively, and they
are not set with interrupt or overlap functions, the PR executor sends this lead
PR command to the motion command generator. The generator creates the
cycle profile for this speed command. When this lead PR command completes
(either by delay or interrupt), the second PR speed command is sent to the
motion command generator and the cycle profile is created using the second
command (which now becomes the lead PR).
If the first speed command is used with a DLY (Delay Time), the DLY begins
timing once the motor reaches the target speed. When the DLY expires, the
second speed command is executed as described earlier and shown in
Figure 169.

> **Figure 169** — Speed Command

Command with Delay
Command without Delay
PR
executor
Motion
command
generator
Time
Speed (rpm)
PR#1
Speed
DLY=[0] 0 ms
100 rpm
Acc=[11] 100 ms
Dec=[11] 100 ms
PR#2
Speed
DLY=[0] 0 ms
500 rpm
Acc=[0] 33.3 ms
Dec=[0] 33.3 ms

(1 ms command cycle)
(1 ms command cycle)

PR
executor
Motion
command
generator
Time
PR#1
Speed
DLY=[1] 100 ms
100 rpm
Acc=[11] 100 ms
Dec=[11] 100 ms
PR#2
Speed
DLY=[0] 0 ms
500 rpm
Acc=[0] 33.3 ms
Dec=[0] 33.3 ms
100ms
Speed (rpm)

(1 ms command cycle)
(1 ms command cycle)
Motion
command
generator
PR
executor
PR
executor
Motion
command
generator
Command with Delay
Command without Delay

<!-- page 77 -->

## Multiple Commands

This section shows how multiple commands are processed by the drive, as
shown in Figure 170.
In the first ms, after a command is triggered, the PR queue sends a position
command to the PR executor. The PR executor receives a position command
and sends this command to the motion command generator, then the cycle
profile is generated.
In the second ms, the PR executor receives a write command and executes it
immediately.
In the third ms, the PR executor receives a jump command and executes it
right away. These two commands are not sent to the motion command
generator; the PR executor and the motion command generator can execute
commands independently.
In the fourth ms, the PR executor receives a position command.

> **Figure 170** — Sequence Command - Multiple Commands

PR#1
 Position
DLY=[0] 0 ms
5000 PUU
ABS
200 rpm
PR#5
 Position
DLY=[0] 0 ms
10000 PUU
ABS
500 rpm
PR
executor
Motion
command
generator
Time
Speed
PR#2
Write
DLY=[0] 0 ms
P5.045=100
PR#3
Jump
DLY=[0] 0 ms
PR#5
(1 ms command cycle)
(1 ms command
cycle)
(1 ms command
cycle)
(1 ms command cycle)
Motion
command
generator
PR
executor

<!-- page 78 -->

## Command Interrupts Execution

Interruption (INS) is an action that results in a change with the motion
command (and the cycle profile). The current motion command is interrupted
with the second motion command. Results of the interruption differ based on
the command types. There are two types of interruption: internal and external,
as shown in Figure 171.

> **Figure 171** — Internal and External Interruption

## Internal Interrupts

With a typical sequence of PR commands that use Auto (Auto execute the next
command), the system processes the next command when the current
command is completed.
However, if the next command is of the type shown for an internal interrupt
(shown in Figure 172), the drive processes this command immediately. For
example, a Point to point PR is considered an internal interrupt. If Interrupt
Previous PR is selected with this PR command, the drive immediately changes
the executing cycle profile to reflect this new PR command
Position Commands
See Figure 172 to use these two examples.
•
Example 1: When the PR executor receives three consecutive position
commands with the second command using the interrupt setting
(PR#01 -> PR#02 (I)->PR#03), the PR executor treats the PR#01 and
PR#02 as one command. Because both the PR commands occurred in the
same 1 ms cycle, and there is no DLY used, the PR executor replaces
PR#01 with PR#02. It sends the second PR to the motion command
generator for execution. When PR#02 is complete, the PR executor sends
PR#03 to the generator.
•
Example 2: Using the same three consecutive position commands, if
PR#01 uses a 100 ms DLY (Time Delay), the PR executor processes PR#01
and PR#02, because the DLY is used with PR#01, the PR#01 movement
executes while DLY is timing. When the delay has expired, and PR#01 has
not reached its target position, the PR executor then processes PR#02
and sends it to the motion command generator and the command profile
executes PR#02 (still a point-to-point index). Once PR#02 is completed,
PR#03 is sent to the motion command generator and PR#03 is executed.
ID 300 (P5.007) PRCmdTrigger
Special Trigger (CAP/CMP/E-CAM)
External Interruption
Software Trigger
Event Trigger
DI Trigger
Homing
Position command
Speed command
Jump command
Write-in command
Internal interruption

<!-- page 79 -->

> **Figure 172** — Internal Interruption - Position Command

The REL and INC position command types operate the same way. The target
position is the previous target position (30,000 in Figure 173) plus the new
Command Position (60,000 in Figure 173).

> **Figure 173** — Example of Relative and Incremental Position Command for Internal Interruption

PR#1
 Position
DLY=[0] 0 ms
1000 PUU
ABS
100rpm
PR#3
 Position
DLY=[0] 0 ms
10000 PUU
ABS
500rpm
PR
executor
( 1 ms Cmd cycle )
Motion
command
generator
Time
Speed
PR#2 (I)
 Position
DLY=[0] 0 ms
5000 PUU
ABS
200rpm
(1 ms Cmd cycle)
PR#1
 Position
DLY=[1] 100 ms
1000 PUU
ABS
100 rpm
PR#3
 Position
DLY=[0] 0 ms
10000 PUU
ABS
500 rpm
PR
executor
Motion
command
generator
Time
Speed
PR#2 (I)
 Position
DLY=[0] 0 ms
5000 PUU
ABS
200 rpm
100 ms
(1 ms Cmd cycle)
(1 ms Cmd cycle)
(1 ms Cmd cycle)
Command with Delay
Command without Delay
Motion
command
generator
Command with Delay
Command without Delay
PR
executor
Motion
command
generator
PR
executor
Internal interruption
REL (I)
60000 PUU

10000 20000 30000 40000 50000 60000 70000 80000 90000
100000
Motorćs
current position
(Fb_PUU)
Target
position
Endpoint of
previous command
(Cmd_E)
60000
Internal interruption
INC (I)
60000 PUU

10000 20000 30000 40000 50000 60000 70000 80000 90000
100000
Motorćs
current position
(Fb_PUU)
Target
position
Endpoint of
previous command
(Cmd_E)
60000
Internal interruption
REL (I)
60000 PUU
Internal interruption
INC (I)
60000 PUU
Endpoint of
previous command
(Cmd_E)
Endpoint of
previous command
(Cmd_E)
Target
position
Motor
current position
(Fb_PUU)
Motor
current position
(Fb_PUU)
Target
position

<!-- page 80 -->

## Speed Commands

See Figure 174 to use these two examples.
•
Example 1: When the PR executor receives three consecutive speed
commands with the second command using the interrupt setting
(PR#01 -> PR#02 (I)->PR#03), the PR executor treats the PR#01 and
PR#02 as one command. Because both the PR commands occurred in the
same 1 ms cycle, and there is no DLY used, the PR executor replaces
PR#01 with PR#02. It sends the second PR to the motion command
generator for execution. When PR#02 is complete, the PR executor sends
PR#03 to the generator.
•
Example 2: Using the same three consecutive speed commands, if PR#01
uses a 100 ms DLY (Time Delay), the PR executor processes PR#01 and
PR#02, because the DLY is used with PR#01, the PR#01 target speed
executes while DLY is timing. When the delay has expired, the PR
executor then processes PR#02 and sends it to the motion command
generator and the command profile executes PR#02 (still a constant
speed type). Once PR#02 is completed (reaches target speed), PR#03 is
sent to the motion command generator and PR#03 is executed.

<!-- page 81 -->

> **Figure 174** — Internal Interruption - Speed Command

## Multiple Commands

The PR queue updates once every 1 ms Command cycle. If all PR commands are
set with interrupt function, the queue can read at least 20 PR commands in
1 ms.
If these multiple PR commands contain multiple motion commands, the PR
queue only issues the last command it receives to the motion command
generator for execution. Therefore, in the same PR group, only one PR
command with motion command is executed. This sequence is different for
non-motion PR types where jump and write commands are executed once
received by the PR queue (see Figure 175).
Command with Delay
Command without Delay

PR
executor
Motion
command
generator
Time
Speed (rpm)
PR#2 (I)
Speed
DLY=[0] 0 ms
200 rpm
Acc=[12] 333 ms
Dec=[12] 333 ms
PR#3
Speed
DLY=[0] 0 ms
500 rpm
Acc=[0] 33.3 ms
Dec=[0] 33.3 ms
PR#1
Speed
DLY=[0] 0 ms
100 rpm
Acc=[11] 100 ms
Dec=[11] 100 ms
( 1 ms command cycle )
(1 ms command cycle)

PR
executor
Motion
command
generator
Time
 Speed (rpm)
PR#2 (I)
Speed
DLY=[0] 0 ms
200 rpm
Acc=[12] 333 ms
Dec=[12] 333 ms
PR#3
Speed
DLY=[0] 0 ms
500 rpm
Acc=[0] 33.3 ms
Dec=[0] 33.3 ms
PR#1
Speed
DLY=[1] 100 ms
100 rpm
Acc=[11] 100 ms
Dec=[11] 100 ms
100 ms
(1 ms Cmd cycle)
(1 ms Cmd cycle)
(1 ms Cmd cycle)

Motion
command
generator
Command with Delay
Command without Delay
PR
executor
PR
executor
Motion
command
generator

<!-- page 82 -->

If one of the PR types uses a delay, the PR queue schedules all subsequent
commands on the basis of the PR type that uses a DLY (Delay Time).
•
Example 1 shows multiple commands that are received in the same 1 ms
Command period. You can see that the last PR (PR#07) is the command
that is executed, as shown in Figure 175.

> **Figure 175** — Internal Interruption - Multiple Commands without Delay

•
Example 2 shows multiple commands that are received in the same 1 ms
update with DLY (Delay Time) used. In this case, the first motion
commanded PR begins executing while the DLY is occurring; the DLY is
grouped with the first 1 ms cycle. Once DLY expires, the next PR (PR#07)
begins executing, which occurs in the second 1 ms cycle, as shown in
Figure 176.

> **Figure 176** — Internal Interruption - Multiple Commands with Delay

PR#1 (I)
 Position
DLY=[0] 0 ms
1000 PUU
ABS
200 rpm
PR#7 (I)
 Position
DLY=[0] 0 ms
10000 PUU
ABS
500 rpm
PR
executor
Motion
command
generator
Time
Speed
PR#2 (I)
Write
DLY=[0] 0 ms
P5.045=100
PR#8 (I)
Jump
DLY=[0] 0 ms
PR#10
Approx. 20 PR Commands
(1 ms Cmd cycle)
Multiple Commands without Delay
Motion
command
generator
Multiple Command without Delay
PR
executor
PR#1 (I)
 Position
DLY=[0] 0 ms
1000 PUU
ABS
200 rpm
PR#7 (I)
 Position
DLY=[0] 0 ms
10000 PUU
ABS
500 rpm
PR
executor
Motion
command
generator
Time
Speed
PR#6 (I)
Write
DLY=[1] 100 ms
P5.045=100
PR#8 (I)
Jump
DLY=[0] 0 ms
PR#10
Approx. 20 PR Commands
DLY 100 ms
(1 ms Cmd cycle)
(1 ms Cmd cycle)
Multiple Commands with Delay
Motion
command
generator
Multiple Command without Delay
PR
executor

<!-- page 83 -->

## External Interrupts

When an external interrupt is used, it uses an external interrupt trigger
method to execute another PR command, see Figure 171.
When the PR queue receives a PR position command with the interrupt setting
(Interrupt Previous PR=1), it is sent to the motion command generator
immediately and any required motion command changes integrate with any
currently executing motion. The use of DLY does not change the result of an
external interruption. The external interruption of Speed or Position
command types operate the same way. Therefore, you can integrate a Constant
Speed PR type with a Positioning type and vice versa.

> **Figure 177** — External Interruption

PR#1
 Position
DLY=[0] 0 ms
1000 PUU
ABS
100 rpm
PR#11
 Position
DLY=[0] 0 ms
10000 PUU
ABS
500 rpm
PR
executor
Motion
command
generator
Time
Speed
PR#10 (I)
 Position
DLY=[0] 0 ms
5000 PUU
ABS
200 rpm
PR#2
 Position
DLY=[0] 0 ms
10000 PUU
ABS
200 rpm
External
interruption
(1 ms command cycle)
(1 ms command cycle)
(1 ms command cycle)
(1 ms command cycle)
PR executor
Motion
command
generator
Time
Speed (rpm)
PR#10 (I)
Speed
DLY=[0] 0 ms
200 rpm
Acc=[12] 333 ms
Dec=[12] 333 ms
PR#11
Speed
DLY=[0] 0 ms
500 rpm
Acc=[0] 33.3 ms
Dec=[0] 33.3 ms
PR#1
Speed
DLY=[1] 100 ms
100 rpm
Acc=[11] 100 ms
Dec=[11] 100 ms
PR#2
Speed
DLY=[0] 0 ms
500 rpm
Acc=[11] 33.3 ms
Dec=[11] 33.3 ms
External
interruption
(1 ms command cycle)
(1 ms command cycle)
(1 ms command cycle)
(1 ms command cycle)

(A) - External Interruption - Position Command
(B) - External Interruption - Speed Command
Motion
command
generator
(A) - External Interruption - Position Command
PR
executor
PR
executor
Motion
command
generator
(B) - External Interruption - Speed Command

**Extracted table (page 83, #1):**

|  | PR#1 |  | 0 (I) |  |
| --- | --- | --- | --- | --- |
|  | Posi |  | tion |  |
|  | DLY=[0 5000 AB 200 r |  | ] 0 ms PUU S pm |  |

**Extracted table (page 83, #2):**

|  |  | PR# |  | 10 (I) |  |
| --- | --- | --- | --- | --- | --- |
|  |  | Sp |  | eed |  |
|  |  | DLY=[ 200 Acc=[12 Dec=[12 |  | 0] 0 ms rpm ] 333 ms ] 333 ms |  |

<!-- page 84 -->

## Overlap Command Execution

The Overlap function is available only with Position Type commands. When
the Overlap function (Overlap next PR =1) is used with a primary PR, the PR
queue (and PR executor) looks for the secondary PR position type so it can
merge the two Position commands while the primary PR command is
decelerating. The calculation for the merging is shown
An Interrupt command has a higher priority over an Overlap command. Thus,
when Overlap function is set in the current position command and the next
motion command is set to apply the Interrupt function, only Interrupt
function is conducted.

> **Figure 178** — Overlap Command

1st target speed (Spd1)
3000
----------------------------------------------------------
Deceleration time (Dec)

2nd target speed (Spd2)
3000
-------------------------------------------------------------
Acceleration time (Acc)

=
IMPORTANT
Do not use the DLY function when you use the Overlap function.
Because you can still select the DLY (Delay Time) when Overlap is used,
this delay occurs at the end of the primary PR motion command and
delays the second PR profile from starting.
If the Deceleration of the primary PR is the same as the Acceleration of the
secondary PR, the velocity transition between the two PRs is very smooth.

PR#2
 Position
DLY=[0] 0 ms
10000 PUU
ABS
500 rpm
Acc=[0] 16.65 ms
Acc=[0] 16.65 ms
PR
executor
Motion
command
generator
Time
Speed
PR#1 (O)
 Position
DLY=[0] 0 ms
5000 PUU
ABS
200 rpm
Acc=[0] 6.67 ms
Dec=[0] 6.67 ms
Deceleration time of the 1st command is different from
acceleration time of the 2nd command
(1 ms command cycle)
(1 ms command cycle)

PR#2
 Position
DLY=[0] 0 ms
10000 PUU
ABS
500 rpm
Acc=[2] 6.67 ms
Dec=[2] 6.67 ms
PR
executor
Motion
command
generator
Time
Speed
PR#1 (O)
 Position
DLY=[0] 0 ms
5000 PUU
ABS
200 rpm
Acc=[0] 6.67 ms
Dec=[0] 6.67 ms
Deceleration time of the 1st command is identical to
acceleration time of the 2nd command
(1 ms command cycle)
(1 ms command cycle)
Command with Delay - Acceleration and Deceleration Time are Identical
Overlap Command - Acceleration and Deceleration Time are Different
PR
executor
Motion
command
generator
Overlap Command - Acceleration and Deceleration Time are Different
Command with Delay - Acceleration and Deceleration Time are Identical
Motion
command
generator
PR
executor
Deceleration time of the 1st command is different
from
Deceleration time of the 1st command is identical to
acceleration time of the 2nd command

<!-- page 85 -->

## Arithmetic Operation Command Execution

Arithmetic commands (typically in Statement type PR commands) have the
same execution priority as Jump and Write commands. When executed
consecutively, Arithmetic operations commands can interrupt the currently
executing PR command but cannot be interrupted by a PR command. This
sequence is to confirm that all arithmetic operations are completed before the
PR commands enter the PR queue (and PR executor). If a PR attempts to
interrupt an arithmetic command by highest priority, either interrupt or
PRCmdTrigger input, the interrupt occurs on the next command cycle (after
the current 1 ms cycle completes).

> **Figure 179** — Multiple Commands with Arithmetic Operations

> **Figure 180** — Writing Trigger Command in Statement Section

PR executor
PR#1 (I)
Write
DLY=[0] 0 ms
P5.045=100
PR#6 (I)
Statement
S0
Exe.Time=3.89μs
true
false
PR#10 (I)
Write
DLY=[0] 0 ms
P5.045=200
PR#20 (I)
Write
DLY=[0] 0 ms
P5.045=300
(1 ms command cycle)
(1 ms

command cycle)
(1 ms command cycle)
PR
executor
(1 ms command cycle)

PR executor
PR#1 (I)
Write
DLY=[0] 0 ms
P5.045=100
PR#6 (I)
Statement
S1
(P5.007 = 30)
Exe.Time=1.89μs
PR#30 (I)
Write
DLY=[0] 0 ms
P5.045=400
(1 ms command cycle)
(1 ms command cycle)
PR
executor
(1 ms command cycle)
(1 ms command cycle)
