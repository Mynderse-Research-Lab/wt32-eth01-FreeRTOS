# Chapter 12: Motion Control Applications

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `13_Ch12_Motion_Control_Apps.pdf` (39 pages)_

<!-- page 1 -->

Motion control applications often include high speed dependencies. These
dependencies includes being able to capture and compare motor positions and
execute high-speed functions with synchronization.
High-speed Position
Capture Function (CAP)
The High-speed Position Capture function uses high speed digital inputs (DI9
and DI10) to quickly and accurately capture the motor position, then store it in
a data array for later use.
The following parameters define the function.
There are two capture functions can be used simultaneously and are
configured using KNX5100C software. To use the capture functions, navigate
to Function List>Motion Control>Capture(CAP)/Compare(CMP) and:
•
Configure the Capture Options
Topic
Page
High-speed Position Capture Function (CAP)

High-speed Position Compare Function (CMP)

E-CAM

This manual links to Kinetix® 5100 Servo Drive Fault Codes Reference
Data, publication 2198-RD001, for fault codes and Kinetix 5100 Servo
Drive Parameters Reference Data, publication 2198-RD002, for
parameters. Download the spreadsheets now for offline access.

> **Table 117** — High-speed Capture Related Parameters

Parameter
Name
ID328 (P5.036)
DI9CaptureStartAddress
ID392 (P5.107)
DI10CaptureStartAddress
ID330 (P5.038)
DI9CaptureRemainedCount
ID394 (P5.109)
DI10CaptureRemainedCount
ID131 (P1.019)
DI9ExtraConfiguration
ID183 (P1.103)
DI10ExtraConfiguration
ID331 (P5.039)
DI9CaptureControl
ID395 (P5.110)
DI10CaptureControl
ID329 (P5.037)
DI9CaptureAxisPosition
ID393 (P5.108)
DI10CaptureAxisPosition
ID132 (P1.020)
DI9CaptureMaskRange
ID184 (P1.104)
DI10CaptureMaskRange
ID368 (P5.076)
DI9FirstCaptureResetPosition
ID396 (P5.110)
DI10FirstCaptureResetPosition

<!-- page 2 -->

•
Choose a source for the position
•
Choose a time interval
•
Determine if you need Online operation or cyclic mode

> **Figure 181** — High-speed Capture

The following example of the Capture function uses Figure 182. This example
uses DI9 as the capture input (in KNX5100C software, the first capture uses
DI9, the second capture uses DI10). Initial setup of the drive/High Speed Input
to use the CAP function:
•
Use KNX5100C software, remove any input assignments at DI9/DI10;
when you use the CAP function, no other association should be present
•
Verify there is no Event Trigger that is using PR 50/60; the Event
execution is separate from the CAP function. They cannot use the same
PR command or DI.
•
Use KNX5100C software, at PR50/60 verify that no other PR command is
presently used; the CAP function uses these PR commands to execute the
CAP index.
Choose the options and settings that you need:
•
CAPTURE Start Address (ID328,P5.036)
Captured data is stored in this shared User Data Array (0…799 elements).
This data is sequentially stored based on the number of CAPTURES
performed in that particular cycle. Once the CAPTURE cycle is complete
and if it is triggered again, the old capture data is overwritten with the
present capture data.
•
CAPTURE Axis Position (ID329, P5.037)
The most recent returned position of the capture function, which is
stored for each capture instance in the User Data Array.
•
Enter a positive value in CAPTURE remained count (DI9)
This option essentially enables the capture function, alternatively, you
can see the status of the function from the Function List>Motion
Control>Parameter Editor, Capture Control DI9 (ID331m P5.039 X.0);

<!-- page 3 -->

when this value is larger than zero, Bit 0 is ON. This state indicates that
the capture function is executing.
When the Capture function is executing, the
DI9CaptureRemainedCount (ID330, P5.038) parameter indicates the
remaining number of captures to be performed. Whenever a capture
cycle occurs, this value decrements by 1 until the value is 0. The zero value
indicates that the Capture function has completed.
•
Choose to perform the Compare (CMP) function when the position is
captured.
This option performs a compare function after the capture has occurred.
•
Choose a PR to execute when the capture is complete.
When the capture is completed, use the pull-down menu to choose a
specific PR# to execute.
•
Choose the source of position for your capture function.
Here you can choose the position feedback source that is used for
capturing. You can also choose Capture Disable which is another way to
disable the capture function if your remaining count is still greater than
zero.
•
Choose the trigger logic.
Choose whether the capture executes when a N.O. (Normally Open) or
N.C. (Normally Closed) condition occurs on DI9 (this condition is
thought of as rising edge, falling edge).
•
Choose the time interval for retriggering the capture function.
This time interval avoids nuisance sensor trips typical in high-speed
applications. You can set the capture function only to occur once this
time has elapsed.
•
Use DI9CaptureMaskRange (ID132, P1.020) if necessary.
To help prevent the same position data (or position data within a small
window) from being captured repeatedly, you can configure the drive to
avoid capturing multiple identical data sets by defining a masking range
for data capture. This range defines the pulses that must occur before
any new data is stored.
The logical evaluation considers DI9 to be configured as a N.O. input. When
this configuration is true, the logic behaves as shown. If DI9 is an N.C. input,
the logic levels are reversed.

<!-- page 4 -->

> **Table 118** — High-speed Position Capture, Additional Information

P5.039 (for DI9)
DI9CaptureControl
Bit
Function
Description
X

Enable capture function
When bit 0 = 1, and ID330 (P5.038) > 0, data capturing starts and DO.CAP_OK signal is off. The value of ID330 (P5.038)
decrements as data capture continues.
When ID330 (P5.038) = 0 it means that data capture is complete. When the data capturing is completed, the
DO.CAP_OK signal is on, and bit 0 is cleared to 0.
If bit 0 is already 1, the written value must not be 1; you have to write 0 to disable the Capture function.

Reset the axis position
when first data is
captured
When bit 1 = 1, after the first data is captured, the Capture axis position is set to the value of ID368 (P5.076)
DI9FirstCaptureResetPosition.

## Enable Compare

function after the first
data is captured(1)
When bit 2 = 1, when the first data is captured, the Compare function is enabled. (ID351.X (P5.059.X) CompareControl
bit 0 = 1 and ID350 (P5.058) CompareRemainedCount resets to the previous setting amount). If the Compare function
is already enabled, then this bit function is ignored.

Execute specific PR
after the last data is
captured
When bit 3 = 1, the drive executes the specific PR once data capture is complete (2).
Y
–
Source of capture axis
0: Disabled
1: Auxiliary encoder (Aux)
2: Pulse command (I/O)
3: Main encoder of motor (MFB)
Z
–
Trigger logic
0: N.O. (normally open)
1: N.C. (normally closed)
U
–
Minimum trigger
interval
–
DC
The specific
PR#1…PR#99
–
(1)
Compare function only supported in first capture.
(2)
See Use IO Mode and Add-On Instruction on page 357. PR mode operation is fixed with DI9 using PR#50 and DI10 using PR#60

**Extracted table (page 4, #1):**

| Bit | Function |
| --- | --- |
| 0 | Enable capture function |
| 1 | Reset the axis position when first data is captured |
| 2 | Enable Compare function after the first data is captured(1) |
| 3 | Execute specific PR after the last data is captured |
| – | Source of capture axis |
| – | Trigger logic |
| – | Minimum trigger interval |
|  | The specific PR#1…PR#99 |

<!-- page 5 -->

> **Figure 182** — High-speed Position Capture Flowchart (DI9 example)

> **Figure 183** — Capture Function Screen

Using PR Command Programming with the Capture Function
You can use KNX5100C software to configure the Capture function. However,
if the functions must change while the application is running, you can use
PR Write command to change it.
With PR write command programming, write commands configure the highspeed position capture function, as well as execute the motion commands once
configuration is complete. See Chapter 11 for details on PR command
programming.

## CAP Axis Source

Normally Open
ID331 (P5.039.Z) = 0
DI9
Normally Closed
ID331 (P5.039.Z) = 1
DI9
CAP Switch
ID331 (P5.039.X) Bit 0
CAP Axis
Position
ID329 (P5.037)
7890
CAP Axis Source
Aux Encoder ID331 (P5.039.Y) = 1
Pulse Command ID331 (P5.039.Y) = 2
Main Encoder ID331 (P5.039.Y) = 3
Data Array
Reset the First Position
ID331 (P5.039.X) Bit 1 = 1
First Position = ID368 (P5.076)
Enable CMP after the
first data is captured.
ID331 (P5.039.X) Bit 2 = 1
CAP Amount
ID330 (P5.038)
Start Address
ID328 (P5.036)
CAP Amount
ID330 (P5.038) = ID330 (P5.038) - 1
CAP Completed?
ID330 (P5.038) = 0
No
Yes
Call PR#50
ID331 (P5.039.X) Bit 3 = 1
Cyclic CAP ID131 (P1.019.X) = 1
ID330 (P5.038) = Previous
Setting Value
1234
2345
5678
6789
7890

**Extracted table (page 5, #1):**

| 1234 |  |
| --- | --- |
| 2345 |  |
| 5678 |  |
|  | 6789 |
| 7890 |  |

<!-- page 6 -->

The following example describes how the PR command works and is
illustrated in Figure 184.
1.
PR#1 confirms that the Capture function is disabled, with ID331.X
(P5.039.X) NAME Bit 0 = 0. This confirmation is done in case the previous
Capture did not complete.
2. PR#2 sets the start address of data array ID 328 (P5.036) to #100.
3.
PR#3 sets the CAPTURE remained count ID 330 (P5.038) as 3, which
executes 3 captures before the capturing function is completed.
4. PR#4 sets the First Capture Reset Position (ID368, P5.076) to 0 for the
first capture point.
5.
PR#5 sets a cyclic capture mode and uses a 1 ms delay before executing
the next PR command.
6. PR#6 enables the capture function and resets the first point. Once the
data capture is complete, the drive executes PR#50. It selects the main
encoder of the motor as the Axis Source and applies 'normally closed'
contact as trigger logic with trigger interval of 2 ms.
7.
PR#7 sets the initial speed command at 50 rpm.
8. Once the last capture occurs, PR#50 executes a Point-to-Point move
(proceed to next), with CMD type CAP (type 11) and
Position CMD Data = 50000 PUU
9. PR#51 uses a PR with command type of Constant speed control.

> **Figure 184** — PR Command with Application of High-speed Capture (DI9 example)

Figure 185 shows how the data is captured in the data array when the capture
function is executed. At (1) after DI9 is triggered, the capturing axis is reset to 0
and the data is stored in data array #100. When DI9 is triggered the second (2)
and third (3) times, the position data is written to the data arrays #101 and
#102, respectively. Once the first capture cycle is completed, the DO:CAP_OK
[0x16] signal turns on and then PR#50 (high speed position capture command)
and PR#51 (motion with fixed speed) are executed. The servo drive will execute
PR#1 (I)
Write
DLY=[0] 0 ms
P5.039=0x2030
PR#3 (I)
Write
DLY=[0] 0 ms
P5.038=3
PR#4 (I)
Write
DLY=[0] 0 ms
P5.076=0
PR#2 (I)
Write
DLY=[0] 0 ms
P5.036=100
PR#50 (I)
 Position
DLY=[0] 0 ms
50000 PUU
CAP
100rpm
PR#7 (I)
Speed
DLY=[0] 0 ms
50 rpm
Acc=[0] 3.33 ms
Dec=[0] 3.33 ms
PR#6 (I)
Write
DLY=[0] 0 ms
P5.039=0x203B
PR#5 (I)
Write
DLY=[10] 1 ms
P1.019=0x0001
PR#51
Speed
DLY=[0] 0 ms
50 rpm
Acc=[0] 3.33 ms
Dec=[0] 3.33 ms

<!-- page 7 -->

the next cycle. The DO:CAP_OK signal is off when procedure is completed and
the capturing amount is set to 3.
When DI9 is triggered for the fourth (1) time, the capture axis position is not
reset and the position data of the capturing axis is written to data array #100
again. Therefore, the data that was logged in the previous cycle is overwritten.
At the moment DI9 is triggered for the fifth and sixth times, the position of
capturing axis is written to data arrays #101 and #102, respectively. As soon as
the second capture cycle is finished, DO:CAP_OK (DO:First CAP procedure
completed) CAP turns on and then PR#50 point-to-point command (Proceed
to next) and PR#51 (constant speed control) are executed.
When applying cyclic capture mode (P1.019.X = 1), once the final capture has
completed, and any PR executes, the cycle of recording new values in the data
arrays is repeated. The capture function resumes storing data in the data array
from CAPTURE Start Address (DI9) (ID 328, P5.036). Any data that was logged
in this data array location is overwritten with the new captured value.

> **Figure 185** — High-speed Capture Application Example

CAP axis
position
(PUU)
Time
10000 PUU
DI9: CAP
DO: [0x16]
CAP_OK
10000 PUU
PR path
#50
#50
#51
#51
#1 ~ 6
#7
Data array

45678
45678
45678
#100

12501
12501
12501
50345
50345
#101

26789
26789
26789
56789
#102
(1)
(1)
(2)
(2)
(3)
(3)

**Extracted table (page 7, #1):**

| Data array |  |  |  |  |  |  |  |  |  |  |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| #100 | 0 |  | 0 |  | 0 |  | 45678 |  | 45678 |  | 45678 |
| #101 | 0 |  | 12501 |  | 12501 |  | 12501 |  | 50345 |  | 50345 |
| #102 | 0 |  | 0 |  | 26789 |  | 26789 |  | 26789 |  | 56789 |

<!-- page 8 -->

## High-speed Position

Compare Function (CMP)
The high-speed position compare function compares the captured actual
position of the motion axis with the value saved in data array. When the
compare condition is fulfilled, a high-speed digital signal (DO4)(1) is output
immediately for motion control. As this function is carried out by the
hardware, KNX5100C software compares the data accurately on high speed
motion axes. When the CMP function is enabled, the servo drive employs DO4
to force an output signal, which is not user-defined.
The following parameters define the function.
To configure the Compare function, you need to:
•
Define and enable the operation of the compare function, including
definition of the axis source and trigger logic
•
Define the start position for data compare in the data array
•
Set the amount of data to be compared
•
Enable the cyclic mode
Figure 188 describes the compare function.
(1)
With execution time 5 µs only.

> **Table 119** — High-speed Capture Related Parameters

Parameter
Name
ID348 (P5.056)
CompareStartAddress
ID349 (P5.057)
CompareAxisPosition
ID350 (P5.058)
CompareRemainedCount
ID351 (P5.059)
CompareControl
ID330 (P5.038)
DI9CaptureRemainedCount
ID331 (P5.039)
DI9CaptureControl
ID131 (P1.019)
DI9ExtraConfiguration
ID133 (P1.023)
NonVolatileCompareDataOffset
ID134 (P1.024)
VolatileCompareDataOffset
ID153 (P1.046)
EncoderOutputResolution
ID179 (P1.097)
EncoderOutputDenominator

<!-- page 9 -->

Use parameter ID351 (P5.059) CompareControl to enable or disable the
Capture function and to define the axis source and trigger logic. See Table 120.

> **Figure 186** — Compare Control

> **Table 120** — High-speed Position Compare, Additional Information

## ID351 P5.059

CompareControl
Bit
Function
Description
X

Compare feature
If bit 0 is set to 1, the compare feature executes when ID350 (P5.058) is > 0.

Cyclic mode
If bit 1 is set to 1 and all compare procedures are completed, ID350 (P5.058) resets to the setting value and then
compare procedure starts again.

Enable Capture function
after data compared
If bit 2 is 1, after all comparing is done, enable the Capture function (Set ID331 (P5.039.X) bit 0 to 1, and set the
previous value to ID330 (P5.038) as the data size to be captured); if Capture function has been enabled, then this
function is invalid.

Reset position of the
comparing axis to 0
If bit 3 is 1, once the last data array element is compared, ID349 (P5.057) is set to zero.
Y
–
Source setting of
comparing axis
0: Capturing axis
1: Auxiliary encoder (AUX)
2: Pulse command (I/O)
3: Main encoder (MFB)
If capturing axis is selected, source of the capturing axis ID331 (P5.039.Y) cannot be changed. If encoder of the
motor is selected, pulse resolution is determined by ID153 (P1.046) (Encoder pulse number output setting) and
ID176 (P1.097).
Z
–
Trigger logic
0: N.O. (normally open)
1: N.C. (normally closed)
U
–
Trigger PR command
If bit 0 is set to 1, PR#50 (DI9) or PR#60 (DI10) is triggered when the last data array segment is compared.
CBA
–
Pulse output duration
(ms)
–

**Extracted table (page 9, #1):**

| Bit | Function |
| --- | --- |
| 0 | Compare feature |
| 1 | Cyclic mode |
| 2 | Enable Capture function after data compared |
| 3 | Reset position of the comparing axis to 0 |
| – | Source setting of comparing axis |
| – | Trigger logic |
| – | Trigger PR command |
| – | Pulse output duration (ms) |

<!-- page 10 -->

Using the Motor Encoder as the Compare Source

> **Figure 187** — Pulse Output

If the capturing axis is selected, the source of the capturing axis ID331
(P5.039.Y) cannot be changed. If the motor encoder is selected, the pulse
resolution is determined by EncoderOutputResolution ID153 (P1.046) and
EncoderOutputDenominator ID179 (P1.097).
When EncoderOutputDenominator ID179 (P1.097) = 0, OA/OB pulse output
(Output Pulse Number from Figure 187) only refers to the setting of
EncoderOutputResolution ID153 (P1.046).
Example 1:
When EncoderOutputDenominator ID179 (P1.097) = 0;
EncoderOutputResolution ID153 (P1.046) = 2500
OA/OB output = EncoderOutputResolution ID153 (P1.046) uses
quadruple (4x the frequency), which is 10,000 pulses.
When EncoderOutputDenominator ID179 (P1.097) has been set (value is not 0),
OA/OB pulse output needs to be calculated via the numerator and
denominator of EncoderOutputResolution ID153 (P1.046) and
EncoderOutputDenominator ID179 (P1.097), and converted using quadruple
(4x frequency).

<!-- page 11 -->

Example 1:
When ID179 (P1.097) = 5; ID153 (P1.046) = 2500
Example 2:
When ID179 (P1.097) = 7; ID153 (P1.046) = 2500

> **Figure 188** — High Speed Position Compare Flowchart

OA/OB output
2500

----------
500 pulses
=
=
OA/OB output
2500

----------
357.142857 pulses
=
=

No
Data array

≥

1234
2345
5678
6789
7890
DO4
DO4
Yes

CAP
amount
ID350 (P5.058)
Start address
ID348 (P5.056)
CMP switch
ID351 (P5.059.X) Bit0
CMP axis
position 7890
CMP axis
position
ID349 (P5.057)
CMP axis source
AUX ENC ID351 (P5.059.Y) = 1
Pulse Cmd ID351 (P5.059.Y) = 2
Main ENC ID351 (P5.059.Y) = 3
CMP trigger logic
Normally open
ID351 (P5.059.Z) = 0
Pulse output duration
ID351 (P5.059.CBA)
Normally closed
ID351 (P5.059.Z) = 1
CMP amount
ID350 (P5.058) = ID350 (P5.058) - 1
CMP completed?
ID350 (P5.058) = 0
Call PR#45
ID351 (P5.059.U) Bit 0 = 1
Cyclic CMP
ID351 (P5.059.X) Bit 1 = 1
ID350 (P5.058) = Previous setting
value
Enable CAP after
completed
ID351 (P5.059.X) Bit 2 = 1
CMP axis position resets to

ID351 (P5.059.X) Bit 3 = 1

<!-- page 12 -->

Using PR Command Programming with the Compare Function
To use the Compare function, you must be in PR operation mode and use PR
commands.
With PR command programming, write commands configure the high-speed
position compare function, as well as execute the motion commands once
configuration is complete. See Chapter 11 for details on PR command
programming.
The following example describes how the PR command works and is
illustrated in Figure 190. Set the numerator ID153 (P1.046)
EncoderOutputResolution and EncoderOutputDenominator ID179 (P1.097),
whose default is based on the comparing an axis using 10000 pulses per motor
rotation.
1.
PR#1 use write commands to edit data array #50.
2. PR#2 use write commands to edit data array #51.
3.
PR#3 use write commands to edit data array #52.
4. PR#4 confirms that the Compare function is disabled
(ID331.X (P5.039.X) Bit 0 = 0).
5.
PR#5 sets the start position to 50.
6. PR#6 sets the compare amount to 3, with a delay of 1 ms to allow the PR
command with the use of Compare function to be executed.
7.
PR#7 enables the Compare function in cyclic mode, which clears the
compare axis to 0 after compare is complete and executes PR#45. It
selects encoder of the motor as the capturing axis, setting 'normally
closed' as the trigger logic with pulse output duration of 100 ms.
8. PR#8 sets the speed command to 50 rpm.
9. PR#45 sets the incremental command to 50000 PUU and then carries on
to PR#46, keeping the speed command setting of 50 rpm.

> **Figure 189** — PR Command with Application of Compare Function

PR#6 (I)
Write
DLY=[10] 1 ms
P5.058=3
PR#5 (I)
Write
DLY=[0] 0 ms
P5.056=50
PR#45 (I)
 Position
DLY=[0] 0 ms
50000 PUU
INC
100rpm
PR#8 (I)
Speed
DLY=[0] 0 ms
50 rpm
Acc=[0] 3.33 ms
Dec=[0] 3.33 ms
PR#46
Speed
DLY=[0] 0 ms
50 rpm
Acc=[0] 3.33 ms
Dec=[0] 3.33 ms
PR#3 (I)
Write
DLY=[0] 0 ms
Arr[52]=40000
PR#4 (I)
Write
DLY=[0] 0 ms
P5.059=
0x00640030
PR#2 (I)
Write
DLY=[0] 0 ms
Arr[51]=30000
PR#7 (I)
Write
DLY=[0] 0 ms
P5.059=
0x0064103B
PR#1
Write
DLY=[0] 0 ms
Arr[50]=20000

<!-- page 13 -->

Figure 190 shows how the data is compared in the data array when the
compare function is executed.

> **Figure 190** — Application Example of High-speed Compare

## Data Array

The data array can store up to 800 data (0…799) elements with each element
being 32 bit. It can be used to store the high-speed capture data and high-speed
compare data as well as E-CAM slave points. You can allocate the data array to
fit your application. There is no pre-defined allocation of data elements, which
provides flexibility.
CMP axis
position
(PUU)
Time
DO4: CMP
PR path
#8
#45
#45
#46
#46
5000
20000
30000
40000
#1 ~ 7
Data array
20000
20000
20000
20000
20000
20000
#50
30000
30000
30000
30000
30000
30000
#51
40000
40000
40000
40000
40000
40000
#52
(1)
(1)
(2)
(2)
(3)
(3)
IMPORTANT
The Data Array is volatile. That means it does not store values through a
power cycle. You can set Force Function ID193 (P2.008) = 30 and then
set Force Function = 35; this operation writes the Data Array values to
EEPROM. Now, they persist through a power cycle. You can see the
present values of the Data Array by navigating KNX5100C software;
Motion Control>Capture/Compare>Data Array Editor.

**Extracted table (page 13, #1):**

| Data array |  |  |  |  |  |  |
| --- | --- | --- | --- | --- | --- | --- |
| #50 | 20000 | 20000 | 20000 | 20000 | 20000 | 20000 |
| #51 | 30000 | 30000 | 30000 | 30000 | 30000 | 30000 |
| #52 | 40000 | 40000 | 40000 | 40000 | 40000 | 40000 |

<!-- page 14 -->

E-CAM
The Kinetix 5100 drive has an electronic camming feature. Position cams, in
effect, provide the capability of implementing non-linear electronic gearing
relationships between the Kinetix 5100 motor (slave) and a master device
(another drive, encoder, or time). No maximum velocity, acceleration, or
deceleration limits are used.
The motion of the master device and the designated cam profile that is derived
from the associated cam table determine the speed, acceleration, and
deceleration of the slave axis.
The E-CAM feature executes a position cam that is created by a user-defined
profile in the KNX5100C software or directly from the user-defined data array.
The mechanical cam and electric cam are illustrated in Figure 191.

> **Figure 191** — Mechanical Cam and E-CAME-CAM

The E-CAM function can be used only in PR Operating mode. After the E-CAM
engagment conditions are met and the E-CAM is active, the slave axis follows
the pre-defined cam profile, and the position of the slave axis is a function of
the master position (or time). As a result of master movement, pulses are
created. Those master pulses are used as the reference for the slave to follow;
and the slave follows those pulses based on its pre-defined profile, shown in
Figure 192. The E-CAM function can be enabled or disable by setting parameter
ID376 (P5.088.X).

> **Figure 192** — E-CAM Curve

(1) - Master Axis Input and Gear
(2) - Mechanical Cam
(3) - Slave Axis Output
(4) - Master Axis Input for E-CAM

<!-- page 15 -->

Figure 193 uses the mechanical cam concept to illustrate the E-CAM parameter
settings.

> **Figure 193** — Electronic cam simulates mechanical cam assembly with servo drive parameters

(1) Master Axis: master axis signal source is set by ID376 (P5.088.Y)
(2) Clutch: time to engage or disengage is set by ID376 (P5.088.UZ), ID375 (P5.087), ID377 (P5.089)
(3) Master Axis Gear Ratio: pulse input resolution is set by ID371 (P5.083), ID372 (P5.084)
(4) E-CAM Curve: curve is set by ID369 (P5.081), ID370 (P5.082), ID373 (P5.085), scale is set by ID311 (P5.019)
(5) Slave Axis Gear Ratio: output signal resolution is set by ID151 (P1.044), ID152 (P1.045)

> **Table 121** — E-CAM General Settings

Parameter
Name
ID245 (P2.073)
ECamConfiguration
ID246 (P2.074)
ECamDIDelayTimeCompensation
ID247 (P2.075)
ECamAlignementTargetPosition
ID248 (P2.076)
ECamControlConfiguration
ID249 (P2.078)
ECAMDOCamArea_RisingEdge
ID250 (P2.079)
ECAMDOCamArea_FallingEdge

<!-- page 16 -->

## E-CAM Control Settings

The format of ID376 (5.088) ECamControl is: (High word h) DCBA:
(Low word L) UZYX

> **Figure 194** — Parameter Format

Definition as follows:
•
X: E-CAM command
•
Y: command source
1: auxiliary encoder
2: pulse command
4: time axis (1 ms)
•
Z: engaging condition
0: immediately
1: trigger DI.CAM
•
U: disengaging condition (2, 4, and 6 cannot be selected at the same time)
•
BA: auto execute the specified PR command
When disengaging condition (ID376 (P5.088.U: 2, 4, 6)) is met, a PR
00…3F (hexadecimal; 00 means no action) is executed automatically.
C
D
BA
Y
Z
U
X
BA
PR command to execute
X
Activation setting of E-CAM
function
C
Reserved
Y
Command source
D
Reserved
Z
Engaging condition
-
-
U
Disengaging condition
Bit
Function
Description

E-CAM activation
0: E-CAM is disabled 1: E-CAM is enabled (relevant parameters cannot be modified once E-CAM is enabled)

E-CAM does not disengage when servo is off
0: when the servo is stopped by alarm or servo is off, the clutch disengages 1: when the servo stops because of
alarm or servo is off, the clutch remains engaged. When the servo enables again, E-CAM can operate directly.

ECamCurveScale ID311 (P5.019) is effective
immediately
0: ID311 (P5.019) is effective after next engagement 1: ID311 (P5.019) is effective immediately

Reserved
-
U
Disengagement condition
Action after disengaged

Never disengage
-

DI.CAM permissive is OFF
In stop status

Master axis reaches the disengage value of ID377 (P5.089) (Sign indicates the direction)
In stop status

Same as 2, but the existing speed continues when disengaging and the engaged length slightly exceeds ID377 (P5.089)
ECamMasterPositionToDisengage. This is suitable for when calling the next PR Position command immediately after
disengaging.

Master axis exceeds the value of disengaging ID377 (P5.089) (sign indicates the direction)
Return to pre-engaged status Lead pulse
is ID380 (P5.092)
ECamSkippedMasterPulses

When U = 1, 2, or 6: disable E-CAM after disengaging
Set X to 0
When U = 4: Avoid jittering when it returns to pre-enagaged status
N/A

**Extracted table (page 16, #1):**

| PR command to execute | X |
| --- | --- |
| Reserved | Y |
| Reserved | Z |
|  | U |

<!-- page 17 -->

•
C: reserved
•
D: reserved

<!-- page 18 -->

## Master Axis Signal Source

When using an electronic cam, you must first determine the source of the
master axis, which can be an auxiliary encoder (via Aux Port), another Kinetix
5100 motor (via PT wiring), or time (ms). The source is determined by
parameter ID376 (P5.088.Y) ECamControl.
•
Auxiliary Encoder - ID376 (P5.088.Y) = 1, the external encoder signal from
the Aux Feedback Connector (AUX) is used as the source of the master
axis signal. And the master axis position is monitored with ID309
(P5.017) AuxEncoderPosition.
•
Pulse Input - ID376 (P5.088.Y) = 2, the pulse input from the I/O connector
is used as the source of the master axis signal. And the master axis
position is monitored with ID310 (P5.018) PulseCmdPosition.
•
Time Axis (1ms) - ID376 (P5.088.Y) = 4, the 1ms pulse generated internally
by the servo drive is used as the source of the master axis signal.
You can monitor the master axis by using parameter ID374 (P5.086).
Digital Output CAM_Area Settings
The Kinetix 5100 drive provides two digital outputs that can operate based on
the present Master position within the E-CAM cycle. The first DO (0x18) ON
transition is determined by Ecam DO.Cam xxxx Rising Edge Angle ID378
(P5.090) and the OFF transition is determined by Ecam DO.Cam xxxx Falling
Edge Angle ID379 (P5.091), as illustrated in Figure 196. The second DO (0x1A) is
operated in a similar manner and is shown in Figure 197.

> **Figure 195** — Digital Output CAM Area

> **Figure 196** — Digital Output 1 - Engagement Timing

Slave axis position (PUU)
E-Cam angle ()

90 135 180 225 270 315 360
DO: 0x18
DO: 0x18
≤

ID378 (P5.090) = 135
ID379 (P5.091) = 240
ID379 (P5.091)
ID379 (P5.091) = 45
ID378 (P5.090)
ID378 (P5.090) = 270
ID378 (P5.090) > ID379 (P5.091)
Slave axis position (PUU)

<!-- page 19 -->

The following parameters define the function.
The relationship between DO.CAM_Area2 and the parameter values is shown
in Figure 197. When E-CAM is not engaged, this signal is always off.

> **Figure 197** — Digital Output 2 - Engagement Timing

> **Table 122** — Relevant Parameters

Parameter
Name
ID378 (P5.090)
ECamDOCamArea1RisingEdgeAngle
ID379 (P5.091)
ECamDOCamArea1FallingEdgeAngle
ID249 (P2.078)
ECamDOCamArea2RisingEdgeAngle
ID250 (P2.079)
ECamDOCamArea2FallingEdgeAngle

E-Cam position (PUU)
E-Cam angle ()

ID249 (P2.078) = 135
ID249 (P2.078) = 270
ID249 (P2.078)
ID249 (P2.078) > ID250 (P2.079)
ID250 (P2.079) = 240
ID250 (P2.079) = 45
ID250 (P2.079)
DO:0x1A
DO:0x1A
≤
E-CAM position (PUU)
E-CAM angle °

<!-- page 20 -->

System Variables for Master Access Monitoring
The Kinetix 5100 drive also provides four system variables to monitor the
master axis:
•
Cumulative Pulse of Master Axis - system variable 059(3Bh): this variable
is the feedback position of the Master. (E-CAMMasterAxisPosition, ID374
(P5.086))
•
Incremental Pulse of Master Axis - system variable 060(3Ch): the
incremental number of pulses of the E-CAM master axis in 1ms.
•
Lead Pulse of Master Axis - this variable is described as a Master Offset
Position. That is, after the E-CAM has been enabled, this position is the
incremental position that occurs (from the point of enabling) before the
E-CAM is engaged (and following the Master).
•
Position of Master Axis - system variable 062(3Eh): the position of the ECAM master axis.
Go to Settings>Monitoring Status and select the variables to monitor by using
Monitoring Items and running the monitor. You can map these variables to
System Variable Monitoring values by using the Settings>Parameter
Editor>Status Monitor. This setting lets you use the Scope to monitor the
values real-time.

> **Figure 198** — Monitoring Parameters

When using the E-CAM, the Pulse Output function
(KNX5100C>Settings>Pulse Output) provides pulses so the next drive can
receive and follow those pulses.
The Kinetix 5100 drive only provides four pulse output pins OA, /OA, OB, /OB
respectively. The pulse can be input to the drive through the I/O connector or
AUX connector. The servo drive output signal source is determined by ID173
(P1.074.Y). If the AUX connector is used as the pulse input channel, as shown in
Figure 199, then the value of ID173 (P1.074.Y) of each drive shall be set to 1. If the
I/O connector is used as the pulse input channel, as shown in Figure 200, then
the value of ID173 (P1.074.Y) of each drive is set to 2.

<!-- page 21 -->

> **Figure 199** — Using Kinetix TBIO Pulses (Master) and Auxiliary Feedback (AUX) Connector (Slave)

> **Figure 200** — Using Kinetix 5100 TBIO Pulses (Master) and Kinetix 5100 TBIO Pulses (Slave)

I/O
AUX
Pin 21 AMOUT+ OA
Pin 1 AM+ Opt_A
Pin 22 AMOUT- /OA
Pin 2 AM- Opt_/A
Pin 25 BMOUT+ OB
Pin 3 BM+ Opt_B
Pin 26 BMOUT- /OB
Pin 4 BM- Opt-/B
I/O
I/O
Pin 21 AMOUT+ OA
Pin 43 AM+ PULSE
Pin 22 AMOUT- /OA
Pin 41 AM- /PULSE
Pin 25 BMOUT+ OB
Pin 36 BM+ SIGN
Pin 26 BMOUT- /OB
Pin 37 BM- SIGN

<!-- page 22 -->

Clutch Engagement and Disengagement
When the E-CAM function is enabled, the state of the clutch determines
whether the slave axis start operates according to the received master axis
signal or not. When the clutch is engaged, the slave axis operates according to
the received master axis pulses and the cam curve. When the cam is
disengaged, the slave axis does not operate according to the cam curve even if
the slave axis receives master axis pulses.
Condition for Engagement
When the E-CAM function is enabled, the slave axis can only be operated
according to the master axis signal and the cam curve when the clutch is in the
engaged state, as shown in Figure 201. The timing condition of the clutch
engagement can be set by ECamControl parameter ID376 (P5.088.Z). The
Kinetix 5100 drive provides two different timing conditions that are selectable.
•
Immediate engagement (ECamControl ID376 (P5.088.Z) = 0) — The
clutch is engaged immediately when the E-CAM function is active.
•
Engagement using a Digital Input E-CAM engaging control [0x36]
(ECamControl ID376 (P5.088.Z) = 1) — The clutch is engaged when the
digital input DI:E-CAM engaging control [0x036] transitions on. When
this DI is on, the clutch remains engaged until a disengagement
condition is reached.

> **Figure 201** — Clutch Engagement Diagram

IMPORTANT
The E-CAM function does not allow E-CAM engagement based on the
absolute position of the master feedback source.

<!-- page 23 -->

In addition, the initial lead pulse number of the master axis before
engagement is set by ECamLeadPulseBeforeEngaged parameter ID375
(P5.087). The E-CAM begins executing the profile when E-CAM function is
enabled and the number of master pulses from ID375 (P5.087) occur, which is
considered a type of offset from the immediate engagement type.

> **Figure 202** — Clutch Engagement Lead Pulses

Condition for Disengagement
When the E-CAM function is enabled and the clutch is engaged, the slave axis
follows master pulses based on the cam curve. When the slave axis completes
the cam cycle, it is stopped either by disabling the E-CAM function or
disengaging the clutch. This action is shown in Figure 203. If the clutch is
disengaged, the slave axis remains stationary regardless of the action of the
master axis.

> **Figure 203** — Clutch Disengagement Diagram

ID375
(P5.087)

<!-- page 24 -->

You can choose the disengagement condition based on your requirements by
setting ECamControl ID376 (P5.088). The Kinetix 5100 drive provides five
disengagement timing conditions:
•
Continuous (ID376 (P5.088.U) = 0 — The clutch does not disengage until
the E-CAM function is disabled.
•
Disengagement using Digital Input E-CAM engaging control
(ID376 (P5.088.U) = 1) — The clutch is disengaged when the digital input
(DI:E-CAM engaging control [0x036]) transitions off. It remains
disengaged when this DI is OFF.
•
Disengagement once ID376 (P5.088.U) = 2 — The clutch is disengaged
and stops immediately when the number of master pulses reaches the
value in ID377 (P5.089), which is shown in Figure 204. This
disengagement condition is suitable for applications where the slave axis
must be accurately stopped.

> **Figure 204** — Immediate Stop After Disengagement

•
Deceleration stop after disengagement (ID376 (P5.088.U) = 6) — The
clutch is disengaged and decelerates smoothly to stop when the number
of pulses of master axis reached the value set by ID377 (P5.089). Then the
E-CAM system enters the stop state, as shown in Figure 205. This
disengagement condition is suitable for the application where the slave
axis must be slowly decelerated to a stop.

> **Figure 205** — Deceleration Stop After Disengagement

ID377 (P5.089)
ID377 (P5.089)

<!-- page 25 -->

•
Resuming PR Mode control after disengagement (ID376 (P5.088.U) = 4)
— slave axis returns to the PR Mode operation after the master pulses
have reached the value of ECamMasterPositionToDisengage ID377
(P5.089).
The E-CAM system enters the pre-engage state, as shown in Figure 206.
Once the ECamSkippedMasterPulses ID380 (P5.092) value occurs, the
clutch engages and the next cam cycle begins.

> **Figure 206** — Loop Mode After Disengagement

Pay attention to the difference between the lead pulses before engaged ID375
(P5.087) and the skipped master position pulses before re-engage ID380
(P5.092). ID375 (P5.087) takes effect before the first engagement, and, ID380
(P5.092) takes effect for each E-CAM cycle. Figure 207 shows the schematic
diagram of the combination of these two.

> **Figure 207** — Lead pulses before engaged and Skipped master position pulses before re-engage

ID377 (P5.089)
ID377 (P5.089)
ID380 (P5.092)
ID377 (P5.089)
ID377 (P5.089)
ID380
(P5.092)
ID375
(P5.087)

<!-- page 26 -->

These are the disengagement conditions.
•
Immediate stop after disengagement
•
Deceleration stop after disengagement
•
Entering loop mode after disengagement (ID376 (P5.088.U) = 2, 4, 6)

> **Figure 208** — Disengagement Dialog Box

You can set the PR command after the disengagement condition is immediate
stop after disengagement, deceleration stop after disengagement, or entering
PR control mode after disengagement (ID376 (P5.088.U) = 2, 4, 6). When there
is a non-zero value in instance ID376 (P5.088.BA), that PR command is
executed. (ID376 (P5.088.U) = 4), the slave axis continues in the next motion
cycle after PR command complete. That is because the E-CAM function doesn't
support interrupt.

<!-- page 27 -->

## E-CAM Alignment

E-CAM phase alignment is a cam compensation method provided by the servo
drive. You must first set the phase of the cam alignment and the position of the
external sensor. Each time the cam runs to the position of the external sensor,
the drive compares the difference between the actual phase and the correct
phase. The difference is stored in the PR program. You can choose to
compensate immediately or later.
E-CAM Alignment Operation Setting
The settings for ID245 (P2.073)ECamConfiguration are described below.
•
YX - range of filter (0…95%)
When the DI.ALGN Electronic cam phase alignment [0x35] signal is
triggered, the E-CAM alignment function is enabled. The system detects
the current E-CAM position. When the difference between the current ECAM position and its previous alignment position is less than the
parameter's range as a percentage, the filter function is enabled.
Otherwise, the system uses the new position to do the alignment.
•
UZ - maximum allowable correction rate (0…100%)
When alignment correction is enabled, the limitation of the maximum
allowable correction rate (C) is defined as follows:
|C|<= [(ID372) (P5.084) / ID371 (P5.083)] x ID245 (P2.073.UZ) %
When the alignment error is too large, correcting this error once may
cause motor vibration or overloading. Using this parameter can divide
the alignment correction into several stages to smooth the process, but it
may need more time to complete the alignment correction.

> **Table 123** — E-CAM Alignment Settings

Parameter
Name
ID245 (P2.073)
ECamConfiguration
ID246 (P2.074)
ECamDIDelayTimeCompensation
ID247 (P2.075)
ECamAlignementTargetPosition
BA
PR number
YX
Range of filter (0…95%)
DC
Masking range (0 … 95%)
UZ
Maximum allowable correction rate
(0…100%)
h
High bit
L
Low bit
YX

01…5F
Function
Filter disabled
| Error | <= (1 to YX)%: filter enabled
Using the filter allows the alignment to be more stable and reduces any position
errors caused by DI noise and results in smoother operation.
C
D
BA
YX
UZ

**Extracted table (page 27, #1):**

| PR number | YX |
| --- | --- |
| Masking range (0 … 95%) | UZ |
| High bit | L |

<!-- page 28 -->

•
BA: PR number (PR#0…PR#99)
After each alignment, any shortage of pulse numbers from the slave axis
is stored in a specified PR. This PR can compensate for the slave position
at the appropriate timing point. If BA is set to 0, any shortage of pulse
numbers is not stored in PR.
•
DC: masking range (0…95%)
When the DI.ALGN signal is triggered, the next alignment action is
allowed only after the increasing pulses of the master axis are greater
than the distance (M) masking.
M >= ((ID372) (P5.084) / ID371 (P5.083)) x ID245 (P2.073.DC) %
E-CAM Alignment Control Switch
The settings for ID248 (P2.076)ECamControlConfiguration are described
below.
•
X: E-CAM alignment control
The format of this parameter is HEX. Thus, to set PR#11, write 0B to BA.
IMPORTANT
This masking function only works with increasing master pulses, and
does not work for decreasing master pulses.
X
E-CAM alignment control
UZ
Alignment forward
direction allowable rate
(0…100%)
Y
Filter intensity (0 - F)
-
-
Bit
Function
Description

Enable alignment
Set this bit to 0 to disable this function; set this bit to 1 to enable this
function.
If enabled, the E-CAM alignment correction is executed when DI.ALGN is
on.

## Trigger PR

immediately
Set this bit to 1 to enable this function. When the E-CAM alignment is
executing, the correction is stored in the PR data location specified by
ID245 (P2.073), which triggers the PR immediately.
Set this bit to 0 to disable this function. When the E-CAM alignment is
executing, it does not trigger PR immediately to compensate the
correction. You must use the PR ID376 (P5.088.BA) when E-CAM
disengages in order to execute it.

Position of the mark
0: if the mark is on the master axis, the position of the mark is not
affected when aligning.
1: if the mark is on the slave axis, the position of the mark is affected
when aligning.

Reserved
-
Y
UZ
X

Bit

<!-- page 29 -->

•
Y: filter intensity (0…F)
Indicates average of 2^(value). Set to 0 to disable the filter. When the
value of Y increases, the correction is slower which can avoid large
amounts of correction during E-CAM adjustment.
This can also avoid disturbances caused by sensor noise for a smoother
operation. Setting ID248 (P2.076) too high causes the alignment to not
work properly. The recommended value is 3.
Example: when the filter intensity value is 3, the actual filter intensity =
2^3 = 8.
•
UZ: alignment forward direction allowable rate (0…100%)
Value
Alignment direction
Value
Alignment direction

Backward alignment only

Forward 80%, backward 20%

Forward 30%, backward 70%
>= 100
Forward alignment only

Alignment with the shortest distance
-
-
Data Array

#50
3000

#51
20000

#52
4000

#53
45000

#54
3000

#55
20000

#56
20000

#57
5000

**Extracted table (page 29, #1):**

| Alignment direction | Value |
| --- | --- |
| Backward alignment only | 80 |
| Forward 30%, backward 70% | >= 100 |
| Alignment with the shortest distance | - |

<!-- page 30 -->

## E-CAM Profile Types

The electronic cam profile is a variable relationship between a master and slave
signal. The cam profile can be built in a variety of ways and can be built using
other mathematical software tools. The KNX5100C software uses the E-CAM
editor to create and edit different types of profiles. Once the profile is created,
it is stored in the Data Array. The following are some important
considerations:
•
One single profile can have up to 720 points.
•
The Master data must all be within a value of 360. It is typical to think of
this value as degrees because the drive does not use Position Units as
scaling. This value can later be normalized in the E-CAM configuration to
match your application, for example, pulses required for one machine
cycle.
•
The master data is not changeable when it is entered into the data table
and is evenly spaced depending on the amount of points that are
entered.
Take Figure 209 as an example, if a mechanical cam is to be replaced by an
electronic cam, the mechanical cam must be divided into several equal parts.
The more granular the divisions, the higher the accuracy. This example divides
it into 8 equal parts. Each part is separated by 45 degrees (for example, the
actual application may have finer definition (and granularity). The starting
point of 0 degrees and the last point of 360 degrees is the same point. It is
typical for these points to be used to define an entire machine cycle. Therefore,
a total of 9 point pairs of data must be filled in to create the table of the
electronic cam curve.

> **Figure 209** — E-CAM Curve Table Creation

E-CAM angle °
Slave Axis Position

<!-- page 31 -->

You can use KNX5100C software to create electronic cam curves. Navigate to
Motion Control>E-CAM editor, shown in Figure 210.
The E-CAM editor contains a wizard that uses 4 steps to complete the E-CAM
configuration. Step 1 is used to choose the approach to use for your E-CAM.

> **Figure 210** — E-CAM Edit Dialog Box

Manual
If the E-CAM table is already known, you can enter the points to complete the
cycle profile. As shown in Figure 209, the E-CAM curve is created based on the
cam curve-to-edge distance corresponding to each angle of the mechanical
cam, which is relationship between the angle and the slave axis position. You
must use your master machine cycle profile as degrees. These values can be
normalized back into counts in the E-CAM editor Parameter Setting Step. The
KNX5100C software E-CAM table manual creation interface is shown in
Figure 211. The following are the steps to manually create the table:
1.
Set the number of E-CAM segments.
A single cam can be divided into up to a maximum of 720 segments (721
points). For a period of 360 degrees, every 0.5 degrees corresponds to a
slave axis position. The more points, the higher the resolution. To select
the most suitable number of segments, consider the resolution of the
curve and the resource usage of the data array.

<!-- page 32 -->

2. When the number of E-CAM segments is set, click Create Table.
The software automatically divides the table evenly into 360 degrees by
using the total number of segments.
3.
Fill in the position (Position Y) of the slave axis.
The position corresponding to each segment angle is filled in the table in
units of PUU. When you click Draw, the software automatically plots the
E-CAM simulation curve and the position, velocity and acceleration
curves. Pay special attention to the continuity of the slave axis speed in
manual creation to use manageable axis dynamics.
4. After confirming that the curve is correct, click Download Table, to write
the E-CAM curve to the data array.
Because the E-CAM data is stored in the Data Array, it is volatile, which
means that it does not persist through a power cycle. Once your E-CAM
table is finalized, check 'Burn to EEPROM when download' and click
Download. The data is saved and persists through a power cycle.

> **Figure 211** — E-CAM Table Manual Creation

<!-- page 33 -->

## Import Points

If you use the third-party software (such as: Excel) to create the table, you must
save the position of each point as a text file (.txt). Separator symbol between
each point should be indicated by Space, Tab, Enter, ‘|’ or comma. Figure 212
illustrates the following steps.
1.
Open the E-CAM editor in KNX5100C software, selecting the manual
table and specify the number of E-CAM segments (ID370 (P5.082)).
2. Click Create Table, the table displays the E-CAM phase corresponding to
each E-CAM segment.
3.
Right-click the form and select Import Points.
The import points dialog box appears.
4. Click Browse to open the stored text file, and select the separator symbol
you used in the text file.
5.
Click OK, to import the data points in the text file.
6. Click Draw and the software draws the designed E-CAM curve according
to the data points.
You can also export the data points to text files by selecting ‘Export Points’. The
KNX5100C software provides a Batch Values Change feature, which includes
functions of increment, decrement, add, subtract, multiply, divide, copy and
exchange for you to quickly adjust the E-CAM curve. You can also right-click to
insert and delete single-position positions.

> **Figure 212** — E-CAM Import or Export Points

<!-- page 34 -->

## Speed Fitting

This method creates a custom index profile on the slave that is based on the
motion of the master. This method works best when the master is moving at a
constant velocity. This method divides an E-CAM cycle into five zones: waiting
zone, acceleration zone, constant speed zone, deceleration zone, and stop
zone, as shown in Figure 213. The proportion of each zone can be adjusted. The
E-CAM curve is designed from the position point of view. The corresponding
speed to the master-slave axis is determined by the position change per time
unit. The KNX5100C E-CAM table creation by using the speed fitting method is
shown in Figure 214.
1.
To plan the E-CAM curve, determine the proportion of waiting zone,
acceleration zone, constant speed zone, deceleration zone, and stop zone
in one cycle according to the required distribution.
2. Set the motion distance.
The total travel distance of the slave axis in one cycle, the unit is PUU.
3.
Set the smoothness of the position curve at the turning point.
The larger the set value, the smoother the motor change during
acceleration and deceleration. A smoother curve extends the running
time in one cycle. The value of the S curve is usually the same as the
number of data points in the stop zone or less than the number of data
points in the stop zone.
4. After confirming that the curve is correct, click Download Table, then the
E-CAM curve is written to the data array.
Because the E-CAM data is stored in the Data Array, it is volatile, which
means that it does not persist through a power cycle. Once your E-CAM
table is finalized, check 'Burn to EEPROM when download' and click
Download. The data is saved and persists through a power cycle.

> **Figure 213** — Speed Zone Definition

## Slave Axis Position

(PUU)
E-CAM angle °
Waiting Zone
Acceleration
Zone
Constant
Speed
Zone
Deceleration
Zone
Stop
Zone

<!-- page 35 -->

> **Figure 214** — E-CAM Speed Fitting Table

## Cubic Curve

When the master-slave axis only has position correspondence, such as pointto-point correspondence, then the cubic curve can be used to create an
electronic cam curve. When using the cubic curve to create a table, you only
need to fill in the phase angle with the corresponding slave axis position. The
software automatically draws and optimizes the curve. Some applications
require a point-to-point motion trajectory, such as a straight line or a curve.
The E-CAM curve creation is simplified by the cubic curve creation method. As
shown in Figure 215, the starting angle N1 (the angle from the starting point)
and the ending angle N2 (the angle of entering the target point) can be set
according to the application requirements. There are three different types of
curve:
•
Straight line — There is a straight line between the two data points of the
cam. The starting angle and the ending angle are not adjustable.
•
Constant acceleration — A unidirectional increasing or decreasing curve
with an equal acceleration and deceleration. Only the starting angle can
be adjusted.

<!-- page 36 -->

•
Cubic curve — Both the starting angle and the ending angle can be
adjusted. The change of the angle will affect the speed change when
leaving the starting point and entering the target point. Improper angle
setting causes the speed to change sharply.

> **Figure 215** — Starting and Ending Angle

The KNX5100C software E-CAM table creation by cubic curve method is shown
in Figure 216. The following is the operation steps of the cubic curve table
creation:
1.
Set E-CAM curve.
The cubic curve table data includes angle, slave axis position, curve type,
starting angle and ending angle. You can change the data corresponding
to each point by dragging the turning point in the Cubic Curve
Simulation diagram, and can also insert or delete a specific turning
point. When dragging, inserting, or deleting a turning point, the data
content in the Cubic Data changes accordingly. However, when directly
inputting the desired content into Cubic Data, click Create Cubic Curve
to see the cubic curve simulation.
2. After completing the setting of the turning point, set the sample angle
(Sample ang.) and click Convert to E-CAM table.
The software automatically fills the data of each sampling point into the
E-CAM table according to the curve. The more points, the more precise
the E-CAM curve. If the position of the slave axis is too small, that can
cause the speed jitter. You can adjust the parameter instance ID311
(P5.019) ECamCurveScale to enlarge the value in the table to improve the
speed jitter.
3.
After confirming that the curve is correct, click Download Table.
The E-CAM curve is written to the data array. If you have selected ‘Burn
to EEPROM when download’, when you click the download button, the
data array is written to the EEPROM that can be held after the power is
turned off.

<!-- page 37 -->

> **Figure 216** — E-CAM Cubic Curve Table

> **Table 124** — E-CAM Wizard Parameter Definitions

ID
Parameter
Name
Description

P5.081
E-CAMStart Address
This is the starting address of the Data Array that is used as the first point in the E-CAM profile.

P5.082
E-CAM Area Number
This is the length of the Cam profile - this value uses the data array (max 720).

P5.085
E-CAM Engaged Area Number
Data Array index value used as the first cam point to be executed.

P5.086
E-CAM Master Axis position
Displays the feedback Position of the Master feedback source.

P5.087
E-CAM Lead Pulse Before Engaged
Master Offset Position - from when the enable permissives are met and the E-CAM is ready to execute, this
number of pulses occurs before the slave begins following.

P5.090
E-CAM DO.CAM_Area Valid Start Angle Specify the rising edge of the DO that can be used to show the E-CAM is engaged (Master pulses).

P5.091
E-CAM DO.CAM_Area Valid End Angle
Specify the falling edge of the DO that can be used to show E-CAM is Engaged (Master units).

P5.019
E-CAM Curve Scale
Used to change the amplitude of the cycle profile in terms of speed (positioning follows the cycle profile).

P5.083
E-CAM SlaveCycleNumber M
This profile is executed according to this value. If 2 is entered, the entire cycle profile executes 2 times
and the speed is also 2x as fast. If 3 is entered, the cycle profile is executed 3 times and the speed is 3
times as fast

P5.084
E-CAM MasterPulseNumber P
The E-CAM editor uses 0…360 for master units and is not changeable. If your machine cycle is not
representative within the 0…360 units; for example, using time and 4000 ms for one machine cycle, enter
the value used to represent one machine cycle. This value is the normalizing parameter used for the ECAM.

**Extracted table (page 37, #1):**

| Parameter | Name |
| --- | --- |
| P5.081 | E-CAMStart Address |
| P5.082 | E-CAM Area Number |
| P5.085 | E-CAM Engaged Area Number |
| P5.086 | E-CAM Master Axis position |
| P5.087 | E-CAM Lead Pulse Before Engaged |
| P5.090 | E-CAM DO.CAM_Area Valid Start Angle |
| P5.091 | E-CAM DO.CAM_Area Valid End Angle |
| P5.019 | E-CAM Curve Scale |
| P5.083 | E-CAM SlaveCycleNumber M |
| P5.084 | E-CAM MasterPulseNumber P |

<!-- page 38 -->

Notes:

<!-- page 39 -->

The 2198-Exxxx-ERS servo drives are equipped for hardwired Safe Torque Off
(STO). The hardwired STO function meets the requirements of Performance
Level d (PLd) and safety category 3 (CAT 3) per ISO 13849-1 and SIL 2 per IEC
61508, IEC 61800-5-2 and IEC 62061.
The 2198-Exxxx-ERS servo drives use the STO connector for wiring external
safety devices and cascading hardwired safety connections from one drive to
another.

> **Figure 217** — Hardwired Safe Torque Off

Topic
Page
Certification

Description of Operation

Average Frequency of a Dangerous Failure per Hour

Safe Torque Off Connector Data

Wire the Safe Torque Off Circuit

Safe Torque Off Feature

Safe Torque Off Specifications

Safe Torque Off Wiring Diagrams

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
1606-XL
Power Supply
Input
Allen-Bradley
AC Input Power
Safety
Device
1606-XLxxx
24V DC Functional Safety Power
(customer-supplied)
Safe Torque Off (STO) Connectors
Kinetix® 5100 Servo Drives
(2198-E1020-ERS drives are shown)
