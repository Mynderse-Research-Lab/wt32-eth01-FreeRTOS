# Chapter 9: Tuning

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `10_Ch09_Tuning.pdf` (34 pages)_

<!-- page 1 -->

## Tuning Process

Reset Gains to Default
The drive uses default tuning gains when the drive and motor catalog number
are chosen. If you are unsure of the present drive tuning, it might be necessary
to revert the drive to the default gains. To revert to the default, use
GainAdjustMode ID217 (P2.032) = 4. This change resets the gains to default
values. Once the gain reset is complete, restore the value to its original setting
where:
•
0 = manual
•
1 = Mode1,
•
2 = Mode2,
•
3 = reserved
•
4 = reset to default
The autotuning test moves the motor (and the load if attached) and attempts to
determine the optimal settings for the drive/motor combination relating to the
gains and filters. If autotuning does not provide suitable performance, then
tuning mode 1, tuning mode 2, and manual tuning mode can be used. You can
use the System Analysis tool to generate a system response after the different
tuning types are executed.
The flowchart in Figure 84 provides an overview of the tuning process.
Before we examine gains (and consider changing them) if the default gain
model is not sufficient for your application, understanding some control
theory terms and concepts is important. It can help you understand what is
important about the tools and gain settings.
Bandwidth
Bandwidth is the measure of the system performance. The bandwidth is
typically measured in Hertz (Hz). When the Bandwidth is higher, that
indicates a responsive system, as can be seen from this image; the dotted line
indicates a command signal, and the solid line indicates the actual response
from the load. Lower bandwidth systems indicate a lower response. See
MOTION-AT005 for additional details on the bandwidth term.

<!-- page 2 -->

A good way to understand the characteristics of your system (and its
performance) is to use a Bode plot.
As shown in this bode plot using Hz (left), the usable bandwidth is the area
below the -3.0dB point and cutoff frequency. This same representation is
shown in Hz (right) and the bandwidth is indicated.

<!-- page 3 -->

> **Figure 84** — Tuning Procedure Flowchart

Motor runs smoothly
without load.
Satisfactory
performance?
Satisfactory
performance?
Satisfactory
performance?
Satisfactory
performance?
Enter Tuning mode 1.
See page 212.ID217 (P2.032) = 1
Enter Manual mode.See page 218.
ID217 (P2.032) = 0
Enter Autotuning
mode.
Complete
Yes
Yes
Yes
Yes
No
No
No
No
All parameters
can be adjusted
in Manual mode.
Enter Tuning mode 2.
See page 215.ID217 (P2.032) = 2
Satisfactory
performance?
Yes
No

<!-- page 4 -->

Autotuning
Autotuning can be performed via KNX5100C software or via the drive panel.
Currently, autotuning cannot be performed via Studio 5000 Logix Designer®
application. Autotuning works best when performed on a mechanism with a
load ratio (load:motor inertia ratio) of < 50:1.
Through the autotuning function, the servo drive helps you find the most
suitable parameters for your mechanical system. The values of the parameters
listed in the following tables can change as a result of autotuning.
Autotuning Configuration Parameters
Parameters ID264 (P2.105) AutoTuningBandwidth and ID265 (P2.106) Auto
TuningOvershoot can be used to adjust the responsiveness and stiffness,
respectively, in autotuning mode.

> **Table 76** — Gain-related Parameters

Parameter
Name
ID144 (P1.037)
LoadInertiaRatio
ID185 (P2.000)
PositionProportionalGain
ID189 (P2.004)
VelocityProportionalGain
ID191 (P2.006)
VelocityIntegralGain
ID216 (P2.031)
SystemGainResponseLevel
ID217 (P2.032)
GainAdjustMode

> **Table 77** — Filter and Resonance Suppression Parameters

Parameter
Name
ID135 (P1.025)
LowFreqVibrationSuppression1Frequency
ID136 (P1.026)
LowFreqVibrationSuppression1Gain
ID137 (P1.027)
LowFreqVibrationSuppression2Frequency
ID138 (P1.028)
LowFreqVibrationSuppression2Gain
ID208 (P2.023)
NotchFilter1Frequency
ID209 (P2.024)
NotchFilter1Depth
ID210 (P2.025)
ResonanceSuppressionLowPassFilterTime
ID226 (P2.043)
Notch Filter2Frequency
ID227 (P2.044)
Notch Filter2Depth
ID228 (P2.045)
Notch Filter3Frequency
ID229 (P2.046)
Notch Filter3Depth
ID232 (P2.049)
VelocityFeedbackLowPassFilterTime
ID257 P(2.098)
Notch Filter4Frequency
ID258 P(2.099)
Notch Filter4Depth
ID260 P(2.101)
Notch Filter5Frequency
ID261 P(2.102)
Notch Filter5Depth

<!-- page 5 -->

ID264 (P2.105) - AutoTuningBandwidth Parameter
This parameter is used to adjust the system bandwidth in conjunction with
autotuning. If this value is larger than the default response of 11, the
bandwidth after autotuning is higher, this higher bandwidth might be
problematic for your load, causing machine resonances or even instability. If
this value is smaller, the bandwidth after autotuning is lower, and the system
response is lower.

> **Figure 85** — AutoTuningBandwidth Parameter Graph

ID265 (P2.106) - AutoTuningOvershoot Parameter
This parameter is used to adjust the maximum allowable overshoot when
autotuning. The overshoot range is set according to the user or machine. If this
value is larger, the maximum overshoot that is allowed by autotuning is
greater, and the response is faster. If this value is smaller, the maximum
overshoot that is allowed by autotuning is smaller, but the response is slower.

> **Figure 86** — AutoTuningOvershoot Parameter Graph

Autotuning via the Drive Panel
See Figure 87 for an overview of autotuning via the drive panel. Make sure that
the emergency stop and the positive and negative limit works properly before
you start to tune the system.
Value (dB)
Frequency
(Hz)
Bandwidth
Higher
Bandwidth
Moderate
Bandwidth
Lower
ID264 (P2.105)>11
ID264 (P2.105)=11
ID264 (P2.105)<11
Pulse Number (Pulse)
Feedback
Command
ID265 (P2.106)
Time (sec)

<!-- page 6 -->

> **Figure 87** — Autotuning Via the Drive Panel Flowchart

S
M

S
M
S
M
S
M
S
M
S
M
S
M
S
M
S
M
S
M
S
M
S
M
S
M
S
M
S
M
S
M
S
M
S
M
Servo On?
Complete
Press the S key to set the drive
to Servo on.
Press the M key (Mode) and S key (Shift)
for four seconds to enter the Auto-tuning mode.
S-Cmd is displayed on the drive panel is blinking.
You can select the internal or external command
using the Up and Down keys.
Press the S key to set the internal command.
No
Reminds you to change the status
to Servo on.
‘JOG-S’ is blinking to remind
you to set the JOG speed.
Then, you can set the JOG speed
with the Up, Down, and Shift keys.
Press the S key to set the JOG speed.
Press the Up and Down keys
to set Position 1.
Press the S key to complete the
setting of Position 1.
Press the Up and Down keys
to set Position 2.
Then, the system starts to do positioning
between the two points that you just set.
The blinking ‘SPEED’ reminds you
to adjust the positioning speed.
Set the speed with the Up,
Down, and Shift keys.
Press the S key to complete
the speed setting.
Use the Up and Down keys to select
Yes or No. ‘YES’ means the setting is
complete. ‘NO’ means to keep adjusting
the speed.
Press the S key to start auto-tuning.
The panel shows the percent complete.
When you see ‘SET’ displayed on the panel, you can
press the S key to complete the setting or you can
press the M key to exit auto-tuning mode.
Press the S key to complete the setting.
Press the M key to exit
auto-tuning mode.
Yes
The blinking ‘SERVO OFF’ reminds
you that the controller has not
issued the Servo On command to
the drive. Then, it exits
auto-tuning mode.
Yes
No
Servo On?
Check the
servo status.
Issue the external commands
 and then press the S key to
continue tuning.
Press the M key (Mode) and S key (Shift)
for four seconds to enter the Auto-tuning mode.
S-Cmd is displayed on the drive panel is blinking.
You can select the internal or external command
using the Up and Down keys.
Yes
No
No
Yes
Reminds you to change the status
to Servo on.
Press the S key to set the internal command.
Press the S key to set the drive
to Servo on.
‘JOG-S’ is blinking to remind
you to set the JOG speed.
Then, you can set the JOG speed
with the Up, Down, and Shift keys.
Press the S key to set the JOG speed.
Press the Up and Down keys
to set Position 1.
Press the S key to complete the
setting of Position 1.
Press the Up and Down keys
to set Position 2.
Then, the system starts to do positioning
between the two points that you just set.
The blinking ‘SPEED’ reminds you
to adjust the positioning speed.
Set the speed with the Up,
Down, and Shift keys.
Press the S key to complete
the speed setting.
Use the Up and Down keys to select
Yes or No. ‘YES’ means the setting is
complete. ‘NO’ means to keep adjusting
the speed.
Press the S key to start auto-tuning.
The panel shows the percent complete.
When you see ‘SET’ displayed on the panel, you
can
press the S key to complete the setting or you can
press the M key to exit auto-tuning mode.
Press the S key to complete the setting.
Press the M key to exit
auto-tuning mode.
The blinking ‘SERVO OFF’ reminds
you that the controller has not
issued the Servo On command to
the drive. Then, it exits
auto-tuning mode.
Issue the external commands
and then press the S key to
continue tuning.
Check the
servo status.
Servo
Servo

<!-- page 7 -->

## Autotuning via KNX5100C Software

Autotuning can be performed by using KNX5100C software. When the drive
and motor are connected, and the drive is online with the KNX5100C software,
follow these steps to autotune the drive. You can autotune with or without the
load attached. The autotune is effective when used with a Load:Motor inertia
ratio of less than 50:1 with a rigid load.
There are two options available that determine how the motor is commanded
to drive.
•
Host controller plans the path and issues the command to drive the
motor
•
Drive plans the path and issues the command to drive the motor
Connect to Drive and Select Autotuning
1.
Select Add New Drive from the New menu.
2. On the New Device dialog box, click Add.
Once you are connected with your drive and you are Online, the
following window is displayed.
3.
Click Auto Tuning from the Function list Settings>Tuning .
4. Continue with the steps that are shown in Motion Command From
Controller or Motion Command From Servo Drive.
ATTENTION: The motor rotates during this tuning procedure. Hazard of
personal injury exists due to motor shaft rotation and/or machinery motion.

<!-- page 8 -->

## Motion Command From Controller

Follow these steps to have the host controller plan the path and issue the
command to drive the motor. The path for autotuning must be bi-directional
and contain a dwell to support the drive firmware.
1.
Select Controller: Motion Command From External Source and make
sure that the motion path is set correctly.
The controller is generating a bi-directional profile with a dwell segment
between each path. The graphic shows the path.
2. After the setting is done, run the motor repeatedly by using the path you
just set, and then click Next.
Set the motor to operate at least one cycle in both forward and backward
directions.

<!-- page 9 -->

3.
Wait until the tuning progress bar reaches 100%, a dialog box showing
autotuning completed is displayed, and then click OK.
A table is displayed that shows the values of parameters before and after
autotuning.
4. Click Download (apply the tuning result) or Exit (ignore the tuning
result) to complete autotuning.
You can click Emergency Stop to stop the tuning process.

<!-- page 10 -->

## Motion Command From Servo Drive

Follow these steps to have the drive plan the path and issue the command to
move the motor.
1.
Select Drive: Motion Command From Servo Drive.
2. Complete the following steps to set the running path of the motor.
a. Set the system to Servo ON state.
b. Set the acceleration/deceleration time and jog speed.
The default setting of acceleration/deceleration time is 500 ms.
Set the jog speed to no less than 500 rpm. Set these values similarly to
your application requirements.
c. Click Download.
d. When the motor dynamics are set (Step 2 of the Autotuning dialog
box), click Position 1 to register a start position for the bi-directional
autotune index.
e. Use
 or
 to jog the motor away from Position 1 and to generate
Position 2. When you have chosen a location for Position 2, click
Position 2.
f. Then, click Start to move the motor between the two positions. The
motor uses bi-directional movements between Position 1 and Position
2.
g. Click Next.
(a)
(b)
(c)
(d)
(e)
(f)
(g)

<!-- page 11 -->

3.
Wait until the tuning progress bar reaches 100%, a dialog box showing
Auto tuning completed is displayed, and then click OK.
The autotune process can take a few minutes to complete. The drive is
measuring resonance and gain output. You might hear vibrations and
noise from the motor during the autotune, which is normal.
A table is displayed that shows the values of parameters before and after
autotuning.
4. Click Download (apply the tuning result) or Exit (ignore the tuning
result) to complete autotuning.
You can click Emergency Stop to stop the tuning process.

<!-- page 12 -->

Alarms Related to Autotuning
When using Autotune: Motion Command from External Source, the operation
cycle (such as acceleration, constant speed, and deceleration) and dwell time
are vital to the correct execution of the Autotune test. See Figure 88.

> **Figure 88** — Settings Required for Autotuning

If any of these settings are not correct, the servo drive stops and displays a
fault. See Table 78 for possible causes and solutions.
Acc. time
Max.
 speed
Dwell
Operation
cycle
Operation
 cycle
Speed
Time

> **Table 78** — Faults Related to Autotuning

## Fault Code

Fault Name
Possible Causes
Possible Solutions
E 08A
Autotuning command error
The external source command was not issued.
Check the external source command.
Cable connection error.
Check the cable connection.
Position 1 and 2 were the same when command was
issued.
Reconfigure position 1 and 2.
E 08B
Dwell time too short
Dwell time too short.
Setting the dwell time is required. Increase the dwell time to more than
1 second.
E 08C
Inertia estimation error
Acceleration/deceleration time was too long.
Verify that the acceleration/deceleration time for motor to start from
0…3000 rpm is within 1.5 sec.
Speed is too slow.
The lowest possible speed setting is 200 rpm. The maximum speed is
3000 rpm, set the value as high as your application allows.
Inertia mismatch.
Verify that load inertia is not more than 50 times the motor inertia.
Inertia variation is too vigorous.
Resize the system requirements.

**Extracted table (page 12, #1):**

| Fault Name | Possible Causes |
| --- | --- |
| Autotuning command error | The external source command was not issued. |
|  | Cable connection error. |
|  | Position 1 and 2 were the same when command was issued. |
| Dwell time too short | Dwell time too short. |
| Inertia estimation error | Acceleration/deceleration time was too long. |
|  | Speed is too slow. |
|  | Inertia mismatch. |
|  | Inertia variation is too vigorous. |

<!-- page 13 -->

Tuning via Tuning Mode 1
and Tuning Mode 2
Apart from the autotuning function described earlier, there are two other
tuning modes provided to fine-tune the system.
Tuning Mode Process
See Figure 89 for an overview of the tuning mode process.

> **Figure 89** — Tuning Mode Process

Start
Tuning Mode 1
ID217 (P2.032) = 1
Adjust ID216 (P2.031)
Keep estimating ID144 (P1.037)
Satisfactory
Performance?
Satisfactory
Performance?
Yes
No
Yes
Tuning Mode 2
ID217 (P2.032) = 2
No
Adjust ID216 (P2.031), ID144 (1.037)
Enter Manual Mode
ID217 (P2.032) = 0
All parameters can
be adjusted in
Manual Mode.
Satisfactory
Performance?
Yes
No
Complete
Tuning Mode 1
ID217 (P2.032) = 1
Yes
Yes
Yes
No
No
No
Start
Tuning Mode 2
ID217 (P2.032) = 2
Enter Manual Mode
ID217 (P2.032) = 0
Satisfactory
Performance?
Satisfactory
Performance?
Satisfactory
Performance?
Keep estimating ID144 (P1.037)
Adjust ID216 (P2.031)
Adjust ID216 (P2.031), ID144 (P1.037)
Complete
All parameters can
be adjusted in
Manual Mode

<!-- page 14 -->

Tuning Mode 1
In this mode, the drive keeps estimating the mechanical inertia and updating
the value of parameter ID144 (P1.037). As shown in Table 79, note that you can
adjust the parameters in the Manual Tuning column while in Tuning Mode 1,
but the parameters in the Autotune column are still adjusted automatically.
Inertia Estimation
Inertia Estimation occurs while the motor is indexing and under acceleration
and deceleration. It does not estimate while the motor is at standstill. When
the present value of Inertia is similar to the estimated value, the present value
is maintained.
Requirements for Inertia estimation:
•
User defined movement (an indexing PR for example).
•
Motor speed increases from 0 rpm…3000 rpm within 1.5 seconds.
•
It is suggested to set the speed to 500 rpm or greater. Set the lowest
speed no less than 200 rpm.
•
The load inertia should be less than 50 times the motor inertia.
•
The change in the external force or inertia ratio cannot be too great.

> **Table 79** — Tuning Mode 1, Related Parameters

ID217 (P2.032)
Setting Value
Tuning Mode
Inertia Estimation
Parameter
Manual Tuning
Autotuning

Tuning Mode 1
Real-time estimation
ID216 (P2.031)
ID144 (P1.037)
ID185 (P2.000)
ID189 (P2.004)
ID191 (P2.006)(
ID208 (P2.023)
ID209 (P2.024)
ID210 (P2.025)
ID229 (P2.046)
ID226 (P2.043)
ID 227 (P2.044)
ID228 (P2.045)
ID232 (P2.049)
ID257 (P2.098)
ID258 (P2.099)
ID260 (P2.101)
ID261 (P2.102)

**Extracted table (page 14, #1):**

| Tuning Mode | Inertia Estimation |  |
| --- | --- | --- |
|  |  | Manual Tuning |
| Tuning Mode 1 | Real-time estimation | ID216 (P2.031) |

<!-- page 15 -->

Tuning Mode 1 in KNX5100C Software
You can use KNX5100C software for manual tuning in Mode 1 by choosing
Manual Tuning from the Function List and selecting Mode 1.

> **Figure 90** — Selecting Mode 1 Manual Tuning

The Smoothing and Filtering tab lets you configure the parameters related to
the Low Pass and Moving filters and S-curve, depending upon your configured
Operating mode. See Chapter 10 for details on filters and s-curves.

<!-- page 16 -->

Tuning Mode 2
When Tuning Mode 1 does not meet your performance requirements, you can
try Tuning Mode 2 to tune the servo system. In Tuning Mode 2, the system
does not automatically estimate the inertia, but rather it lets you choose the
Inertia Estimation to occur once by using a user-defined movement created in
the KNX5100C software. As shown in Table 80, note that the parameters in the
Manual Tuning column can be adjusted while in Tuning Mode 2, but the
parameters in the Autotune column are still adjusted automatically.
The correct mechanical inertia ratio must be entered in parameter ID144
(P1.037).

> **Table 80** — Tuning Mode 2, Related Parameters

ID217 (P2.032)
Setting Value
Tuning Mode
Inertia Estimation
Parameter
Manual Tuning
Autotuning

Tuning Mode 2
Value of ID144 (P1.037)
ID144 (P1.037)
ID216 (P2.031)
ID185 (P2.000)
ID189 (P2.004)
ID191 (P2.006)
ID208 (P2.023)
ID209 (P2.024)
ID210 (P2.025)
ID226 (P2.043)
ID227 (P2.044)
ID228 (P2.045)
ID229 (P2.046)
ID232 (P2.049)
ID257 (P2.098)
ID258 (P2.099)
ID260 (P2.101)
ID261 (P2.102)

**Extracted table (page 16, #1):**

| Tuning Mode | Inertia Estimation |  |
| --- | --- | --- |
|  |  | Manual Tuning |
| Tuning Mode 2 | Value of ID144 (P1.037) | ID144 (P1.037) ID216 (P2.031) |

<!-- page 17 -->

## Setting ID216 (P2.031) SysGainResponseLevel

Parameter ID216 (P2.031) SysGainResponseLevel is provided to tune the servo
system in an easy and user-friendly way. When using the fixed inertia ratio
and increasing this parameter, the servo bandwidth is also increased. If
resonance occurs, lower the bandwidth levels and it is possible to use
resonance mitigation techniques to resolve the resonance. Adjust the
bandwidth level according to the actual application.
For instance, if the setting value of ID218 (P2.031) was 30, the bandwidth level
can be reduced to 28. When adjusting the value of this parameter, its
corresponding parameters is adjusted by the servo system, such as ID185
(P2.000) PositionProportionalGain and and ID189 (P2.004)
VelocityProportional Gain.

> **Figure 91** — Settings for SysGainResponseLevel

Command
Response
Feedback
Position
Time
Before
Command
Response
Feedback
Position
Time
After
Servo
Bandwidth
Level Increases
84 Hz
26 Hz
ID216 (P2.031) = 30
ID216 (P2.031) = 20

## Inertia Ratio

ID144 (P1.037)

<!-- page 18 -->

Tuning Mode 2 in KNX5100C Software
You can use KNX5100C software for manual tuning in Mode 2 by choosing
Manual Tuning from the Function List and selecting Mode 2.

> **Figure 92** — Selecting Mode 2 Manual Tuning

The Smoothing and Filtering tab lets you configure the parameters related to
the Low Pass and Moving filters and S-curve, depending upon your configured
Operating mode. See Chapter 10 for details on filters and s-curves.

<!-- page 19 -->

Tuning in Manual Mode
Nested P-I Loop Gain Adjustment
There are two types of gain adjustment for the position and velocity loops: auto
and manual.
Auto adjustment is achieved when an Auto Tuning procedure is executed in
the Kinetix 5100 drive.
For a detailed description, refer to Autotuning on page 203.
Manual adjustment is when the inside-out tuning method is used and user
adjustments are made.
Manual Mode Tuning
Manual tuning can result in the optimum performance for complex
mechanisms. See Motion System Tuning, publication MOTION-AT005 for
more information regarding tuning in manual mode. You can reference this
publication for best practices that are common to inside-out tuning.
Table 81 lists parameters used in Manual Mode tuning.
The machinery stiffness and the application determines the selection of the
position and speed response frequency. Generally, for applications or
machines that require high speed and high precision, higher bandwidth is
required. However, increasing the bandwidth might cause resonance.

> **Table 81** — Manual Mode Tuning

ID 217 (P2.032)
Gain Adjustment
Mode Setting
Tuning Mode Inertia estimation
Parameter
Manual Tuning
Autotuning

Manual
Value of ID144 (P1.037)
ID144 (P1.037)
–
ID188 (P2.000)
ID189 (P2.004)
ID191 (P2.006).
ID208 (P2.023)
ID 209 (P2.024)
ID210 (P2.025)
ID211 (P2.026)
ID226 (P2.043)
ID227 (P2.044)
ID228 (P2.045)
ID229 (P2.046)
ID232 (P2.049)
ID257 (P2.098)
ID258 (P2.099)
ID260 (P2.101)
ID 261 (P2.102

**Extracted table (page 19, #1):**

| Tuning Mode | Inertia estimation |  |
| --- | --- | --- |
|  |  | Manual Tuning |
| Manual | Value of ID144 (P1.037) | ID144 (P1.037) |
|  |  | ID188 (P2.000) |
|  |  | ID189 (P2.004) |
|  |  | ID191 (P2.006). |
|  |  | ID208 (P2.023) |
|  |  | ID 209 (P2.024) |
|  |  | ID210 (P2.025) |
|  |  | ID211 (P2.026) |
|  |  | ID226 (P2.043) |
|  |  | ID227 (P2.044) |
|  |  | ID228 (P2.045) |
|  |  | ID229 (P2.046) |
|  |  | ID232 (P2.049) |
|  |  | ID257 (P2.098) |
|  |  | ID258 (P2.099) |
|  |  | ID260 (P2.101) |
|  |  | ID 261 (P2.102 |

<!-- page 20 -->

When the resonance frequency is unknown, you can gradually increase the
gain parameter values to increase the system response bandwith until you hear
the sound of resonance. Then, decrease the gain parameter values until the
resonance is removed. You can use the System Analysis test to diagnose
resonant frequencies, also there are many FFT (Fast Fourier Transform) tools
that you can use to diagnose resonant frequencies. You can use the drive
filtering described in Filter on page 256. Generally, if the dominant resonant
frequency is within the servo loop bandwidth, the gain values (and system
response) must be lowered.
The following are the descriptions of the gain adjustment parameters used
with different application types.

> **Table 82** — Gain Selection Based on Application Type

## Application Type

Applications
KPI
KVI
Integrator Hold
KVFF
KAFF
Basic
Basic smooth motion
X
Tracking
• Converting
• Printing
• Web
• Flying shear
• Coordinated motion
• Rotary knife
• Packaging
X
X
X
Point to Point
• Pick and place
• Indexing
• Robotics
• Palletizing
X
X
Constant Speed
• Conveyors
• Line shafts
• Cranks
X
X
Positioning
High performance position control
X
X
X

**Extracted table (page 20, #1):**

| Applications | K PI | K VI | Integrator Hold | K VFF |
| --- | --- | --- | --- | --- |
| Basic smooth motion |  |  |  | X |
| • Converting • Printing • Web • Flying shear • Coordinated motion • Rotary knife • Packaging |  | X |  | X |
| • Pick and place • Indexing • Robotics • Palletizing | X |  | X |  |
| • Conveyors • Line shafts • Cranks |  | X |  | X |
| High performance position control | X |  |  | X |

<!-- page 21 -->

Gain Adjustment of the Position Loop
The position loop gain should not be larger than the velocity loop gain.
There are three types of gain:
1.
Proportional gain: a larger gain increases the response of its loop.
2. Integral Gain: a larger gain increases the steady-state performance.
3.
Feed forward gain: reduces the deviation of phase delay.
By using inside-out tuning, we tune the inner loop (velocity) first. The
VelocityProportional Gain ID189 (P2.004) and VelocityIntegralGain
ID191 (P2.006) are in the Velocity (Speed) loop and once they are set, you can
manually change the outer loop (position) gains. The
PositionProportionalGain ID185 (P2.000), PositionIntegralGain ID235
(P2.053), and VelocityFeedforwardGain ID187 (P2.002).
The actual position curve changes from (1…3) with the increase in the KPP
value.

> **Table 83** — Relevant Parameters

Parameter
Name
ID185 (P2.000)
PositionProportionalGain (KPP)
ID235 (P2.053)
PositionIntegralGain (KPI)
ID187 (P2.002)
VelocityFeedForwardGain (KVFF)
fp
KPP = 2 × π × fp
Where:
fv = response bandwidth of speed loop (Hz)
fp = response bandwidth of position loop (Hz)
Example: If the desired position bandwidth is 20 Hz, then
 adjust the KPP (ID185, P2.000) to 125 (2 × π × 20 Hz = 125).
fv

≤
Position Feed
Forward Gain
ID187 (P2.002)
Position
Control Gain
ID185 (P2.000)
Max.
Speed Limit
ID160 (P1.055)
Speed Command
Position Control Unit
Position
Command
Position
Loop Izone
ID654 (P2.123)
Integrator
ID235 (P2.053)
Differentiator
Position Counter
Encoder
Smooth Constant
of Position Feed
Forward Gain
ID188 (P2.003)
Changing Rate
of Position
Control Gain
ID186 (P2.001)
Gain switching
condition and
method selection
ID212 (P2.027)
+
+
+
+
+
+
–

<!-- page 22 -->

ID185 (P2.000) PositionProportionalGain [KPP]
This parameter determines the response of the position loop. The larger the
KPP value, the higher the response frequency of the position loop. This lowers
following error and position error, and shortens the settling time. However, if
you set the value too high, it can cause instability. The calculation of position
loop frequency response is as follows:
ID 235 (P2.053) PositionIntegralGain (Kpi)
This gain may not be used (zero). This gain is used in positioning and tracking
applications to improve the steady-state positioning . Set this gain such that:
0 ≤ Kpi ≤ Kpp/4
See Nested P-I Loop Gain Adjustment on page 218 for more information.
ID187 (P2.002) VelocityFeedforwardGain [PFG]
This parameter can reduce the position error and shorten the settling time.
However, if you set the value too high, it might cause overshoot in positioning.
Frequency response bandwidth of position loop (Hz)
KPP
2
----------
=

<!-- page 23 -->

Gain Adjustment of Velocity Loop
Manual Mode
When the Gain Adjustment Mode parameter ID217 (P2.032) is set to 0, Manual
Mode tuning is used and you must set parameters VelocityProportionalGain
ID189 (P2.004), VelocityIntegralGain ID191 (P2.006), and
AccelFeedforwardGain ID192 (P2.007). More detail about adjusting the gains is
as follows:
•
Velocity loop gain: The higher the gain, the bigger bandwidth of velocity
loop response is.
•
Integral gain: Increasing this gain will increase the low frequency
rigidity and reduce the steady-state error. However, phase margin is
smaller. If this gain is set too high, the system stability will be reduced.
•
Feed forward gain: Diminish the deviation of phase delay.
Theoretically, a stepping response can be used to explain proportional gain
(KVP), integral gain (KVI), and feed forward gain (KVF). Speed over time
diagrams are shown below to illustrate the basic principle.
Feed
Forward Gain
ID192 (P2.007)
Speed
Control Gain
ID189 (P2.004)
Torque Constant
Reciprocal
1/KT
Speed Control Unit
Speed Integral
Compensation
ID191 (P2.006)
Integrator
ID235 (P2.053)
Load Initiator
ID144 (P1.037)
System Inertia J
(1 + ID144/P1.037)*JM
Motor Inertia
JM
Low-pass Filter
ID232 (P2.049)
Differentiator
Speed Estimator
Current Command
Torque Command
Encoder
Changing Rate
of Speed
Control Gain
ID190 (P2.005)
Gain switching
condition and
method selection
ID212 (P2.027)
+
+
+
+
+
+
+
+
–
Gain switching
condition and
method selection
ID212 (P2.027)

> **Table 84** — Relevant Parameters

Parameter
Name
ID189 (P2.004)
VelocityProportionalGain (KVP)
ID190 (P2.006)
VelocityIntegralGain (KVI)
ID191 (P2.007)
AccelFeedForwardGain (KAFF)

<!-- page 24 -->

## Timing Diagrams

ID189 (P2.004) VelocityProportionalGain [KVP]
This parameter determines the response of velocity loop. The larger the KVP
value, the higher the response frequency of the velocity loop and the lower the
velocity error. However, if you set the value too high, it could cause instability.
Typically, the response frequency of the velocity loop must be 4…6 times higher
than the response frequency of the position loop; otherwise, instability can
occur. The calculation of velocity loop frequency response is as follows:
JM= Motor Inertia; JL: Load Inertia; ID144 (P1.037): 0.1 (times)
When ID144 (P1.037) (auto estimation or manually set value) is equal to the real
inertia ratio (JL / JM), the real velocity loop frequency response is:
Impact of Speed Proportional Gain (KVP) Setting
The higher the KVP value, the larger the bandwidth, and
the speed increase time also shortens. However, if the
value is set too high, the phase margin is too small. The
effect is not as good as KVI for the steady-state error
but is better for the effect on following.
Impact of Speed Integral Gain (KVI) Setting
The higher the KVI value, the larger the low frequency
gain. It shortens the time for the steady-state error to
reduce to zero. However, it does not significantly reduce
the following error.
Impact of Acceleration Feedforward Gain (KVF) Setting
The closer the KVF value is to 1, the more complete the
forward compensation. The following error becomes
very small. But a KVF value that is set too high also
causes vibration.
fv
KVP
2
----------





P1.037

10

+

JL
JM
------
+
-------------------------------------








Hz

=
fv
KVP
2
-------------



Hz
=

<!-- page 25 -->

ID191 (P2.006) VelocityIntegralGain [KVI]
KVI is used to provide better tracking during motion. The larger the value, the
smaller the tracking error. Set the value as follows:
ID210 (P2.025) ResonanceSuppressionLowPassFilterTime [NLP]
A large inertia mismatch forces a reduction in the frequency response of the
velocity loop. Therefore, you must increase the KVP value to maintain the
response frequency. Increasing KVP value might cause machinery resonance.
Use this parameter to mitigate the resonance. The higher the value, the better
the capability for reducing high-frequency noise. However, if you set the value
too high, it can cause instability in the velocity loop and overshoot in
positioning. It is suggested that you set the value as follows:
ID211 (P2.026) AntiInterferenceGain [DST]
Use this parameter to increase the ability to resist external force and mitigate
overshoot during acceleration / deceleration. The default value is 0. This value
is not typically set when you use Manual Tuning. It is set when you use Mode 1,
Mode 2, or Autotune.
KVI (P2.006)

Kvi
Kvp 4




NLP (P2.025)
10000

Speed loop frquency response (Hz)

----------------------------------------------------------------------------------


<!-- page 26 -->

Manual Mode Tuning in KNX5100C Software
You can use KNX5100C software for manual mode tuning by choosing
Function List>Tuning>Manual Tuning.

> **Figure 93** — Selecting Manual Mode Tuning

The Smoothing and Filtering tab lets you configure the parameters related to
the Low Pass and Moving filters and S-curve, depending upon your configured
Operating mode. See Chapter 10 for details on filters and S-curves.

<!-- page 27 -->

Low Frequency Vibration Suppression in Position Mode
If the mechanism is compliant, the resonance can be present even when the
motor stops running after positioning command is executed completely. The
Low Frequency vibration suppression can reduce, or remove, the resonance.
The suppression range is between 1.0 Hz and 100.0 Hz. Both manual setting
and auto setting are available.
Automatic Setting
If you have difficulty determining the resonant frequency, you can enable the
auto low frequency vibration suppression function, which searches for the
resonant frequency. When parameter LowFreqVibrationSuppressionMode
ID139 (P1.029) is set to 1, the drive automatically searches for the resonant
frequency. This state remains active (ID139 (P1.029) =1) until a resonant
frequency is determined, or no vibration or resonance is detected. Once the
vibration suppression is applied, and the resonance is removed, this parameter
is reset to 0, and the resonant frequency that was obtained, is stored
inLowFreqVibrationSuppression1Frequency (ID135, P1.025). We recommend
that you use a LowFreqVibrationSuppressionGain ID138 (P1.028) of 1. The
second frequency suppression is used when multiple resonances are present. A
second resonance suppression operates the same way the first resonance
suppression operates.
LowFreqVibrationDetectionLevel ID140(P1.030) is used when the
LowFreqVibrationSuppressionMode =1. The detection level value is the size of
the resonance in encoder counts. Setting this value too large can result in
misrepresenting a resonance for actual motion or missing the resonance
entirely. Setting a value too small, typical motor current noise can be
misdiagnosed as a resonance. The default for this value is 8000 counts. This
value changes based on your E-Gear scaling value.
You can use any FFT tool to diagnose the resonant frequency that exists on the
mechanism.

> **Table 85** — Relevant Parameters

Parameter
Name
ID135 (P1.025)
LowFreqVibrationSuppression1Frequency
ID136 (P1.026)
LowFreqVibrationSuppression1Gain
ID137 (P1.027)
LowFreqVibrationSuppression2Frequency
ID138 (P1.028)
LowFreqVibrationSuppression2Gain
ID139 (P1.029)
LowFreqVibrationSuppressionMode
ID140 (P1.030)
LowFreqVibrationDetectionLevel
IMPORTANT
When the detection level is set too small, noise might be regarded as
low-frequency vibration.

<!-- page 28 -->

## Mechanical Resonance Suppression

Figure 94 shows an overview of the procedure to suppress mechanical
resonance. Five sets of notch filters are provided to suppress mechanical
resonance. All five sets can be set to auto-resonance suppression (set by
parameter ID230 (P2.047) ResonanceSuppressionConfig) and manual
adjustment.

> **Figure 94** — Mechanical Resonance Suppression Flowchart

Use the analytic tool provided by KNX5100C
to display the point of resonance.
The servo issues the command to accelerate
and decelerate the motor alternatively.
High-frequency
resonance?
No
Yes
Complete
Save the value of resonance frequency to
ID208 (P2.023) and set ID209 (P2.024) to 4.
Resonance
eliminated?
Yes
No
Increase the value of
ID209 (P2.024)
Tuning
Completed
Use the analytic tool provided by KNX5100C
software to display the point of resonance.
The servo drive issues the command to
accelerate and decelerate the motor
alternatively.
Save the value of resonance frequency to ID208 (P2.023)
and set ID209 (P2.024) to 4.
Increase the value of ID209
(P2.024)
Complete
Highfrequency
Resonance
eliminated?
Tuning
Complete
No
Yes
No
Yes

<!-- page 29 -->

## System Analysis

The Kinetix 5100 drive can create a bode plot by using the System Analysis test.
The bode plot lets you see the frequency response of a system. By applying an
input signal and comparing it to an output signal, we can see a variation in the
magnitude and shift in phase of that signal. The Bode plot shows these
variations as part of the System Analysis test in the KNX5100C software.
Phase Margin and Gain Margin
The phase margin is the amount of open-loop change required to make a
closed loop system unstable. In other words, the available Phase (degree)
before the system becomes unstable. The phase margin is measured at the 0 dB
Magnitude point of the bode plot. When the Phase (degree) reaches -180, the
signal flips, which causes instability. The Phase Margin is the available phase
from the System's phase measurement to the -180-degree point. If the Phase
margin is close to zero (or negative), the system is susceptible to ringing and
overshoot, which is shown in Figure 95.
The gain margin is a measure of gain amplification used to reduce error
between the input and output signals. Gain Margin is the available gain before
the system becomes unstable. When the Phase (deg) reaches -180 degrees, the
signal will flip causing instability. The Gain Margin is measured at this point of
the bode plot and is shown as the available gain until the 0 dB point is reached,
which is shown in Figure 95.

> **Figure 95** — Phase and Gain Margin

<!-- page 30 -->

Guidelines for Gain/Phase Margin
•
Gain and phase margin must be positive values.
•
Phase margin range between 30…60 is considered acceptable. A value
lower than 30 will lead to instability. This means you must lower gains to
raise the Phase margin, which means you lower the performance
(Bandwidth). A value greater than 45 means you have a stable system,
and you can increase the gains further, which means you raise the
performance (Bandwidth).
•
Gain margin range between 5…10 dB is considered acceptable. This
should be positive, if its negative, the system is unstable.
•
If the response curves never cross 0 dB or -180 degrees, the system has
low performance, but is likely stable.
•
The gain and phase margin values vary by machine (mechanism) type
and by the inertia reflected to the motor shaft.

> **Figure 96** — System Analysis Tool - Bode plot

Method for Using the System Analysis Tool
You can use the System Analysis tool to develop a bode plot; and ultimately
characterize your system. This type of information is useful with complex
mechanisms that are difficult to tune.
Before any tuning is attempted, a baseline System Analysis test should be
performed so you can quantify the improvements from any tuning changes
(default, autotune, or manual tuning). Once tuning changes are made, the
System Analysis can be used again to verify the response and see if it has
improved.
The System Analysis can also show resonances that exist with your system. If
you are not able to use an FFT (Fast Fourier Transform) tool, you can use the
System Analysis tool.

<!-- page 31 -->

## System Analysis Tool

The System Analysis can be accessed from KNX5100C software Function
List>Settings>System Analysis. This Analysis will attempt to estimate the
Phase Margin and Gain Margin for your load.
Analysis Type:
•
Speed Open-Loop - Speed control is performed open-loop. The bode plot
analysis is based on Kvp (speed loop proportional gain) and Kvi (speed
loop integral gain). This is the method that is generally used for good
results.
•
System Module - System module attempts to analyze your mechanism.
This analysis type will not provide gain or phase margin; instead, it
attempts to provide a mechanical representation of the system and will
not be impacted by your existing gains. This method should not be used
on a compliant mechanism as variation of these mechanisms cannot
typically be transferred to the bode plot. When you need to analyze the
allowable command response or resonance, or if abnormal vibration
occurs that cannot be removed, the System Module can be used to
analyze the mechanism.
•
Rated Current (%) - This is the current level to use for the test. This can
be set up to 300%. The larger the load inertia of the mechanism, the
higher this setting should be. However, the setting is typically below
100%. The test results may be incorrectly reported using values that are
too large (or too small).
•
Enable Low Frequency Analysis - This is typically used when you want the
analysis to be focused on low frequency response, ie: within the servo
loop bandwidth. This is typically not used.
Execute the System Analysis
When the system is ready to be tested, press Run. The test will generate small
oscillations at different frequencies. When the test is complete, you can click
Ok and the Bode plot is generated. The figure below shows the usable
Bandwidth (-3dB point - green area of graph). The system analysis test shows a
grey line to show the -3dB point of your system.
•
Line A - Figure 96 on page 229 shows the Line-A plot (Red curves). These
curves are the measurements after the System Analysis test is performed.
After each test is executed, these curves are the only values that change.
•
Line B - Figure 96 on page 229 shows the Line-B plot. These curves are the
measurements before the System Analysis is performed. To transfer
these measurements from Line-A to Line-B, click >>.

<!-- page 32 -->

## System Analysis Results

Now that the bode plot is understood, we can show some examples from the
test.
The larger the magnitude of gain above 0 dB, the better the ability to track
(command vs. actual). Generally, a larger magnitude means the command is
being tightly followed by the motor and a higher performing system. If gains
are changed, you can execute the System Analysis to see the new results from
the bode plot, which tells you if the system response has improved.
Once we pass the crossover frequency, we want a high level of attenuation on
the signal, which indicates good noise rejection.
The bode plot is a good way to see resonance that occurs either naturally or
because of a problem in your system. It is common to use an FFT (Fast Fourier
Transform) tool that can help diagnose the frequency of the resonance (see
MOTION-AT005). If such a tool is not available, you can also see resonance
(and anti-resonance) frequencies from the bode plot.

<!-- page 33 -->

Notes:

<!-- page 34 -->

The following sections describe the operation of each mode, including the
mode structure, command source, selection and processing of the command,
and gain adjustment.
Topic
Page
Select Operation Mode and Direction Control

Position Control

Speed Mode

Torque Mode

Filter

Speed and Torque Limit Functions

Dual and Multi-modes

IO Mode

Analog Outputs and Monitoring

This manual links to Kinetix® 5100 Servo Drive Fault Codes Reference
Data, publication 2198-RD001, for fault codes and Kinetix 5100 Servo
Drive Parameters Reference Data, publication 2198-RD002, for
parameters. Download the spreadsheets now for offline access.

> **Table 86** — Kinetix 5100 Drive Control Modes

## Control Mode

Short Name
Description
Position mode (I/O terminal block
input)
PT
This mode is sometimes referred to as Pulse Train Output or Step and Direction. The servo drive receives the Position
command and commands the motor to run to the target position. The Position command is communicated through the I/
O terminal block and the signal type is pulse.
Position mode (register input)
PR
This mode contains the drive indexing capabilities. The servo drive receives the Position command and commands the
motor to run to the target position. Position commands are issued from the program registers (99 sets in total). You can
select the register number with binary-weighted DI signals or through communication.
Speed mode
S
The servo drive receives the Speed command and commands the motor to run at the target speed. The Speed command
is issued from preset speed internal registers (3 sets in total) or by analog voltage (-10V…+10V) that is communicated
through the analog inputs on the I/O terminal block. You can select the command with binary-weighted DI signals.
Speed mode (no analog input)
Sz
The servo drive receives the Speed command and commands the motor to run at the target speed. The Speed command
can only be issued from the preset speed internal registers (4 sets in total, one is fixed at 0). You can select the
command with binary-weighted DI signals.
Torque mode
T
The servo drive receives the Torque command and commands the motor to run using the target torque. The Torque
commands can be issued from the preset torque internal registers (3 sets in total) and by analog voltage (-10V…+10V)
that is communicated through the I/O terminal block. You can select the command with binary-weighted DI signals.
Torque mode (no analog input)
Tz
The servo drive receives the Torque command and commands the motor to run using the target torque. The Torque
command can only be issued from the preset torque internal registers (4 sets in total, one is fixed at 0). You can select
the command with binary-weighted DI signals.
IO mode
IO
The servo drive receives commands from the Logix controller through the EtherNet/IP network Class 1 connection.
Commands are issued through the Add-On Profile (AOP) and uses the Add-On Instruction instructions in the Logix
Designer application.
