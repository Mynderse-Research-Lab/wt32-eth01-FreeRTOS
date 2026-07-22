# Chapter 16: Troubleshooting

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `17_Ch16_Troubleshooting.pdf` (8 pages)_

<!-- page 1 -->

## Safety Precautions

Observe the following safety precautions when you troubleshoot your
Kinetix® 5100 drive.
Status Indicators
There are three status indicators on front of the Kinetix 5100 drive: module
(MOD), network (NET), and CHARGE.
Topic
Page
Safety Precautions

Status Indicators

View Status and Faults

Drive Stopping Behavior

Clear Faults

General Troubleshooting

ATTENTION: DC bus capacitors may retain hazardous voltages after input
power has been removed. Before working on the drive, measure the DC bus
voltage to verify it has reached a safe level or wait the full time interval
listed on the drive warning label. When the DC bus voltage is above 50V DC,
the charge LED light on the drive must be on. Failure to observe this
precaution could result in severe bodily injury or loss of life.
Do not attempt to defeat or override the drive fault circuits. You must
determine the cause of a fault and correct it before you attempt to operate
the system. If you do not correct a drive or system malfunction, it could
result in personal injury and/or damage to the equipment as a result of
uncontrolled machine system operation.
Test equipment (such as an oscilloscope or chart recorder) must be properly
grounded. Failure to include an earth ground connection could result in a
potentially fatal voltage on the oscilloscope chassis.

> **Table 141** — MOD LED Status Indicators

## Operation Mode

LED Color
Status
Description
I/O
Steady off
No power
If no power is supplied to the device, the module status indicator shall be steady off.
Steady green
Device operational
If the device is operating correctly (IO connection is established successfully), the module status
indicator shall be steady green.
Flashing green
Standby
If the device has not been configured, the module status indicator shall be flashing green.
Flashing red
Major recoverable fault or
Minor recoverable fault
If the device has detected a major or minor recoverable fault, the module status indicator shall be
flashing red.
IMPORTANT: An incorrect or inconsistent configuration could be considered a major recoverable
fault.
Steady red
Major unrecoverable fault
If the device has detected a major unrecoverable fault, the module status indicator shall be steady
red.
Flashing green/red
Self-test
While the device is performing its power up testing, the module status indicator shall be flashing
green/red.

**Extracted table (page 1, #1):**

| LED Color | Status |
| --- | --- |
| Steady off | No power |
| Steady green | Device operational |
| Flashing green | Standby |
| Flashing red | Major recoverable fault or Minor recoverable fault |
| Steady red | Major unrecoverable fault |
| Flashing green/red | Self-test |

<!-- page 2 -->

View Status and Faults
Drive Fault Code Display
The drive display panel indicates a fault or warning on the display.
For more information on faults and warnings, see Clear Faults on page 454.
Standalone
Steady off
No power
If no power is supplied to the device, the module status indicator shall be steady off.
Steady green
Device operational
If the device is operating correctly (after the drive boots successfully), the module status indicator
shall be steady green.
Flashing red
Major recoverable fault or
Minor recoverable fault
If the device has detected a major or minor recoverable fault, the module status indicator shall be
flashing red.
IMPORTANT: An incorrect or inconsistent configuration could be considered a major recoverable
fault.
Steady red
Major unrecoverable fault
If the device has detected a major unrecoverable fault, the module status indicator shall be steady
red.
Flashing green/red
Self-test
While the device is performing its power up testing, the module status indicator shall be flashing
green/red.

> **Table 141** — MOD LED Status Indicators (Continued)

## Operation Mode

LED Color
Status
Description

> **Table 142** — NET LED Status Indicators

## LED Color

Status
Description
Steady off
No power or no IP address
The device is powered off, or is powered on but with no IP address configured.
Steady green
Connected
An IP address is configured, at least one CIP™ connection (any transport class) is
established, and an Exclusive Owner connection has not timed out.
Flashing green
Not connected
An IP address is configured but no CIP connections are established, and an Exclusive Owner
connection has not timed out.
Flashing red
Connection timeout
An IP address is configured, and an Exclusive Owner connection where this device is the
target has timed out.
Steady red
A duplicate IP address has been identified The device has detected that (at least one of) its IP address is already in use.
Flashing green/red
Self-test
While the device performs its power up testing.

> **Table 143** — CHARGE LED Status Indicators

## LED Color

Status
Description
Steady off
No power
When no power is supplied to the device.
Steady orange
DC bus voltage operational
When the DC bus voltage is above 50V DC.
This manual links to Kinetix® 5100 Servo Drive Fault Codes Reference
Data, publication 2198-RD001, for fault codes and Kinetix 5100 Servo
Drive Parameters Reference Data, publication 2198-RD002, for
parameters. Download the spreadsheets now for offline access.
Fault
Warning

**Extracted table (page 2, #1):**

| LED Color | Status |
| --- | --- |
| Steady off | No power |
| Steady green | Device operational |
| Flashing red | Major recoverable fault or Minor recoverable fault |
| Steady red | Major unrecoverable fault |
| Flashing green/red | Self-test |

<!-- page 3 -->

Monitoring Status in KNX5100C Software
In KNX5100C software, you can monitor the status of the servo drive.
To monitor the servo drive, perform the following steps.
1.
In the Function List of the KNX5100C software, click Monitoring Status.
On the Monitoring Items tab, the monitored items and their setting
values are shown.
On the Select Monitoring Items tab, you can select what items to
monitor.

<!-- page 4 -->

2. Click Save Changes if any changes are made, which are shown
subsequently on the Monitoring Items tab.
Fault Information in the KNX5100C Software
In the KNX5100C software, you can use Fault Information to identify any
faults of the servo drive.
To identify any faults, perform the following steps.
1.
In the Function List of the KNX5100C software, click Fault Information.
On the Fault Information tab, the most current fault is shown with detailed
information, including possible causes and subsequent corrective actions.
2. Click either available button for the following reasons:
• Click Show Faults to refresh the current fault information.
• Click Fault Reset to reset fault and remove the current list of fault
information if fault is cleared.
IMPORTANT
Click Run to run the Monitoring Status function. If you do, the same
button toggles to Stop, which you then click when you want to stop
the monitoring status.

<!-- page 5 -->

Click the Fault History tab to see the latest fault codes recorded in the
servo drive.
Fault and Status Information in Studio 5000 Application
Major faults and minor faults (warnings) are shown on the Module Info tab in
the Studio 5000 Logix Designer® application.
To view faults the Studio 5000 Logix Designer application, perform the
following steps.
1.
Under the hierarchy, click Module Info.
Any fault is shown in the Status field.
2. If the fault is recoverable, click Reset Module to clear the fault.

<!-- page 6 -->

## Drive Stopping Behavior

A fault (code E nnn) triggers the shutdown (servo off) behavior of the drive.
Parameter ID675 (P1.032), MotorStopMode, configures the drive stopping
behavior.
Clear Faults
The two methods for clearing faults depends on the type.
Warnings (A nnn): When the condition that caused the warning is corrected,
the warning clears automatically. Warnings can also be referred to as Alarms.
Faults (E nnn): Clear the fault by one of the following methods:
•
For a major unrecoverable fault, cycle power to clear the fault. If the issue
persists, contact your distributor or Rockwell Automation
representative.
•
For a major recoverable fault:
- Click Reset Module in the Studio 5000 Logix Designer software. For
more information, see Fault and Status Information in Studio 5000
Application on page 453.
- By using the DI.ARST signal.
- By clicking Fault Reset in the Fault Information dialog of the
KNX5100C software (see Fault Information in the KNX5100C Software
on page 452).
- Set ID101 (P0.001) FltWarnCode to 0.
- By using the raC_xxx_K5100_MAFR add-on instruction in the Studio
5000 Logix Designer software (for more information on add-on
instructions, see Appendix C, Use Add-On Instructions).
MotorStopMode Setting
Drive Behavior
0000 (default)
Dynamic brake stop - This stop type is similar to current decel (available
torque used to decelerate motor) with an internal regenerative type of
stop that stops the motor as quickly as possible.
0010
Disable and coast.
0020
Use dynamic brake first, then let the motor run freely once the speed is
slower than the value of parameter ID145 (P1.038), ZeroSpeedWindow.
0030
Enable vertical load control.

<!-- page 7 -->

## General Troubleshooting

The following conditions do not always result in a fault code, but can require
troubleshooting to improve performance.

> **Table 144** — Troubleshooting

Condition
Potential Cause
Possible Resolution
Axis or system is
unstable.
The position feedback device is incorrect or open.
Check wiring.
Unintentionally in torque mode.
Check to see what primary operation mode was programmed.
Motor tuning limits are set too high.
Run Tune by using KNX5100C software or the LED panel.
See Tuning Process on page 200.
Position loop gain or position controller acceleration/deceleration
rate is improperly set.
Run Tune by using KNX5100C software or the LED panel.
See Tuning Process on page 200.
Improper grounding or shielding techniques are causing noise to be
transmitted into the position feedback or velocity command lines,
causing erratic axis movement.
Check wiring and ground.
Motor select limit is incorrectly set (servo motor is not matched to
axis module).
• Check setups.
• Run Tune in the Logix Designer application.
Mechanical resonance.
Notch filter or output filter can be required.
You cannot obtain
the motor
acceleration/
deceleration that
you want.
Torque Limit limits are set too low.
Verify that torque limits are set properly.
Incorrect motor selected in configuration.
Select the correct motor.
Run Tune by using KNX5100C software or LED panel.
The system inertia is excessive.
• Check motor size versus application need.
• Review servo system sizing.
The system friction torque is excessive.
Check motor size versus application need.
Available current is insufficient to supply the correct acceleration/
deceleration rate.
• Check motor size versus application need.
• Review servo system sizing.
Acceleration limit is incorrect.
Verify limit settings and correct them, as necessary.
Velocity limits are incorrect.
Verify limit settings and correct them, as necessary.
The motor is operating in the field-weakening range of operation.
Reduce the commanded acceleration or deceleration.
Motor does not
respond to a
command.
The axis cannot be enabled until stopping time has expired.
Disable the axis, wait for 1.5 seconds, and then enable the axis.
The motor wiring is open.
Check the wiring.
The motor cable shield connection is improper.
• Check feedback connections.
• Check cable shield connections.
The motor has malfunctioned.
Repair or replace the motor.
The coupling between motor and machine has broken (for example,
the motor moves but the load/machine does not).
Check and correct the mechanics.
Primary operation mode is set incorrectly.
Check and properly set the limit.
Velocity or torque limits are set incorrectly.
Check and properly set the limits.
Brake connector not wired.
Check the brake wiring.
Presence of noise
on command or
motor feedback
signal wires.
Recommended grounding per installation instructions have not been
followed.
• Verify grounding.
• Route wire away from noise sources.
• Refer to System Design for Control of Electrical Noise, publication
GMC-RM001.
Line frequency can be present.
• Verify grounding.
• Route wire away from noise sources.
Variable frequency can be velocity feedback ripple or a disturbance
caused by gear teeth or ball screw. The frequency can be a multiple
of
the motor power transmission components or ball screw speeds,
resulting in velocity disturbance.
• Decouple the motor for verification.
• Check and improve mechanical performance, for example, the
gearbox or the ball screw mechanism.

<!-- page 8 -->

No rotation
The motor connections are loose or open.
Check motor wiring and connections.
Foreign matter is lodged in the motor.
Foreign matter is lodged in the motor. Remove foreign matter.
The motor load is excessive.
The motor load is excessive. Verify the servo system sizing.
The bearings are worn.
The bearings are worn. Return the motor for repair.
The motor brake is engaged (if supplied).
• Check brake wiring and function.
• Return the motor for repair.
The motor is not connect to the load.
Check the coupling.
Motor overheating
The duty cycle is excessive.
Change the command profile to reduce acceleration/deceleration, or
increase time.
The rotor is partially demagnetized causing excessive motor current. Return the motor for repair.
Abnormal noise
Motor tuning limits are set too high.
Run Tune by using KNX5100C software or the LED panel. See Tuning
Process on page 200.
Loose parts are present in the motor.
• Remove the loose parts.
• Return motor for repair.
• Replace motor.
Through bolts or coupling is loose.
Tighten bolts.
The bearings are worn.
Return motor for repair.
Mechanical resonance.
Notch filter can be required.
Erratic operation–
Motor locks into
position, runs
without control, or
with reduced
torque.
Motor power phases U and V, U and W, or V and W reversed.
Check and correct motor power wiring.

> **Table 144** — Troubleshooting (Continued)

Condition
Potential Cause
Possible Resolution
