# Chapter 14: Absolute Position Recovery

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `15_Ch14_Absolute_Pos_Recovery.pdf` (8 pages)_

<!-- page 1 -->

This section introduces the absolute positioning feature of the Kinetix® 5100
drive, the steps to set up the feature, and the procedures for initializing and
operating the feature for the first time.
System Requirements
A complete absolute servo system should include a Kinetix 5100 servo drive,
motor with absolute feedback device, and when a Kinetix TLP motor is present,
a backup battery box containing a battery (see page 73 for battery
specifications). When the battery supplies power to the motor feedback device,
the encoder is able to retain position throughout a power cycle event.
Moreover, an absolute type encoder records the motor position, including
when the motor shaft is rotated while the power is removed. A configured
absolute servo system must work with an absolute encoder. If the system is
configured with an incremental encoder and the related parameters of an
absolute system are enabled, E 069 Wrong motor encoder error occurs.
Compatible Servo Motors
The following motors are compatible with Kinetix 5100 drives. They are listed
with variables, but include the appropriate encoder (V/E/M/S/D) types. See the
Motor and Auxiliary Feedback Configurations on page 20 for information
about encoder types.
Topic
Page
System Requirements

Compatible Servo Motors

Install the Battery

System Initialization

This manual links to Kinetix® 5100 Servo Drive Fault Codes Reference
Data, publication 2198-RD001, for fault codes and Kinetix 5100 Servo
Drive Parameters Reference Data, publication 2198-RD002, for
parameters. Download the spreadsheets now for offline access.
Kinetix TL Servo Motor(1)
(1)
Battery backup is required for -B encoders.
TL-A1xxx-B
TL-A2xxx-B
TL-A25xxx-B
TL-A4xxx-B

<!-- page 2 -->

All MPL absolute encoders provide absolute positioning without requiring a
battery backup and when configured as an Absolute system.
Kinetix TLY Servo Motor
TLY-A1xxx-B
TLY-A2xxx-B
TLY-A25xxx-B
TLY-A3xxx-B
Kinetix TLP Servo Motor (1)
(1)
Battery backup is required for -D encoders when the system is
configured as an Absolute system.
TLP-A046-xxx-D
TLP-A/B070-xxx-D
TLP-A/B090-xxx-D
TLP-A100-xxx-D
TLP-A/B115-xxx-D
TLP-A/B145-xxx-D
TLP-A/B200-xxx-D
TLP-A/B235-xxx-D
Kinetix MPL Low Inertia Motors (200V-class)
MPL-A15xxx-V/E
MPL-A2xxx-V/E
MPL-A3xxx-M/S
MPL-A4xxx-M/S
MPL-A45xxx-M/S
MPL-A5xxx-M/S
Kinetix MPL Low Inertia Motors (400V-class)
MPL-B15xxx-V/E
MPL-B2xxx-V/E
MPL-B3xxx-M/S
MPL-B4xxx-M/S
MPL-B45xxx-M/S
MPL-B5xxx-M/S
MPL-B6xxx-M/S
MPL-B8xxx-M/S
MPL-B9xxx-M/S
Kinetix MPM Medium Inertia Motors (200V-class)
MPM-A115xx-M/S
MPM-A130xx-M/S
MPM-A165xx-M/S
MPM-A215xx-M/S

<!-- page 3 -->

Install the Battery
When using a Kinetix TLP motor, a battery is required to make absolute
position retention operate properly. For instructions on the motor feedback
cable preparation, see Wire the Motor Feedback Connector on page 100.
If you are using the pre-assembled TLP motor feedback cables, see the
Kinetix 5100 Feedback Battery Box Installation Instructions,
publication 2198-IN022, for information on how to install or replace a battery
box, install a battery, and prepare a feedback cable for a battery box
installation.
For information on wiring flying-lead feedback cables, see Chapter 4. That
chapter provides information on motor feedback cables and provides wire
terminations for encoder signals to the motor feedback (MFB) connector on
Kinetix 5100 drives.
Kinetix MPM Medium Inertia Motors (400V-class)
MPM-B115x-M/S
MPM-B130x-M/S
MPM-B165x-M/S
MPM-B215x-M/S
Kinetix MPF Food Grade Motors (200V-class)
MPF-A3xxx-M/S
MPF-A4xxx-M/S
MPF-A45xxx-M/S
MPF-A5xxx-M/S
Kinetix MPF Food Grade Motors (400V-class)
MPF-B3xxx-M/S
MPF-B4xxx-M/S
MPF-B45xxx-M/S
MPF-B5xxx-M/S
Kinetix MPS Stainless Steel Motors (200V-class)
MPS-A3xxx-M/S
MPS-A45xxx-M/S
Kinetix MPS Stainless Steel Motors (400V-class)
MPS-B3xxx-M/S
MPS-B45xxx-M/S
MPS-B5xxx-M/S
Installation of a battery within the battery box is required for using the Absolute
system with Kinetix TLP motors.

<!-- page 4 -->

> **Figure 234** — Battery box that is connected to Kinetix 5100 system

## System Initialization

When you initialize the absolute system for the first time, a fault (A 060
Absolute Position Lost) occurs because the axis has not been homed. Clear the
fault by configuring homing and homing the axis successfully. See Homing on
page 298 to configure and perform homing on your axis. Additional causes of
an A060 (Absolute Position Lost) alarm can be:
•
Backup battery failure (insufficient charge)
•
Main power supply failure
•
Incorrect motor feedback detected
When the PUU feedback value range exceeds -214748346…214783647, A289
(Feedback position (PUU) counter overflow) alarm occurs.
When your system is configured as incremental or absolute and the PUU
counts exceeds the range -2147483648…2147483647, the A289 alarm occurs. This
alarm can occur with a constant movement application (like a conveyor). To
avoid this alarm, you can set DataReadUnitSelection ID243 (P2.070) bit 2 = 0.
See Figure 235.

> **Figure 235** — Overflow Warning

The system that is shown is an example.
Your system can be different.
MFB Connector (Female)
MFB Connector (Male)

<!-- page 5 -->

1.
Initialize the absolute coordinates (Home the axis).
When the coordinate setting is complete, A 06A (or A 060) is
automatically cleared. There are three ways for you to initialize the
Kinetix 5100 drive: by using the Enable Homing Input (DI), setting the
parameters below, or in IO Mode by using the AOI Homing Command
(raC_xxx_K5100_MAH).
2. When the system is power cycled, the absolute position can be accessed
using KNX5100C software or via Ethernet/IP communication.
Based on the setting of ID243 (P2.070), the Kinetix 5100 drive can select
either the PUU or the pulse value, within one turn.
Pulse Number
When the motor is running in the clockwise direction, MultiTurnAbsPosition
ID110 (P0.051) is expressed as a positive value. When the motor runs in the
counterclockwise direction, MultiTurnAbsPosition ID110 (P0.051) is expressed
as a negative value.
Homing Parameters
Name
ID269 (P5.004)
HomingMode
ID298 (P5.005)
HomingSpeed
ID299 (P5.006)
HomingCreepSpeed
ID397 (P6.000)
HomingSetting
ID398 (P6.001)
HomePosition

> **Table 132** — Relevant Parameters and Faults

Fault code
Name
A 060
Absolute Position Lost
A 06A
Absolute Position is not Initialized
A 289
Feedback Position [PUU] Counter Overflow
Parameters
Name
ID110 (P0.051)
MultiTurnAbsPosition
ID117 (P1.001)
ControlMode
ID151 (P1.044)
GearRatioSlaveN1
ID152 (P1.045)
GearRatioMasterCounts
ID194 (P2.009)
DIDebounceTime
ID243 (P2.070)
DataReadUnitSelection
ID398 (P6.001)
HomePosition

<!-- page 6 -->

In these examples, an example Kinetix TLP motor with the encoder resolution
of 16777216 (single turn) is used. In addition to the cycle counter
(MultiTurnAbsPosition ID110 P0.051), there are 16,777,216 pulses (0…16777215)
in one rotation. Pay attention to the motor's running direction.
Pulse number = m (cycle number) x 16777216 + pulse number (0 … 16777215). The
conversions between pulse number and PUU are as follows:
When the rotation direction is defined as clockwise (CW) in ID117
(P1.001.Z=0), then the PUU number =
pulse number × [ID152 (P1.045) / ID151 (P1.044)] + ID398 (P6.001).
When the rotation direction is defined as counter-clockwise (CCW) in ID117
(P1.001.Z=1), then the PUU number =
(-1) × pulse number × [ID152 (P1.045) / ID151 (P1.044)] + ID398 (P6.001).
0…16777215
0…16777215
0…16777215
0…16777215
0…16777215
0…16777215
16777215 - 1
Pulse
MultiTurnAbsPosition
ID110 (P0.051)
Pulse number
in one turn
ID111 (P0.052)

<!-- page 7 -->

## PUU Number

The PUU number is a signed 32-bit value. When the motor is running in the
forward direction, the PUU number increases; when the motor is running in
the reverse direction, the PUU number decreases. The forward direction is
determined in KNX5100C software Function List>Settings>General Setting:
Rotation Direction ID117 (P1.001 Z). The following example shows how the
overflow is used in the Kinetix 5100 drive.
Example:
When the E-Gearing Ratio (Position scaling) is set for 16777216/100000, the
motor needs 100,000 PUU to complete a motor revolution. To determine the
maximum number of motor revolutions: 2,147,483,647 ÷ 100,000 = 21,474.8,
when the motor exceeds 21,474.8 motor revolutions in the forward direction,
A 289 alarm occurs.
Initializing the Absolute Coordinates with Parameters
It is not common to reset the absolute coordinates with parameters. It is
typical to reset the coordinates by using the homing operation in PR Mode, or
raC_xxx_K5100_MAH Add-On Instruction in IO Mode. The Absolute
Coordinates are write protected. In the remote case that you must perform an
initialization without the ability to use these two methods, the sequence to
initialize the absolute coordinates is:
•
ForceFunction (ID193, P2.008) = 271
•
ResetAbsolutePosition (ID244, P2.071) = 1
After initializing the absolute coordinate system (homing is completed), any
change to ID117 (P1.001.Z) or the E-Gear ratio [ID151 (P1.044) and ID152 (P1.045)]
changes the original setting of the absolute coordinate system. The system must
be re-initialized (Homed).

<!-- page 8 -->

Notes:
