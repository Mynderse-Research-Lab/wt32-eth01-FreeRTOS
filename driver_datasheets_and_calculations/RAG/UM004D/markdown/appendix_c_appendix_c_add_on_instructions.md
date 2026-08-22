# Appendix C - Add-On Instructions

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `20_AppC_AddOn_Instructions.pdf` (37 pages)_

<!-- page 1 -->

## Appendix C

Use Add-On Instructions
Use of the Add-On
Instruction Library
When the Kinetix® 5100 drive is configured for IO Mode operation and is used
with Studio 5000 Logix Designer®, the use of the pre-defined Add-On Profile
(AOP) and Add-On Instruction library provide an easy way to program your
simple motion control application.
The Kinetix 5100 drive was designed to use the Add-On Instruction library
since its launch with major revision 1.xx. This library has evolved to include a
Device Object handler (Figure 256) designed to provide a robust method to
control the read/write functions of the drive assemblies. This Device Object
handler has many advantages including:
•
Integration with the Power Device Library and its framework for
programming
•
Optional HMI faceplate
•
Creation of Position Units
•
Removal of user created logic (CPS) and interlocks that allow the Motion
Add-On Instructions to operate effectively
Table 150 shows the differences between the Add-On Instruction libraries and
the use of the device object handler.
Topic
Page
Use of the Add-On Instruction Library

Download the Add-On Instruction Files and Data Types

Import the Add-On Instruction Files and Data Types (version 1.xxx)

Dvc Add-On Instruction Configuration (version 1.xxx)

Opr Add-On Instruction Configuration

Add-On Instruction Details

Error Codes

IMPORTANT
Induction and linear motors are not supported with the Add-On
Instruction library.

> **Table 150** — Device Object Handler Availability

## Kinetix Firmware Major Rev

Device Handler Add-On Instruction Add-On Instruction Library
1.xx
NO
raC_Dvc_K5100_xxx
2.xx
YES
raC_Opr_K5100_xxx

<!-- page 2 -->

## Appendix C Use Add-On Instructions

Kinetix 5100 Drive Device Object Add-On Instructions
The Device Object Add-On Instruction comes with the Add-On Instruction
library that is downloaded from the PCDC as part of the Power Device Library.
This Add-On Instruction works together with a commonly developed faceplate
created with the Studio 5000 View Designer® application. Each Kinetix 5100
drive requires a unique instance of the Device Object Add-On Instruction. This
Add-On Instruction provides a single software interface that is used by each
drive that commands the drive and provides the drive status you can use with
your application logic.

> **Figure 256** — Device Object Add-On Instruction

IMPORTANT
If you are using the Kinetix 5100 drive with major revision 2.xx, you must
use the Device Handler Add-On Instruction with the Opr Add-On
Instruction Library. Motion Add-On Instruction versions 1.xx and 2.xx are
not interchangeable and cannot be combined to perform motion
control.
The Add-On Instruction library containing raC_Dvc instructions is
available for legacy applications. They can be downloaded from the
Rockwell Automation Product Compatibility Download Center (PCDC)
website; use keyword Kinetix 5100.
For new applications that are using the Kinetix 5100 drive with major
revision 2.xx, use the Device Object handler Add-On Instruction.
These Add-On Instructions can be downloaded from the PCDC website;
use keyword Power Device Library.

<!-- page 3 -->

## Appendix C Use Add-On Instructions

The Kinetix 5100 drive Add-On Instructions are aimed to provide necessary
simple motion functions. Table 151 lists the Kinetix 5100 drive Add-On
Instructions.

> **Table 151** — Add-On Instruction List for Kinetix 5100 Drives

Name(1)
(1)
The xxx in the name can be Dvc (legacy applications) or Opr (new applications).
Description
raC_xxx_K5100_MSO
Motion Servo On.
Use the Motion Servo On instruction to activate the drive output and to activate
the drive servo loops.
raC_xxx_K5100_MSF
Motion Servo Off.
Use the Motion Servo Off instruction to deactivate the drive output and to
deactivate the drive servo loops.
raC_xxx_K5100_MAJ
Motion Axis Jog
Use the Motion Axis Jog instruction to accelerate or decelerate the motor at a
constant speed without termination.
raC_xxx_K5100_MAT
Motion Axis Torque
Use the Motion Axis Torque instruction to use torque limiting while a pre-defined
speed is used to move the motor.
raC_xxx_K5100_MAM
Motion Axis Move
Use the Motion Axis Move instruction to move the motor to a specified position.
raC_xxx_K5100_MAH
Motion Axis Home
Use the Motion Axis Home instruction to home the motor.
raC_xxx_K5100_MAG
Motion Axis Gear
Use the Motion Axis Gear instruction to set the gear ratio between a pulsesource and follower drive.
IMPORTANT: This Add-On Instruction changes the drive E-Gear ratio; Slave/
Follower ID151 (P1.044) and Master ID152 (P1.045) Counts. If your drive is
positioning, be aware that the units are impacted because the E-Gear ratio
controls the counts/motor rotation value.
raC_xxx_K5100_MAS
Motion Axis Stop
Use the Motion Axis Stop instruction to stop a specific motion process on the
motor or to stop the motor completely.
raC_xxx_K5100_MAFR
Motion Axis Fault Reset.
Use the Motion Axis Fault Reset instruction to clear many motion faults for the
drive. Some faults cannot be cleared until you power cycle the drive. The faults,
which can be cleared by raC_xxx_K5100_MAFR, are listed in the fault list section.
raC_xxx_K5100_MAI
Motion Axis Index
Use the Motion Axis Index instruction to execute the specified PR (index)
function of the drive. Use K5100C configuration software or explicit messaging
to set the PR (index) parameters. The raC_xxx_K5100_MAI instruction specifies
the PR (index) number to be executed.

<!-- page 4 -->

## Appendix C Use Add-On Instructions

Download the Add-On
Instruction Files and Data
Types
The Add-On Instructions files and Data Types for the Kinetix 5100 drive are
available for download at the Rockwell Automation Product Compatibility
Download Center (PCDC) website. Follow these steps to download the Add-On
Instruction files and Data Types from the PCDC website,
https:rok.auto/pcdc.
•
For legacy applications: Enter 2198-Exxxx-ERS in the Search PCDC
window.
•
For new applications: Enter Power Device Library in the Search PCDC
window.
Import the Add-On
Instruction Files and Data
Types (version 1.xxx)
In Legacy applications, (version1.xxx) follow these steps to import the Add-On
Instruction files and Data Types to your Studio 5000 Logix Designer
application. Use this method when you want to import individual Add-On
Instructions. This method also contains some simple interlocking to begin
your application programming.
1.
From the File menu, click Import Component>Add-On Instruction to
import Add-On Instruction Files.
Or, click Import Component>Data Type to import Data Types.
The Import Add-On Instruction/Data Types dialog box appears.
2. Browse to the Add-On Instruction files/Data Types you downloaded and
select a file to add to your Logix Designer application and click Open.
3.
Repeat step 1 and step 2 for the other Add-On Instruction file/Data Types.

<!-- page 5 -->

## Appendix C Use Add-On Instructions

Your Add-On Instruction files appear in the Controller Organizer under the
Add-On Instructions folder, along with the Add-On-Defined Data Types,
which appear in the Controller Organizer under Data Types> Add-On-Defined
folder. Your Data Types appear in the Controller Organizer under Data
Types>User-Defined folder.
There are 11 Add-On Instruction files and five user-defined data types for the
Kinetix 5100 drive firmware revision 2 or later to provide a necessary function
block with assembly output instance 106 or 'Connection' is 'Data with
Camming' of AOP version 2 or later. If the assembly output instance 104 is
configured or 'Connection' is 'Data' of AOP version 2 or later, MAG and MAT
must use the files in 'Version 1'.
The Add-On Instruction files also appear in the ladder logic toolbox.
To avoid incorrect data types, or incorrectly setting the data types, when
using version 2.xxx, use the Add-On Instructions that are designed for new
applications.

<!-- page 6 -->

## Appendix C Use Add-On Instructions

Dvc Add-On Instruction
Configuration (version
1.xxx)
For legacy applications (version 1.xxx), follow these steps to configure your
Add-On Instruction.
Create the Add-On Instruction Tag
1.
In the Controller Organizer, right-click Controller Tags and click New
Tag.
The New Tag dialog box appears.
2. Type a name (for example, MSO_1) for the Tag.
3.
In the Data Type field, click Browse and choose an Add-On Instruction
(for example, raC_Dvc_K5100_MSO).
4. Click OK.
The Add-On Instruction tag that you created, with the module-defined
data types, populates in the Controller Tags group.
For new applications (version 2.xxx), use the PCDC keyword ‘Power Device
Library’, and download the video from the PCDC website to install the Opr
Add-On Instructions.

<!-- page 7 -->

## Appendix C Use Add-On Instructions

Create the Ref_Axis Tag
To use the Kinetix 5100 drive Add-On Instructions, you must create a tag
Ref_Axis whose type is raC_UDT_Dvc_K5100_Assm. Follow these steps to
create a Ref_Axis tag.
1.
In the Controller Organizer, right-click Controller Tags and click New
Tag.
The New Tag dialog box appears.
2. Type a name (for example, K5100_Axis) for the Tag.
3.
In the Data Type field, click Browse and choose an Add-On Instruction
(for example, raC_UDT_Dvc_K5100_Assm).
4. Click OK.
The Add-On Instruction tag that you created, with the module-defined
data types, populates in the Controller Tags group.
IMPORTANT
All Add-On Instructions use the K5100_Axis as the operation object.

<!-- page 8 -->

## Appendix C Use Add-On Instructions

Configure the Add-On Instruction
1.
Double-click the entry of raC_Dvc_K5100_MSO argument and choose the
MSO_1 Tag created earlier.
2. Double-click the entry of Ref_Axis argument and choose the K5100_Axis
tag that you created earlier.
The error disappears after you configured the Add-On Instruction
arguments.
General Execution Rules for Add-On Instructions
See IO Mode on page 271 for the input assembly data for the Kinetix 5100 drive.
To map the K5100_Axis to the Kinetix 5100 drive, before any Kinetix 5100 drive
motion Add-On Instruction is used, you must use the CPS function to copy all
input assembly data of the Kinetix 5100 drive to the Input element of
K5100_Axis. After all Kinetix 5100 drive motion Add-On Instructions are used,
you must use the CPS function to copy the Output element of K5100_Axis to
the output assembly data of the Kinetix 5100 drive. This figure shows an
example for mapping the K5100_Axis to the Kinetix 5100 drive.
CommandInProcess in the input assembly indicates the new motion
command has been received by the Kinetix 5100 drive. It toggles between 0 and
1 after a new motion command has been received by the Kinetix 5100 drive.
The CommandInProcess bit remains in the toggled state until a new command
is received.

<!-- page 9 -->

## Appendix C Use Add-On Instructions

Opr Add-On Instruction
Configuration
Use the video on the PCDC to create and install the Device Object and Motion
Instruction Add-On Instructions. Once the Operation Add-On Instructions are
created, use the sample logic that is created as guidance for creating your
application logic.
When the Device Object Add-On Instruction is created, it references Data
Types that interface with the drive to exchange command and status
information. You must create each Device Object Add-On Instruction as a
unique instance.

> **Figure 257** — Device Object

raC_UDT_Itf_K5100_Cfg
raC_UDT_Itf_K5100_Cfg is the Power Motion Common Control Interface
User-Defined Data Type for device configuration. Its members provide
selection between drive units (counts) or user units (PU).
This selection is very useful because the Kinetix 5100 drive natively supports
only drive units. When the Operating Units = 1, the Motion Resolution and
ConversionConstant values are used. Position Scaling originates from the
KNX5100C software and is used together with the Cfg tags to derive user
scaling units.
Example Configuration with Position Units
The E-Gear ratio (KNX5100C>Function List>E-Gear Ratio) is always used to
provide a representation of positioning (units or counts) or to define a PulsePulse Following relationship (MAG/PT). When the E-Gear ratio is changed, the
positioning of the drive is changed. When not using the MAG Add-On
Instruction or PT operation mode, the E-Gear ratio is used to define position
scaling.

> **Table 152** — raC_UDT_Itf_K5100_Cfg Data Types

Member
Description
DataType
OperatingUnits
0 = Drive Units; 1 = UserUnits
DINT
MotionResolution
Motion Counts per Motor Revolution
DINT
ConversionConstant
Motion Counts per Position Unit
REAL

<!-- page 10 -->

## Appendix C Use Add-On Instructions

When Operating Units =1, Position Units are used, and we can define
application units instead of using drive counts. In KNX5100C software, the EGear ratio is defined to provide Position Scaling. This is encoder counts (or
pulses)/motor rotation.
By using KNX5100C software, navigate to Settings>E-Gear Ratio.

> **Figure 258** — Position Units Configuration

All Position Unit configurations must:
•
Configure GearRatioFollowerCounts ID151 (P1.044) to be the same as the
motor feedback effective resolution.
•
Configure GearRatioMasterCounts ID152 (P1.045) to provide motor
feedback counts/motor rotation.
•
You define this value and can be any count value, default values with
high-resolution encoders are 100,000 counts/motor rotation. The E-Gear
configuration is used with the Device Object Cfg tags.

> **Figure 259** — Position Unit Configuration Tag

The Device Object Cfg values must:
•
Set Cfg.MotionResolution = GearRatioMasterCounts ID152 (P1.045) ->
Motion Counts/Motor Revolution
•
Set Cfg.ConversionConstant based on the Counts/Position Unit ->
Motion Counts/Position Unit that is required for your application.
The example in Figure 259 results in Position Units = motor rotations. Now,
entry values that originally used drive counts can be entered as motor
rotations.
raC_UDT_Itf_K5100_Set
raC_UDT_Itf_K5100_Set is the Power Motion Common Control Interface
User-Defined Data Type for device settings. Its members provide application
program access to allow or inhibit commands and settings from the device
faceplate or other external sources. The table below shows member names,
descriptions, and tag data types.

<!-- page 11 -->

## Appendix C Use Add-On Instructions

For example, to inhibit write commands from the device faceplate or other
external sources write a 1 to the ModuleName_AOI_CtlrSet.InhibitCmd
program tag from your application program. This write prevents a jog
command from the device faceplate.

> **Table 153** — raC_UDT_Itf_K5100_Set Data Types

Member
Description
DataType
bInhibit
Bit overlay for external access restriction
DINT
InhibitCmd
1 = Inhibit user Commands from external sources; 0 =
Allow Control. This is only used with the optional device
faceplate.
BOOL
InhibitSet
1 = Inhibit user Settings from external sources; 0 = Allow
This member is only used with the optional device
faceplate.
BOOL
OperatingMode
Determines the drive operating mode when ‘Start
Motion’ has a zero-to-one transition.
1 - Position mode
2 - Speed mode
3 - Home mode
4 - Torque mode
5 - Gear mode (Fixed Ratio, based on present E-Gear
ratio)
6 - Index mode
7 - Reserved
8 - Gear Mode (Variable Ratio, based on Master/Slave
tag values)
9 - Enhanced MAT mode
DINT
MoveType
Specify the type of move.
0 = Absolute
1 = Incremental
2 = Rotary Shortest Path
3 = Rotary Positive
4 = Rotary Negative
7 = Relative
8 = Capture
DINT
PositionCommandOverlap
Allows overlapping of successive movements.
BOOL
PositionCommandOverride
Allows interruption of current movement, replacing it
with a new movement.
BOOL
CapturedPositionSelect
Capture position selection (First capture or second
capture).
BOOL
Position
Determines the command position.
REAL
Velocity
Determines the command speed.
REAL
Accel
Determines the command acceleration.
REAL
Decel
Determines the command deceleration.
REAL
Torque
Determines the command torque.
DINT
TorqueRampTime
Determines the command torque ramp time.
DINT
StartingIndex
This entry is the PR (Position Register) the drive should
execute.
DINT
HomingMethod
Homing Method.
DINT
HomeReturnSpeed
Determines the command home return speed.
REAL
CamMasterReference
Future: Determines the master position reference of
CAM.
DINT
CamExecutionSchedule
Future: Determines the method used to execute the CAM
profile.
DINT
CamExecutionMode
Future: Determines if the cam profile is executed only
one time or repeatedly.
DINT
CamStopMode
Future: Determines the stop mode of CAM.
BOOL
CamSlaveScaling
Future: Scales the total distance covered by the slave
axis through the cam profile.
DINT
CamLockPosition
Future: Determines the starting location in the cam
profile
DINT
CamMasterLockPosition
Future: Determines the master location where the slave
axis locks to the mater axis.
DINT

<!-- page 12 -->

## Appendix C Use Add-On Instructions

raC_UDT_Itf_K5100_Cmd
raC_UDT_Itf_K5100_Cmd is the Power Motion Common Control Interface
User-Defined Data Type for device commands. Its members provide
application program access to common basic device commands.
Table 154 shows member names, descriptions, and tag data types.
All the commands are available whether operating the device physically or
virtually.
While it is possible, it is not typical to modify any of these UDT values directly.
The Motion Operation Add-On Instructions manipulate these values as a result
of their operation.
CamMasterLeadingCounts
Future: Determines the leading counts (master axis)
before the cam profile is executed.
DINT
CamMasterUnlockCounts
Future: Determines the unlock counts (master axis)
when the cam profile is executed.
DINT
CamMasterCyclicLeadingCounts
Future: Determines the cyclic leading counts (master
axis) during the cam profile is executed.
DINT
GearRatioSlaveCounts
Integer value representing slave counts. This value is
P1.044 Gear Ratio Follower Counts from the E-Gear ratio
in KNX5100C software.
DINT
GearRatioMasterCounts
Integer value representing master counts. This value is
P1.045 Gear Ratio Master Counts from the E-Gear ratio
in KNX5100C software.
DINT

> **Table 154** — raC_UDT_Itf_K5100_Cmd Data Types

Member
Description
DataType
bCmd
Commands (Bit Overlay)
DINT
Physical
1 = Operate as a physical device
BOOL
Virtual
1 = Operate as a virtual device
BOOL
ResetWarn
1 = Reset device warning
BOOL
ResetFault
1 = Reset device trip or fault
BOOL
Activate
1 = Activate Output Power Structure
BOOL
Deactivate
1 = DeActivate Output Power Structure
BOOL
StartMotion
A zero-to-one transition means the motion command is
issued from the external controller.
BOOL
StopMotion
A zero-to-one transition will stop any active motion
command in the drive.
BOOL

> **Table 153** — raC_UDT_Itf_K5100_Set Data Types (Continued)

Member
Description
DataType

<!-- page 13 -->

## Appendix C Use Add-On Instructions

raC_UDT_Itf_K5100_Sts
raC_UDT_Itf_K5100_Sts is the Power Motion Common Control Interface
User-Defined Data Type for device status. Its members provide application
program access to device states, status, and diagnostic data. The table below
shows member names, descriptions, and tag data types.

> **Table 155** — raC_UDT_Itf_K5100_Sts Data Types

Input
Description
DataType
eState
Enumerated state value:
0 = Unused
1 = Initializing
2 = Disconnected
3 = Disconnecting
4 = Connecting
5 = Idle
6 = Configuring
7 = Available
DINT
FirstWarning
Capture the First Alarm Bit to trigger. Display the
respective Description and Time Stamp on Faceplate. Log
the same in Event Queue.
raC_UDT_Event
FirstFault
Capture the Fault Code of the device. Display the
respective code, description, and timestamp on faceplate.
Log the same in Event Queue.
raC_UDT_Event
eCmdFail
Enumerated command failure code
DINT
bSts
Status (Bit Overlays)
DINT
Physical
1 = Operating as a physical device
BOOL
Virtual
1 = Operating as a virtual device
BOOL
Connected
1 = PAC to device connection has been established.
BOOL
Available
1 = The automation device is available for interaction with
the user program
BOOL
Warning
1 = A warning is active on the automation device
BOOL
Faulted
1 = A fault is active on the automation device
BOOL
Ready
1 = Device is ready to be Activated
BOOL
Active
1 = Device power structure is active
BOOL
ZeroSpeed
1 = Motor is within zero speed tolerance (this tolerance is
defined in KNX5100C software)
BOOL
Homed
Indicates whether the drive completed the home operation. BOOL
AtReference
Depending on the motion command (position, speed,
torque), AtReference is 1 when the actual reference =
command reference.
BOOL
CommandInProgress
Toggles state when a motion command is active in the
drive. This bit changes state (toggles between 0 and 1)
when a new command is executed from the drive.
IMPORTANT: Once this bit changes state, it remains in that
state for the duration of the command; it toggles to the
opposite state (and remains in that state) once a new
command is received.
BOOL
FaultCode
Active Fault Code in the drive
DINT
WarningCode
Active Warning Code in the drive
DINT
OperatingMode
Indicate which operating mode is currently used.
DINT
MotorType
Indicate which type of motor is connected to the drive.
Rotary Motor = 1
Linear Motor =2 (Future)
DINT
ActualPosition
Actual position of the motor. Units depend on the Cfg
settings. These can be drive counts or Position Units.
REAL
ActualVelocity
Actual speed of the motor. Units depend on the Cfg
settings. These can be 0.1 RPM/sec or Position Units.
REAL
ActualTorque
When the operating mode is 4, Torque Mode, this
represents the % motor torque.
REAL
ActiveIndex
Indicates the currently executing Position Register PR
(index).
DINT

<!-- page 14 -->

## Appendix C Use Add-On Instructions

raC_UDT_LookupMember_STR0082
Use these Ctrlxxx tags where possible instead of the Operation Add-On
Instruction status bits. For example, instead of evaluating the
raC_Opr_K5100_MSO.Sts_DN =1 to indicate an Active state, use
_Drive01_CtrlSts.Active = 1. This bit is inclusive of interlocks that check for
connection status as well as the drive being Ready and not faulted.

> **Figure 260** — Ctrlxxx Tag Example

When using the Device Object Add-On Instructions, using the Input/Output
Assemblies is not required because the Device Object Add-On Instruction is
now the interface between the drive and controller.
ParameterMonitor1Value
Parameter Monitor 1 Value
You can use ID60 (P0.035) to specify the mapping
parameter instance ID number. The content of the
parameter that is specified by ID60 (P0.035) is shown in
ID55 (P0.025).
DINT
ParameterMonitor2Value
Parameter Monitor 2 Value
You can use ID61 (P0.036) to specify the mapping
parameter instance ID number. The content of the
parameter that is specified by ID61 (P0.036) is shown in
ID56 (P0.026).
DINT
ParameterMonitor3Value
Parameter Monitor 3 Value
You can use ID62 (P0.037) to specify the mapping
parameter instance ID number. The content of the
parameter that is specified by ID62 (P0.037) is shown in
ID56 (P0.027).
DINT
ParameterMonitor4Value
Parameter Monitor 4 Value
You can use ID63 (P0.038) to specify the mapping
parameter instance ID number. The content of the
parameter that is specified by ID63 (P0.038) is shown in
ID57 (P0.028).
DINT
ParameterMonitor5Value
Parameter Monitor 5 Value
You can use ID64 (P0.039) to specify the mapping
parameter instance ID number. The content of the
parameter that is specified by ID64 (P0.039) is shown in
ID57 (P0.028).
DINT

> **Table 156** — raC_UDT_LookupMember_STR0082 Data Types

Member
Description
DataType
Code
Stores the value of device fault code.
DINT
Desc
Stores the Messages related to fault code.
STRING

> **Table 155** — raC_UDT_Itf_K5100_Sts Data Types (Continued)

Input
Description
DataType

<!-- page 15 -->

## Appendix C Use Add-On Instructions

raC_UDT_Event
raC_UDT_Event is an array of size 4 and is used to log the FirstWarning and
FirstFault capture. The data is captured in FIFO order. The faceplate displays
the same data. This UDT is created as part of the Device Object, however, is
only used with the optional faceplate. You assign this Device Object to record
events in Logix. When using Machine Builder Libraries, a suite of instructions
is available to customize event handling.

> **Table 157** — raC_UDT_Event Data Types

Member
Description
Data Type
Type
1 = Status
2 = Warning
3 = Fault
4….n = User
DINT
ID
User definable event ID
DINT
Category
User definable category (Electrical, Mechanical,
Materials, Utility)
DINT
Action
User definable event action code
DINT
Value
User definable event value or fault code
DINT
Message
Event message text
STRING
EventTime_L
Timestamp
LINT
EventTime_D
Timestamp (Y,M,D,h,m,s,us)
DINT[7]

<!-- page 16 -->

## Appendix C Use Add-On Instructions

Add-On Instruction Details
When the Device Object is used with the current version of the Add-On
Instruction library, using the Device Object states to represent the axis is
preferred, as these states incorporate the instruction and drive information in
one location, which results in accurate drive state representation.
This section provides details for each instruction.
raC_xxx_K5100_MSO(1)
Use the Motion Servo On (raC_xxx_K5100_MSO) instruction to activate the
drive amplifier for the specified axis and to activate the servo control loops.

> **Figure 261** — MSO Ladder Diagram

Operands
IMPORTANT
Although the Dvc instructions function similarly to the Opr instructions,
the Opr are described in this section. The Opr instructions are used for
new applications.
(1) The xxx in the name can be Dvc (legacy applications) or Opr (new applications).
Operand
Type
Format
Description
Instance
raC_xxx_K5100_MSO
Tag
Unique instance of the MSO Add-On Instruction
Ref_Ctrl_Set
raC_UDT_Itf_PowerMotionSA_Set
Tag
Interface for Ctrl_Set of the Device Object
Ref_Ctrl_Cmd
raC_UDT_Itf_PowerMotionSA_Cmd
Tag
Interface for Ctrl_Cmd of the Device Object
Ref_Ctrl_Sts
raC_UDT_Itf_PowerMotionSA_Sts
Tag
Interface for Ctrl_Sts of the Device Object
Mnemonic
Description
Sts_EN (Enable)
This bit is set when the rung makes a false-to-true transition and remains set as the
message transaction to activate the drive is initiated and in process. It remains set while the
rung-in condition is true and no faults are active.
Sts_DN (Done)
This bit is set when the rung makes a false-to-true transition and the cmd to activate the
drive has been acknowledged.
Sts_ER (Error)
This bit is set when the rung makes a false-to-true transition and there is an error that has
occurred with the instruction. (This instruction error can be a result of a fault on the drive
itself). See Sts_ERR for details on the cause of the error.

**Extracted table (page 16, #1):**

| Type | Format |
| --- | --- |
| raC_xxx_K5100_MSO | Tag |
| raC_UDT_Itf_PowerMotionSA_Set | Tag |
| raC_UDT_Itf_PowerMotionSA_Cmd | Tag |
| raC_UDT_Itf_PowerMotionSA_Sts | Tag |

<!-- page 17 -->

## Appendix C Use Add-On Instructions

Description
Use the raC_xxx_K5100_MSO instruction to activate the motor. This
instruction must be used while there are no active faults on the drive and the
drive is in a Ready State. The resulting state of the drive is reflected when
Ref_CtrlSts.Active is one.
Error Codes
•
100 - Kinetix 5100 drive is not ready
•
101 - Kinetix 5100 drive is faulted
•
102 - Another raC_xxx_K5100_MSO message is executing
•
103 - raC_Dvc_K5100_MSF is executing
•
127 - Previous command has not completed
•
129 - Motor not connected
See Error Codes on page 524 for details.
raC_xxx_K5100_MSF(1)
Use the Motion Servo Off (raC_xxx_K5100_MSF) instruction to deactivate the
drive output for the specified axis and to deactivate the motor.

> **Figure 262** — MSF Ladder Diagram

Operands
IMPORTANT
The instruction execution can take multiple scans to execute
because it requires multiple RPI updates to complete the request.
The Done (Sts_DN) bit is not set immediately, but only after the
request is completed.
You can use the F1 key to view fault error codes.
(1) The xxx in the name can be Dvc (legacy applications) or Opr (new applications).
Operand
Type
Format
Description
Instance
raC_xxx_K5100_MSF
Tag
Unique instance of the MSF Add-On Instruction
Ref_Ctrl_Set
raC_UDT_Itf_PowerMotionSA_Set
Tag
Interface for Ctrl_Set of the Device Object
Ref_Ctrl_Cmd
raC_UDT_Itf_PowerMotionSA_Cmd
Tag
Interface for Ctrl_Cmd of the Device Object
Ref_Ctrl_Sts
raC_UDT_Itf_PowerMotionSA_Sts
Tag
Interface for Ctrl_Sts of the Device Object
Mnemonic
Description
Sts_EN (Enable)
This bit is set when the rung makes a false-to-true transition and remains set as the
message transaction to deactivate the drive is initiated and in process. It remains set
while the rung-in condition is true and no faults are active.
Sts_DN (Done)
This bit is set when the rung makes a false-to-true transition and the message
transaction to deactivate the drive (Sts_EN) is complete.
Sts_ER (Error)
This bit is set when the rung makes a false-to-true transition and there is an error that
has occurred with the instruction. (This instruction error can be a result of a fault on
the drive itself). See Sts_ERR for details on the cause of the error.

**Extracted table (page 17, #1):**

| Type | Format |
| --- | --- |
| raC_xxx_K5100_MSF | Tag |
| raC_UDT_Itf_PowerMotionSA_Set | Tag |
| raC_UDT_Itf_PowerMotionSA_Cmd | Tag |
| raC_UDT_Itf_PowerMotionSA_Sts | Tag |

<!-- page 18 -->

## Appendix C Use Add-On Instructions

Description
The raC_xxx_K5100_MSF instruction deactivates the motor. This instruction
must be used when there are no active faults on the drive and the drive is in the
Ready state. The resulting state of the drive is reflected when RefCtrlSts.Active
is zero.
Error Codes
•
100 - Kinetix 5100 drive is not ready
•
101 - Kinetix 5100 drive is faulted
•
104 - Another raC_Dvc_K5100_MSF message is executing
•
129 - Motor not connected
See Error Codes on page 524 for details.
raC_xxx_K5100_MAFR(1)
Use the Motion Axis Fault Reset (raC_xxx_K5100_MAFR) instruction to clear
some drive faults. When the fault is no longer active in the drive, this
instruction clears the fault. This instruction does not clear any faults that are
still active in the drive.

> **Figure 263** — MAFR Ladder Diagram

Operands
IMPORTANT
The instruction execution can take multiple scans to execute
because it requires multiple RPI updates to complete the request.
The Done (Sts_DN) bit is not set immediately, but only after the
request is completed.
You can use the F1 key to view fault error codes.
(1) The xxx in the name can be Dvc (legacy applications) or Opr (new applications).
Operand
Type
Format
Description
Instance
raC_xxx_K5100_MAFR
Tag
Unique instance of the MAFR Add-On Instruction
Ref_Ctrl_Set
raC_UDT_Itf_PowerMotionSA_Set
Tag
Interface for Ctrl_Set of the Device Object
Ref_Ctrl_Cmd
raC_UDT_Itf_PowerMotionSA_Cmd
Tag
Interface for Ctrl_Cmd of the Device Object
Ref_Ctrl_Sts
raC_UDT_Itf_PowerMotionSA_Sts
Tag
Interface for Ctrl_Sts of the Device Object
Mnemonic
Description
Sts_EN (Enable)
This bit is set when the rung makes a false-to-true transition and the message
transaction to Reset is initiated and in process. It remains high until the rung-in
condition is false and no faults are active.
Sts_DN (Done)
This bit is set when the rung makes a false-to-true transition and the message
transaction to Reset the drive (Sts_EN) is complete.
Sts_ER (Error)
This bit is set when the rung makes a false-to-true transition and there is an error that
has occurred with the instruction. (This instruction error can be a result of a fault on
the drive itself). See Sts_ERR for details on the cause of the error.

**Extracted table (page 18, #1):**

| Type | Format |
| --- | --- |
| raC_xxx_K5100_MAFR | Tag |
| raC_UDT_Itf_PowerMotionSA_Set | Tag |
| raC_UDT_Itf_PowerMotionSA_Cmd | Tag |
| raC_UDT_Itf_PowerMotionSA_Sts | Tag |

<!-- page 19 -->

## Appendix C Use Add-On Instructions

Description
The raC_xxx_K5100_MAFR instruction attempts to clear any active fault on the
specified axis. If the active fault condition is still present, the drive remains
faulted.
Error Codes
•
100 - Kinetix 5100 drive is not ready
•
101- Drive is faulted
•
106 - Another raC_Dvc_K5100_MAFR message is executing
•
129 - Motor is not connected
See Error Codes on page 524 for details.
IMPORTANT
The instruction execution can take multiple scans to execute
because it requires multiple RPI updates to complete the request.
The Done (Sts_DN) bit is not set immediately, but only after the
request is completed.
You can use the F1 key to view fault error codes.

<!-- page 20 -->

## Appendix C Use Add-On Instructions

raC_xxx_K5100_MAS(1)
Use the Motion Axis Stop (raC_xxx_K5100_MAS) instruction to stop motion on
an axis. The drive remains active when the stop instruction is complete.

> **Figure 264** — MAS Ladder Diagram

Operands
Description
Use the raC_xxx_K5100_MAS instruction when you want a controlled stop for
any controlled motion. The instruction stops the motion without disabling the
motor. This Add-On Instruction stops any motion that is generated by motion
Add-On Instruction including the MAJ, MAM, or MAG.
(1) The xxx in the name can be Dvc (legacy applications) or Opr (new applications).
Operand
Type
Format
Description
Instance
raC_xxx_K5100_MAS
Tag
Unique instance of the MAS Add-On Instruction
Ref_Ctrl_Cfg
raC_UDT_Itf_PowerMotionSA_Cfg
Tag
Interface for Ctrl_Cfg of the Device Object
Ref_Ctrl_Set
raC_UDT_Itf_PowerMotionSA_Set
Tag
Interface for Ctrl_Set of the Device Object
Ref_Ctrl_Cmd
raC_UDT_Itf_PowerMotionSA_Cmd
Tag
Interface for Ctrl_Cmd of the Device Object
Ref_Ctrl_Sts
raC_UDT_Itf_PowerMotionSA_Sts
Tag
Interface for Ctrl_Sts of the Device Object
Set_DecelReference
REAL
Tag
The Deceleration Rate in 0.1 RPM/s for rotary
motor. Range: 458…30,000,000
Mnemonic
Description
Sts_EN (Enable)
This bit is set when the rung makes a false-to-true transition and the message
transaction to Stop is initiated and in process. It remains high until the rung-in
condition is false and no faults are active.
Sts_DN (Done)
This bit is set when the rung makes a false-to-true transition and the message
transaction to Stop the drive (Sts_EN) is complete.
Sts_ER (Error)
This bit is set when the rung makes a false-to-true transition and there is an error that
has occurred with the instruction. (This instruction error can be a result of a fault on
the drive itself). See Sts_ERR for details on the cause of the error.
Sts_IP (In Progress)
This bit is set when the rung makes a false-to-true transition, the Stop message
transaction is successful, and the motor begins to decelerate. This bit remains set as
the motor is executing the stop.
Sts_PC (Process
Completed)
This bit is set when the rung makes a false-to-true transition, the Sts_IP is set, and
Zero Speed is reached. Zero Speed is defined using KNX5100C software>General
Setting.

**Extracted table (page 20, #1):**

| Type | Format |
| --- | --- |
| raC_xxx_K5100_MAS | Tag |
| raC_UDT_Itf_PowerMotionSA_Cfg | Tag |
| raC_UDT_Itf_PowerMotionSA_Set | Tag |
| raC_UDT_Itf_PowerMotionSA_Cmd | Tag |
| raC_UDT_Itf_PowerMotionSA_Sts | Tag |
| REAL | Tag |

<!-- page 21 -->

## Appendix C Use Add-On Instructions

Error Codes
•
100 - Kinetix 5100 drive is not ready
•
101 - Kinetix 5100 drive is faulted
•
103 - MSF is executing
•
105 - Drive is disabled
•
107 - Another raC_xxx_K5100_MAS message is executing
•
113 - Decel_Rate is out of range
•
127 - Previous command has not completed
•
129 - Motor is not connected
See Error Codes on page 524 for details.
raC_xxx_K5100_MAJ(1)
Use the Motion Axis Jog (MAJ) instruction to move an axis at a constant speed
until the command is terminated.

> **Figure 265** — MAJ Ladder Diagram

Operands
You can use the F1 key to view fault error codes.
(1) The xxx in the name can be Dvc (legacy applications) or Opr (new applications).
Operand
Type
Format
Description
Instance
raC_xxx_K5100_MAJ
Tag
Unique instance of the MAJ Add-On Instruction
Ref_Ctrl_Cfg
raC_UDT_Itf_PowerMotionSA_Cfg
Tag
Interface for Ctrl_Cfg of the Device Object
Ref_Ctrl_Set
raC_UDT_Itf_PowerMotionSA_Set
Tag
Interface for Ctrl_Set of the Device Object
Ref_Ctrl_Cmd
raC_UDT_Itf_PowerMotionSA_Cmd
Tag
Interface for Ctrl_Cmd of the Device Object
Ref_Ctrl_Sts
raC_UDT_Itf_PowerMotionSA_Sts
Tag
Interface for Ctrl_Sts of the Device Object
Set_SpeedReference
REAL
Immediate or Tag
Units are 0.1 rpm for rotary motors.
Range: -80,000…+80,000
Set_AccelReference
REAL
Immediate or Tag
Units are 0.1 rpm/s for rotary motors.
Range: 458…30,000,000
Set_DecelReference
REAL
Immediate or Tag
Units are 0.1 rpm/s for rotary motors.
Range: 458…30,000,000
Mnemonic
Description
Sts_EN (Enable)
This bit is set when the rung makes a false-to-true transition and the message transaction to Jog is initiated and in process. It remains set while
the rung-in condition is true and no faults are active.
Sts_DN (Done)
This bit is set when the rung makes a false-to-true transition and the message transaction to Jog the drive (Sts_EN) is complete.

**Extracted table (page 21, #1):**

| Type | Format |
| --- | --- |
| raC_xxx_K5100_MAJ | Tag |
| raC_UDT_Itf_PowerMotionSA_Cfg | Tag |
| raC_UDT_Itf_PowerMotionSA_Set | Tag |
| raC_UDT_Itf_PowerMotionSA_Cmd | Tag |
| raC_UDT_Itf_PowerMotionSA_Sts | Tag |
| REAL | Immediate or Tag |
| REAL | Immediate or Tag |
| REAL | Immediate or Tag |

<!-- page 22 -->

## Appendix C Use Add-On Instructions

Sts_ER (Error)
This bit is set when the rung makes a false-to-true transition and there is an error that has occurred with the instruction. (This instruction error
can be a result of a fault on the drive itself). See Sts_ERR for details on the cause of the error.
Sts_IP (In Process)
This bit is set when the rung makes a false-to-true transition, the Jog message transaction is successful, and the motor begins to move. This bit
remains set as the motor is moving towards the target speed (Accel or Decel). It remains set while the Jog is active, regardless of the rung-in
condition.
Sts_AtSpeed (Process
Complete)/Sts_AtSpeed
This bit is set when the rung makes a false-to-true transition, the Sts_IP is set, and the Target Speed is reached. This bit remains set while the
Jog is active and AtSpeed condition is true.
Mnemonic
Description

<!-- page 23 -->

## Appendix C Use Add-On Instructions

Description
Use the MAJ instruction to move an axis at a constant speed until the
command is terminated.
Error Codes
•
100 - Kinetix 5100 drive is not ready
•
101 - Kinetix 5100 drive is faulted
•
103 - raC_xxx_K5100_MSF is running
•
105 - Drive is disabled
•
107 - raC_xxx_K5100_MAS is executing
•
108 - Other motion Add-On Instruction is sending the command
•
111 - Input speed is out of range
•
112 - Accel_rate is out of range
•
113 - Decel_rate is out of range
•
127-Previous command has not completed
•
129 - Motor is not connected
See Error Codes on page 524 for details.
raC_xxx_K5100_MAM(1)
Use the Motion Axis Move (raC_xxx_K5100_MAM) instruction to move (index)
an axis to a specified position.

> **Figure 266** — MAM Ladder Diagram

You can use the F1 key to view fault error codes.
(1) The xxx in the name can be Dvc (legacy applications) or Opr (new applications).

<!-- page 24 -->

## Appendix C Use Add-On Instructions

Operands
Operand
Type
Format
Description
Instance
raC_xxx_K5100_MAM
Tag
Unique instance of the MAM Add-On Instruction
Ref_Ctrl_Cfg
raC_UDT_Itf_PowerMotionSA_Cfg
Tag
Interface for Ctrl_Cfg of the Device Object
Ref_Ctrl_Set
raC_UDT_Itf_PowerMotionSA_Set
Tag
Interface for Ctrl_Set of the Device Object
Ref_Ctrl_Cmd
raC_UDT_Itf_PowerMotionSA_Cmd
Tag
Interface for Ctrl_Cmd of the Device Object
Ref_Ctrl_Sts
raC_UDT_Itf_PowerMotionSA_Sts
Tag
Interface for Ctrl_Sts of the Device Object
Set_PositionReference
REAL
Immediate or Tag
Set the Target Distance/Position Reference
(PUU) Range: -2,147,483,648…+2,147,483,647
Set_SpeedReference
REAL
Immediate or Tag
Units are 0.1 rpm for rotary motors
Range: -80,000…+80,000
Set_AccelReference
REAL
Immediate or Tag
Units are 0.1 rpm/s for rotary motors
Range: 458…30,000,000
Set_DecelReference
REAL
Immediate or Tag
Units are 0.1 rpm/s for rotary motors
Range: 458 …30,000,000
Set_MoveType
INT
Tag
Specify the type of move:
0 = Absolute
1 = Incremental
2 = Rotary Shortest Path
3 = Rotary Positive
4 = Rotary Negative
7 = Relative
8 = Capture (see page 337, and page 514 for
details)
Set_PositionCommandOverride
BOOL
Tag
0 = Do not interrupt previous movement
1 = Interrupt previous movement (see page 514
for details)
Set_PositionCommandOverlap
BOOL
Tag
0 = Current movement is not overlapped with
next
1 = Current movement is overlapped with next
movement (see page 514 for details)
Set_CapturedPositionSelect
BOOL
Tag
0 = First High Speed Capture (triggered by DI9)
1 = Second High Speed Capture (triggered by
DI10)
Mnemonic
Description
Sts_EN (Enable)
This bit is set when the rung makes a false-to-true transition and the message transaction
to Index is initiated and in process. It remains high until the rung-in condition is false and no
faults are active.
Sts_DN (Done)
This bit is set when the rung makes a false-to-true transition and the message transaction
to Index the drive (Sts_EN) is complete.
Sts_ER (Error)
This bit is set when the rung makes a false-to-true transition and there is an error that has
occurred with the instruction. (This instruction error can be as a result of a fault on the drive
itself). See Sts_ERR for details on the cause of the error.
Sts_IP (In Progress)
This bit is set when the rung makes a false-to-true transition, the Index message transaction
is successful, and the motor begins to move. This bit remains set as the motor is executing
the index.
Sts_PC (Process
Completed)
This bit is set when the rung makes a false-to-true transition, the Index message transaction
is successful, and the motor reaches the Target position.

**Extracted table (page 24, #1):**

| Type | Format |
| --- | --- |
| raC_xxx_K5100_MAM | Tag |
| raC_UDT_Itf_PowerMotionSA_Cfg | Tag |
| raC_UDT_Itf_PowerMotionSA_Set | Tag |
| raC_UDT_Itf_PowerMotionSA_Cmd | Tag |
| raC_UDT_Itf_PowerMotionSA_Sts | Tag |
| REAL | Immediate or Tag |
| REAL | Immediate or Tag |
| REAL | Immediate or Tag |
| REAL | Immediate or Tag |
| INT | Tag |
| BOOL | Tag |
| BOOL | Tag |
| BOOL | Tag |

<!-- page 25 -->

## Appendix C Use Add-On Instructions

Description
The raC_xxx_K5100_MAM instruction moves an axis to using a target position
specified and uses the Move Type to perform the move (index).
Error Codes
•
100 - Kinetix 5100 drive is not ready
•
101 - Kinetix 5100 drive is faulted
•
103 - MSF is executing
•
105 - Drive is disabled
•
107 - raC_xxx_K5100_MAS is executing
•
108 - Other motion Add-On Instruction is sending the command
•
111 - SpeedReference is out of range
•
112 - AccelReference is out of range
•
113 - DecelReference is out of range
•
117 - NonCyclicMoveType is out of range
•
118 - CyclicMoveType is out of range
•
119 - TravelMode is out of range
•
126 - Homing not completed
•
127 - Previous command has not completed
•
129 - Motor is not connected to drive
See Error Codes on page 524 for details.
You can use the F1 key to view fault error codes.

<!-- page 26 -->

## Appendix C Use Add-On Instructions

Set_MoveType
•
Positioning Operation (Cfg_Selection is set to 2 (0b0010)): Four types of
move operations (Set_MoveType) are executed as shown.
•
Positioning Operation (Cfg_Selection is set to 10 (0b1010)): Three types
of move operations (Set_MoveMethod) are executed as shown.
- Define 'Indexing Coordinate':
- Rotate Positive or Rotate Negative or Rotate Shortest Path.
The Rotary move types are used to provide a way to index while
observing the natural rollover of the feedback device. For example, if
the motor could only index positive, the Rotary Positive is used. When
the feedback device transitions through its natural unwind (typically
2.1 billion counts), the movements always index positive.

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
IMPORTANT
At this time, the Kinetix 5100 drive does not have a user defined
Unwind function. The rotary selections in this Add-On Instruction
do not refer to user defined rotary axis types.

<!-- page 27 -->

## Appendix C Use Add-On Instructions

•
Position Command with Overlap option.
The executing index is interrupted during its deceleration. The new
index is started before the deceleration is complete
•
Position Command with Interrupt option.
The executing index (Index 1) is terminated. The new index (Index 2) is
executed using its dynamics. This is shown in the graphic below. The red
arrow is the point where the command for Index 2 is received by the
drive
raC_xxx_K5100_MAI(1)
It may be useful to execute a motion control internal register (PR) while in the
IO operation mode. This can be for performing an action that is not able to be
performed by one of the Add-On Instructions in the motion library. Using this
means that the PR has to be pre-configured in the drive by using KNX5100C
software, which is done before the IO Mode connection is established.

> **Figure 267** — MAI Ladder Diagram

(1) The xxx in the name can be Dvc (legacy applications) or Opr (new applications).

<!-- page 28 -->

## Appendix C Use Add-On Instructions

Operands
Description
Use the Motion Axis Index (raC_xxx_K5100_MAI) instruction to execute
motion control by internal register (PR) in Kinetix 5100 drives. The 99 built-in
command registers selects the PR command source. See Chapter 11, Motion
Control in PR Mode on page 285 for details.
Error Codes
•
100 - Kinetix 5100 drive is not ready
•
101 - Kinetix 5100 drive is faulted
•
103 - MSF is executing
•
105 - Drive is disabled
•
107 - raC_xxx_K5100_MAS is executing
•
108 - Other motion Add-On Instruction is sending the command
•
115 - StartingIndex is out of range
•
127 - Previous command is not completed
•
129 - Motor is not connected
See Error Codes on page 524 for details.
Operand
Type
Format
Description
Instance
raC_xxx_K5100_MAI
Tag
Unique instance of the MAI Add-On Instruction
Ref_Ctrl_Set
raC_UDT_Itf_PowerMotionSA_Set
Tag
Interface for Ctrl_Set of the Device Object
Ref_Ctrl_Cmd
raC_UDT_Itf_PowerMotionSA_Cmd
Tag
Interface for Ctrl_Cmd of the Device Object
Ref_Ctrl_Sts
raC_UDT_Itf_PowerMotionSA_Sts
Tag
Interface for Ctrl_Sts of the Device Object
Set_StartingIndex
INT
Tag
Enter the pre-configured PR# to execute
Sts_ActiveIndex
INT
Tag
Reads the current PR# that is executing in the
drive
Mnemonic
Description
Sts_EN (Enable)
This bit is set when the rung makes a false-to-true transition and the message transaction
to MAI is initiated and in process. It remains high until the rung-in condition is false and no
faults are active.
Sts_DN (Done)
This bit is set when the rung makes a false-to-true transition and the message transaction
to MAI the drive (Sts_EN) is complete.
Sts_ER (Error)
This bit is set when the rung makes a false-to-true transition and there is an error that has
occurred with the instruction. (This instruction error can be a result of a fault on the drive
itself). See Sts_ERR for details on the cause of the error.
Sts_IP (In Progress)
This bit is set when the rung makes a false-to-true transition, the MAI message transaction
is successful, and the PR command has been sent to the drive. This bit remains set until the
AtReference bit is set.
Sts_PC (Process
Completed)
This bit is set when the rung makes a false-to-true transition, the Sts_IP is set, and the MAI
has sent the PR execution and the AtReference bit is set.
You can use the F1 key to view fault error codes.

**Extracted table (page 28, #1):**

| Type | Format |
| --- | --- |
| raC_xxx_K5100_MAI | Tag |
| raC_UDT_Itf_PowerMotionSA_Set | Tag |
| raC_UDT_Itf_PowerMotionSA_Cmd | Tag |
| raC_UDT_Itf_PowerMotionSA_Sts | Tag |
| INT | Tag |
| INT | Tag |

<!-- page 29 -->

## Appendix C Use Add-On Instructions

raC_xxx_K5100_MAG(1)
Use the Motion Axis Gear (MAG) Add-On Instruction to execute a pulse-pulse
relationship with the drive. The MAG Add-On Instruction uses the E-Gear ratio
configured in the KNX5100C software. The E-Gear ratio dialog box is shown in
Figure 268. When the MAG Add-On Instruction is used, the drive behaves like
it is in PT (Position Terminal - or Pulse Train) mode and the drive uses the EGear ratio to respond to master pulses.
Figure 268 describes the values in the E-Gear ratio dialog box. Not all values
shown here are used with the Motion Operation Add-On Instruction.

> **Figure 268** — E-Gear Ratio Dialog Box

The PT Mode is a pulse-pulse follower relationship. When the variable ratio is
used and the ratio is changed, there is no positioning ability. Therefore, when
you are finished using the MAG Add-On Instruction, your position scaling
(which also uses the E-Gear ratio) might have changed if you used variable
GearingMode and changed the Master ratio because of your application
requirements.
(1) The xxx in the name can be Dvc (legacy applications) or Opr (new applications).
Item
Description

Gear Ratio Selection pull-down menu - You can choose from four different ratios (N1…N4) (Not used
with Add-on Instructions)

Gear Ratio Follower Counts (N1) - Set this value as the motor feedback resolution.

Gear Ratio Master Counts (M) - Set this value as the counts/motor resolution. This value is set for
whatever your application requires. Typical values are 100,000 counts for a high-resolution encoder.

GNUM0/1 - These values are mapped to the Digital Inputs that represent binary weighted values to
select the Gear Ratio value. (Not used with Add-on Instructions)

<!-- page 30 -->

## Appendix C Use Add-On Instructions

The MAG Set_SlaveCounts is sometimes called the Gear Ratio Follower Counts
or numerator (shown as 2 in Figure 268) because it is used to determine the
internal 'ratio' of the drive (shown as 1677.72 in Figure 268). For our purposes,
the E-Gear ratio Follower = MAG Set_SlaveCounts = motor feedback resolution
(from the KNX5100C>Function List>Motor Selection>Feedback dialog box).

> **Figure 269** — Set the Follower Counts

The MAG Set_MasterCounts is sometimes called the Gear ratio Master counts
or denominator (shown as 3 in Figure 268). Any gearing relationship must
consider the actual motor mechanics, like a gearbox, or actuator pitch, and use
those mechanics to relate back to a motor rotation. Gear Ratio Master counts is
desired counts/motor rotation. Desired counts are not used for positioning;
but defines how many counts your motor moves in one rotation based on the
number of feedback pulses you expect to receive from the source input, which
is used to determine your gearing relationship. So, this Master counts value is
used to define the pulse-pulse relationship.
IMPORTANT
The MAG Add-On Instruction can affect your positioning. The issuing
Kinetix 5100 drive (slave) uses the E-Gear ratio to define how it follows
pulses from a source (a master). While the result is that the issuing
Kinetix 5100 drive (slave) follows pulses from another source (master),
the way the function operates can affect positioning of the drive.
Regardless of Operation Mode, the E-Gear ratio is always used to
provide a representation of positioning (units or counts) or to define a
Pulse-Pulse Following relationship (MAG/PT). When the E-Gear ratio is
changed, the positioning of the axis is changed.
EXAMPLE
Gearing example:
The master in our system is a 4000 ppr encoder. When the encoder
makes one revolution, we expect the Slave1 drive to see: 4000
pulses.
Our application requirement is that we want to follow this encoder
at a 1:2 relationship. So, when the master encoder moves one
encoder revolution, the motor rotates two times.
The Master PPR is not entered anywhere, but is required that we
know this value. We calculate the MAG Set_MasterCounts value
knowing the Master PPR counts and the relationship we want in the
Slave1 motor.
We set the MAG Set_SlaveCounts = Motor Feedback Resolution =
16,777,216.
We set MAG Set_MasterCounts = 2000, so when the Slave1 drive
sees 2000 master pulses, the Slave1 motor moves one rotation, and
thus, as the Master encoder moves 4000 pulses, Slave1 would have
moved two rotations.

<!-- page 31 -->

## Appendix C Use Add-On Instructions

There are two modes of the MAG function that can be used. These modes are
defined by the Cfg_GearingMode entry. This entry is not visible and is set for
Fixed initially. You must intentionally change this setting. Fixed mode does
not impact positioning because it uses the existing E-Gear ratio in the Kinetix
5100 drive. Therefore, we can follow a master source at this fixed ratio and
when gearing is disabled, we can continue positioning without losing the
position scaling for the drive.
Variable mode lets you change the E-Gear ratio by manipulating the master/
slave counts values. The variable mode lets you change the ratio. When the
ratio is changed, the motor positioning is affected because the E-Gear ratio is
also used to define Position Scaling. If you require positioning after using the
variable gearing, issue a Homing Sequence to re-establish an origin.

> **Figure 270** — MAG Ladder Diagram

Operands
Operand
Type
Format
Description
Instance
raC_xxx_K5100_MAG
Tag
Unique instance of the MAG Add-On Instruction
Ref_Ctrl_Cfg
raC_UDT_Itf_PowerMotionSA_Cfg
Tag
Interface for Ctrl_Cfg of the Device Object
Ref_Ctrl_Set
raC_UDT_Itf_PowerMotionSA_Set
Tag
Interface for Ctrl_Set of the Device Object
Ref_Ctrl_Cmd
raC_UDT_Itf_PowerMotionSA_Cmd
Tag
Interface for Ctrl_Cmd of the Device Object
Ref_Ctrl_Sts
raC_UDT_Itf_PowerMotionSA_Sts
Tag
Interface for Ctrl_Sts of the Device Object
Set_MasterCounts
DINT (not visible)
Immediate or Tag
Sets the value of E-Gear Ratio; Denominator ID152 (P1.045)
Set this value to represent the desired counts/motor
rotation. This value defines the number of pulses/motor
rotation and when used with the feedback pulses you
expect to see from the source input (also pulses/
revolution) provides a gearing relationship.
Set_SlaveCounts
DINT (not visible)
Immediate or Tag
Sets the value of E-Gear Ratio; Numerator ID151 (P1.044)
Set this value the same as the Motor Feedback Resolution.
Cfg_GearingMode
BOOL (not visible)
Tag
0 = Fixed
1 = Variable
Set_AccelReference (future)
REAL (not visible)
Immediate or Tag
Units are 0.1 rpm/s for rotary motor
Range: 458…30,000,000
Mnemonic
Description
Sts_EN (Enable)
This bit is set when the rung makes a false-to-true transition and the message transaction
to MAG is initiated and in process. It remains high until the rung-in condition is false and no
faults are active.
Sts_DN (Done)
This bit is set when the rung makes a false-to-true transition and the message transaction
to MAG (Sts_EN) is complete.
Sts_ER (Error)
This bit is set when the rung makes a false-to-true transition and there is an error that has
occurred with the instruction. (This instruction error can be a result of a fault on the drive
itself). See Sts_ERR for details on the cause of the error.
Sts_IP (In Progress)
This bit is set when the rung makes a false-to-true transition, the MAG message transaction
is successful, and the drive begins following. This bit remains set as the motor is executing
the gearing. It remains set while the MAG is active, regardless of the rung-in condition.

**Extracted table (page 31, #1):**

| Type | Format |
| --- | --- |
| raC_xxx_K5100_MAG | Tag |
| raC_UDT_Itf_PowerMotionSA_Cfg | Tag |
| raC_UDT_Itf_PowerMotionSA_Set | Tag |
| raC_UDT_Itf_PowerMotionSA_Cmd | Tag |
| raC_UDT_Itf_PowerMotionSA_Sts | Tag |
| DINT (not visible) | Immediate or Tag |
| DINT (not visible) | Immediate or Tag |
| BOOL (not visible) | Tag |
| REAL (not visible) | Immediate or Tag |

<!-- page 32 -->

## Appendix C Use Add-On Instructions

Error Codes
•
100 - Kinetix 5100 drive is not ready
•
101 - Kinetix 5100 drive is faulted
•
103 - MSF is executing
•
105 - Drive is disabled
•
107 - raC_xxx_K5100_MAS is executing
•
108 - Other motion Add-On Instruction is sending the command
•
112 - AccelReference is out of range
•
127 - Previous command is not completed
•
129 - Motor is not connected
•
131 - Slave count is out of range 1 to (229…1)
•
132 - Master counts is out of range 1 to (231…1)
•
133 - Gear ratio is out of range (262144…1)
See Error Codes on page 524 for details.
raC_xxx_K5100_MAH(1)
Use the Motion Axis Home (raC_xxx_K5100_MAH) instruction to command a
homing procedure in the drive. Homing is used to define an origin for your
motor and to establish an absolute positioning reference for the motor. The
description of the different homing configurations is shown on page 298.

> **Figure 271** — MAH Ladder Diagram

You can use the F1 key to view fault error codes.
(1) The xxx in the name can be Dvc (legacy applications) or Opr (new applications).

<!-- page 33 -->

## Appendix C Use Add-On Instructions

Operands
Error Codes
•
100 - Kinetix 5100 drive is not ready
•
101 - Kinetix 5100 drive is faulted
•
103 - raC_xxx_K5100_MSF is running
•
105 - Drive is disabled
•
107 - raC_xxx_K5100_MAS is executing
•
108 - Other motion Add-On Instruction is sending the command
•
111 - SpeedReference is out of range
•
112 - AccelReference is out of range
•
113 - DecelReference is out of range
•
122 - HomingMethod is out of range
•
127 - Previous command has not completed
•
129 - Motor is not connected
See Error Codes on page 524 for details.
Operand
Type
Format
Description
Instance
raC_xxx_K5100_MAH
Tag
Unique instance of the MAH Add-On Instruction
Ref_Ctrl_Cfg
raC_UDT_Itf_PowerMotionSA_Cfg Tag
Interface for Ctrl_Cfg of the Device Object
Ref_Ctrl_Set
raC_UDT_Itf_PowerMotionSA_Set Tag
Interface for Ctrl_Set of the Device Object
Ref_Ctrl_Cmd
raC_UDT_Itf_PowerMotionSA_Cm
d
Tag
Interface for Ctrl_Cmd of the Device Object
Ref_Ctrl_Sts
raC_UDT_Itf_PowerMotionSA_Sts Tag
Interface for Ctrl_Sts of the Device Object
Set_HomingMethod
SINT
Tag
0…38 - See Table 112 on page 301 for Homing methods.
TIP: You can press F1 to see the Homing Methods from the Add-On Instructions help.
Set_PositionReferenc
e
REAL
Tag
The feedback position when a homing procedure is completed. Range: -2,147,483,648…+2,147,483,647
Set_SpeedReference
REAL
Tag
The first (high) speed reference. Units are 0.1 rpm for rotary motors. Range: 1…20,000
Set_HomeReturnSpee
d
REAL
Tag
The second (low) speed reference. Units are 0.1 rpm for rotary motors. Range: 1…5000
Set_AccelReference
REAL
Tag
Units are 0.1 rpm/s for rotary motors. Range: 458…30,000,000
Set_DecelReference
REAL
Tag
Units are 0.1 rpm/s for rotary motor. Range: 458…30,000,000
Mnemonic
Description
Sts_EN (Enable)
This bit is set when the rung makes a false-to-true transition and the message
transaction to Home is initiated and in process. It remains high until the rung-in
condition is false and no faults are active.
Sts_DN (Done)
This bit is set when the rung makes a false-to-true transition and the message
transaction to Home the drive (Sts_EN) is complete.
Sts_ER (Error)
This bit is set when the rung makes a false-to-true transition and there is an error that
has occurred with the instruction. (This instruction error can be a result of a fault on
the drive itself). See Sts_ERR for details on the cause of the error.
Sts_IP (In Process)
This bit is set when the rung makes a false-to-true transition, the Home message
transaction is successful, and the homing begins. This bit remains set if the homing is
executing.
Sts_PC(Process
Complete)
This bit is set when the rung makes a false-to-true transition and the Homing
Sequence is completed. Once homing is successfully completed, the CtrlSts.Homed bit
= 1.
You can use the F1 key to view fault error codes.

**Extracted table (page 33, #1):**

| Type | Format |
| --- | --- |
| raC_xxx_K5100_MAH | Tag |
| raC_UDT_Itf_PowerMotionSA_Cfg | Tag |
| raC_UDT_Itf_PowerMotionSA_Set | Tag |
| raC_UDT_Itf_PowerMotionSA_Cm d | Tag |
| raC_UDT_Itf_PowerMotionSA_Sts | Tag |
| SINT | Tag |
| REAL | Tag |
| REAL | Tag |
| REAL | Tag |
| REAL | Tag |
| REAL | Tag |

<!-- page 34 -->

## Appendix C Use Add-On Instructions

raC_xxx_K5100_MAT(1)
The Motion Axis Torque (raC_xxx_K5100_MAT) instruction lets you use torque
limiting while a pre-defined speed is used to move the motor. The first time the
pre-defined torque limit is reached, the Sts_TorqueReached bit is set. While
the Sts_TorqueReached bit is set, the MAT operation remains active until it is
terminated by an raC_xxx_K5100_MAS (Motion Axis Stop)/MSF (Motion Servo
Off), or a drive fault. The torque and speed entries are bi-directional.

> **Figure 272** — MAT Ladder Diagram

Operands
(1) The xxx in the name can be Dvc (legacy applications) or Opr (new applications).
Operand
Type
Format
Description
Instance
raC_xxx_K5100_MAT
Tag
Unique instance of the MAT Add-On
Instruction
Ref_Ctrl_Cfg
raC_UDT_Itf_PowerMotionSA_Cfg
Tag
Interface for Ctrl_Cfg of the Device Object
Ref_Ctrl_Set
raC_UDT_Itf_PowerMotionSA_Set
Tag
Interface for Ctrl_Set of the Device Object
Ref_Ctrl_Cmd
raC_UDT_Itf_PowerMotionSA_Cmd
Tag
Interface for Ctrl_Cmd of the Device
Object
Ref_Ctrl_Sts
raC_UDT_Itf_PowerMotionSA_Sts
Tag
Interface for Ctrl_Sts of the Device Object
Set_TorqueReference
DINT
Immediate or Tag
The limited value of motor torque, in the
unit of 0.1% of the motor rated torque.
Range: -4000…+4000
Set_TorqueRampTime
DINT
Immediate or Tag
Torque Ramp Time, the time (ms) it takes
to ramp up from 0 to the
TorqueReference. Range: 1…65500
Set_Speedlimit
REAL
Immediate or Tag
Speed limit that is used during the
constant torque operation: unit is 0.1 rpm
for rotary motor. Range: -
80,000…+80,000
Mnemonic
Description
Sts_EN (Enable)
This bit is set when the rung makes a false-to-true transition and remains set as the
message transaction to execute the MAT is initiated and in process. It remains high
until the rung-in condition is false and no faults are active.
Sts_DN (Done)
This bit is set when the rung makes a false-to-true transition and the message
transaction to the drive (Sts_EN) is complete.
Sts_ER (Error)
This bit is set when the rung makes a false-to-true transition and there is an error that
has occurred with the instruction. (This instruction error can be a result of a fault on
the drive itself). See Sts_ERR for details on the cause of the error.
Sts_IP (In Progress)
This bit is set when the rung makes a false-to-true transition, the MAT message
transaction is successful, and the motor begins to move. This bit remains set while the
MAT operation is active.
Sts_PC (Process
Completed)/
Sts.TorqueReached
This bit is set when the rung makes a false-to-true transition, the Sts_IP is set, and the
Set_TorqueReference value is reached. This bit is set (and remains set) on the first
occurrence of this condition.

**Extracted table (page 34, #1):**

| Type | Format |
| --- | --- |
| raC_xxx_K5100_MAT | Tag |
| raC_UDT_Itf_PowerMotionSA_Cfg | Tag |
| raC_UDT_Itf_PowerMotionSA_Set | Tag |
| raC_UDT_Itf_PowerMotionSA_Cmd | Tag |
| raC_UDT_Itf_PowerMotionSA_Sts | Tag |
| DINT | Immediate or Tag |
| DINT | Immediate or Tag |
| REAL | Immediate or Tag |

<!-- page 35 -->

## Appendix C Use Add-On Instructions

Error Codes
•
100 - Kinetix 5100 drive is not ready
•
101 - Kinetix 5100 drive is faulted
•
103 - raC_xxx_K5100_MSF is executing
•
105 - Drive is disabled
•
107 - raC_xxx_K5100_MAS is executing
•
108 - Other motion Add-On Instruction is sending the command
•
111 - Speed limit is > 80000 or < -80000
•
116 - TorqueReference is out of range
•
125 - TorqueRampTime is out of range
•
127 - Previous command has not completed
•
129 - Motor is not connected
See Error Codes on page 524 for details.
You can use the F1 key to view fault error codes.

<!-- page 36 -->

## Appendix C Use Add-On Instructions

Error Codes
Motion Error Codes (Sts_ERR) describes the error for Logix Designer.
Table 158 lists the error codes for Logix Designer application motion
instructions for the Kinetix 5100 drive.
You can use the F1 key to view fault error codes.

> **Table 158** — Kinetix 5100 Drive Add-On Instruction Error Codes

Error
Code
Description
Instruction Name(1)
MSO
MSF
MAFR
MAS
MAJ
MAT
MAI
MAM
MAH
MAG

Drive is not ready
X
X
X
X
X
X
X
X
X
X

Drive is faulted
X
X
X
X
X
X
X
X
X
X

raC_xxx_K5100_MSO is executing
X

raC_xxx_K5100_MSF is executing
X
X
X
X
X
X
X
X

Another raC_xxx_K5100_MSF is executing
X

Drive is disabled
X
X
X
X
X
X
X

Another raC_xxx_K5100_MAFR message is executing
X

raC_xxx_K5100_MAS is executing
X
X
X
X
X
X
X

Another RA motion Add-On Instructions is sending the
command
X
X
X
X
X
X

SpeedReference is out of range
X
X
X

AccelReference is out of range
X
X
X
X

DecelReference is out of range
X
X
X
X

StartingIndex is higher than 99
X

Torque is out of range
X

NonCyclicMoveType is higher than 3
X

CyclicMoveType is higher than 2
X

TravelMode is not either 2 or 10
X

HomingMethod is out of range
X

TorqueRampTime is out of range
X

Homing is not completed
X

Previous command has not completed
X
X
X
X
X
X
X

Motor is not connected
X
X
X
X
X
X
X
X
X
X

Gear slave counts is out of range
X

Gear master count is out of range
X

Gear ratio is out of range
X

Operation is not supported when device is virtual
X
X
X
X
X
X
X

Motor type not supported (Linear)
X
X
X
X
X
X
X
(1)
The xxx in the name can be Dvc (legacy applications) or Opr (new applications).

**Extracted table (page 36, #1):**

| Description |  |  |  |  |  |  |  |  |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
|  | MSO | MSF | MAFR | MAS | MAJ | MAT | MAI | MAM | MAH |
| Drive is not ready | X | X | X | X | X | X | X | X | X |
| Drive is faulted | X | X | X | X | X | X | X | X | X |
| raC_xxx_K5100_MSO is executing | X |  |  |  |  |  |  |  |  |
| raC_xxx_K5100_MSF is executing | X |  |  | X | X | X | X | X | X |
| Another raC_xxx_K5100_MSF is executing |  | X |  |  |  |  |  |  |  |
| Drive is disabled |  |  |  | X | X | X | X | X | X |
| Another raC_xxx_K5100_MAFR message is executing |  |  | X |  |  |  |  |  |  |
| raC_xxx_K5100_MAS is executing |  |  |  | X | X | X | X | X | X |
| Another RA motion Add-On Instructions is sending the command |  |  |  |  | X | X | X | X | X |
| SpeedReference is out of range |  |  |  |  | X |  |  | X | X |
| AccelReference is out of range |  |  |  |  | X |  |  | X | X |
| DecelReference is out of range |  |  |  | X | X |  |  | X | X |
| StartingIndex is higher than 99 |  |  |  |  |  |  | X |  |  |
| Torque is out of range |  |  |  |  |  | X |  |  |  |
| NonCyclicMoveType is higher than 3 |  |  |  |  |  |  |  | X |  |
| CyclicMoveType is higher than 2 |  |  |  |  |  |  |  | X |  |
| TravelMode is not either 2 or 10 |  |  |  |  |  |  |  | X |  |
| HomingMethod is out of range |  |  |  |  |  |  |  |  | X |
| TorqueRampTime is out of range |  |  |  |  |  | X |  |  |  |
| Homing is not completed |  |  |  |  |  |  |  | X |  |
| Previous command has not completed |  |  |  | X | X | X | X | X | X |
| Motor is not connected | X | X | X | X | X | X | X | X | X |
| Gear slave counts is out of range |  |  |  |  |  |  |  |  |  |
| Gear master count is out of range |  |  |  |  |  |  |  |  |  |
| Gear ratio is out of range |  |  |  |  |  |  |  |  |  |
| Operation is not supported when device is virtual |  |  |  | X | X | X | X | X | X |
| Motor type not supported (Linear) |  |  |  | X | X | X | X | X | X |

<!-- page 37 -->

## Appendix D

Full Closed Loop Control
The full closed (dual)-loop control can be useful when compliance or slip exists
between the motor and the load. A second encoder is used to close the position
loop. This encoder is located at the load side where accurate positioning is
required.
The full closed-loop control function is used to reduce the position error/offset
and improve the positioning accuracy at the load, since the load feedback
device is typically located on the load side of the mechanism.

> **Figure 273** — Wiring Diagram

Topic
Page
Full Closed-loop Control

IMPORTANT
This function applies to PT, PR, and IO operation modes.
Drive
Encoder
U
V
W
Motor
Coupling
Mechanism
Main
Shaft
Encoder
MFB
AUX
Encoder (A, B, Z) Signals
