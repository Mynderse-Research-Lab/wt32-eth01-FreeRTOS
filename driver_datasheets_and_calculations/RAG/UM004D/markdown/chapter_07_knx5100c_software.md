# Chapter 7: KNX5100C Software

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `08_Ch07_KNX5100C_Software.pdf` (60 pages)_

<!-- page 1 -->

Before You Begin
This section provides instructions to download and install KNX5100C
configuration software. It also provides procedures to launch KNX5100C and
configure your Kinetix® 5100 drive system with a Micro800™ controller by
using the Connected Components Workbench™ software.
For help using the Connected Components Workbench as it applies to
configuring the Micro800 controllers, see Additional Resources on page 14.
The Connected Components Workbench application, version 11.00 or later,
makes it possible to launch KNX5100C Configuration Tool and to configure a
Kinetix 5100 drive.
These procedures assume that you have wired your Kinetix 5100 drive system.
KNX5100C software is required for all applications. It is used to commission,
configure, and possibly program the Kinetix 5100 drive.
Topic
Page
Before You Begin

Download KNX5100C Software

Connect to the Drive/Set Your COM Port

Configure Drive Settings

Set the IP Address

Configure the Motor Selection in KNX5100C Software

Parameter Editor

Choose an Operation Mode

Configure Settings

Configure Position, Velocity, and Current Loops

Digital I/O and Jog Function in KNX5100C Software

<!-- page 2 -->

## Download KNX5100C

Software
KNX5100C configuration software is available for download at the Product
Compatibility Download Center (PCDC) website:
https://compatibility.rockwellautomation.com/Pages/home.aspx
and search Kinetix 5100.
To download the KNX5100C configuration software, perform the following
steps.
1.
Go to the Product Compatibility Download Center website.
The Compatibility and Downloads webpage appears.
2. In the Search PCDC window, enter Kinetix 5100.
3.
Click the appropriate software version and follow prompts to download.
4. Extract the zip file and run Setup.
Launch KNX5100C Configuration Tool
To launch the KNX5100C configuration tool, perform the following steps.
1.
Use one of two methods to launch the KNX5100C software:
a. On your personal computer desktop, double-click the KNX5100C
shortcut icon.

<!-- page 3 -->

b. If you have Connected Components Workbench software installed,
open it and select Tools > KNX5100C.
When the KNX5100C software is launched, it starts with a new project. If
you create a New Project, you can choose to add a New Device. When you
click Add, the drive is automatically created. If you cancel the New
Project prompt, you can open an existing file.
2. Choose the COM port associated with your drive.
If you are connected to your drive, click the Com port pull-down menu and select
a COM port. In some cases, the mini-USB cable has to be removed and reinserted
to refresh the COM port state.

<!-- page 4 -->

3.
Click Add to add your drive to the Function List.
By default, Kinetix 5100 is used as the device name.
Connect to the Drive/Set
Your COM Port
You can change the COM port associated with your drive if it has changed after
initial configuration. In some cases, the mini-USB cable has to be removed and
re-inserted to refresh the COM port state.
To change the COM port, perform the following steps.
1.
From the Function List, click Communication Setting.
2. Choose the COM port associated with your drive.
To add more drives that are connected your computer, from the File menu,
choose Add New Drive or right-click Start/Add Device in the Function List.
You can choose the COM port from the pull-down menu, or click
 to refresh
the COM port.

<!-- page 5 -->

3.
Click Connect to connect your drive.
The online process options dialog appears.
4. From the three options, choose the one that is best suited as your online
process, and then click OK.
5.
If you select the first option, an uploading message box appears until the
process is completed.
6. In the Communication Setting dialog, communication status is shown as
online.

<!-- page 6 -->

## Configure Drive Settings

If an I/O connection is established with a controller, the KNX5100C software is
then set in 'read-only' mode and configuration changes are not permitted. In
this situation, you can exit the I/O connection by checking ‘Inhibit Module’ on
the AOP 'Connection' page or when you disconnect the network cable from the
drive.
Set the IP Address
You can change the IP address (network) setting through the Function List.
To change the IP address, perform the following steps.
1.
Under the Function List, click Drive IP Address Setting.
2. Choose between STATIC IP and DHCP.
See Configure IP Address by Using BOOTP-DHCP Tool on page 115
If Static IP, then configure the IP address, Gateway, and Subnet mask.
For more information on setting the network parameters, refer to Chapter 5,
Set Up EtherNet/IP Communication.
Configure the Motor
Selection in KNX5100C
Software
In the KNX5100C software, navigate to Function List > Settings >
Motor Selection. There are three data sources available to configure the motor/
actuator through the Function List.
IMPORTANT
Settings are stored in nonvolatile memory, and changes to the IP
address take effect after power is cycled.

<!-- page 7 -->

You can specify the motor data source and configure the related parameter
value via the motor pages according to data source and motor type.
Data Source
You can specify the data source where the motor feedback values originate.
The available data sources are:
•
Motor NV: Motor parameter values come from the nonvolatile memory
of a motor. When the motor has an intelligent feedback device, the drive
is able to identify the motor when using NV parameters from the
feedback device.
•
Catalog number: Motor parameter values come from the selected catalog
number. Select the motor from a pull-down menu where the motor
database is embedded into the KNX5100C software.
•
Nameplate datasheet: Motor parameter values come from a motor
nameplate and datasheet. You must input the motor nameplate data
manually.
The parameters displayed in the Motor Selection dialog box depend on the data
source and the type of motor selected.
IMPORTANT
When the nameplate datasheet is selected, the parameter field of
nameplate/datasheet parameter section of the Motor, Model, and
Motor Feedback pages are visible, and you can enter these values.
If a data source other than Nameplate datasheet is selected, the
parameter field of nameplate/datasheet parameter section of the
Motor, Model, and Motor Feedback pages are dimmed and the motor
parameters values are read either from the intelligent encoder or the
KNX5100C software motor database.
The Motor Overload Limit parameter is not dimmed and can be
modified, see Motor NV on page 136.
When the data source is Catalog Number and a Kinetix LDC or Kinetix
LDL motor is selected, you must manually enter the parameters in the
Motor Feedback dialog box.

<!-- page 8 -->

## Motor NV

When Motor NV is selected as the data source and the attached motor has a
intelligent encoder with internal memory, the drive can identify the motor
automatically and read all motor parameter values.
If the attached motor does not have a intelligent encoder when Motor NV is
selected, then fault E 60B, Motor Selection Error, occurs. If the attached motor
has a Nikon encoder that is not a intelligent encoder and no motor parameters
are stored, then fault E 004, Motor Combination Error, occurs.
When Motor NV is selected as the data source, then all parameter data is
dimmed on the Motor Device Specification dialog box except for the Motor
Overload Limit field and the Next button.
1.
Type an appropriate Motor Overload Limit and click Next.
2. On the Motor Model Phase screen, click Next.

<!-- page 9 -->

3.
On the Motor Feedback Device screen, click Download to download all
parameters to the drive.
The following screen notifies you that the drive requires a reset and power
cycle.
After you click Yes, the drive performs the following sequence:
•
Sets ID628 (PN.000) MotorDataSource to 0 (Motor NV selection)
•
Sets the value of ID629 (PN.001) MotorOverloadLimit
•
Automatically triggers a drive reset and power cycle
After the reset and power cycle, the identified motor catalog number is shown
in the motor page, and the motor related parameters are updated according to
encoder internal memory.

<!-- page 10 -->

With Motor NV selected and the drive is power cycled, these other changes are
made:
•
Commutation alignment is fixed as Motor offset (A in the next figure)
•
The default Startup Method is set as Incremental (B in the next figure)
•
The position feedback is zero
If your application requires absolute positioning, you can change the startup
method to Absolute from the pull-down menu. If you do, click Download to
initiate the change. Then AbsoluteSystemSetting parameter ID242 (P2.069) is
set to 1 and the drive automatically triggers a power cycle. Warning A 06A
(Absolute position is not initialized) occurs after the power cycle, so you must
initialize the absolute position (using a Homing Command) to clear this fault
to let the absolute operation start.
IMPORTANT
If the startup method is Absolute and the TLP motor is configured, a
battery must be used to establish absolute positioning; regardless of
single or multi-turn operation. See Table 36 on page 73 for battery
specifications.
A
B

<!-- page 11 -->

## Catalog Number

When the catalog number is selected as the data source, you must select the
appropriate motor from the Change Catalog Number dialog box.
To change the catalog number, perform the following steps.
1.
Choose Function List > Settings > Motor Selection, and select Catalog
Number from the Data Source pull-down menu.
2. Click Change Catalog…
IMPORTANT
If the startup method is set to Absolute but the attached motor is not
equipped with an appropriate encoder or is not configured correctly,
then the following faults are posted:
Kinetix MP Motor-related Faults
Single Turn
(-E, -S)
Multi-turn
(-V, -M)
Incremental
(-H)
Change startup method to Absolute
E 069
A 06A
E 069
Kinetix TLP and TL/TLY (-B)
Motor-related Faults
Without Battery
With Battery
Change startup method to Absolute
A 060
A 06A

<!-- page 12 -->

3.
In the Change Catalog Number dialog box, find and select the motor for
your application.
4. Click OK to select that catalog number and to close the Change Catalog
Number dialog box.
When you click OK, all motor parameter values for that catalog number
are retrieved from the KNX5100C software motor database.
5.
From the Motor Device Specification dialog box, click Download.
6. Drive power is automatically cycled.
When power is restored to the drive, all motor parameter values for the
selected catalog number are downloaded to the drive and take effect.
Catalog number availability is from the motor database embedded into the
KNX5100C software. By using the catalog number, the drive can validate that the
correct motor is connected.
Use the Family and Feedback Type pull-down menus to further filter the related
motors.
IMPORTANT
If your motor has an intelligent encoder and you select the wrong catalog
number, an E 60A (Catalog Number Match Error) fault occurs at the drive.

<!-- page 13 -->

## Nameplate Datasheet

Motor parameter values are from a motor nameplate and datasheet. You must
input this data manually on the Motor and Model pages.
To select Nameplate datasheet as your data source, perform the following
steps.
1.
Choose Function List>Settings>Motor Selection>Motor Device
Specification, and select Nameplate Datasheet from the Data Source
pull-down menu.
Motor types available for selection are the following: Rotary permanent
magnet, linear permanent magnet, and induction motors.
2. After you add data manually to the fields under the Nameplate/
Datasheet-Phase to Phase Parameters section, click Next.
3.
From the Motor Model Phase to Neutral Parameters dialog box, add data
manually.
When you are done, click Next.

<!-- page 14 -->

4. From the Motor Feedback Device Specification dialog box, select
applicable feedback type from the Type pull-down menu.
5.
Click Download.
After the motor parameters are downloaded, you can use the motor
analyzer feature, where the drive analyzes the motor and provides
suggested parameter values.
• Motor Analyzer - Rotary Motors and Linear Motors on page 142
• Dynamic Motor Analyzer - Induction Motors on page 143
• Static Motor Analyzer - Induction Motors on page 144
• Inertia Estimation Motor Analyzer - Induction Motors on page 145
If you click Accept, drive power is automatically cycled. When power is
restored to the drive, all motor parameter values that you added manually are
downloaded to the drive and take effect.
Motor Analyzer - Rotary Motors and Linear Motors
Depending on the motor type you choose, the motor analyzer test is visible. To
use the motor analyzer feature for rotary and linear motors, perform the
following steps.
1.
Select Motor > Analyzer.
2. Click Start to initiate the analyze process.
3.
After each step of the analyzing process is completed, a confirmation
window appears; click OK.
A results window appears with suggested parameter values.
Motor Type
Feedback Type
Rotary Permanent Magnet and
Linear Permanent Magnet
Digital AqB
Digital AqB with UVW
Hiperface
Sine/Cosine
Induction Motor
Digital AqB

<!-- page 15 -->

4. Click Accept if you want to use those values, or click Cancel if you want to
stay with the parameters that you added manually.
Dynamic Motor Analyzer - Induction Motors
The dynamic motor test generates motion, be sure your motor is un-coupled to
perform this test. To use the motor analyzer feature for induction motors,
perform the following steps.
1.
Select Motor > Analyzer.
2. Select Dynamic Analysis and click Start to initiate the analysis.
3.
After each step of the analyzing process is completed, a confirmation
window appears; click OK.
A results window appears with suggested parameter values.
4. Click Accept if you want to use those values, or click Cancel if you want to
stay with the parameters that you added manually.

<!-- page 16 -->

## Static Motor Analyzer - Induction Motors

The static motor test does not generate motion. If Dynamic Analysis is not
possible for your system, select Static Analysis. To use the motor analyzer
feature for induction motors, perform the following steps.
1.
Select Motor > Analyzer.
2. Select Static Analysis and click Start to initiate the analysis.
3.
After each step of the analyzing process is completed, a confirmation
window appears; click OK.
A results window appears with suggested parameter values.
4. Click Accept if you want to use those values, or click Cancel if you want to
stay with the parameters that you added manually.
IMPORTANT
For Static Analysis, the test results (using an induction motor) are
affected by the value of the flux current parameter. If you cannot use
Dynamic Analysis to analyze (for example, due to mechanical
restrictions), we recommend that you set the value of rated flux current
to 40% of motor rated current to execute Static Analysis analysis. The
allowable setting range for rated flux current is 10…100% of rated
motor current.
IMPORTANT
If the drive is in the servo-on status before executing the Static
Analysis function, after you click Start, the KNX5100C software displays
a notification ‘Servo off before executing this function’ message.

<!-- page 17 -->

## Inertia Estimation Motor Analyzer - Induction Motors

To use the Inertia Estimation feature for induction motors, perform the
following steps.
1.
Select Motor > Analyzer.
2. Select Inertia Estimation and click Start to initiate the analysis.
A confirmation dialog box displays.
3.
Click Yes and a dialog box appears indicating that the drive will perform
servo-on to execute dynamic analysis.
A results window appears with suggested parameter values.

<!-- page 18 -->

4. Click Accept if you want to use those values, or click Cancel if you want to
stay with the parameters that you added manually.
Selection of Motor Thermal Models
The Kinetix 5100 drives contain two motor thermal-overload protection
algorithms that are used to prevent the motor from overheating.
Generic Motors
The default thermal model is a generic I2 T Class 10 overload protection
algorithm. This model is active if the MotorWindingToAmbientResistance
ID635 (PN.007) or the MotorWindingToAmbientCapacitance ID636 (PN.008)
values are 0.0. The purpose of this algorithm is to limit the time a motor is
operating with excessive levels of current.

> **Figure 77** — Motor Overload Curve

The relationship between Motor Overload Factory Limit trip-time and motor
output current is shown in Figure 77. Hot vs cold is determined by applying a
1st order filter, with a 20 minute time constant, to the output current. The hot
curve corresponds to a filtered output current of 100% or greater rated current.
The cold curve corresponds to a filter output current of 0%.
% Full Load Current
Class 10
Approximate Trip Time (s)
Hot
Cold

<!-- page 19 -->

You can use the MotorOverloadLimit ID629 (PN.001) attribute (default of
100%, max of 200%) to increase the motor overload trip-time by artificially
increasing the motor rated current (for thermal protection only). Increase
MotorOverloadLimit above 100% only if cooling options are applied.
Increasing MotorOverloadLimit causes MotorCapacity ID656 (PN.038) to
increase more slowly. The generic motor thermal model also derates the motor
rated current (for thermal protection only) when operating at low speeds. The
derating factor is 30% at 0 Hz and 0% at 20 Hz, with linear interpolation
between. Operating at output frequencies less than 20 Hz causes
MotorCapacity to increase more quickly. When the generic motor thermalmodel is active, the MotorCapacity attribute increases only if the motor output
current is greater than the effective motor rated current (taking into account
the MotorOverloadLimit and low speed derating factor). The default
MotorThermalOverloadFactoryLimit and MotorThermalOverloadUserLimit
values for this thermal model are both 100%.
Thermally Characterized Motors
If the ID635 (PN.007) MotorWindingToAmbientResistance and
MotorWindingToAmbientCapacitance ID636 (PN.008) attribute values are
both non-zero, the motor is considered thermally characterized and an
alternate motor thermal model is run. The purpose of this algorithm is to limit
the time a motor is operating with excessive levels of current. This thermal
model uses the firstorder time constant determined from the
MotorWindingToAmbientResistance and
MotorWindingToAmbientCapacitance values to estimate the motor thermal
capacity based on the motor output current. The MotorOverloadLimit ID629
(PN.001) attribute (default of 100%, max of 200%) can be used to increase the
motor overload trip-time by increasing the
MotorThermalOverloadFactoryLimit value. The MotorOverloadLimit should
be increased above 100% only if cooling options are applied. Increasing
MotorOverloadLimit does not change the behavior of MotorCapacity ID656
(PN.038). This thermal model supports setting the MotorOverloadAction
attribute as Current Foldback. Selecting the Current Foldback action results in
a reduction in the current reference via the MotorThermalCurrentLimit
attribute value that is reduced in proportion the percentage difference
between the MotorCapacity and the MotorOverloadLimit values. When this
thermal model is active, the MotorCapacity attribute is non-zero if the motor
output current is non-zero. The default MotorThermalOverloadFactoryLimit
and MotorThermalOverloadUserLimit values for this thermal model are both
110%.
IMPORTANT
The generic motor-thermal model does not support Current Foldback as
a Motor Overload Action.
IMPORTANT
This thermal model does not derate the motor-rated current when
operating at low speeds. Operating at low output frequencies does not
cause the MotorCapacity behavior to change.

<!-- page 20 -->

## Motor Feedback

With some motor type selections, the feedback type is automatically selected
for the corresponding motor. For others, the selection can be changed.

> **Figure 78** — Supported Motor Feedback

IMPORTANT
If you select Digital AqB when the attached motor has a Hall sensor,
the drive ignores the Hall sensor.

<!-- page 21 -->

To choose a motor feedback type, perform the following steps.
1.
Under Motor, select Motor Feedback.
2. On Motor Feedback Device Specification, choose either Incremental or
Absolute as the startup method.
3.
Under Commutation, choose one of the following from the Alignment
pull-down menu.
• Motor Offset: This setting is the default if the selected catalog number
has an intelligent encoder. The drive reads the internal commutation
offset saved in the intelligent encoder.
• Self-sense: This setting enables automatic magnetic field detection,
where the commutation offset is detected automatically from the drive
whenever the drive is powered up.
• Drive Offset: You must input the commutation offset and encoder
polarity manually.
Considerations for Drive Offset Alignment
If you select Drive Offset as the Commutation Alignment type, the valid range
of the Commutation Offset is calculated based on the Commutation Offset
from the encoder’s internal value (x):
x-85 < Commutation Offset setting < x+85 (unit: degree)
The chosen startup method determines how the device applies the feedback
count value during drive startup.
For Incremental, the drive uses zero for the feedback count accumulator at drive
startup.
For Absolute, the drive uses the absolute feedback position value from the
encoder at startup.
IMPORTANT
Kinetix 5100 drives support only incremental feedback on Kinetix LDAT
motors.
Any commutation offset that is detected by self-sense alignment is not shown in
the Offset field on Motor Feedback Device Specification.
Self-sense alignment cannot read commutation offset from CommutationOffset
parameter ID602 (PM.007).

<!-- page 22 -->

For example:
By using the KNX5100C software, choose
Start>Parameter Editor>Motor>CommutationOffset (ID 602, (PM.007)).
With Motor Offset selected as the commutation alignment, the parameter
ID602 (PM.007) CommutationOffset is 11.2°.
Therefore, the Commutation Offset setting for the Drive Offset type must be in
the range of 0…96.2° and 286.2…360°.
If you click Download and the input value of 200 exceeds the range, the
following warning appears.
You must change the input value before you can click Download again and
have the input value accepted.
Run a Commutation Test
The commutation test determines an unknown commutation offset and can
also be used to determine (or verify) the polarity of the start-up commutation
wiring.
You can choose to keep or discard the test results.
The following parameters are updated after commutation test:
•
Phase Sequence ID601 (PM.006)
•
Commutation Offset ID602 (PM.007)
•
Hall Hysteresis Width ID603 (PM.008)
To test commutation, perform the following steps.
1.
Uncouple the motor from the load.
IMPORTANT
This test mainly applies to third-party or custom permanent-magnet
motors equipped with (TTL with Hall) incremental encoders that are
not available as a catalog number in the Motion Database.
This test also applies to Kinetix MP and Kinetix TLP motors that are
available as a catalog number in the Motion Database, and use to
verify a known commutation offset or use the test result other than
the commutation offset specified in the motion Database.
ATTENTION: To avoid personal injury or damage to equipment, you
must uncouple the motor from each load you test as uncontrolled
motion can occur if an axis with an integral motor brake is released
during the test.

<!-- page 23 -->

2. In the Alignment field, select Drive Offset.
Different encoder types result in different configuration fields.
3.
Type an offset.
4. If the attached motor feedback type is Digital AqB with UVW (A in the
following figure) and the Commutation Alignment is Drive Offset (B),
then you must add additional data to the Hall Hysteresis Width (C) and
Hall Feedback Polarity (D) fields.
5.
Click Test Commutation.
After you click Test Commutation, the following message appears to alert
you that this operation resets the drive.
6. Click Yes.
A message window alerts you that the process might take some time to
complete.
When the process is complete, a results window appears with suggested
parameter values.
A
B
C
D

<!-- page 24 -->

7.
Click Accept to use the test result values, or click Cancel to stay with the
original parameter values.
Parameter Editor
You can read all Kinetix 5100 drive parameters of the servo drive and upload
them to your personal computer by using KNX5100C software, choose
Function List>Kinetix Drive>Parameter Editor. You can also use the
Parameter Editor to view or modify all Kinetix 5100 drive parameters then
download them to the servo drive.
The Parameter Editor consists of parameter categories (page 153), a
toolbar (page 154), and a status indicator that includes the firmware version
and other information.
To access the Parameter Editor, select Start > Kinetix 5100 > Setting >
Parameter Editor in the Function List.
Toolbar
Categories
Status Values

<!-- page 25 -->

All Kinetix 5100 drive parameters are divided into the following categories:
•
Motor
•
Drive
•
General
•
Status monitor
•
Control
•
I/O
•
Communication
•
Diagnosis
•
Motion
Click each parameter group tab to toggle between tabs.
The following information is displayed for each parameter:
•
ID
•
Name - for example, MotorType
•
Status - R (read only), S (set when servo disabled), P (requires a power
cycle, V (volatile; reset on power cycle)
•
Value - Click the box to the left of the parameter value to poll the drive for
the latest value
•
* - Indicates that a setting has changed
•
Unit
•
Min (value)
•
Max (value)
•
Default (value)
•
Parameter Number
•
Description
The Parameter Editor provides the status of the parameter:
•
(R) Read-only
•
(S) Value is set when servo power is off
•
(P) Value is applied after a Power Cycle
•
(V) Value is volatile (cleared once power is cycled)
The firmware version is also shown in this window.

<!-- page 26 -->

## Parameter Wizard

Double-click a parameter value to open the parameter wizard, which provides
a simple method to change parameter values.
Parameter Toolbar
The toolbar at the top of Parameter Editor contains six icons.
The function of each icon is as follows.
Save Parameters File
All Kinetix 5100 parameters that are shown on the screen (which are those also
saved to your personal computer) are saved as a *.par file.
Open Parameter File
All Kinetix 5100 parameter files (.par) on your personal computer can be
opened and displayed.
Read Parameters
All Kinetix 5100 parameters are read.
Download Parameters
When online with the Kinetix 5100 drive, a dialog box lets you choose to
download all the parameters or just the parameters that have been modified.
Stop Operation
This stops any operation in progress.

<!-- page 27 -->

## Compare Parameters

Use this function to compare the file you open with existing parameters.
1.
On your personal computer, navigate to your target saved *.par file.
2. Select the file and click Open.
A message appears that the comparison process has started.

<!-- page 28 -->

Choose an Operation Mode
There are three ways to change the operation mode in the KNX5100C software:
•
By using the Operation Mode Selection List
•
By using the Setting dialog box
•
By using the Parameter Editor
For detailed information on how the drive operates in each mode, see
Chapter 10, Modes of Operation.
Using the Operation Mode Selection List
To select an operation mode, perform the following steps.
1.
Use the pull-down menu to select an operating mode setting.
The operating modes are critical to the usability of different features in
the drive, to see which Operating Modes are available and their use, see
Chapter 10.
When you choose a new operation mode setting, the following message
appears.
If you click Yes, the drive is reset. We recommend this choice.
If you click No, you are returned to the Settings view without a power cycle.
However, a reminder appears until you initiate a power cycle.
IMPORTANT
ControlMode Parameter ID117 (P1.001) is valid only after power cycle.

<!-- page 29 -->

Using the Setting Page
To change the operation mode setting, perform the following steps.
1.
From the Function List, select Start > Kinetix 5100 > Settings.
2. From the Operation Mode pull-down menu, select an operation mode.
3.
Click
.
When you choose a new operation mode setting, the following message
appears.
Using the Parameter Editor
To change the control mode in the Parameter Editor, perform the following
steps.
1.
From Function List, select Start > Kinetix 5100 > Setting > Parameter
Editor.
If you click Yes, the drive is reset.
If you click No, the original operation mode is used.

<!-- page 30 -->

2. Change the ControlMode value by clicking directly in its value field and
typing a new value.
• Another way to change the parameter is to double-click the value,
which opens the ControlMode dialog box.
3.
In KNX5100C software, choose the Operation Mode from the main
screen pull-down menu.
After the parameter value is changed, the following reminder appears.
When the parameter value is changed and has not been downloaded to the drive,
there will be a '*' mark next to the value field.
Click the box immediately to the left of the parameter to see the present value of
the parameter in the drive. The drive is polled immediately and updates the
Value field.

<!-- page 31 -->

## Configure Settings

From the Settings selection, you can view and change the operation mode. The
Operation Mode control loop diagram is updated based on the mode.
When the Operation Mode value is changed from the pull-down menu,
subsequent pull-down menus can appear.

<!-- page 32 -->

The Operation Mode block control loops change as values are selected from
both pull-down menus.
Configure General Settings
The General Setting page lets you configure parameters related to the motor,
shunt, and brake operation.
To access the General Setting page, perform the following steps.
1.
From the Function List, select Start > Kinetix 5100 > Settings > General
Setting.
2. In General Settings, change the fields manually as needed.
Any changes to these settings require the motor to be disabled.
3.
Click Download to send any changed parameters to the servo drive.
The gray boxes show the functions included in the each operation mode.
You can double-click a box to open a dialog box and to configure the related
functions.

<!-- page 33 -->

## Enable Vertical Load Control (Motor Holding Brake)

A common requirement for a load is to be held using a holding brake. A
holding brake can be used with a vertical or horizontal load. A vertical load is
classified as a load that stores potential energy either by gravity or spring
effect. In this type of load, the Kinetix servo motor must hold part or all the
load, even when the motor is not moving, but is still powered by the drive. A
horizontal load does not store potential energy (either by spring or gravity
effect) when the motor is disabled. Vertical or horizontal loads can use a motor
holding brake to keep the load stationary while the motor is disabled. Holding
brakes are not designed to stop a motor, but rather to hold a motor when it is
stationary and disabled.
While this feature is named 'Vertical Load Control' it can be used to setup any
load type used with a holding brake. This feature is enabled in the KNX5100C
software Function List>Settings>General Setting and enables the automatic
control of the motor holding brake.
Once the checkbox is selected:
•
The Brake DO setting is chosen and a DO is assigned as [0x08] Brake
Control
•
The Brake Engage / Disengage times are set to non-zero values (these
must be changed to match your motor holding brake engage/disengage
times, that data is obtained in the Kinetix Rotary Motion Specifications
Technical Data, publication KNX-TD001)
•
The Zero Speed Range is set to a non-zero value; this value is used as one
of the conditions to engage the holding brake
•
MotorStopMode ID675 (P1.032).Y is set to use Enable Vertical Load
Control
There must be a DI that is user assigned to provide the enable/disable function
for the motor with the associated timing that is used with the Brake control
DO.
There are three DIs that can be used, the characteristics of each DI is shown in
Table 71.

> **Table 71** — Holding Brake Digital Inputs

## DI Signal

Characteristic
[0x01] Servo On
• Uses brake timing.
• Provides one signal for enable/disable operation of motor.
• Can be used with explicit read/write operation.
• By default uses dynamic braking when deceleration is active (similar to a current
deceleration operation), which can be aggressive.
• Can use disable and coast MotorStopMode ID675 (P1.032).
[0x47] Profile Quick Stop
• Uses brake timing.
• This input is solely used to decelerate and disable the motor.
• Requires an enabling signal (second input is used to enable the motor).
• Uses a programmable deceleration profile AutoProtectionDecelTime ID296
(P5.003).
• Alarm is issued at the end of the disable operation.

<!-- page 34 -->

Servo on with Holding Brake is available starting from firmware revision 2 and
is the preferred input when using a motor holding brake.
The holding brake timing is shown in the KNX5100C software. Configure the
vertical load control feature using these steps:
1.
Select Enable Vertical Load Control checkbox.
2. Verify the Enagage Delay Time for your motor (this is negative indicating
that the motor remains enabled while the holding brake is engaging).
Verify the Disengage Delay Time for your motor.
3.
Navigate to Function List > Settings > Parameter Editor > Motion, and
find ID296 (P5.003); double-click the Value.
4. Use the pull-down menu for C: PFQS (Profile Quick Stop) and select the
deceleration time to use before the motor is disabled.
(AutoProtectionDecelTime ID296 (P5.003))
This value is programmable by using the PR Mode Editor > Speed and
Time Setting > Accel/Decel Time.
5.
Navigate to Function List > Digital IO/Jog Control and check 'Edit DIO
Configuration', then using an available DI, use the pull-down menu to
select Servo on with Holding Brake and make this a NO type of input.
6. Verify that the motor holding brake has been selected; if it is not selected,
use the pull-down menu and associate a DO as Brake Control.
[0x48] Servo on with
Holding Brake
• Uses brake timing.
• Uses a programmable deceleration profile AutoProtectionDecelTime ID296
(P5.003).
• In IO mode, this input is not required to be assigned, these operations occur with
raC_xxx_K5100_MSO and raC_xxx_K5100_MSF commands.

> **Table 71** — Holding Brake Digital Inputs

## DI Signal

Characteristic

<!-- page 35 -->

<!-- page 36 -->

The scope trace below shows the holding brake timing:
1.
DI is on and transitions off.
2. Motor is decelerated (blue pen is motor speed).
3.
Motor reaches zero speed
4. DO Brake Control is off and Brake EngageTime begins timing
5.
Once the BrakeEngageTime expires, the motor is disabled
Configure the Command Source
The selected operation mode allows the modification of different parameters.
The configuration of the Command Source is available with: I/O Terminal
Block input, Speed Mode, and Torque Mode. Command Source is not available
for Position Register (PR) or IO Modes.
Configure the Command Source for Position mode (I/O Terminal block input -PT Mode)
To configure the Command Source for Position mode, perform the following
steps.
1.
From the Function List, choose Start>Kinetix 5100>Setting and select the
Operating Mode as PT:Position Mode (I/O terminal block input).
In Command Source, the Position Mode (I/O terminal block input) tab is
used.
IMPORTANT
If you have configured a dual or multiple operation mode, more than
one tab is visible. For example, if you have PT/S mode, you get a
Position mode (I/O terminal block input) tab and a Speed mode tab.

<!-- page 37 -->

2. On the Position Mode tab, select either Pulse Train or Analog Input as the
position command source.
3.
If you select Pulse train, you can specify the External Pulse Input Type,
Source (the drive port from where the command originates), Filter Type,
and Pulse Filter Width.
4. Select the Source to be used.
When the Control Mode is PT (Position Terminal), the source indicates
which port the Master Pulses originate.
5.
Click Download to download any changed parameters to the servo drive.
For more information, see PT Mode (Position Command with I/O
Terminal Block Input) on page 237.
Configure the Command Source for Speed Mode (-S mode)
1.
From the Function List, choose Start>Kinetix 5100> Setting and select
the Operating Mode as S:Speed Control Mode.
2. Click the Speed Command box.
IMPORTANT
The Speed Mode tab is visible if you have configured a dual or
multiple operation mode, such as PT/S mode.

<!-- page 38 -->

3.
On the Speed Mode tab, select the speed command source from either an
analog input or preset speed registers that is triggered by using binary
weighted digital inputs signals DI.SPD0 and DI.SPD1.
You can change the values of the Speed registers 1…3 by using
ID125…ID127 (P1.009…P1.011).
4. Click Download to send any changes to the servo drive.
For more information, see Configure and Select the Preset Speeds on
page 246.
Configure the Command Source for Torque Mode (-T Mode)
1.
From the Function List, choose Start>Kinetix 5100>Setting and select the
Operating Mode as T:Torque Control Mode.
2. Click the Torque Command box.
IMPORTANT
The Torque Mode tab is visible if you have configured a dual or
multiple operating mode.

<!-- page 39 -->

3.
In the Torque Mode tab, select the torque command source from either
an analog input or preset torque registers that can be triggered by using
binary weighted digital inputs signals DI.TCM0 and DI.TCM1.
You can change the values of the torque registers 1…3 by using
ID128…ID130 (P1.012…P1.014).
For more information, see Selection of Torque Command on page 252.

<!-- page 40 -->

Configure the Pulse Outputs
Use the Pulse Output as a form of buffered encoder outputs. Use these pulses
to provide master feedback signal to another drive, to provide feedback for
closed loop operation, or pulse train control is used with an external controller.
To access and use the Pulse Outputs, perform the following steps.
1.
From the Function List, select Start > Kinetix 5100 > Setting > Pulse
Output.
2. In the Pulse Output dialog box, you can configure the following:
• The output polarity of OA/OB/OZ to either forward or reverse
• The source of pulse output from Motor Encoder (MFB), Auxiliary
Encoder (Aux), or Pulse Command (I/O)
• The Output Pulse Type to by-pass, Ratio Output without Quadruple, or
Ratio Output with Quadruple
• The Output Pulse Number is the output pulse count that you can
change to match your application requirements. The Output Pulse
Number is visible when Ratio Output with Quadruple is selected. This
value is set as a default that you can change to match your application.
When Ratio Output without Quadruple is selected, you can change the
numerator and denominator to match your application.
3.
You can also use the Settings>Parameter Editor>General to manually
change the Encoder Output Resolution ID153 (P1.046), which is counts.
4. Click Download to send any changes to the servo drive.
IMPORTANT
Output Pulse Type selection depends on the source of the pulse
output. For example, by-pass is only available when Pulse Command
(I/O) is chosen as the source.

<!-- page 41 -->

## Configure Electronic Gear (E-Gear) Ratio

The E-Gear ratio configuration is important and serves two purposes
depending on the Operation Mode:
•
PT Operation mode/IO mode - (raC_xxx_K5100_MAG Add-On
Instruction only). The E-Gear ratio represents a pulse-pulse relationship
between a master and slave source (Pulse-Pulse follower). In this mode,
the drive internally has no conversion constant (counts/mm or inch), any
user position conversions must be considered as pulses.
•
PR Operation Mode/IO mode. When in this mode, the E-Gear ratio
represents Position Scaling (PUU). This mode allows you to define the
number of feedback counts/motor rotation.
Regardless of the Operation Mode, the E-Gear ratio is always used to provide
either a representation of Position Scaling (PUU) or a Pulse Following
relationship.
If you change the gear ratio in your application, when servo is off, you can
define multiple numerators but only one denominator. The numerator of the
E-Gear ratio can be selected via the DI.GNUM0 and DI.GNUM1 signals. If the
DI.GNUM0 and DI.GNUM1 signals are not defined, ID151 (P1.044) is the
default numerator. To avoid mechanical vibration, switch the DI.GNUM0 and
DI.GNUM1 signals during servo off status.
See Description of Digital Input Functions on page 433.

> **Table 72** — Relevant Parameters(1)

(1)
Do not change the setting value in servo on state.
Parameter
Name
ID151 (P1.044)
GearRatioslaveCountsN1
ID236 (P2.060)
GearRatioslaveCountsN2
ID237 (P2.061)
GearRatioslaveCountsN3
ID238 (P2.062)
GearRatioslaveCountsN4
ID152 (P1.045)
GearRatioMasterCounts (denominator)
ID170 (P1.068)
PositionCmdMovingFilterTime
IMPORTANT
– If you change gear ratio during operation, you can cause an abrupt, uncontrolled
motion event.
– ID236 (P2.060), ID237 (P2.061), and ID238 (P2.062) are for PT Mode only. ID151 (P1.044) is
for both PR Mode and PT Mode. All of the parameters can be changed in PT Mode only.
– If PT/PR Mode is selected, and ID151 (P1.044) is changed in PT Mode, it remains active
through any subsequent execution if you switch to PR Mode.
GNUM0, GNUM1
Position Command
Moving Filter
Time Constant
ID170 (P1.068)
Pulse
GearRatioFollowerCounts N1 - ID151 (P1.044)
GearRatioFollowerCounts N2 - ID236 (P2.060)
GearRatioFollowerCounts N3 - ID237 (P2.061)
GearRatioFollowerCounts N4 - ID238 (P2.062)
GearRatioMasterCounts - ID152 (P1.045)
Numerators
Denominator
Position Command
Low Pass Filter
Time Constant
ID124 (P1.008)
Feedback Pulse
Pulse
Error

<!-- page 42 -->

To configure the E-Gear ratio, open the E-Gear Ratio dialog box in the
KNX5100C software (from the Function List, select Start > Kinetix 5100 >
Setting > E-gear Ratio). The different settings in the E-Gear Ratio dialog box
are explained.

> **Figure 79** — E-Gear Ratio

The Gear Ratio Selection (1) and GNUM0/1 (4) are not used in IO mode. They
are used in PT Mode to select different gear ratios.
Item
Description

Gear Ratio Selection - Pull-down menu to choose the different ratios (N1…N4).

Gear Ratio Follower Counts (N1) - Set this value as the motor feedback resolution.

Gear Ratio Master Counts (M) - This value is set depending on the Operation Mode.
Default values are 100,000 counts for a high-resolution encoder.

GNUM0/1 - These choices are mapped to the Digital Inputs that represent binary
weighted values to select the Gear Ratio value.

<!-- page 43 -->

## Setting Gear Ratio Follower Counts (ID151, P1.044)

The Gear ratio Follower Counts is sometimes called the numerator, because
when you look at how it is used in the drive, it is used to determine the internal
'ratio' of the drive (shown as 1677.72 in Figure 79). Therefore, the E-Gear ratio
Follower equals the Effective Resolution of the motor feedback.

> **Figure 80** — Motor Selection > Feedback Dialog Box

Setting Gear Ratio Master Counts (ID152, P1.045) - PT Operation Mode
The Gear ratio Master Counts parameter ID152 (P1.045) is sometimes called the
denominator, which by default is defined as 100,000 counts/motor rev. You
can also use with IO mode - raC_xxx_K5100_MAG Add-On Instruction.

> **Figure 81** — Gear Ratio Master Counts

When the Follower counts is set equal to motor effective resolution, then the
Gear Ratio Master counts is desired counts/motor rotation. Desired counts, in
this case, are not used for position scaling, but are used to define the gearing
ratio. The Master counts are set based on the number of feedback pulses you
expect to receive from the Master source input and is used to define your
overall gearing relationship. So, this Master counts value is used to define the
effective 'ratio' of your gearing relationship.
IMPORTANT
The numerator is always set as the Effective Resolution. We use Master
counts (denominator) to represent your ratio (depending on your
Operation Mode). When using DI.GNUM0 and DI.GNUM1 for different gear
ratios, we are changing the Slave/Follower Counts entry. DI.GNUM0/1
can only be entered as Follower Counts (ID236…ID238, P2.060…P2.062),
this means the desired ratio must be normalized - because we can only
represent DI.GNUM0/1 as Follower Counts value, but the Master Counts
represents your ratio. Normalizing is done by changing the Drive Ratio
(shown in Figure 79 as 1677.72).

<!-- page 44 -->

For example, the master in our system is a 4000 ppr encoder. Which means,
when the encoder makes one revolution, we expect the Slave1 drive to see 4000
pulses. Our application requirement is that we want to follow this encoder at a
1:2 relationship. Which means, when the master encoder moves 1 encoder
revolution, the motor rotates 2 revolutions. The Master PPR is not entered
anywhere, but it is required that we know this value. We calculate the E-Gear
Master Counts value knowing the Master PPR counts and the relationship we
want in the motor of Slave1. In our example, the Gear Ratio Master Counts =
2000, which means when Slave 1 sees 2000 pulses from the master, it rotates 1
revolution, and thus, as the master moves 4000 pulses (one rotation), Slave1
moves 2 rotations.
IMPORTANT
When you change the E-Gearing ratio, you are affecting the position
scaling of your drive. So, you cannot expect to position correctly if you
change modes to a positioning mode when you are finished following
pulses. You must first re-establish a home, then set the E-Gear ratio
correctly for your application Position units.
IMPORTANT
There is NO positioning ability while using the pulse follower function in
PT Mode. Furthermore, when the E-Gear ratio is changed in PT Mode,
and you change sub-modes back to a position-based mode (this means
using any other Opr_AOI), your position scaling changes. When you keep
a 'fixed' gearing ratio, then the positioning is maintained because the EGear ratio values did not change. The drive follows pulses based on the
present value of the E-Gear ratio.

<!-- page 45 -->

Setting Gear Ratio Master Counts (ID152, P1.045) - Using any other Positioning Mode
The Gear ratio Master Counts (Figure 81) parameter is sometimes called the
denominator, which by default is defined as 100,000 counts/motor rev.
The Master counts value is desired counts/motor rotation. So, this Master
counts value is used to set the 'position scaling' for your axis. The value can be
any encoder count value that can be converted to give you your position units/
motor rotation, which includes your mechanical transmission.
For example:
The ball screw pitch is 3 mm, when using a 4 bit encoder, if the E-Gear ratio is
set to 16777216/3000, then the workpiece moves 1 μm per 1 pulse command.
You can create Position Units in the Logix environment by using the Device Object
Handler Add-On Instruction
IMPORTANT
When you change the E-Gearing ratio, you are affecting the position
scaling of your drive. So, you cannot expect to position correctly after
you have changed the E-Gear ratio. You must first re-establish a home,
then set the E-Gear ratio correctly for your Position units.
E-Gear Status
Gear Ratio
Moving Distance per 1 Pulse Command
E-Gear is not applied
E-Gear is applied
Motor
Platform
Workpiece
Ball screw pitch:
3 mm = 3000 µm

= 1

(Unit:
)
rev
μm
pulse
μm
rev
pulse
=
=
x
16777216
3000
16777216
3000
=
3000
16777216
1 (Unit:
)
rev
μm
pulse
μm
rev
pulse
=
=
x
16777216
3000
3000
16777216

<!-- page 46 -->

## Configure Filter

There are different filters available to use with the Kinetix 5100. There is a Low
Pass Filter and Moving Average Filter which are both used to remove
unwanted resonance from the drive (these filter types are available for use in
different drive modes). See Filter on page 256 for more information.
Use the Filter dialog box to configure the Kinetix 5100 Low Pass and Moving
Average Filters.
To access Filter, perform the following steps.
1.
From the Function List, choose Start > Kinetix 5100 > Setting > Filter.
2. In the Filter dialog box, you can configure the following:
• The Low Pass Filter smoothing time constant parameters for Position
Command ID124 (P1.008), Speed Command ID122 (P1.006) and
ID123 (P1.007). This filter is a first order low pass filter. This filter is a
smoothing filter that creates a gradually increasing output when an
input step is applied. The time constant determines how fast the Low
Pass Filter output reaches the level of input applied. This filter uses the
Position Command Smoothing Constant ID124(P1.008) to determine
the cutoff frequency. The cutoff frequency is 1/(2*PI*P1.008). The
units for ID124(P1.008) are in 10 ms, which means if 100 is entered into
ID124(P1.008), the actual value that is used by the formula is 1000 ms.
The same cutoff frequency formula is used for the Speed and Torque
Command Smoothing Constant (ID122(P1.006) and ID123(P1.007))
except the units are ms and not 10 ms.
• The Moving Average Filter time constant parameters for Position
Command ID170 (P1.068), and Speed Command ID164 (P1.059).

<!-- page 47 -->

• S-curve.
These features are only applicable to S-curve profile types. The
Acceleration ID141(P1.034) and Deceleration ID145(P1.035) Constants
come from the cycle profile Acc/Dec settings. In this image, the PR
Index Accel/Decel values are 200 ms. These values are populated in
ID141(P1.034) and ID145(P1.035).
The smoothing constant (TSL) provides a way to add smoothing to the
jerk properties and is shown in this diagram.
Filters available for Position Mode, Speed Mode, and Torque Mode are
described in Filter on page 256 in Chapter 10, Modes of Operation.
3.
Click Download to send any changes to the servo drive.

<!-- page 48 -->

## Configure Notch Filter

The Notch Filter is used to attenuate a specific resonant frequency. The
Kinetix 5100 drive uses up to five Notch Filters simultaneously.
To access the Notch Filter, perform the following steps.
1.
From the Function List, choose Start > Kinetix 5100 > Setting > Notch
Filter.
2. In the Notch Filter dialog box, you can modify the five notch filters.
3.
Click Download to send any changes to the servo drive.
For more information, see Resonance Suppression (Notch Filter, Speed
Mode) on page 259.
To edit any notch filter parameter, you must first check Manual for that
notch filter.
You can use any frequency diagnosing tool (FFT) to diagnose the frequency
and magnitude of the resonance.
You can use the System Analysis tool to diagnose the frequency of
resonance.

<!-- page 49 -->

## Configure Limits

From the Function List, choose Limit to configure Position, Speed, and Torque
Limits.
In the Limit dialog box, there are two tabs: Position and Speed Limit, and
Torque Limit.
Position and Speed Limit
On the Position and Speed Limit tab, the Software Limits define a linear
counts range for valid travel. When motor travel is outside the travel limit, an
alarm is generated (A283/A284) and motion is permitted in the opposite
direction of the limit.
You can change the accel/decel rates used for the detection of the software
limits in Function List>Motion Control>PR Mode Editor>Deceleration Time
for Auto-protection ID296 (P5.003)
Click Download to send any changes to the servo drive.
For more information, see Analog Outputs and Monitoring on page 282.

<!-- page 50 -->

## Torque Limit

On the Torque Limit tab, you can enable or disable torque limits and set torque
limit values. You can use limited torque values by setting P1.002.Y to 1 (Enable)
or 0 (Disable) or by using the digital input DI:TRQLM. You can also select
torque command preset values by changing binary weighted digital inputs,
TCM1 and TCM2.
For more information on the torque limit, see Analog Outputs and Monitoring
on page 282. For more information on digital input function descriptions, see
Description of Digital Input Functions on page 433.
Configure Analog I/O
The Kinetix 5100 servo drive can use different analog input signals for
commands. These operations include using analog inputs representing
Position, Speed, or Torque from another command source (another controller
for example).
The drive can use up to two analog outputs to display selected drive
parameters. This output is typically used to provide status information to
another device. This servo drive provides two output channels for this purpose,
MON1 and MON2.
From the Function List, choose Analog I/O to select the type of analog
command input and the output data to be monitored.
In the Analog I/O dialog box, there are two tabs: Command Input and Output
Monitor.
On the Command Input tab, you can configure three types of analog input:
position mode (I/O terminal block input), speed input, and torque input.

<!-- page 51 -->

Position Mode (I/O terminal block input) Tab
This mode is useful when you want to relate an analog voltage command to
motor position. By using the analog input, you can relate an analog voltage to a
number of motor rotations. Use this feature for positive (up to +10V) or
negative (down to -10V) rotations. The conversion of volts/motor revs used
here is: 10V yields a maximum of P1.066 (Maximum Rotation Number) motor
revs. The volts/rev is scaled using this formula. On this tab, you can enable the
analog position function, set the initial position of the motor, and the
maximum motor position value (in motor rotations).

<!-- page 52 -->

## Speed Input Tab

This mode is useful when you want to relate an analog voltage command to
motor speed. By using the analog input, you can relate an analog voltage to a
motor speed in RPM. Use this feature to define positive (up to +10V) or
negative (down to -10V) speed. The conversion of volts/motor RPM used here
is:
10V yields a maximum of P1.040 (Maximum Output Speed) RPM.
The volts/RPM is scaled using this formula. For example, if a ControlLogix®
1756-M02AE module was configured for Velocity mode, its analog output could
be used with this speed input for the Kinetix 5100 drive to provide full closedloop control.
On this tab, you can set the maximum output speed by using parameters
ID147 (P1.040) and ID679 (P1.081). Select the Max Speed value by using a
Digital Input.
Click Download to write any changed parameters to the servo drive.
Torque Input Tab
This mode is useful when you want to relate an analog voltage command to
motor torque. By using the analog input, you can relate an analog voltage to a
motor torque. Use this feature to define positive (up to +10V) or negative
(down to -10V) torque limits. The conversion of volts/motor torque used here
is: 10V yields a maximum of P1.041 (Maximum Output Torque). The volts/
motor torque % is scaled by using this formula. For example, if a ControlLogix
1756-M02AE module was configured for Torque mode, its analog output could
be used with this torque input for the Kinetix 5100 drive to provide full closedloop control.

<!-- page 53 -->

On this tab, you can change the maximum output torque command by using
parameter ID148 (P1.041).
Click Download to write any changed parameters to the servo drive.
Output Monitor Tab
On the Output Monitor tab, you can change the Monitored Value using a pulldown menu. You can change the scaling, proportion, and the polarity of the
output.
The Mon Calculator lets you enter a unique Motor Speed (can be the maximum
Motor Speed) with your desired Analog Voltage at that speed. Click Calculate
to determine the corresponding Maximum values and analog scaling
ID120 (P1.004). For more information, see Analog Outputs and Monitoring on
page 282.

<!-- page 54 -->

Configure Position,
Velocity, and Current Loops
The Operation Mode selection dictates which control loops are active and that
you can modify. Each dialog box lets you configure the parameters for the gain
and filter values that correspond to the command type.
Configure Position Loop
From the Function List, choose Position Loop to view or change the
parameters that apply to the position command.
Click Download to download any changed parameters to the servo drive.

> **Table 73** — Position Loop Parameters

Parameter
Name
ID185 (P2.000)
PositionProportionalGain
ID187 (P2.002)
VelocityFeedforwardGain
ID188 (P2.003)
VelocityFeedforwardLowPassFilterTimeConstant
ID235 (P2.053)
PositionIntegralGain

<!-- page 55 -->

## Configure Velocity Loop

From the Function List, choose Velocity Loop to view or change the parameters
that apply to the velocity command.
Click Download to write any changed parameters to the servo drive.

> **Table 74** — Velocity Loop Parameters

Parameter
Name
ID189 (P2.004)
VelocityProportionalGain
ID191 (P2.006)
VelocityIntegralGain
ID192 (P2.007)
AccelerationFeedforwardGain
ID232 (P2.049)
VelocityFeedforwardLowPassFilterTimeConstant
ID144 (P1.037)
LoadInertiaRatio

<!-- page 56 -->

## Configure Current Loop

From the Function List, choose Current Loop to view or change the parameters
that apply to the current command.
The affected parameter is ID210 (P2.025) Resonance Suppression Low Pass
Filter Time Constant. This Low Pass Filter is used to dampen an aggressive
output that can potentially cause an unwanted resonance with the mechanical
equipment.
Click Download to write any changed parameters to the servo drive.
Digital I/O and Jog Function
in KNX5100C Software
From the Function List, choose Digital IO/Jog Control to view or change the
I/O function and to see the status of digital inputs (DI) and digital outputs
(DO), or to control the I/O signals manually.
There are three sections in the dialog box: Digital Input (DI), Digital Output
(DO), and Jog Control.

<!-- page 57 -->

Configuration and Status of Digital Input (DI) and Digital Output (DO)
Signals
In the Digital Input (DI) and Digital Output (DO) sections, the user defined
signals are shown with their individual configurations.
If the contact type of the DI or DO signal is normally closed, 'NC' is added at
the end of the signal name. Click ‘Edit DIO Configurations’ to change the
configuration of the signal.
The Status column shows the status of the digital I/O. This is the LOGICAL
level of the input that is based on the use of N.O. or N.C. This is NOT the actual
voltage on the terminals (0V DC = OFF, 24V DC = ON).
An example is shown in Figure 82. DI3 is configured as an N.C. input. The
demo box has DI3 toggle switch ON (24V DC to the input). Notice that the
Status in the Control Panel is Off. This is because the configuration is NC, the
drive interprets (and expects) this state/condition as being OFF.

> **Figure 82** — Status Example

This dialog box also shows the On/Off status of the DI or DO signals and offers
manual control of the DI or DO signal state. This control is useful when testing
or troubleshooting the signals.

<!-- page 58 -->

## Control Digital Input Signals Manually

To control the digital input signals manually, go online with the drive, then
clear the Edit DIO Configurations checkbox and click the Enable checkbox.
Use the On/Off buttons on the right side to enable the DI and DO control. You
can use these On/Off buttons to control the DI or DO signals while the drive is
connected.
To change and control the DI signals manually, perform the following steps.
1.
Check Enable DIO Configurations.
2. Configure the Digital Inputs as required.
3.
Clear the Edit DIO Configurations checkbox.
4. Go online with the drive. You might have to download your KNX5100C
project file.
5.
Check Enable so that On/Off is visible.
6. Click On/Off to change the status of the DI signals directly.
You can see the signal status by looking at the Status window.
Control Digital Output Signals Manually
To change and control the digital outputs via the communication software
settings when the servo drive is connected, perform the following steps.
1.
Check Edit DIO Configurations.
2. Configure the Digital Outputs as required.
3.
Clear the Edit DIO Configurations checkbox.
4. Go online with the drive. You might have to download your KNX5100C
project file.
5.
Check Enable DO Override so that On/Off is visible.
6. Click On/Off to change the status of the DO signals directly.
You can see the signal status by looking at the Status window.

<!-- page 59 -->

## Edit DIO Configurations

When Edit DIO Configurations is checked, the digital input (DI) and digital
output (DO) actual state is always Off. To change the function and status of DI
and DO signals, perform the following steps.
1.
Check the box next to Edit DIO Configurations to enable the editing
function.
2. Use the pull-down menu to change the DIO function in the drive.
3.
Click OK to save the changes and write to the drive.
4. When you have configured all your I/O, clear the Edit DIO
Configurations checkbox.

<!-- page 60 -->

## Jog Function

See Description of Digital Input Functions on page 433 for information on the
individual DI functions.
See Description of Digital Output Functions on page 437 for information on
the individual DO functions.
The Jog Function commands the motor to run at a constant speed (Jog Speed).
To control the jog operation, enter the desired jog speed and then determine
the motor rotation direction. The existing motor rotation (Settings > General
Setting > Rotation Direction) is used as the directional context in this dialog
box.
There are two ways to initiate a Jog function on the Kinetix 5100 drive.
1.
Use DIO to configure the Jog and Servo On Digital Inputs. Use Enable
On/Off of DI1 (Servo On) signal manually.
a. Check Enable DIO Configurations.
b. Configure the Servo On Digital Input.
c. Configure the Motor JOGs in the forward direction input and, if
required, configure the reverse direction.
d. Clear the Edit DIO Configurations checkbox.
e. Go online with the drive.
f. Check Enable so that On/Off is visible.
g. Click On/Off to change the status of the DI signals directly.
You can see the signal status by looking at the Status window. To jog the
motor, first enable the drive by using Servo On, then Jog forward or
reverse as your application requires.
2. Use the Jog Control Panel and check Forced Servo On.
a. When Forced Servo ON is checked, or your digital input is ON, click
the left and right arrows to jog the motor in that specific direction.
b. Stop clicking the left and right arrows to stop the motor rotation.
c. If the observed rotation is opposite to what is desired, check Invert
Direction.
The direction of the jog command is inverted.
