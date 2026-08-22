# Chapter 6: Keypad Interface

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `07_Ch06_Keypad_Interface.pdf` (12 pages)_

<!-- page 1 -->

Keypad Input and Panel
Display
The Kinetix® 5100 drive is equipped with a diagnostic status indicator and five
push buttons that are used to display information and to edit a limited set of
parameter values.
The drive has three status indicators: Charge, MOD, and NET. For an
explanation of their functions, see Status Indicators on page 449. For more
information on how the keypad and status indicator can be used to do tuning,
see Autotuning via the Drive Panel on page 204.

> **Figure 72** — Keypad and Display

Topic
Page
Keypad Input and Panel Display

Drive Displays

Edit Settings From the Display

Item
Key
Name
Description

—
Display
A 5-digit, 7-segment status indicator that displays the monitoring
values, parameters, and setting values.

Mode key
Use this key to return to the parent menu or, if there is one, to return to
the previous menu.

Up key
Use this key to return to the previous menu. It is also increases the
values that you edit while in the configuration and parameter edit
screens.

Down key
Use this key to advance to the next menu. It also decreases the values
that you edit while in the configuration and parameter edit screens.

Shift key
Use this key to toggle between the digits or menus in the same level.

Set key
Use this key to enter a sub-menu, if one exists, or to confirm a value
that you have edited.
5100
QUALITY
S
M

**Extracted table (page 1, #1):**

| Key Na | me |
| --- | --- |
| — Di | splay |
| Mo | de key |
| Up | key |
| Do | wn key |
| Sh | ift key |
| Se | t key |

<!-- page 2 -->

## Drive Displays

After the drive boots up successfully, it displays ‘Kinetix 5100’ briefly and then
transitions to the Drive Status display, assuming the drive is disabled/servo
off. If the drive is enabled/servo on with no faults, then the Real Time display is
shown. If a fault occurs, then the fault screen displays an E, followed by the
fault code. If a warning occurs, then the fault screen displays an A, followed by
the warning code.
For more information on fault codes, see Chapter 16, Troubleshoot the Kinetix
5100 Drive System.
Main displays are as follows.
•
Real time data
•
Drive status
•
Setting
Real Time Data
Real Time Data screen shows the real-time value of the selected parameter. The
Real Time Data display format is as follows.
Use the
 keys to move between parameters.
When the drive is enabled/servo on, you can use the
 key to access the Drive
Status screen to view information. If no button is pressed and no fault or alarm
occurs within one minute, then the display reverts back to Real Time Data.
Boot
Fault
Power Up/
Reset
Boot
Completed
Drive
Status
Setting
Boot
Version
Information
Device
Information
Drive
Disable/
Real Time
Display
Drive Enable/
Servo On
Drive is Running
No key event/fault
Information Display/Setup
FB.PUU˽00000˽UU
Units
Space
Parameter value
Parameter Name
(first parameter is
shown in this example)
Space

<!-- page 3 -->

> **Figure 73** — Real Time Data Display

For example:
On the Real Time Display, the drive displays FB.PUU. When you press
, the
drive displays C-PUU, which is the input pulse number. See Table 68 for the
description of the Real Time Display symbols.

> **Table 68** — Real Time Display Symbols

Code
Real Time Displayed
Symbol
Description
Unit

Motor feedback pulse number after the scaling of
electronic gear ratio.
User unit

Input pulse number of pulse command after the
scaling of electronic gear ratio.
User unit

The deviation between control command pulse and
feedback pulse number.
User unit

Motor feedback pulse number (encoder unit) (1.28
million count/rev)
Count

Input pulse number of pulse command before the
scaling of electronic gear ratio. (encoder unit)
Count

Error pulse number after the scaling of electronic
gear ratio. (encoder unit)
Count

Input frequency of pulse command.
kHz

Motor speed.
rpm

Speed command.
Volt
P3
P2
P1
...
...
...
PN
Drive
Status
Drive
Disable/
Real Time
Display
Drive Enable/
Servo On
Drive is Running
No key event/fault
Information Display/
Setup Mode
Fault
Occurred
Fault
Mode
Fault
Clear

**Extracted table (page 3, #1):**

| Real Time Displayed Symbol |  | Description |
| --- | --- | --- |
|  |  | Motor feedback pulse number after the scaling of electronic gear ratio. |
|  |  | Input pulse number of pulse command after the scaling of electronic gear ratio. |
|  |  | The deviation between control command pulse and feedback pulse number. |
|  |  | Motor feedback pulse number (encoder unit) (1.28 million count/rev) |
|  |  | Input pulse number of pulse command before the scaling of electronic gear ratio. (encoder unit) |
|  |  | Error pulse number after the scaling of electronic gear ratio. (encoder unit) |
|  |  | Input frequency of pulse command. |
|  |  | Motor speed. |
|  |  | Speed command. |

<!-- page 4 -->

Speed command.
rpm

Torque command.
Volt

Torque command.
%

Average torque.
%

Peak torque.
%

Main circuit voltage.
Volt

Load/motor inertia ratio.
If 13.0 is displayed, the actual inertia is 13.
1 time

IGBT temperature.
°C

Resonance frequency. Low byte is the first resonance
and high byte is the second resonance.
Hz

The absolute pulse number of encoder Z phase
equals the homing value, 0. The absolute pulse
number can be +5000 or -5000 pulses depending on
whether the motor rotates in a forward or reverse
direction.
—

Map parameter number 1 displays the content of
parameter ID55 (P0.025). Specify the map target by
using ID60 (P0.035).
—

Map parameter number 2 displays the content of
parameter ID56 (P0.026). Specify the map target by
using ID61 (P0.036).
—

Map parameter number 3 displays the content of
parameter ID57 (P0.027). Specify the map target by
using ID62 (P0.037).
—

Map parameter number 4 displays the content of
parameter ID58 (P0.028). Specify the map target by
using ID62 (P0.038).
—

Monitor variable number 1 displays the content of
parameter ID663 (P0.009). Specify the monitor
variable code by using ID668 (P0.017).
—

Monitor variable number 2 displays the content of
parameter ID664 (P0.010). Specify the monitor
variable code by using ID669 (P0.018).
—

Monitor variable number 3 displays the content of
parameter ID665 (P0.011). Specify the monitoring
variable code by using ID670 (P0.019).
—

Monitor variable number 4 displays the content of
parameter ID666 (P0.012). Specify the monitoring
variable code by using ID667 (P0.020).
—

Offset value between motor position and Z phase in
PUU unit. The value is 0 when the position overlaps
with Z phase. The greater the value, the greater the
offset.
User Unit

Current drive fault.
—

> **Table 68** — Real Time Display Symbols (Continued)

Code
Real Time Displayed
Symbol
Description
Unit

+5000
+5000

**Extracted table (page 4, #1):**

| Real Time Displayed Symbol |  | Description |
| --- | --- | --- |
|  |  | Speed command. |
|  |  | Torque command. |
|  |  | Torque command. |
|  |  | Average torque. |
|  |  | Peak torque. |
|  |  | Main circuit voltage. |
|  |  | Load/motor inertia ratio. If 13.0 is displayed, the actual inertia is 13. |
|  |  | IGBT temperature. |
|  |  | Resonance frequency. Low byte is the first resonance and high byte is the second resonance. |
| 0 +5000 0 +5000 0 |  | The absolute pulse number of encoder Z phase equals the homing value, 0. The absolute pulse number can be +5000 or -5000 pulses depending on whether the motor rotates in a forward or reverse direction. |
|  |  | Map parameter number 1 displays the content of parameter ID55 (P0.025). Specify the map target by using ID60 (P0.035). |
|  |  | Map parameter number 2 displays the content of parameter ID56 (P0.026). Specify the map target by using ID61 (P0.036). |
|  |  | Map parameter number 3 displays the content of parameter ID57 (P0.027). Specify the map target by using ID62 (P0.037). |
|  |  | Map parameter number 4 displays the content of parameter ID58 (P0.028). Specify the map target by using ID62 (P0.038). |
|  |  | Monitor variable number 1 displays the content of parameter ID663 (P0.009). Specify the monitor variable code by using ID668 (P0.017). |
|  |  | Monitor variable number 2 displays the content of parameter ID664 (P0.010). Specify the monitor variable code by using ID669 (P0.018). |
|  |  | Monitor variable number 3 displays the content of parameter ID665 (P0.011). Specify the monitoring variable code by using ID670 (P0.019). |
|  |  | Monitor variable number 4 displays the content of parameter ID666 (P0.012). Specify the monitoring variable code by using ID667 (P0.020). |
|  |  | Offset value between motor position and Z phase in PUU unit. The value is 0 when the position overlaps with Z phase. The greater the value, the greater the offset. |
|  |  | Current drive fault. |

<!-- page 5 -->

## Drive Status Display

This display provides drive information and access to the Setting display,
which shows network and parameter information. When the drive is enabled/
servo on, the information can only be viewed.
Use the
 keys to move between status information displays.
Device Information Screen
Version Information Screen
This screen displays hardware and firmware versions of the drive. The display
string is
. h represents the hardware version, and F
represents the firmware version.
Setting Screen
When the drive is disabled/servo off, the Setting screen lets you edit the
network address or drive parameters, or reset the drive.
For more information, see Edit Settings From the Display on page 122.

Position feedback from the auxiliary encoder.
User Unit

Position deviation between the position feedback
and the command from the auxiliary encoder.
User Unit

Feedback position deviation between main encoder
and auxiliary encoder.
User Unit
Drive Status Name
Display String Description
Standby
Stdby
If the drive has passed boot steps and self-testing, but the drive has
not been configured.
• DHCP off: Stdby 192.168.1.180
• DHCP on (before IP address is assigned): Stdby_DHCP_0.0.0.0
• DHCP on (after IP address is assigned): Stdby_DHCP_192.168.1.180
Stopped
StoP
If the drive has been configured, but it is still not enabled/servo on.
Running
RUN
The drive is enabled/servo on.
Fault
A fault or warning occurred during operation.

> **Table 68** — Real Time Display Symbols (Continued)

Code
Real Time Displayed
Symbol
Description
Unit
Drive
Status
Setting
Version
Information
Device
Information
Information Display/Setup
h 01.002
01.102
F

**Extracted table (page 5, #1):**

| Real Time Displayed Symbol |  |  |  |  |  |  | Description |
| --- | --- | --- | --- | --- | --- | --- | --- |
|  |  |  |  |  |  |  | Position feedback from the auxiliary encoder. |
|  |  |  |  |  |  |  | Position deviation between the position feedback and the command from the auxiliary encoder. |
|  |  |  |  |  |  |  | Feedback position deviation between main encoder and auxiliary encoder. |

<!-- page 6 -->

Edit Settings From the
Display
Access the Setting display from the Drive Status display by pressing the

key.
When the drive is disabled/servo off, perform the following steps.
1.
Press the
 key to enter the editing mode.
Use the
 keys to scroll through the Network setting, reset, and
parameter setting displays.
2. Press the
 key to return to the previous display.

> **Figure 74** — State Switch Machine of Information Display and Setup

## Edit Network Settings

From the Setting display, perform the following steps.
1.
Press
 to go to Network Setting display.
2. Press
 again to enter the Static IP display.
From that display, there are two choices:
• Press
 again to set a static IP address. See Set Static IP Address.
• Press
 to turn DHCP on or off. See Turn DCHP On or Off on
page 123.
Set Static IP Address
On the Static IP Address display, perform the following steps.
1.
Press the
 key to enter the IP Setting display.
2. Press the
 key to enter the edit display.
3.
Use the
 keys to move between the IP Address, Gateway, and Subnet
setting screens.
See Figure 75.
4. Press
 in any one of those settings displays and use the
 keys to
edit the values.
5.
Press the
 key twice to set the values and return to the original display.
Drive
Status
Setting
Version
Information
Device
Information
Information Display/Setup

<!-- page 7 -->

> **Figure 75** — State Switch Machine of Setting

Turn DCHP On or Off
On the DHCP display, perform the following steps.
1.
Press the
 key to enter the DHCP editing display.
2. Press the
 key to turn the switch OFF or ON.
3.
Press the
 key to return to the DHCP editing display.
4. Press the
 key to return to the IP Setting display.
Setting
Network
Setting
Static IP
Subnet
Setting
IP Setting
Gateway
Setting
Number
Editing
Change
Numbers
Number
Editing
Number
Editing
Change
Numbers
Change
Numbers
Setting
Network
Setting
Static IP
DHCP
ON/OFF

<!-- page 8 -->

## Edit Parameter Settings

From Settings display, perform the following steps.
1.
Press the
 key to get to Network Settings.
2. Press the
 key for Parameters.
3.
Press the
 key for the parameter editing mode.
Starting with group 0, use
 to move between parameter numbers
within the group.
4. Use
 to move to the next parameter group.
Use the
 keys to move between parameter numbers within a group.
5.
Once you have navigated to the parameter you want to edit, press the

key to select it.
Use the
 keys to change the value of the selected digit (flashing). Use
the
 key to move between digits.
6. Press the
 key to confirm the edit or the
 key to cancel the change
and to return to the Parameter display.
When you press the
 key, the drive saves the value and displays Saved or
another status message on the display. See Table 69 on page 125.
Setting
Network
Setting
Parameter
Parameter
Parameter
Parameter
Parameter
Parameter
Parameter
Parameter
Mode
Parameter Group 0
Parameter Group 0
Change Value
Confirm
Change Digits
Cancel

<!-- page 9 -->

## Save Display

When you have set the parameter, press the
 key to save. The display shows
one of the following symbols for one second.
Example
Reset the Drive via Keypad
To reset the drive, perform the following steps.
1.
From the Parameter display, press the
 key to get to the Reset display.
2. On the Reset display, press the
 key.
The reset string blinks.
3.
Press the
 key again.

> **Table 69** — Parameter Status Display

## Displayed Symbol

Meaning
Description
Saved
Correctly saved the setting value.
Out of Range
Incorrect value or the input is reserved for this value.
Power On
The parameter will be effective after the servo drive is repowered.
S
S
S
Power On
Stand by Display
Press
 key for Set menu.
Press
 key to select and
press
keys for Parameter
Press
 key to select.
ID100 (P0.000) is displayed.
Press
 key to select.
Power cycle the Kinetix 5100 to
activate the ControlMode setting.
Press
 key to navigate to ID116 (P1.000).
Then press
keys to for ID117 (P1.001).
It displays the current value of the ControlMode ID117 (P1.001)
parameter. Use the
 key change the value to required setting.
Press
 key to select.

<!-- page 10 -->

The drive resets.
4. Press the
 key to return to the Parameters display.
Display Low Byte, High Byte, and Negative Values
In the real-time data display, all values are scrolled and displayed as decimal or
hexadecimal. On the parameter editor screen, the value range can be shown in
one of two ways:
•
The real-time value is ‘short’ or 16 bits (can be shown in one screen).
•
The real-time value is ‘long’ or 32 bits (must be shown in two screens).
The first screen is the high byte and the second screen is the low byte. In
these instances, use the
 key to move between screens.
How the panel displays 16-bit and 32-bit values is shown in Table 70.
Figure 76 shows the panel display of positive and negative signs.

> **Table 70** — 16-Bit and 32-Bit Display Formats

Example of the displayed value
Description
 (Dec)
16 bits
If the value is positive 12345, the display shows
12345 in decimal format.
 (Hex)
If the value is 0x011F, the display shows 0x011F in
hexadecimal format; the highest digit is not
shown.
 (Dec high)
32 bits
If the value is (positive) 1230478900, the display
for the high byte shows 12304, and display for the
low byte shows 78900, both in decimal format.
 (Dec low)
 (Hex high)
If the value is 0x001F0000, the display for the
high byte shows h001F, and display for the low
byte shows L0000 in hexadecimal format.
 (Hex low)

> **Figure 76** — Panel Display

## Positive Sign

Negative Sign

<!-- page 11 -->

## Display Fault Record

From the Diagnosis display, use the
 key to move between high and low
byte. Parameter ID274 (P4.000) FaultRecordN displays the most recent fault. It
is not read-only and can be set to 0 to reset all fault records.
Parameters ID275…ID278 (P4.001 …P4.004) are read-only.
Diagnosis Parameters via Keypad
You can use the keypad to display the status of digital inputs and outputs.
Digital Input Diagnosis Operation
When external output signal triggers DI1…DI10, the display shows the
corresponding signal by bit. When the bit is equal to 1, the DI is on.
The second recent error.
The first recent error.
The third recent error.
The fourth recent error.
The fifth recent error.
S
S
S
S
S

<!-- page 12 -->

For example, if hexadecimal number 3FE1 is displayed, the binary equivalent
for E is 1110, then DI6…DI8 are on.
Digital Output Diagnosis Operation
The output signal DO1…DO5 are triggered and the corresponding signals are
shown on the display by bit. When 1 is displayed, the DO is on.
For example, if hexadecimal number 1F, is displayed, the binary equivalent for
F is 1111, then DO1…DO4 are on.
Display Firmware Upgrade Information
See Upgrade Kinetix 5100 Drive Firmware on page 477.
S
 1 1 1 1 1 1
1 1 1 0
0 0 0 1
14 13
12 11 10 9

Binary code
Corresponding DI status
The display is in hexadecimal format.
S
0 0 0 1
1 1 1 1

Binary code
Corresponding DI status
The display is in hexadecimal format.
