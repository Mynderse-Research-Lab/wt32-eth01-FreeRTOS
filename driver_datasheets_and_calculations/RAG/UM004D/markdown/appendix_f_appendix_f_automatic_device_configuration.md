# Appendix F - Automatic Device Configuration

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `23_AppF_Automatic_Device_Config.pdf` (6 pages)_

<!-- page 1 -->

## Appendix F

Automatic Device Configuration
Automatic Device Configuration (ADC) function is supported in Kinetix® 5100
firmware revision 2 and later.
This function is only available with IO mode operation and allows for an
original drive configuration (parameters) to be automatically downloaded to a
new Kinetix 5100 drive. This is useful when a replacement drive is used, drive
parameters have been changed and no longer match the original drive
configuration (parameters), or when KNX5100C software is not accessible.
Before ADC is activated on the original Kinetix 5100 drive, any typical drive
settings or changes to the drive must be completed. This includes tuning, IO
configuration, E-Gear ratio, hookup/polarity, and other common drive
configuration settings.
New Kinetix Drive ADC
Preparation
To prepare the new Kinetix 5100 drive:
1.
Apply logic power to the new drive
2. Remove any Ethernet port connections (Port1/Port2)
3.
Change the Kinetix 5100 drive IP address:
• Use the keypad to match the IP address, subnet, and gateway settings
to the original drive. See Edit Network Settings on page 122.
4. Verify the Kinetix 5100 drive Operation Mode:
• If this is a new drive, the factory default operation mode is PT
(Position mode-terminal block input). The drive must be set for IO
Operation Mode. Change the Operation Mode using the drive keypad
(see Table 69 on page 125). Control Mode ID117(P1.001) = 0x0c.
It is optional to cycle power to the drive. With power applied, re-connect the
Ethernet port connections. After a short synchronizing period, the new drive
updates with the controller's stored configuration (parameters) and the
Kinetix 5100 Module AOP status is Running.
Topic
Page
Get Started

Compare the Configuration Data

Upload the Configuration Data

Overwrite the Configuration Data

<!-- page 2 -->

Appendix F Automatic Device Configuration

> **Figure 275** — Module Configuration for ADC

> **Figure 276** — Module Properties>General Settings Shows ADC Configuration Is Active

> **Figure 277** — Configuration Screen

IMPORTANT
Automatic Device Configuration (ADC) does not support induction and
linear motor parameters.

<!-- page 3 -->

## Appendix F Automatic Device Configuration

Get Started
Version 2 and later of the Kinetix 5100 module AOP and Kinetix 5100 drive
firmware lets you change the Kinetix 5100 Module Properties (General >
Configured By) setting for:
•
Configured By: External Means, or
•
Configured By: This Controller
When you choose ‘Configured By: This Controller’, the ADC function is active
and the established Configuration Signature is used to validate the original
Kinetix 5100 drive. IO operation mode must be used, and the Ethernet/IP™
connection between the controller and drive must be active.
When an active connection exists between the controller and the Kinetix 5100
drive, changing any Kinetix 5100 drive parameters using Class 3 messaging or
KNX5100C software is not permitted. Controller-drive data exchange only
occurs when the drive and controller signatures are synchronized. The ADC
function is only active with I/O mode when the control mode parameter ID 117
(P1.001) value is 0X0C.

> **Figure 278** — Module Definition

IMPORTANT
Updating Kinetix 5100 drives Major Firmware levels can cause loss of
ADC functionality. Configuration with K5100C software followed by
correlating the configuration from the drive into Logix might be
required.
‘Configured By: External Means’ allows KNX5100C software to create and modify
the configuration, then download it to the Kinetix 5100 drive via the Kinetix 5100
drive USB port.
The different values in the 'Connection' field determines the assembly output
instance and motion function.
• Data with Camming: Assembly output instance 106 is configured to use
raC_Opr_K5100_XXX Add-On Instructions and the Device Object Handler can be
used.
• Data: Assembly output instance 106 is configured for use with Version 1
raC_Dvc_K5100_XXX legacy Add-On Instructions.

<!-- page 4 -->

## Appendix F Automatic Device Configuration

Compare the Configuration
Data
When first activating ADC, the Configuration Signatures are different. If the
configuration data in the controller and the Kinetix 5100 drive are different,
data exchange between the drive and controller cannot be established. In this
case, the Kinetix 5100 drive Ethernet/IP connection is not inhibited and the
drive shows Code 16#000c Service Request Error: Invalid mode or state for
service request.

> **Figure 279** — Configuration Compare Error

Use the Add-On Profile (AOP) Configuration dialog box to determine whether
the configuration signature data in the controller and the Kinetix 5100 drive
are different. The signature and date/time in the controller are once again
stored when one of the following occurs:
•
The AOP is created. The value is based on the original Kinetix 5100 drive
AOP values, which might (or might not) match the replacement drive
values. To view the original drive values, use the KNX5100C software
after resolving differences using ‘Overwrite’.
•
When you click ‘Upload’ in the ‘Resolve Differences’ dialog box to upload
the configuration data into the AOP.
•
When you click ‘Overwrite’ in the ‘Resolve Differences’ dialog box.
The signature of the connected Kinetix 5100 drive is automatically read when
you click 'Refresh'. If the signature in the controller and the connected
Kinetix 5100 drive are different, the Module status is Faulted. The differences
must be resolved or overwritten before the Kinetix 5100 drive Module status is
Running. When that occurs, data exchange between the drive and controller
can begin.

> **Figure 280** — Configuration Signature

<!-- page 5 -->

## Appendix F Automatic Device Configuration

Upload the Configuration
Data
Click 'Upload' in the 'Resolve Differences' dialog box to upload the
configuration data from the Kinetix 5100 drive into the AOP.

> **Figure 281** — Upload Configuration Data

After the upload is complete, the Configuration Signature and Date/Time in
the controller is changed to be the same as the connected Kinetix 5100 drive
when the Upload was clicked. After you click 'Apply', the Configuration data is
saved in the Studio 5000® project and controller. These configuration settings
(parameters) are applied and executed by the controller to the Kinetix 5100
drive when the Module Status is Running.

> **Figure 282** — Apply Configuration Data

<!-- page 6 -->

## Appendix F Automatic Device Configuration

Overwrite the Configuration
Data
To overwrite the configuration data in the Kinetix 5100 drive with the settings
that are stored in the Studio 5000 project and the controller, click 'Overwrite'
in the 'Resolve Differences' dialog box.

> **Figure 283** — Overwrite the Configuration Data

After you click 'Overwrite', the Date/Time of the configuration data stored in
the Studio 5000 project and the controller is updated, which gives those
configuration settings the latest time stamp. After you click 'Apply', the
Configuration data with the newer date is saved in the Studio 5000 project and
the controller. These configuration settings (parameters) are applied and
executed by the controller to the Kinetix 5100 drive when the Module Status is
Running.

> **Figure 284** — Apply Controller Configuration Data
