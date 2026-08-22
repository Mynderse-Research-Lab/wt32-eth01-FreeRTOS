# Chapter 8: Studio 5000

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `09_Ch08_Studio_5000.pdf` (11 pages)_

<!-- page 1 -->

## Designer Application

Studio 5000 Logix Designer
Application
For help using the Studio 5000 Logix Designer® application as it applies to
configuring the ControlLogix® or CompactLogix™ controllers, see the
Additional Resources on page 14.
Version History
Each release of the Studio 5000 Logix Designer application makes possible the
configuration of additional Allen-Bradley® motors, actuators, power supplies,
and drive features not available in previous versions.
Table 75 shows whether the AOP must be downloaded and installed. In later
versions of Logix Designer application, the AOP is already installed.
Topic
Page
Studio 5000 Logix Designer Application

Configure the Logix 5000 Controller

Configure the Kinetix 5100 Drive Modules

Support Automatic Device Configuration (ADC) in AOP Version 2 and Later

Connection RPI

Inhibiting/Un-inhibiting an I/O Connection

Download the Program

IMPORTANT
Support for Studio 5000 Logix Designer is available only when you set
Control Mode parameter (ID117 (P1.001)) to IO Mode (0x0C).
IMPORTANT
To configure additional motors, actuators, and drive features with
your Kinetix® 5100 servo drive, you must have drive firmware
revision 1.xxx or 2.xxx. Refer to Table 75 to determine if you need to
install the Kinetix 5100 drive Add-on Profile.

<!-- page 2 -->

Install the Kinetix 5100 Add-On Profile
If Table 75 indicates you must download the AOP, download Add-On profiles
(AOP) from the Product Compatibility Download Center (PCDC) website:
rok.auto/pcdc.
Follow these steps to download the Kinetix 5100 Add-On Profile.
1.
Go to the Product Compatibility Download Center.
The Compatibility & Downloads webpage appears.
2. Click Download.
3.
Enter Kinetix 5100 in the Search PCDC window.
4. Click the appropriate AOP revision and follow prompts to download.
5.
Extract the AOP zip file and run Setup.
Configure the Logix 5000
Controller
These procedures assume that you have wired your Kinetix 5100 drive system.
These procedures show the dialog boxes for following devices.
•
ControlLogix 5570 controller with a 1756-EN2TR EtherNet/IP™
communication module
•
CompactLogix 5370 controller with an embedded EtherNet/IP
connection
See the list of other compatible Logix PAC® and PLC Controller Platforms in
Kinetix 5100 Drive System Overview on page 17.

> **Table 75** — AOP Installation Requirement

## Drive Firmware Revision

Logix Designer Application Version
Kinetix 5100 AOP Needed
1.xxx or 2.xxx
30.00, 31.00, 32.00
Yes
33.00 or later
No
IMPORTANT
Although the AOP can come installed with the Logix Designer
application, you might require interim features or functions that occur
between Logix Designer application major releases. In this case, you
would have to install your AOP manually.
IMPORTANT
To use your Kinetix 5100 servo drive with the provided AOP and predefined Add-On Instructions, you must configure your Kinetix 5100
drive in KNX5100C software first and change the control mode to IO
Mode. See Download KNX5100C Software on page 130.

<!-- page 3 -->

To configure your controller do the following.
1.
Apply power to your controller and run the Studio 5000 Logix Designer
application.
2. From the Create menu, choose New Project.
The New Project dialog box appears.
In this example, the typical dialog boxes for 1756-ENxT EtherNet/IP modules
and CompactLogix 5370 controllers with embedded Ethernet are shown.
Follow these steps to configure your Logix 5000™ controller.
1.
Expand the Logix 5000 controller family and select your controller.
2. Type the file Name.
3.
Click Next.
The New Project dialog box appears.
4. From the Revision pull-down menu, choose your software revision.
5.
Click Finish.
The new controller appears in the Controller Organizer under the I/O
Configuration folder.
CompactLogix 5370 Controller
ControlLogix 5570 Controller
CompactLogix 5370 Controller
ControlLogix 5570 Controller
CompactLogix 5370 Controller
ControlLogix 5570 Controller

<!-- page 4 -->

6. Right-click I/O Configuration in the Controller Organizer and choose
New Module.
7.
By using the filters, check Communication and Allen-Bradley, and select
1756-EN2T, 1756-EN2TR, or 1756-EN3TR as appropriate for your hardware
configuration. In this example, the 1756-EN2T module is selected.
8. Click Create.
The New Module dialog box appears.
IMPORTANT
If your project includes a ControlLogix or GuardLogix® 5570
controller, you need to add an EtherNet/IP communication module to
your Bulletin 1756 chassis and configure it for use in your application.
• For ControlLogix 5570, and GuardLogix 5570 controllers, go to step 6.
• For CompactLogix 5370, Compact GuardLogix 5370,
CompactLogix 5380, ControlLogix 5580, or GuardLogix 5580
controllers, go to step 13.
Refer to the EtherNet/IP Network Configuration User Manual,
publication ENET-UM006 for more information on EtherNet/IP
modules.

<!-- page 5 -->

9. Configure the new module.
a. Type the module Name.
b. Enter the Logix EtherNet/IP module slot (leftmost slot = 0).
c. Select an Ethernet Address option.
In this example, the Private Network address is selected.
d. Enter the address of your EtherNet/IP module.
In this example, the last octet of the address is 32.
e. Click Change in the Module Definition area.
The Module Definition dialog box opens.
10. To close the Module Definition dialog box, click OK.
11. When prompted to confirm your module definition changes, click Yes.
12. To close the New Module dialog box, click OK.
Your new 1756-ENxT Ethernet module appears under the I/O
Configuration folder in the Controller Organizer.
13. Click OK.

<!-- page 6 -->

Configure the Kinetix 5100
Drive Modules
In this example, a 2198-E1004-ERS drive is configured.
Follow these steps to configure Kinetix 5100 drives.
1.
Right-click Ethernet/IP Module and choose New Module.
The Select Module Type dialog box appears.
2. By using the filters, check Motion and Allen-Bradley, and select your
2198-Exxxx-ERS drive as appropriate for your hardware configuration.
3.
Click Create.
The New Module dialog box appears.
IMPORTANT
To configure Kinetix 5100 drive systems, you must be using the Logix
Designer application, version 30.00 or later.

<!-- page 7 -->

4. Configure the new drive.
a. Type the drive Name.
b. Select an Ethernet Address option.
In this example, the Private Network address is selected.
c. Enter the address of your 2198-Exxxx-ERS drive.
In this example, the last octet of the address is 2.
d. Under Module Definition click Change.
The Module Definition dialog box appears.
e. Set Series and Revision to match your drive.
f. Choose an Electronic Keying option.
The electronic keying feature automatically compares the expected
module, as shown in the configuration tree, to the physical module
before communication begins. We recommend using either `Exact
Match' or `Compatible Keying'. You cannot use Disable keying with
safety applications. For more information about electronic keying, see
the Electronic Keying in Logix 5000 Control Systems Application
Technique, publication LOGIX-AT001.
g. From the Connection pull-down menu, choose the Connection mode
for your motion application.
IMPORTANT
For new applications, it is typical to use Data with Camming. For
legacy applications, use the following guidance to choose Data
or Data with Camming.

<!-- page 8 -->

When Connection is Data, assembly output instance 104 is configured,
or AOP revision 2 is used, then use the Add-On Instructions:
raC_Dvc_K5100_MAG, raC_Dvc_K5100_ MAT, and structure
AssemblyOutIOM from the ‘Version 1’ firmware folder.
When Connection is Data with Camming or assembly output instance
106 is configured, AOP version 2 or later must be used. The Operation
Add-On Instructions can be used. (raC_Opr_K5100_xxxx)
h. From the Configured By: pull-down menu, when ‘External Means’ is
used with AOP version 2.0 or later, ADC (Automatic Device
Configuration) is not used. When ‘This Controller’ is used with AOP
version 2.0 or later, ADC is used.
5.
To close the Module Definition dialog box, click OK.
6. To close the Module Properties dialog box, click OK.
7.
To close the Select Module Type dialog box, click Close.
Your 2198-xxxx-ERS drive appears in the Controller Organizer under the
Ethernet network in the I/O Configuration folder.
Support Automatic Device
Configuration (ADC) in AOP
Version 2 and Later
ADC function can be enabled by setting 'Configured by' as 'This Controller'.
CompactLogix 5370 Controller
ControlLogix 5570 Controller

<!-- page 9 -->

## Connection RPI

Choose the RPI (Requested Packet Interval) for your drive. In previous
firmware revisions, the default was 2.0ms. We recommend 20 ms since this is
a simple I/O device and not an integrated motion on EtherNet/IP (CIP) drive.
Inhibiting/Un-inhibiting an
I/O Connection
To inhibit the I/O connection, check ‘Inhibit Module’, then click ‘Apply’.
To uninhibit the I/O connection, uncheck ‘Inhibit Module’, then press ‘Apply’.

> **Figure 83** — Inhibit Module

IMPORTANT
The KNX5100C software does not permit configuration changes while
an I/O connection exists with the controller. To enable configuration
changes using the KNX5100C software, check ‘Inhibit Module’ and apply
changes. If ADC is not configured, uncheck ‘Inhibit Module’ and apply
changes after configuration changes are completed. If ADC is
configured, import the configuration changes into the Studio 5000
project and controller before un-inhibiting the I/O connection.

<!-- page 10 -->

Download the Program
When the Logix Designer application development is complete, the file is
saved. You must download your program to the Logix 5000 controller.
For legacy applications, we have developed some sample logic that you can use
to import the entire Add-On Instruction library into your Studio 5000 Logix
Designer application by using an .L5X file as the import mechanism. See
Appendix C on page 489 for details on the Add-On Instruction library.
For new applications, use the Logix Designer application with the Plug-in
Wizard once the Power Device library (which includes the new Add-On
Instructions) is downloaded from the PCDC site.

<!-- page 11 -->

This chapter provides information about tuning.
See Motion System Tuning, publication MOTION-AT005 for more
information regarding tuning.
Topic
Page
Tuning Process

Autotuning

Tuning via Tuning Mode 1 and Tuning Mode 2

Tuning in Manual Mode

System Analysis

This manual links to Kinetix® 5100 Servo Drive Fault Codes Reference
Data, publication 2198-RD001, for fault codes and Kinetix 5100 Servo
Drive Parameters Reference Data, publication 2198-RD002, for
parameters. Download the spreadsheets now for offline access.
IMPORTANT
Linear motors are not supported with Autotune.
