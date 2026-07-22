# Appendix B - Upgrade Firmware

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `19_AppB_Upgrade_Firmware.pdf` (12 pages)_

<!-- page 1 -->

## Appendix B

Upgrade Kinetix 5100 Drive Firmware
This appendix provides procedures for upgrading your Kinetix® 5100
firmware.
The firmware update procedure uses the Ethernet/IP port of the drive. You
must have an Ethernet/IP cable connected to the drive and any active Class 1
connection must be inhibited.
You can upgrade your Kinetix 5100 drive firmware by using either of these two
methods:
•
ControlFLASH Plus™ software
•
ControlFLASH™ software
To upgrade drive firmware, you must configure a path to your drive, select the
drive module to upgrade, and complete the firmware upgrade procedure.
Before You Begin
The firmware revision for software must be as shown for EtherNet/IP™
networks.
Topic
Page
Before You Begin

Upgrade the Firmware

Verify the Firmware Upgrade

We recommend that you use ControlFLASH Plus software for firmware upgrades.
See the ControlFLASH Plus Quick Start Guide, publication CFP-QS001, for more
information.

> **Table 149** — Kinetix 5100 System Requirements

Description
Firmware Revision
Logix Designer application
30.00.00 or later
RSLinx® software (1)
(1)
Only required when using ControlFLASH software.
3.60.00 or later
FactoryTalk® Linx software (2)
(2)
Only required when using ControlFLASH Plus software.
6.20.00 or later
ControlFLASH software kit(3)
(3)
Download the ControlFLASH software kit from the Product Compatibility and Download Center at: rok.auto/pcdc. For
more ControlFLASH software information (not Kinetix 5100 specific), refer to the ControlFLASH Firmware Upgrade Kit
User Manual, publication 1756-UM105.
14.01.00 or later
ControlFLASH Plus software kit (3)
3.01 or later

<!-- page 2 -->

## Appendix B Upgrade Kinetix 5100 Drive Firmware

Gather this information before you begin your firmware upgrade.
•
Network path to the targeted Kinetix 5100 drives you want to upgrade.
•
Catalog numbers of the targeted Kinetix 5100 drives you want to
upgrade.
Inhibit the Module
If the drive is configured as IO Mode, you must inhibit the connection before
performing the firmware upgrade.
Follow these steps to inhibit the connection.
1.
Open your Logix Designer application.
2. Right-click the 2198-Exxxx-ERS drive
and choose Properties.
The Module Properties dialog box
appears.
3.
Select the Connection category.
4. Check Inhibit Module.
5.
Click OK.
6. Save your file and download the program to the controller.
IMPORTANT
Control power at L1C and L2C (200V drives) and 24V+ and 24V- (400V
drives) must be present prior to upgrading your target module.
IMPORTANT
The state on the display must be STDBY (STANDBY) in IO Mode before
upgrading your target module.
The state on the display must be STOP (STOPPED) in other modes
before upgrading your target module.
ATTENTION: To avoid personal injury or damage to equipment during the
firmware upgrade due to unpredictable motor activity, do not apply the main
power to the drive. Do apply the control power to the drive.

<!-- page 3 -->

## Appendix B Upgrade Kinetix 5100 Drive Firmware

Upgrade Your Firmware
Use either ControlFLASH Plus software or ControlFLASH software to upgrade
your firmware.
•
To use ControlFLASH Plus software, see Use ControlFLASH Plus
Software to Upgrade Your Drive Firmware on page 479.
•
To use ControlFLASH software, see Use ControlFLASH Software to
Upgrade Your Drive Firmware on page 482.
Use ControlFLASH Plus Software to Upgrade Your Drive Firmware
Follow these steps to select the Kinetix 5100 drive to upgrade.
1.
Start ControlFLASH Plus software.
2. Click the Flash Devices tab. If the device is not already present in
Browsing from path:, complete these steps:
a. Click
.
b. In the Network Browser dialog box, locate and select the device to
upgrade.
c. Click OK.
The Status field displays Non-DMK Firmware must be installed
manually. This is accomplished with the ControlFLASH MSI file.
You can choose to select and upgrade the firmware for all drive modules
in your system. However, in this procedure only one drive is selected for a
firmware upgrade.

<!-- page 4 -->

## Appendix B Upgrade Kinetix 5100 Drive Firmware

3.
To download the ControlFLASH MSI file, go to the Product Compatibility
and Download Center (PCDC).
a. Check Firmware, click Downloads, and follow the prompts to
download the ControlFLASH MSI file.
b. Install the Kinetix 5100 ControlFLASH MSI file.
c. Click Refresh Firmware.
The Building firmware inventory dialog box opens and the firmware
inventory installs.
When the refresh is complete, the Status field is empty.
4. If a warning dialog box appears, read the warning, complete any
recommendations, and click Close.

<!-- page 5 -->

## Appendix B Upgrade Kinetix 5100 Drive Firmware

5.
After acknowledging all warnings and confirming the desired revisions,
click Flash to begin the firmware upgrade.
The Status bar appears to show the progress of the firmware update.
Also, the status display scrolls ‘Updating. Do Not Turn Off’, which
indicates that the upgrade is in progress.
After the upgrade information is sent to the drive, the drive resets and
performs diagnostic checking.
After the download, the drive applies the new firmware and reboots. This
can take several minutes.
After the drive reboots, ControlFlash Plus software indicates success or
failure of the update.
6. When the upgrade has completed, click Close.
7.
To complete the process and close the application, click Done.
IMPORTANT
Do not cycle power to the drive during this process. A power cycle
results in an unsuccessful firmware upgrade and an inoperable
module.
IMPORTANT
You must return to the drive Module Properties>Connection category
to clear the Inhibit Module checkbox before resuming normal
operation.

<!-- page 6 -->

## Appendix B Upgrade Kinetix 5100 Drive Firmware

Use ControlFLASH Software to Upgrade Your Drive Firmware
Before using ControlFLASH software you need to configure the
communication path by using RSLinx software.
Configure Your Communication Path with RSLinx Software
This procedure assumes that your communication method to the target device
is the Ethernet network. It also assumes that any Ethernet communication
module or Logix 5000™ controller in the communication path has already been
configured.
For more controller information, see Additional Resources on page 14.
Follow these steps to configure the communication path to the target device.
1.
Open your RSLinx Classic software.
2. From the Communications menu, choose Configure Drivers.
The Configure Drivers dialog box appears.
3.
From the Available Driver Types pull-down menu, choose Ethernet
devices.
4. Click Add New.
The Add New RSLinx Classic Driver dialog box appears.
5.
Type the new driver name.
6. Click OK.
The Configure driver dialog box appears.

<!-- page 7 -->

## Appendix B Upgrade Kinetix 5100 Drive Firmware

7.
Type the IP address of your Ethernet Module or Controller that bridges
between the Ethernet network and the EtherNet/IP network.
8. Click OK.
The new Ethernet driver appears under Configured Drivers.
9. Click Close.
10. Minimize the RSLinx application dialog box.
Start the ControlFLASH Software
Follow these steps to start ControlFLASH software and begin your firmware
upgrade.
1.
In the Logix Designer application, from the Tools menu, choose
ControlFLASH.
2. In the Logix Designer application, from the Tools menu, choose
ControlFLASH.
The Welcome to ControlFLASH dialog box appears.
3.
Click Next.
You can also open ControlFLASH software by choosing
Start>Programs>FLASH Programming Tools>ControlFLASH.

<!-- page 8 -->

## Appendix B Upgrade Kinetix 5100 Drive Firmware

The Catalog Number dialog box appears.
4. Select your drive module.
In this example, the 2198-E1004-ERS drive is selected.
5.
Click Next.
The Select Device to Update dialog box appears.
6. Expand your Ethernet node, Logix backplane, and EtherNet/IP network
module.
7.
Select the servo drive to upgrade.
8. Click OK.
If your catalog number does not appear, click Browse, select the
monitored folder where the firmware kit (DMK files) is located. Click Add
and OK.

<!-- page 9 -->

## Appendix B Upgrade Kinetix 5100 Drive Firmware

The Firmware Revision dialog box appears.
9. Select the firmware revision to upgrade.
10. Click Next.
The Summary dialog box appears.
11. Confirm the drive catalog number and firmware revision.
12. Click Finish.
This ControlFLASH warning dialog box appears.
13. To complete the update now, click Yes.

<!-- page 10 -->

## Appendix B Upgrade Kinetix 5100 Drive Firmware

This ControlFLASH warning dialog box appears.
14. Acknowledge the warning and click OK.
The Progress dialog box appears and the update begins.
The state on the display changes from
STDBY (STANDBY), or STOP
(STOPPED) to F_UPD (FIRMWARE
UPDATE), which indicates that the
upgrade is in progress.
After the upgrade information is sent
to the drive, the drive resets and
performs diagnostic checking.
15. Wait for the Progress dialog box to time out.
It is normal for this process to take several minutes.
16. Verify that the Update Status dialog box appears and indicates success or
failure as described below.
IMPORTANT
Do not cycle power to the drive during this process. A power
cycle results in an unsuccessful firmware upgrade and an
inoperable module.

<!-- page 11 -->

## Appendix B Upgrade Kinetix 5100 Drive Firmware

17. Click OK.
Verify the Firmware
Upgrade
Follow these steps to verify that your firmware upgrade was successful.
1.
Open your RSLinx software.
2. From the Communications menu, choose RSWho.
3.
Expand your Ethernet node, Logix backplane, and EtherNet/IP network
module.
4. Right-click the drive module and choose Device Properties.
The Device Properties dialog box appears.
5.
Verify the new firmware revision level.
6. Click Close.
Upgrade Status
If
Success
Update complete appears in a GREEN Status dialog box, then go to step 17.
Failure
Update failure appears in a RED Status dialog box, then see the ControlFLASH Firmware
Upgrade Software User Manual, publication 1756-UM105 for troubleshooting information.
IMPORTANT: If the power is lost during the firmware upgrade, the update fails. When the
power is restored, three different situations happen depending on when the power went off.
• Panel display shows UPt 1: Update the firmware again.
• Panel display is blank:, The drive automatically finishes the last firmware update in 20
seconds, then it resets to complete the update.
• Panel display shows UPt 3: Update the firmware again.
IMPORTANT
If you checked Inhibit Module on the Connection tab in Module
Properties, you must clear the Inhibit Module check box before
resuming normal operation.
Verifying the firmware upgrade is optional.

<!-- page 12 -->

## Appendix B Upgrade Kinetix 5100 Drive Firmware

Notes:
