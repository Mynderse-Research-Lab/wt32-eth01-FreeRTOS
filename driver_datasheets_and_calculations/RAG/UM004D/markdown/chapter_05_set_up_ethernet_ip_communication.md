# Chapter 5: Set Up EtherNet/IP Communication

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `06_Ch05_Setup_EtherNetIP.pdf` (5 pages)_

<!-- page 1 -->

## Set Network Parameters by

Using the Keypad Interface
Follow these steps to set network parameters.

> **Figure 71** — Keypad and Display

1.
Apply power to your drive.
BOOTx appears on the display as the drive boots up. After a successful
boot process, the drive display scrolls 5100 192.168.1.1, then STOP
192.168.1.1.
2. In the Device Information screen or Drive Status screen, the current IP
address is shown.
3.
Press
 key.
SETTING appears on the display.
4. Press
 key.
NET SETTING scrolls across the display.
5.
Press
 key.
STATIC IP scrolls across the display.
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

Drive
Status
Device
Information
Version
Information
Setting
Information Display/Setup Mode

**Extracted table (page 1, #1):**

| Key Na | me |
| --- | --- |
| — Dis | play |
| Mo | de key |
| Up | key |
| Do | wn key |
| Sh | ift key |
| Se | t key |

<!-- page 2 -->

To enter the IP ADDR, press
 key, and use the
 keys to enter the IP
Address octets.
To enter the SUBNET, press
 key, and use the
 keys to enter the
subnet address octets.
To enter the GATE, press
 key, and use the
 keys to enter the
Gateway octets.
Press
 to return to the Static IP display.
6. Press
 key.
DHCP appears on the display.
To show the current DHCP setting (OFF or ON), press
 key.
To change the DHCP setting, and press
 or
 key.
The display toggles between OFF and ON.
To apply the setting, press
 key or to exit the setting press

keypad.
See Chapter 6, Use the Keypad Interface for help with setting the network
parameters

<!-- page 3 -->

## Set Network Parameters by

Using KNX5100C Software
The KNX5100C software reads the parameters from your drive. Follow these
steps to configure network parameters.
1.
From the Function List, click Drive IP Address Setting.
The Drive IP Address Setting dialog box appears, and the current IP
Settings are displayed.
2. Choose between STATIC IP and DHCP.
The default setting is STATIC IP.
3.
If STATIC IP, then configure the following parameters:
- IP address
- Gateway
- Subnet mask
4. Click Apply.
5.
To have the IP Settings take effect, click Reset Module from the tool bar.

<!-- page 4 -->

## Configure IP Address by

Using BOOTP-DHCP Tool
Follow these steps to configure the IP address.
1.
Connect your PC with a Kinetix 5100 drive via Ethernet cable, and then
apply the power to the drive.
2. Open BOOTP-DHCP tool in your workstation, and select network
interface according to your environment as shown.
In this example, the IP address of workstation is 192.168.1.25.
The BOOTP-DHCP tool automatically scans the devices that are
configured to DHCP on the network, and displays the device MAC
addresses as shown.
You can find the MAC address of your drive on the drive label. In this
example, the MAC address 00:00:BC:01:01:01 is used.
The Kinetix 5100 drive supports DHCP only, not BootP. You can use other DHCP
server software to configure the IP address of the Kinetix 5100 drive. This
example shows a software tool called BootP-DHCP Tool, which is installed
together with the Studio 5000 application.

<!-- page 5 -->

3.
From the dialog box, double-click the MAC address of your drive.
The New Entry dialog appears.
4. Type your specified IP address in the New Entry dialog, and then click
OK.
