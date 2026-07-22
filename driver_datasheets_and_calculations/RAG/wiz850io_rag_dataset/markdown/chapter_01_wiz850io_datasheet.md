# WIZ850io — Compact-sized network module

Source: **WIZnet WIZ850io Datasheet (Version 1.0.4)**

<!-- page 1 -->
WIZ850io Datasheet (WIZnet Co., Ltd)

## WIZ850io Datasheet
(Version V1.0)

©2016 WIZnet Co., Ltd. All Rights Reserved.
☞ For more information, visit our website at http://www.wiznet.co.kr

<!-- page 2 -->
WIZ850io Datasheet (WIZnet Co., Ltd)
## Document Revision History

Date
Revision
Changes
2016-04-28
V1.0
## Official Release

| Date | Revision | Changes |
| --- | --- | --- |
| 2016-04-28 | V1.0 | Official Release |
|  |  |  |
|  |  |  |

<!-- page 3 -->
WIZ850io Datasheet (WIZnet Co., Ltd)

내용
1.
Overview ........................................................................................................................................................... 4
1.1. Feature ....................................................................................................................................................... 4
2.
Hardware Specification ............................................................................................................................. 5
2.1. WIZ850io Hardware Summary ....................................................................................................... 5
2.2. Pinout .......................................................................................................................................................... 5
2.3. Pin Description ...................................................................................................................................... 6
3.
Characteristic ................................................................................................................................................. 7
3.1. DC Characteristic ................................................................................................................................. 7
3.2. Power Dissipation ................................................................................................................................ 7
4.
SPI Operations............................................................................................................................................... 7
5.
Timing Diagram ............................................................................................................................................. 8
5.1. Reset Timing ........................................................................................................................................... 8
5.2. SPI Timing ................................................................................................................................................ 8
6.
Schemetic ........................................................................................................................................................ 9
7.
Dimension ..................................................................................................................................................... 10

<!-- page 4 -->
WIZ850io Datasheet (WIZnet Co., Ltd)
1. Overview
WIZ850io is a compact size network module that includes a W5500 (TCP/IP hardwired chip and PHY
embedded), a transformer and RJ45. It can be used as a component and no effort is required to
interface W5500 and Transformer.
The WIZ850io is an ideal option for users who want to develop their Internet enabling systems
rapidly. WIZ850io is hardware pin compatible with WIZ820io.
WIZ820io users, to migrate to WIZ850io, need to modify the Firmware.

For the detailed information on implementation of Hardware TCP/IP, refer to the W5500 Datasheet.

1.1.
Feature

Supports following Hardwired TCP/IP Protocols : TCP, UDP, ICMP, IPv4, ARP, IGMP, PPPoE

Supports 8 independent sockets simultaneously

Supports Power down mode

Supports Wake on LAN over UDP

Supports High Speed Serial Peripheral Interface(SPI MODE 0, 3)

Internal 32Kbytes Memory for Tx/Rx Buffers

10BaseT/100BaseTX Ethernet PHY embedded

Support Auto Negotiation (Full and half duplex, 10 and 100-based)

Not support IP Fragmentation

3.3V operation with 5V I/O signal tolerance

Not support auto MDIX

<!-- page 5 -->
WIZ850io Datasheet (WIZnet Co., Ltd)
2. Hardware Specification
2.1.
## WIZ850io Hardware Summary

Plugin Network Module.

Hardware pin compatible with WIZ820io.

Usable without H/W design for W5500, Transformer & RJ45.

Fast evaluation for W5500 & MCU in the target board.

Support high speed SPI interface.

Support power down mode and Wake-on-LAN function

Operating Temperature : -40℃ ~ 85℃

Very small form factor : 23mm x 25.75mm x 18mm

1 x 6, 2.54mm Pin Header x 2

2.2.
Pinout

<!-- page 6 -->
WIZ850io Datasheet (WIZnet Co., Ltd)
2.3.
## Pin Description

Pin No.
## Pin Type
## Pin Name
Description
J1
P
GND
Ground
P
GND
Ground
I
MOSI
## SPI Master Out Slave In
This pin is used for SPI MOSI signal pin
I
SCLK
## SPI Clock
This pin is used for SPI Clock Signal pin
I
SCNn
## SPI Slave Select
This pin is used for SPI Slave Select Signal Pin when
using SPI interface
I
INTn
W5500 Interrupt : Low activity
This pin is used for indicating event like socket TCP
connection, disconnection, data receiving timeout,
WOL(Wake on Lan) and so on occurred in W5500 inside
WIZ550io.
The interrupt is cleared by writing IR register or Sn_IR.
All interrupts are maskable.

Pin No.
## Pin Type
## Pin Name
Description
J1
P
GND
Ground
P
3.3V
Power : 3.3V power supply
P
3.3V
Power : 3.3V power supply
-
NC
## Not Connect
I
RSTn
Reset : Low activity
This pin is to initialize WIZ850io.
Hold at least 500us after asserted to LOW and keep
HIGH until next Reset needed.
User need to wait for 50ms after this pin is changed to
HIGH to communicate with WIZ850io.
(Refer to 5. Timing Diagram)
O
MISO
## SPI Master In Slave Out
This pin is used for SPI MISO signal pin

※ P : Power , I : Input , O : Output

|  |  |  |  | Pin No. |  |  | Pin Type |  |  | Pin Name |  |  | Description |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| J1 |  |  | 1 |  |  | P |  |  | GND |  |  | Ground |  |  |
|  |  |  | 2 |  |  | P |  |  | GND |  |  | Ground |  |  |
|  |  |  | 3 |  |  | I |  |  | MOSI |  |  | SPI Master Out Slave In This pin is used for SPI MOSI signal pin |  |  |
|  |  |  | 4 |  |  | I |  |  | SCLK |  |  | SPI Clock This pin is used for SPI Clock Signal pin |  |  |
|  |  |  | 5 |  |  | I |  |  | SCNn |  |  |  | SPI Slave Select |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | This pin is used for SPI Slave Select Signal Pin when |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | using SPI interface |  |
|  |  |  | 6 |  |  | I |  |  | INTn |  |  |  | W5500 Interrupt : Low activity |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | This pin is used for indicating event like socket TCP |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | connection, disconnection, data receiving timeout, |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | WOL(Wake on Lan) and so on occurred in W5500 inside |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | WIZ550io. |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | The interrupt is cleared by writing IR register or Sn_IR. |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | All interrupts are maskable. |  |

|  |  |  |  | Pin No. |  |  | Pin Type |  |  | Pin Name |  |  | Description |  |  |  |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| J1 |  |  | 1 |  |  | P |  |  | GND |  |  |  | Ground |  |  |  |  |
|  |  |  | 2 |  |  | P |  |  | 3.3V |  |  |  | Power : 3.3V power supply |  |  |  |  |
|  |  |  | 3 |  |  | P |  |  | 3.3V |  |  |  | Power : 3.3V power supply |  |  |  |  |
|  |  |  | 4 |  |  | - |  |  | NC |  |  |  | Not Connect |  |  |  |  |
|  |  |  | 5 |  |  | I |  |  | RSTn |  |  |  | Reset : Low activity |  |  |  |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | This pin is to initialize WIZ850io. |  |  |  |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | Hold at least 500us after asserted to LOW and keep |  |  |  |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | HIGH until next Reset needed. |  |  |  |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | User need to wait for 50ms after this pin is changed to |  |  |  |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | HIGH to communicate with WIZ850io. |  |  |  |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | (Refer to 5. Timing Diagram) |  |  |  |  |
|  |  |  | 6 |  |  | O |  |  | MISO |  |  |  | SPI Master In Slave Out |  |  |  |  |
|  |  |  |  |  |  |  |  |  |  |  |  |  | This pin is used for SPI MISO signal pin |  |  |  |  |

<!-- page 7 -->
WIZ850io Datasheet (WIZnet Co., Ltd)
3. Characteristic
3.1.
## DC Characteristic
Symbol
Parameter
Pins
Min
Typ
Max
Unit
VDD
Supply voltage
3.3V
2.97
3.3
3.63
V
VIL
High level input voltage
ALL
2.0
-
5.5
V
VIH
Low level input voltage
ALL
-0.3
-
0.8
V
VOL
Low level output voltage
ALL
-
-
0.4
V
VOH
High level output voltage
ALL
2.4
-
-
V
LOL
Low level output Current
ALL
8.6
13.9
18.9
mA
LOH
High level output Current
ALL
12.5
26.9
47.1
mA
IDD
## Supply Current
(Normal operation mode)
3.3V
-
132
-
mA
LDD
## Supply Current
(Power Down mode)
3.3V
-
13
-
mA

3.2.
## Power Dissipation
Condition
Min
Typ
Max
Unit
100M Link
-
128
-
mA
10M Link
-
75
-
mA
Un-Link
(Auto- negotiation mode)
-
65
-
mA
100M Transmitting
-
132
-
mA
10M Transmitting
-
79
-
mA
Power Down mode
-
13
-
mA

4. SPI Operations
As WIZ850io consists of W5500 and others, SPI operation of WIZ850io follows W5500 SPI
specification. For more information about SPI operation of WIZ850io, please refer to W5500
Datasheet.

|  | Symbol |  |  | Parameter |  |  |  |  | Pins |  | Min |  |  |  |  | Typ |  |  |  |  | Max |  |  |  |  | Unit |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
|  | V DD |  |  |  | Supply voltage |  |  | 3.3V |  |  | 2.97 |  |  |  |  |  | 3.3 |  |  |  |  | 3.63 |  |  | V |  |  |
|  | V IL |  |  | High level input voltage |  |  |  | ALL |  |  |  | 2.0 |  |  | - |  |  |  |  |  |  | 5.5 |  |  | V |  |  |
|  | V IH |  | Low level input voltage |  |  |  |  | ALL |  |  |  | -0.3 |  |  | - |  |  |  |  |  |  | 0.8 |  |  | V |  |  |
|  | V OL |  |  | Low level output voltage |  |  |  | ALL |  |  | - |  |  |  | - |  |  |  |  |  |  | 0.4 |  |  | V |  |  |
|  | V OH |  |  | High level output voltage |  |  |  | ALL |  |  | 2.4 |  |  |  | - |  |  |  |  | - |  |  |  |  | V |  |  |
|  | L OL |  |  | Low level output Current |  |  |  | ALL |  |  | 8.6 |  |  |  | 1 |  | 3.9 |  |  |  |  | 18.9 |  |  | mA |  |  |
|  | L OH |  |  | High level output Current |  |  |  | ALL |  |  | 12.5 |  |  |  | 26.9 |  |  |  |  |  |  | 47.1 |  |  | mA |  |  |
| I DD | I DD |  | Supply Current (Normal operation mode) |  |  |  |  | 3.3V |  |  | - |  |  |  | 132 |  |  |  |  | - |  |  |  |  | mA |  |  |
| L DD |  |  | Supply Current (Power Down mode) |  |  |  |  | 3.3V |  |  | - |  |  |  | 13 |  |  |  |  | - |  |  |  |  | mA |  |  |

|  | Condition |  | Min |  | Typ |  | Max |  | Unit |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
|  | 100M Link | - |  | 128 |  | - |  | mA |  |  |
|  | 10M Link | - |  | 75 |  | - |  | mA |  |  |
|  | Un-Link (Auto- negotiation mode) | - |  | 65 |  | - |  | mA |  |  |
|  | 100M Transmitting | - |  | 132 |  | - |  | mA |  |  |
|  | 10M Transmitting | - |  | 79 |  | - |  | mA |  |  |
|  | Power Down mode | - |  | 13 |  | - |  | mA |  |  |

<!-- page 8 -->
WIZ850io Datasheet (WIZnet Co., Ltd)
5. Timing Diagram
5.1.
## Reset Timing

Symbol
Description
Min
Max
TRC
## Reset Cycle Time
500us
-
TPL
## Internal Auto Configuration
-
50ms

5.2.
## SPI Timing

Symbol
Description
Min
Max
Units
FSCK
## SCLK Clock Frequency
-
80
MHz
TWH
SCLK High duration
-
ns
TWL
SCLK Low duration
-
ns
TCS
nSCS High duration
-
ns

|  | Symbol |  |  | Description |  |  | Min |  |  | Max |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
|  | T RC |  | Reset Cycle Time |  |  | 500us |  |  | - |  |  |
|  | TPL |  | Internal Auto Configuration |  |  | - |  |  | 50ms |  |  |

|  | Symbol |  |  | Description |  |  | Min |  |  | Max |  |  | Units |  |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
|  | F SCK |  | SCLK Clock Frequency |  |  | - |  |  | 80 |  |  | MHz |  |  |
|  | T WH |  | SCLK High duration |  |  | 6 |  |  | - |  |  | ns |  |  |
|  | T WL |  | SCLK Low duration |  |  | 6 |  |  | - |  |  | ns |  |  |
|  | T CS |  | nSCS High duration |  |  | 5 |  |  | - |  |  | ns |  |  |

<!-- page 9 -->
WIZ850io Datasheet (WIZnet Co., Ltd)
6. Schemetic

<!-- page 10 -->
WIZ850io Datasheet (WIZnet Co., Ltd)
7. Dimension
3D PDF Link : http://wizwiki.net/wiki/doku.php?id=products:wiz850io:start&#dimension

Total Height: 2mm(Tap) +13.61mm(RJ45) + 1.6mm(PCB) + 8.54mm(Pin Header) = 25.75mm