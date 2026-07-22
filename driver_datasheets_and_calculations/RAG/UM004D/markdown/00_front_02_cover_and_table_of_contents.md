# Cover and Table of Contents

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `00_Cover_and_TOC.pdf` (12 pages)_

<!-- page 1 -->

## User Manual

Original Instructions
Kinetix 5100 EtherNet/IP
Indexing Servo Drives
Catalog Numbers 2198-E1004-ERS, 2198-E1007-ERS,
2198-E1015-ERS, 2198-E1020-ERS, 2198-E2030-ERS,
2198-E2055-ERS, 2198-E2075-ERS, 2198-E2150-ERS,
2198-E4004-ERS, 2198-E4007-ERS, 2198-E4015-ERS,
2198-E4020-ERS, 2198-E4030-ERS, 2198-E4055-ERS,
2198-E4075-ERS, 2198-E4150-ERS
This manual links to Kinetix 5100 Servo Drive Fault Codes Reference Data, publication
2198-RD001, for fault codes and Kinetix 5100 Servo Drive Parameters Reference Data,
publication 2198-RD002, for parameters. Download the spreadsheets now for offline access.

<!-- page 2 -->

## Kinetix 5100 EtherNet/IP Indexing Servo Drives User Manual

Important User Information
Read this document and the documents listed in the additional resources section about installation, configuration, and operation of this equipment before
you install, configure, operate, or maintain this product. Users are required to familiarize themselves with installation and wiring instructions in addition to
requirements of all applicable codes, laws, and standards.
Activities including installation, adjustments, putting into service, use, assembly, disassembly, and maintenance are required to be carried out by suitably
trained personnel in accordance with applicable code of practice.
If this equipment is used in a manner not specified by the manufacturer, the protection provided by the equipment may be impaired.
In no event will Rockwell Automation, Inc. be responsible or liable for indirect or consequential damages resulting from the use or application of this
equipment.
The examples and diagrams in this manual are included solely for illustrative purposes. Because of the many variables and requirements associated with
any particular installation, Rockwell Automation, Inc. cannot assume responsibility or liability for actual use based on the examples and diagrams.
No patent liability is assumed by Rockwell Automation, Inc. with respect to use of information, circuits, equipment, or software described in this manual.
Reproduction of the contents of this manual, in whole or in part, without written permission of Rockwell Automation, Inc., is prohibited.
Throughout this manual, when necessary, we use notes to make you aware of safety considerations.
These labels may also be on or inside the equipment to provide specific precautions.
The following icon may appear in the text of this document.
Rockwell Automation recognizes that some of the terms that are currently used in our industry and in this publication are not in alignment
with the movement toward inclusive language in technology. We are proactively collaborating with industry peers to find alternatives to such
terms and making changes to our products and content. Please excuse the use of such terms in our content while we implement these
changes.
WARNING: Identifies information about practices or circumstances that can cause an explosion in a hazardous environment,
which may lead to personal injury or death, property damage, or economic loss.
ATTENTION: Identifies information about practices or circumstances that can lead to personal injury or death, property
damage, or economic loss. Attentions help you identify a hazard, avoid a hazard, and recognize the consequence.
IMPORTANT
Identifies information that is critical for successful application and understanding of the product.
SHOCK HAZARD: Labels may be on or inside the equipment, for example, a drive or motor, to alert people that dangerous
voltage may be present.
BURN HAZARD: Labels may be on or inside the equipment, for example, a drive or motor, to alert people that surfaces may
reach dangerous temperatures.
ARC FLASH HAZARD: Labels may be on or inside the equipment, for example, a motor control center, to alert people to
potential Arc Flash. Arc Flash will cause severe injury or death. Wear proper Personal Protective Equipment (PPE). Follow ALL
Regulatory requirements for safe work practices and for Personal Protective Equipment (PPE).
Identifies information that is useful and can help to make a process easier to do or easier to understand.

<!-- page 3 -->

Table of Contents
Preface
Summary of Changes. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 13
Conventions. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 13
Access Fault Codes and Parameter List . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 13
Additional Resources . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 14

About the Kinetix 5100 Drive System . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 17
Typical Hardware Configuration. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 19
Motor and Auxiliary Feedback Configurations. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 20
Typical Communication Configurations. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 21
Linear Topology. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 21
Ring Topology . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 22
Star Topology. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 23
Typical Control Configurations . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 24
Logix Enabled Using a Class 1 EtherNet/IP Connection . . . . . . . . . . . . . . . . . . . . . . 25
Micro800 Using a Class 3 EtherNet/IP Connection. . . . . . . . . . . . . . . . . . . . . . . . . . 26
Pulse Train Output Control with Motion User Defined Function Block . . . . . . . . . . 27
Catalog Number Explanation. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 28
Agency Compliance . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 29

## Drive System

System Design Guidelines . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 31
System Mounting Requirements . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 31
AC Line Filter Selection. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 32
Circuit Breaker/Fuse Selection. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 33
Transformer Selection . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 35
Passive Shunt Considerations. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 35
Enclosure Selection . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 36
Minimum Clearance Requirements. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 38
Electrical Noise Reduction. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 40
HF Bond the Drives . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 40
HF Bond Multiple Subpanels. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 42
Establish Noise Zones . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 43
Cable Categories for Kinetix 5100 Drive Systems . . . . . . . . . . . . . . . . . . . . . . . . . . . 44
Noise Reduction Guidelines for Drive Accessories. . . . . . . . . . . . . . . . . . . . . . . . . . 44
Mount Your Kinetix 5100 Drive. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 47
Drill-hole Patterns. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 47
Mount the Drive. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 48

Descriptions
Kinetix 5100 Connector Data. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 50
Safe Torque-off Connector Pinout . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 52
Power Connector Pinouts. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 53
Auxiliary Feedback Connector Pinout. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 56

<!-- page 4 -->

Table of Contents
Ethernet Communication Connector Pinout . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 57
Control Signal Specifications . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 57
Digital Inputs . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 57
Digital Outputs. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 60
Analog Inputs. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 63
Pulse Inputs. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 63
Analog Outputs . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 65
Buffered Encoder Outputs . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 65
Ethernet Communication Specifications. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 67
Motor Brake Circuit. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 68
Control Power . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 71
Feedback Specifications . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 72
Motor Feedback Supported by Using the MFB Connector . . . . . . . . . . . . . . . . . . . . 73
Auxiliary Feedback Specifications . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 74
Encoder Phasing Definitions . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 75
Absolute Position Feature . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 76
Safe Torque Off Feature . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 77
Operation Modes. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 77

System
Basic Wiring Requirements. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 79
Build Your Own Cables . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 80
Route Power and Signal Wiring . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 80
Determine the Input Power Configuration. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 80
Three-phase Power Wired to Three-phase Drives . . . . . . . . . . . . . . . . . . . . . . . . . . 81
Single-phase Input Power used with Single-phase Drives. . . . . . . . . . . . . . . . . . . . 82
Three-phase Input Power used with Single-phase Drives. . . . . . . . . . . . . . . . . . . . 83
Voiding of CE and UK Compliance. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 84
Using Isolation Transformers with Grounded Power Configurations . . . . . . . . . . . 84
Ground the Drive System . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 85
Ground Your Drive to the System Subpanel. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 85
Ground Multiple Subpanels. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 86
Wiring Requirements . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 86
Wiring Guidelines . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 88
Wire the Input Power Connectors. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 89
Wire the I/O Connector. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 91
Wire the Safe Torque Off Connector . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 91
Wire the Motor Power Connector . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 91
Servo Motor and Motor Cable Compatibility. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 93
Motor Power and Brake Cables . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 94
Maximum Cable Length . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 95
Cable Preparation for Kinetix TLP Servo Motors. . . . . . . . . . . . . . . . . . . . . . . . . . . . 96
Cable Preparation for Kinetix MP Servo Motors. . . . . . . . . . . . . . . . . . . . . . . . . . . . . 97
Cable Preparation for Kinetix TL and TLY Motor Power Cables. . . . . . . . . . . . . . . . 98
Apply the Cable Shield Clamp . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 98
Motor Brake Connections. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 100
Wire the Motor Feedback Connector . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 100
Cable Preparation for Kinetix TLP Feedback Cables . . . . . . . . . . . . . . . . . . . . . . . 101
Cable Preparation for 2090-CFBM7Dx Feedback Cables . . . . . . . . . . . . . . . . . . . . 102

<!-- page 5 -->

Table of Contents
Cable Preparation for Kinetix TL and TLY Feedback Cables . . . . . . . . . . . . . . . . . 102
Motor Feedback Cable Preparation. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 103
Kinetix 2090 Feedback Cable Pinouts . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 105
External Passive-shunt Resistor Connections . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 108
Ethernet Cable Connections . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 110

Communication
Set Network Parameters by Using the Keypad Interface . . . . . . . . . . . . . . . . . . . . . . . 112
Set Network Parameters by Using KNX5100C Software. . . . . . . . . . . . . . . . . . . . . . . . . 114
Configure IP Address by Using BOOTP-DHCP Tool . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 115

Keypad Input and Panel Display . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 117
Drive Displays . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 118
Real Time Data . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 118
Drive Status Display . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 121
Edit Settings From the Display . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 122
Edit Network Settings. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 122
Edit Parameter Settings. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 124
Reset the Drive via Keypad . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 125
Display Low Byte, High Byte, and Negative Values . . . . . . . . . . . . . . . . . . . . . . . . . 126
Display Fault Record. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 127
Diagnosis Parameters via Keypad . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 127
Display Firmware Upgrade Information. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 128

## KNX5100C Software

Before You Begin . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 129
Download KNX5100C Software . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 130
Launch KNX5100C Configuration Tool . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 130
Connect to the Drive/Set Your COM Port . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 132
Configure Drive Settings . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 134
Set the IP Address. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 134
Configure the Motor Selection in KNX5100C Software . . . . . . . . . . . . . . . . . . . . . . . . . . 134
Data Source . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 135
Selection of Motor Thermal Models. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 146
Motor Feedback. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 148
Run a Commutation Test . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 150
Parameter Editor . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 152
Parameter Wizard. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 154
Parameter Toolbar . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 154
Choose an Operation Mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 156
Using the Operation Mode Selection List . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 156
Using the Setting Page. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 157
Using the Parameter Editor. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 157
Configure Settings . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 159
Configure General Settings . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 160
Configure the Command Source. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 164
Configure the Pulse Outputs . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 168

<!-- page 6 -->

Table of Contents
Configure Electronic Gear (E-Gear) Ratio. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 169
Configure Filter . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 174
Configure Notch Filter . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 176
Configure Limits . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 177
Configure Analog I/O. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 178
Configure Position, Velocity, and Current Loops . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 182
Configure Position Loop. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 182
Configure Velocity Loop. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 183
Configure Current Loop . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 184
Digital I/O and Jog Function in KNX5100C Software . . . . . . . . . . . . . . . . . . . . . . . . . . . 184
Configuration and Status of Digital Input (DI) and Digital Output (DO) Signals . . 185
Edit DIO Configurations . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 187
Jog Function . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 188

5000 Logix Designer Application
Studio 5000 Logix Designer Application . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 189
Version History . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 189
Install the Kinetix 5100 Add-On Profile. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 190
Configure the Logix 5000 Controller . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 190
Configure the Kinetix 5100 Drive Modules . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 194
Support Automatic Device Configuration (ADC) in AOP Version 2 and Later . . . . . . . . 196
Connection RPI . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 197
Inhibiting/Un-inhibiting an
I/O Connection . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 197
Download the Program. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 198

Tuning Process . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 200
Reset Gains to Default . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 200
Bandwidth . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 200
Autotuning . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 203
Autotuning Configuration Parameters . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 203
Autotuning via the Drive Panel . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 204
Autotuning via KNX5100C Software . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 206
Alarms Related to Autotuning . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 211
Tuning via Tuning Mode 1 and Tuning Mode 2. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 212
Tuning Mode Process . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 212
Tuning Mode 1 . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 213
Tuning Mode 2 . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 215
Tuning in Manual Mode. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 218
Nested P-I Loop Gain Adjustment . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 218
Manual Mode Tuning . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 218
Gain Adjustment of the Position Loop . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 220
Gain Adjustment of Velocity Loop. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 222
Low Frequency Vibration Suppression in Position Mode . . . . . . . . . . . . . . . . . . . . 226
Mechanical Resonance Suppression . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 227
System Analysis . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 228
Phase Margin and Gain Margin . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 228
Guidelines for Gain/Phase Margin. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 229

<!-- page 7 -->

Table of Contents
Method for Using the System Analysis Tool. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 229
System Analysis Results. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 231

Select Operation Mode and Direction Control . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 234
Change using the Parameter Editor by using KNX5100C Software or
Programmatically . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 234
Position Control . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 236
PT Mode (Position Command with I/O Terminal Block Input) . . . . . . . . . . . . . . . . 237
Pulse Command Input Inhibitor (INHP). . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 240
Analog Input. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 240
PR Mode (Position Command with Internal Register Input) . . . . . . . . . . . . . . . . . . 243
Control Structure of Position Mode. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 244
Position Mode Timing (PR Mode) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 245
Speed Mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 246
Configure and Select the Preset Speeds . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 246
Scaling the Analog Command (Speed Mode) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 247
Control Structure of Speed Mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 248
Speed Mode Timing. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 249
Zero Speed Threshold Function . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 249
Torque Mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 252
Selection of Torque Command . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 252
Control Structure of Torque Mode. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 253
Scaling of Analog Command (Torque Mode) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 254
Torque Mode Timing. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 255
Filter. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 256
Position Mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 256
Speed Mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 257
Torque Mode Low Pass Filter. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 262
Speed and Torque Limit Functions. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 263
Speed Limits . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 263
Apply a Speed Limit . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 264
Torque Limits . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 265
Apply the Torque Limit . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 267
Enable/Disable Limits by using VelocityTorqueLimitAction . . . . . . . . . . . . . . . . . 268
Dual and Multi-modes. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 269
IO Mode. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 271
IO Mode - Position. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 279
IO Mode - Home . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 279
IO Mode - Gear. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 280
IO Mode - Index . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 281
IO Mode - Speed. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 281
IO Mode - Torque. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 281
Analog Outputs and Monitoring. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 282

Detailed Operation in PR Mode . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 286
Parameter Editor. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 286
PR Mode Definitions . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 288

<!-- page 8 -->

Table of Contents
Shared PR Parameters . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 293
Monitoring Variables in PR Mode. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 294
Homing. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 298
Setting Homing Mode ID297 (P5.004) - PR Mode . . . . . . . . . . . . . . . . . . . . . . . . . . . 298
Configuring Homing Setting ID397 (P6.000) - PR Mode . . . . . . . . . . . . . . . . . . . . . 299
Homing Speed and Position. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 300
Operation of Homing Types . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 300
Constant Speed Control . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 324
Position Control Command . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 326
Position Command Types . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 327
Jump Command . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 329
Write Command . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 331
Index Position Command. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 333
Hex Settings for Index Coordinate System . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 334
Index Coordinates Settings Wizard. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 335
Index Position Command Operation . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 338
Arithmetic Operation Commands . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 339
Arithmetic Operations . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 339
Use the PR Mode Editor in KNX5100C Software. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 342
Speed and Time Settings . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 343
General Parameter Settings. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 344
Homing Setting . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 345
PR Mode Setting . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 346
Display of PR Procedure in KNX5100C Software. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 348
Parts of the PR Display. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 348
Homing PR Display . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 349
Speed Command PR Display . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 350
Position Command PR Display. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 351
Jump Command PR Display. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 352
Write Command PR Display . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 352
Index Position Command PR Display . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 353
Arithmetic Operation PR Display. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 354
Trigger Methods for PR Commands . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 354
Digital Input (DI) Trigger. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 354
Event Trigger. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 356
Use PR Command Trigger ID300 (P5.007) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 357
Use IO Mode and Add-On Instruction . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 357
PR Execution Process . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 358
Sequence Command Execution. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 359
Command Interrupts Execution . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 363
Overlap Command Execution. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 369
Arithmetic Operation Command Execution . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 370

High-speed Position Capture Function (CAP) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 371
Using PR Command Programming with the Capture Function . . . . . . . . . . . . . . . 375
High-speed Position Compare Function (CMP) . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 378
Data Array . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 383
E-CAM . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 384

<!-- page 9 -->

Table of Contents
E-CAM Control Settings. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 386
Master Axis Signal Source . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 388
Clutch Engagement and Disengagement. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 392
E-CAM Alignment. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 397
E-CAM Profile Types . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 400

(STO) Feature
Certification . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 410
Important Safety Considerations . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 410
Category 3 Requirements According to ISO 13849-1 . . . . . . . . . . . . . . . . . . . . . . . . 410
Stop Category Definition . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 410
Performance Level (PL) and Safety Integrity Level (SIL) . . . . . . . . . . . . . . . . . . . . 410
Description of Operation . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 411
STO-related Fault Codes. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 412
Average Frequency of a Dangerous Failure per Hour. . . . . . . . . . . . . . . . . . . . . . . . . . . 414
Safe Torque Off Connector Data. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 415
Wire the Safe Torque Off Circuit. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 415
Safe Torque Off Wiring Requirements . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 416
Safe Torque Off Feature . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 416
Safe Torque Off Feature Bypass . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 417
Cascade the Safe Torque Off Signal. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 417
Safe Torque Off Specifications. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 418
Safe Torque Off Wiring Diagrams. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 418

System Requirements . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 423
Compatible Servo Motors. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 423
Install the Battery . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 425
System Initialization. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 426
Pulse Number . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 427
PUU Number . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 429
Initializing the Absolute Coordinates with Parameters . . . . . . . . . . . . . . . . . . . . . 429

Parameters
Organization of Parameters . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 431
Parameter Groups. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 431
Description of Digital Input Functions . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 433
Description of Digital Output Functions. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 437
Description of System Variable Monitoring. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 440
Panel Display . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 440
System Variable Monitoring Parameters . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 440
System Variables List. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 441
Description of Parameter Monitoring. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 445
Use a MSG Instruction to Set Parameters . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 447

<!-- page 10 -->

Table of Contents

## Drive System

Safety Precautions. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 449
Status Indicators . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 449
View Status and Faults. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 450
Drive Fault Code Display. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 450
Monitoring Status in KNX5100C Software. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 451
Fault Information in the KNX5100C Software . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 452
Fault and Status Information in Studio 5000 Application . . . . . . . . . . . . . . . . . . . 453
Drive Stopping Behavior . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 454
Clear Faults . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 454
General Troubleshooting . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 455
Appendix A
Interconnect Diagrams
Interconnect Diagram Notes. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 457
Power Wiring Examples . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 458
Passive Shunt Wiring Examples . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 463
Kinetix 5100 Drive/Rotary Motor Wiring Examples . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 464
Kinetix 5100 Servo Drive and Linear Actuator Wiring Examples. . . . . . . . . . . . . . . . . . 470
System Block Diagram. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 475
Appendix B
Upgrade Kinetix 5100 Drive
Firmware
Before You Begin . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 477
Inhibit the Module . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 478
Upgrade Your Firmware. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 479
Use ControlFLASH Plus Software to Upgrade Your Drive Firmware . . . . . . . . . . . 479
Use ControlFLASH Software to Upgrade Your Drive Firmware . . . . . . . . . . . . . . . 482
Verify the Firmware Upgrade . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 487
Appendix C
Use Add-On Instructions
Use of the Add-On Instruction Library . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 489
Kinetix 5100 Drive Device Object Add-On Instructions. . . . . . . . . . . . . . . . . . . . . 490
Download the Add-On Instruction Files and Data Types . . . . . . . . . . . . . . . . . . . . . . . . 492
Import the Add-On Instruction Files and Data Types (version 1.xxx) . . . . . . . . . . . . . . 492
Dvc Add-On Instruction Configuration (version 1.xxx) . . . . . . . . . . . . . . . . . . . . . . . . . . 494
Create the Add-On Instruction Tag . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 494
Create the Ref_Axis Tag. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 495
Configure the Add-On Instruction. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 496
General Execution Rules for Add-On Instructions. . . . . . . . . . . . . . . . . . . . . . . . . . 496
Opr Add-On Instruction Configuration . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 497
raC_UDT_Itf_K5100_Cfg . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 497
raC_UDT_Itf_K5100_Set . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 498
raC_UDT_Itf_K5100_Cmd . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 500
raC_UDT_Itf_K5100_Sts . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 501
raC_UDT_LookupMember_STR0082 . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 502
raC_UDT_Event . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 503
Add-On Instruction Details. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 504
raC_xxx_K5100_MSO . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 504
raC_xxx_K5100_MSF . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 505

<!-- page 11 -->

Table of Contents
raC_xxx_K5100_MAFR . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 506
raC_xxx_K5100_MAS . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 508
raC_xxx_K5100_MAJ . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 509
raC_xxx_K5100_MAM. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 511
raC_xxx_K5100_MAI. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 515
raC_xxx_K5100_MAG . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 517
raC_xxx_K5100_MAH . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 520
raC_xxx_K5100_MAT . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 522
Error Codes . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 524
Appendix D
Full Closed Loop Control
Full Closed-loop Control. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 526
Appendix E
Use the Scope Function in
KNX5100C Software
Get Started. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 529
Scope Functions . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 530
Quick Setup of Communication Channels . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 530
Select Communication Channels . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 531
Enable Stop Condition . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 534
FFT Display and Show RMS Value . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 535
Fine-Tune the Scope . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 536
Set Preferences . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 537
Use Popup Menu for Save Options . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 538
Appendix F
Automatic Device Configuration
New Kinetix Drive ADC Preparation . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 539
Get Started. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 541
Compare the Configuration Data . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 542
Upload the Configuration Data . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 543
Overwrite the Configuration Data. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 544
Appendix G
History of Changes
 . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 545
Index . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 547

<!-- page 12 -->

Table of Contents
Notes:
