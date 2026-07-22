# Preface

_Source: Rockwell Automation Publication 2198-UM004D-EN-P - December 2022_

_Original file: `01_Preface.pdf` (4 pages)_

<!-- page 1 -->

Preface
This manual provides detailed installation instructions for mounting, wiring,
and troubleshooting your Kinetix® 5100 drive; and system integration for your
drive/motor combination with a Logix controller.
This manual is intended for engineers and technicians that are directly
involved in the installation and wiring of the Kinetix 5100 drive and
programmers who are directly involved in operation, field maintenance, and
integration of the Kinetix 5100 drive.
If you do not have a basic understanding of the Kinetix 5100 drive, contact
your local Rockwell Automation sales representative for information on
available training courses.
Summary of Changes
This publication contains the following new or updated information. This list
includes substantive updates only and is not intended to reflect all changes.
Translated versions are not always available for each revision.
Conventions
These conventions are used throughout this manual:
•
Bulleted lists such as this one provide information, not procedural
steps.
•
Numbered lists provide steps or hierarchical information.
•
Parameters are shown in this format: ID185 (P2.000).
Access Fault Codes and
Parameter List
Topic
Page
Added “and UK” after CE, where applicable
Throughout
Added United Kingdom to Agency Compliance

Added Appendix F, Automatic Device Configuration

This manual links to Kinetix® 5100 Servo Drive Fault Codes Reference
Data, publication 2198-RD001, for fault codes and Kinetix 5100 Servo
Drive Parameters Reference Data, publication 2198-RD002, for
parameters. Download the spreadsheets now for offline access.

<!-- page 2 -->

Preface
Additional Resources
These documents contain additional information concerning related products from
Rockwell Automation.
Resource
Description
Kinetix 5100 Servo Drive Fault Codes Reference Data, publication 2198-RD001 Provides the fault codes for Kinetix 5100 servo drives.
Kinetix 5100 Servo Drive Parameters Reference Data, publication 2198-RD002 Provides the parameters for Kinetix 5100 servo drives.
Kinetix Rotary Motion Specifications, publication KNX-TD001
Provides product specifications for Kinetix VPL, VPC, VPF, VPH, VPS, Kinetix MPL, MPM, MPF,
MPS; Kinetix TL and TLY, Kinetix RDB, and Kinetix HPK rotary motors.
Kinetix Linear Motion Specifications, publication KNX-TD002
Provides product specifications for Kinetix MPAS and MPMA linear stages, Kinetix VPAR,
MPAR, and MPAI electric cylinders, and Kinetix LDC and Kinetix LDL linear motors.
Kinetix 5700, 5500, 5300, 5100 Servo Drives Specifications, publication KNXTD003
Provides product specifications for Kinetix Integrated Motion over the EtherNet/IP network
and EtherNet/IP networking servo drive families.
Kinetix Rotary and Linear Motion Cable Specifications Technical Data,
publication KNX-TD004
Product specifications for Kinetix 2090 motor and interface cables.
Kinetix 3, 300, 350, 2000, 6000, 6200, 6500, 7000 Servo Drives
Specifications, publication KNX-TD005
Provides product specifications for Kinetix Integrated Motion over the EtherNet/IP network
(Kinetix 6500 and Kinetix 350), Integrated Motion over Sercos interface (Kinetix 6200, Kinetix
6000, Kinetix 2000, and Kinetix 7000), and component (Kinetix 3) servo drive families.
Kinetix 5100 Drive Systems Design Guide, publication KNX-RM011
System design guide to select the required (drive specific) drive module, power accessory,
feedback connector kit, and motor cable catalog numbers for your Kinetix 5100 drive
system.
Kinetix 5100 EtherNet/IP Indexing Servo Drive Automatic Device
Configuration Application Technique, publication 2198-AT004
Provides information on how to use Automatic Device Configuration to configure your
Kinetix drive.
Ultra3000 to Kinetix 5100 Servo Drive Migration Guide,
publication 2198-RM003
Provides information on how to migrate from the Ultra™3000 drive to the Kinetix 5100 servo
drive.
Kinetix 300 to Kinetix 5100 Servo Drive Migration Guide,
publication 2198-RM004
Provides information on how to migrate from the Kinetix 300 drive to the Kinetix 5100 servo
drive.
Kinetix 5100 AC Line Filter Installation Instructions, publication 2198-IN017
Provides information on how to install and wire the Kinetix 5100 AC line filters.
Kinetix 5100 Auxiliary Feedback Connector Kit Installation Instructions,
publication 2198-IN018
Provides information on how to attach the Kinetix 5100 auxiliary feedback connector kit to
your shielded, twisted-pair customer-supplied cable.
Kinetix 5100 Feedback Connector Kit Installation Instructions,
publication 2198-IN019
Provides information on how to attach the Kinetix 5100 feedback connector kit to Kinetix
2090 flying lead motor feedback cables.
Kinetix 5100 I/O Terminal Expansion Block Installation Instructions,
publication 2198-IN020
Provides information on how to install and wire the Kinetix 5100 I/O terminal expansion
block.
Kinetix 5700 Shunt Passive Modules Installation Instructions,
publication 2198-IN011
Provides information on how to install and wire 2198-R004 and 2198-R031 passive shunts for
use with Kinetix 5100 servo drives.
Kinetix 300 Shunt Resistor Installation Instructions, publication 2097-IN002
Provides information on how to install and wire 2097-R6 and 2097-R7 shunt resistors for use
with Kinetix 5100 servo drives.
Feedback Battery Box Installation Instructions, publication 2198-IN022
Provides information on how to install or replace a battery box, install a battery, and prepare
a feedback cable for a battery box installation.
Shaft Seal Kits for Kinetix TLP Motors Installation Instructions,
publication 2090-IN044
Provides information about how to remove and replace shaft seals on Kinetix TLP motors.
2090-Series Kinetix TLP Power and Feedback Cables,
publication 2090-IN046
Provides information on how to build cables for Kinetix TLP servo motors.
Build Your Own Kinetix TLP Motor Cables Installation Instructions,
publication 2090-IN048
Provides information on how to attach Kinetix 2090 connector kits to bulk cable and build
your own Kinetix TLP motor power and feedback cables.
Kinetix TLP Multi-purpose Servo Motors Installation Instructions,
publication TLP-IN001
Provides information on how to install the Kinetix TLP multi-purpose servo motor.
System Design for Control of Electrical Noise Reference Manual,
publication GMC-RM001
Provides information, examples, and techniques designed to minimize system failures
caused by electrical noise.
Servo Drive Installation Best Practices Application Technique,
publication MOTION-AT004
Best practice examples to help reduce the number of potential noise or electromagnetic
interference (EMI) sources in your system and to make sure that the noise sensitive
components are not affected by the remaining noise.
Kinetix Motion Control Selection Guide, publication KNX-SG001
Overview of Kinetix servo drives, motors, actuators, and motion accessories that are
designed to help make initial decisions for the motion control products best suited for your
system requirements.
MicroLogix 1100 Programmable Controllers User Manual,
publication 1763-UM001
Provides information on how to install, wire, and troubleshoot the MicroLogix™
programmable controllers.
MicroLogix 1200 Programmable Controllers User Manual,
publication 1762-UM001
MicroLogix 1400 Programmable Controllers User Manual,
publication 1766-UM001

<!-- page 3 -->

Preface
Micro810 Programmable Controllers User Manual, publication 2080-UM001
Provides information on how to install, wire, and troubleshoot the Micro800™ programmable
controllers.
Micro820 Programmable Controllers User Manual, publication 2080-UM005
Micro830, Micro850, Micro870, Programmable Controllers User Manual,
publication 2080-UM002
GuardLogix 5570 Controllers User Manual, publication 1756-UM022
Provides information on how to install, configure, program, and use ControlLogix®
controllers and GuardLogix® controllers in Studio 5000 Logix Designer® projects.
GuardLogix 5580 Controllers User Manual, publication 1756-UM543
Compact GuardLogix 5370 Controllers User Manual, publication 1769-UM022
Provides information on how to install, configure, program, and use CompactLogix™ and
Compact GuardLogix controllers.
Compact GuardLogix 5380 Controllers User Manual, publication 5069-UM001
GuardLogix 5570 and Compact GuardLogix 5370 Controller Systems Safety
Reference Manual, publication 1756-RM099
Provides information on how to achieve and maintain Safety Integrity Level (SIL) and
Performance Level (PL) safety application requirements for GuardLogix and Compact
GuardLogix controllers.
GuardLogix 5580 and Compact GuardLogix 5380 Controller Systems Safety
Reference Manual, publication 1756-RM012
ControlFLASH Firmware Upgrade Kit User Manual, publication 1756-UM105
Provides information on how to upgrade your drive firmware by using ControlFLASH™
software.
Rockwell Automation Product Selection website rok.auto/systemtools
Online product selection and system configuration tools, including AutoCAD (DXF) drawings.
Motion Analyzer System Sizing and Selection Tool
website https://motionanalyzer.rockwellautomation.com/
Comprehensive motion application sizing tool used for analysis, optimization, selection, and
validation of your Kinetix Motion Control system.
EtherNet/IP Network Devices User Manual, ENET-UM006
Describes how to configure and use EtherNet/IP devices to communicate on the EtherNet/IP
network.
Ethernet Reference Manual, ENET-RM002
Describes basic Ethernet concepts, infrastructure components, and infrastructure features.
Safety Guidelines for the Application, Installation, and Maintenance of
Solid-State Control, publication SGI-1.1
Designed to harmonize with NEMA Standards Publication No. ICS 1.1-1987 and provides
general guidelines for the application, installation, and maintenance of solid-state control in
the form of individual devices or packaged assemblies incorporating solid-state
components.
Industrial Automation Wiring and Grounding Guidelines, publication 1770-4.1
Provides general guidelines for installing a Rockwell Automation industrial system.
Product Certifications website, rok.auto/certifications.
Provides declarations of conformity, certificates, and other certification details.
Resource
Description

<!-- page 4 -->

Preface
Notes:
