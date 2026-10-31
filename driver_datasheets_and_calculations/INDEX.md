# driver_datasheets_and_calculations

Vendor PDFs and generated panel artifacts for the gantry. Firmware mechanical
constants cite these documents from `include/axis_drivetrain_params.h`.
Production motion is **EtherNet/IP** (see `docs/EXPECTED_ELECTROMECHANICAL_ASSEMBLY.md`).

## Present (selection)

| File | Role |
|------|------|
| `SCHUNK_design_technical_info.pdf` | Actuator / gripper selection |
| `Technical Information_SCHUNK_ERD_04 Catalog section.pdf` | Theta ERD 04 catalog chapter |
| `Technical Information_SCHUNK_ERD Catalog section.pdf` | Full ERD series catalog chapter |
| `SCHUNK_ERD_Operating_Manual.pdf` | ERD assembly / operating manual |
| `Optimization_Manual_Axis.pdf` | ERD manual tuning heuristic for Bosch-Rexroth/IndraWorks |
| `SCHUNK_ERD-ERT_IndraDrive_CS_Commissioning_instructions.pdf` | IndraDrive CS commissioning |
| `SCHUNK_ERD_motor_parameter-commissioning.zip` (+ extracted tree) | Motor params / IndraDrive `.par` / Motordatenblatt |
| `SCHUNK_ERD04_Motordatenblatt_IndraDrive.pdf` / `SCHUNK_ERD04_Datasheet_IndraDrive.pdf` | ERD04 IndraDrive motor data (from zip) |
| `Technical Information_Beta 100-ZRS*.pdf` | X belt actuator |
| `Technical Information_Beta 80-SRS*.pdf` | Z ballscrew actuator |
| `Technical Information_SCHUNK KGG 100-80*.pdf` | Gripper |
| `AB_Kinetix5100_user_manual.pdf` | Kinetix 5100 |
| `PUB00213R0_EtherNetIP_Developers_Guide.pdf` | ODVA EtherNet/IP Quick Start for Vendors Handbook |
| `PUB00123R1_Common-Industrial_Protocol_and_Family_of_CIP_Networks.pdf` | ODVA overview of CIP and EtherNet/IP (2016) |
| `PUB00028R1-Recommended-IP-Addressing-Methods-for-EtherNetIP-Devices.pdf` | ODVA IP addressing recommendations |
| `EtherNetIP Interoperabiltiy Recommendations.pdf.pdf` | ODVA 2009 conference paper on interoperability and performance testing |
| `PUB00070_Recommended-Functionality-for-EIP-Devices-v10.pdf` | ODVA Roundtable recommendations for EtherNet/IP devices (R10, 2018) |
| CIP Networks Library Vol 2, Ed. 1.18 (not in git) | ODVA EtherNet/IP specification. Single-subscription licence, p. iii: local copy only, do not commit |
| `AB_Kinetix_drive_specs.pdf` / `AB_Kinetix_motor_specs.pdf` | Drive / motor specs |
| `2198-in003_-en-p.pdf` | Kinetix installation |
| `R911325518_*HCS01_Commissioning Manual.pdf` | HCS01 connectors |
| `Rexroth HCS-01 Drive Project Planning Manual*.pdf` | HCS01 type code |
| `OA 1693058*.pdf` / `McMc order acknowledgement.pdf` | Purchase records |
| `SDF08NK8X_manual_V3.0_2024.pdf` | Legacy pulse driver (reference only) |

## Missing / pending

| Document | Needed for |
|----------|------------|
| SCHUNK Trap Move attachments | Inertia / torque commissioning slots in `axis_drivetrain_params.h` |
| 2198-IN020 | Formal TBIO pin drawing |
| MPB-xxVRS Functional Description | HCS01 X31 details if needed beyond EIP |

## Generated artifacts

| File | Generator |
|------|-----------|
| `BOM.xlsx` | `tools/generate_bom.py` |
| `WIRE_SIZE_SELECTION.xlsx` | `tools/generate_wire_size_selection.py` |

Optional PDF→markdown extraction: `tools/convert_pdfs_to_markdown.py` (output not
kept in-repo; regenerate locally if needed for search).
