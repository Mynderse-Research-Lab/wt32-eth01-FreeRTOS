# Vendor catalog

Drive parameter sets in this folder, and where the vendor manuals are.

## In this folder

-   `driver_configs/`: Kinetix 5100 and HCS01 parameter sets, EDS files, the
    IndraWorks project.
-   `SCHUNK_ERD_motor_parameter-commissioning.zip` and its extracted tree: the
    SCHUNK ERD motor package, without its PDFs.
-   `2198-RD001_-en-p/` and `2198-RD002_-en-p/`, with their `.xlsx`: Kinetix
    5100 fault codes and parameter reference as CSV; `tools/eip_test.py` reads
    the fault codes.

## Vendor manuals

The PDFs are in the shared folder [`wt32-eth01-vendor-docs`][shared], at the
paths they had here. The four `AB_Kinetix_*` PDFs,
`SCHUNK_design_technical_info.pdf`, the Beta 100, Beta 80 and KGG 100-80 data
and the two purchase records are in
`Design Work/Gantry Mechanical System (v2 SCHUNK Components)/` instead.

-   **Rockwell Kinetix 5100:** user manual, installation (`2198-in003`), drive
    and motor specs, selection guide.
-   **Rexroth HCS01 and IndraDrive:** commissioning (R911325518), project
    planning (R911322210), MPx-18 functions (R911338673), MPx-16 to MPx-21
    parameters (R911328651), the HCS01 parameter document.
-   **SCHUNK:** ERD catalog sections, operating manual, IndraDrive CS
    commissioning, axis optimization manual, ERD04 and per-controller motor
    data, Beta 100 (X), Beta 80 (Z), KGG 100-80 (gripper).
-   **ODVA:** PUB00213 (EtherNet/IP quick start for vendors), PUB00123 (CIP
    overview), PUB00028 (IP addressing), PUB00070 (device recommendations), a
    2009 interoperability paper.
-   **Other:** WIZ850io (W5500 module), SDF08NK8X (legacy pulse driver),
    purchase records.
-   **Text extracts** of three manuals, with the scripts that made them: search
    aids; cite the PDF.

The CIP Networks Library Vol 1 (Ed. 3.3, all rights reserved) and Vol 2 (Ed.
1.18, single-subscription licence) are local copies only: never commit or share
them.

## Missing

-   SCHUNK Trap Move attachments: the inertia and torque commissioning slots in
    `axis_drivetrain_params.h`.
-   2198-IN020: the TBIO pin drawing.
-   MPB-xxVRS functional description: HCS01 X31, if needed beyond EtherNet/IP.

## See also

-   [Doc index](/docs/INDEX.md).

[shared]: https://drive.google.com/drive/folders/1NL5sW1CT2gorexiiKgt4eoGlQNMBzlNm
