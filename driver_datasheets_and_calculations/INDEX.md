# driver_datasheets_and_calculations

Reference hardware documents that the PulseMotor-based firmware depends on
for per-axis commissioning. File names are normalized so the firmware headers
can cite them unambiguously.

## Present

| File                                      | Describes                                                                                       | Commissioning values supplied                                                   |
|-------------------------------------------|--------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------|
| `SDF08NK8X_manual_V3.0_2024.pdf`          | Bergerda SDF08NK8X servo driver manual (legacy X-axis driver; kept as a pulse-train reference). | PN parameter map, pulse-input bandwidth, I/O wiring.                            |
| `AB_Kinetix5100_user_manual.pdf`          | Allen-Bradley Kinetix 5100 servo drive user manual (X and Y axis drivers).                       | PTI pulse-input bandwidth, electronic gear (`AXIS_{X,Y}_ENCODER_PPR`), alarm and enable wiring. |
| `AB_Kinetix_drive_specs.pdf`              | Kinetix 5100 drive specifications.                                                               | Supply rails, torque, I/O limits.                                              |
| `AB_Kinetix_motor_specs.pdf`              | Kinetix 5100 motor specifications.                                                               | Reflected inertia inputs, torque curves.                                       |
| `AB_Kinetix_motion_control_selection_guide.pdf` | Motion-control sizing methodology for the Kinetix family.                                   | Trap-move / sizing inputs that fill the reserved block in `axis_drivetrain_params.h` Section 3.2.7. |
| `SCHUNK_design_technical_info.pdf`        | SCHUNK gantry-hardware selection (Beta 100-ZRS, Beta 80-SRS, KGG 100-80).                        | X belt lead (200 mm/rev), Y ballscrew pitch (20 mm), 3000 rpm critical speed, gripper open/close times, repeatability tolerances. |

## Missing / pending

Documents referenced by the firmware headers but not yet committed to this
folder. Each row lists where to obtain the document and which firmware values
it will supply.

| Document                              | Source                                                                                  | Commissioning values it will supply                                                  |
|---------------------------------------|------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------|
| SCHUNK ERD 04-40-D-H-N datasheet      | <https://schunk.com/us/en/automation-technology/rotary-actuators/erd/erd-04-40-d-h-n/p/000000000000331220> | Detailed torque-speed curve, HIPERFACE resolution, mechanical envelope (beyond the datasheet values already captured in `axis_drivetrain_params.h`). |
| Custom ERD pulse-train driver parameter sheet | Not yet released by the controls vendor.                                             | `AXIS_THETA_MAX_PULSE_FREQ_HZ`, electronic gear setup (confirms the 36000 ppr default), alarm/enable wiring details. |
| SCHUNK Trap Move attachments (Beta 100-ZRS, Beta 80-SRS, KGG 100-80) | The four PDFs referenced on the final page of `SCHUNK_design_technical_info.pdf`; were not saved with the quote. Rick Hunsucker (SCHUNK) can resend. | Reflected inertia, required torque, cycle-time target. Populate the reserved macro block in `include/axis_drivetrain_params.h` (Section 3.2.7). |

Add rows above when new drivers or actuators are commissioned. Keep the
"Present" and "Missing / pending" tables in sync with the files actually
checked in.
