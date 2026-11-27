# wt32-eth01-base tools

Firmware / documentation helpers for this repo. Host MQTT/serial lab scripts live
in the sibling **ADSB-PI-base** project.

## Scripts kept here

| Script | Purpose |
|--------|---------|
| `generate_bom.py` | Generate `driver_datasheets_and_calculations/BOM.xlsx` |
| `generate_wire_size_selection.py` | Generate `WIRE_SIZE_SELECTION.xlsx` |
| `direct_eip_driver_ui.py` | Direct PC-to-Drive EtherNet/IP & HTTP Suite GUI (unplug from WT32 and plug directly into drive chain) |
| `driver_config_ui.py` | EtherNet/IP Driver Configuration & Tuning GUI over WT32 bridge |
| `lan_debug_ui.py` | Full plant Ethernet TCP console GUI & telemetry monitor |
| `switch_traffic_logger.py` | Raw NIC packet logger for traffic visible at the unmanaged switch uplink |
| `hcs01_eng.py` | HCS01 engineering HTTP (status / C0500 / PM / OM / C0300 / travel / save / verify-origin / C6400); never writes CIP IP |
| `hcs01_comws.py` | Shared Service Tool COMWS client used by `hcs01_eng.py` and `hcs01_set_eip_io_map.py` |
| `hcs01_set_eip_io_map.py` | Load live 18/14 EtherNet/IP cyclic map (no CIP IP write) |

## Other files

| File | Purpose |
|------|---------|
| `cell_net_l2.py`, `example_cell_net_l2.py` | Python implementation of the CellNet L2 protocol, and an example of its use |
| `cellnet_ota_send.cpp` | Test sender for a firmware update over CellNet L2 (N-2) |
| `simulate_vision_output.py` | Simulated vision node: random detections as CellNet `VISION_DETECT` frames |
| `eip_test.py` | Interactive EtherNet/IP test harness for the Kinetix 5100 |
| `eip_jog.py` | Keyboard jog of the Kinetix X and Z axes over EtherNet/IP |
| `eip_position_abs.py` | Bench Position Absolute moves, in place of host-side StopMotion |
| `k5100_cyclic.py`, `hcs01_cyclic.py` | PC-side Class 1 cyclic drivers for the Kinetix 5100 and the HCS01 |
| `hcs01_build_erd04_eip_par.py` | Builds an HCS01 parameter file from `hcs01_Parameter_all_1_cip22.par` |
| `Parameter_default_1.par` | HCS01 parameter export (SERCOS ASCII), the source of the theta profile units |
| `tune_theta_inertia.py` | Tuning wizard for the SCHUNK ERD-04 theta axis on the HCS01 |
| `theta_trace_fetch.py` | Fetches the last `thtrace` into a CSV |
| `ota_flash.py` | Streams a firmware image to the gantry's TCP OTA server |
| `supervisor_ota.py` | Dual-OTA flash through the supervisor's developer toolbox HTTP API |
| `generate_purchasing_bom.py` | Purchasing BOM for building another gantry (writes a local, untracked spreadsheet) |
| `simulate_coordinated_grasp.py` | Coordinated-grasp simulation; uses `test/host/data/coordinated_grasp_reference.json` |
| `gantry_waypoints.txt` | Saved waypoints for `lan_debug_ui.py` |
| `test_lan_debug_ui.py` | Desktop check of `lan_debug_ui.py` (needs Tk; not in the host tests) |
| `test_serial_console.py` | Serial console exercise script (port COM9) |
| `setup_hold_defender_exclusion.ps1` | Windows Defender exclusions for the EtherNet/IP hold and jog state files (run once, elevated) |
| `schematic_reference/` | KiCad reference sheets: drives and motors, controller I/O |
| `requirements.txt` | Python packages for the generators |

Canonical design docs: [`docs/INDEX.md`](../docs/INDEX.md).

```powershell
pip install -r tools/requirements.txt
py -3 tools/generate_bom.py
py -3 tools/generate_wire_size_selection.py

# HCS01 eng HTTP (.22) — PC must be on the EIP daisy chain
py -3 tools/hcs01_eng.py status
py -3 tools/hcs01_eng.py travel --yes
py -3 tools/hcs01_eng.py c0300 --yes
py -3 tools/hcs01_eng.py save --yes
py -3 tools/hcs01_eng.py verify-origin
py -3 tools/hcs01_eng.py c0500

# NIC packet logger (requires Npcap on Windows + admin shell)
py -3 -m pip install scapy
py -3 tools/switch_traffic_logger.py --list-ifaces
py -3 tools/switch_traffic_logger.py --iface "<your-nic>" --bpf "ether proto 0x88b5"
py -3 tools/switch_traffic_logger.py --iface "<your-nic>" --conveyor-only --log logs/conveyor_packets.txt --pcap logs/conveyor_packets.pcap
```
