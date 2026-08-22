# wt32-eth01-base tools

Firmware / documentation helpers for this repo. Host MQTT/serial lab scripts live
in the sibling **ADSB-PI-base** project.

## Scripts kept here

| Script | Purpose |
|--------|---------|
| `generate_bom.py` | Generate `driver_datasheets_and_calculations/BOM.xlsx` |
| `generate_wire_size_selection.py` | Generate `WIRE_SIZE_SELECTION.xlsx` |
| `eip_position_abs.py` / `eip_test.py` | PC-side EIP prove tools |
| `hcs01_eng.py` | HCS01 engineering HTTP (status / C0500 / PM / OM / C0300 / travel / save / verify-origin / C6400); never writes CIP IP |
| `hcs01_comws.py` | Shared Service Tool COMWS client used by `hcs01_eng.py` and `hcs01_set_eip_io_map.py` |
| `hcs01_set_eip_io_map.py` | Load live 18/14 EtherNet/IP cyclic map (no CIP IP write) |

Canonical design docs: [`docs/INDEX.md`](../docs/INDEX.md).

```powershell
pip install openpyxl
py -3 tools/generate_bom.py
py -3 tools/generate_wire_size_selection.py

# HCS01 eng HTTP (.22) — PC must be on the EIP daisy chain
py -3 tools/hcs01_eng.py status
py -3 tools/hcs01_eng.py travel --yes
py -3 tools/hcs01_eng.py c0300 --yes
py -3 tools/hcs01_eng.py save --yes
py -3 tools/hcs01_eng.py verify-origin
py -3 tools/hcs01_eng.py c0500
```
