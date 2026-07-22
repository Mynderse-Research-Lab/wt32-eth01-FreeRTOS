# wt32-eth01-base tools

Firmware / documentation helpers for this repo. Host MQTT/serial lab scripts live
in the sibling **ADSB-PI-base** project.

## Scripts kept here

| Script | Purpose |
|--------|---------|
| `generate_bom.py` | Generate `driver_datasheets_and_calculations/BOM.xlsx` |
| `generate_wire_size_selection.py` | Generate `WIRE_SIZE_SELECTION.xlsx` |
| `eip_position_abs.py` / `eip_test.py` | PC-side EIP prove tools |
| `srs_build/` | SRS Markdown → DOCX export |

Canonical design docs: [`docs/INDEX.md`](../docs/INDEX.md).

```powershell
pip install openpyxl
py -3 tools/generate_bom.py
py -3 tools/generate_wire_size_selection.py
```
