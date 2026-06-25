# wt32-eth01-base tools

This repo is **firmware and hardware documentation** (C++ / ESP-IDF). Host-side lab scripts moved to **[ADSB-PI-base](https://github.com/)** (sibling: `Projects/ADSB-PI-base`).

## Moved to ADSB-PI-base (`tools/`)

- MQTT: `send_frame.py`, `host_mqtt_broker.py`, `lab_net.py`
- USB serial bring-up: `listen_serial.py`, `capture_boot.py`, `probe_console.py`, `poll_max_limit.py`, `reset_and_listen.py`, `run_home_probe.py`, `run_calibrate_probe.py`

Install: `pip install -r requirements.txt` in **ADSB-PI-base**.

## Remaining here (repo build / BOM)

| Script | Purpose |
|--------|---------|
| `generate_bom.py` | Generate `driver_datasheets_and_calculations/BOM.xlsx` |
| `generate_wire_size_selection.py` | Generate wire-size spreadsheet (includes Motion I/O IF hop) |
| `convert_pdfs_to_markdown.py` | Convert PDFs under `driver_datasheets_and_calculations/` to `pdf_markdown/` |
| `srs_build/` | SRS Markdown → DOCX export |

Motion I/O interface design: [`docs/MOTION_IO_INTERFACE.md`](../docs/MOTION_IO_INTERFACE.md).

```bash
pip install openpyxl pymupdf4llm   # BOM generators + PDF conversion
py tools/generate_bom.py
py tools/generate_wire_size_selection.py
py tools/convert_pdfs_to_markdown.py
```
