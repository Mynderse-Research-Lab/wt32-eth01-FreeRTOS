# Rexroth support request — HCS01 Multi-Ethernet EtherNet/IP fieldbus IP not writable

**Date:** 2026-08-11  
**Update (2026-08-18):** CIP **does** answer TCP 44818 on **`192.168.1.23`** (FKM MAC `…B9`). Engineering HTTP/IndraWorks stays on **`192.168.1.22`** (MAC `…B6`). Firmware `EIP_TARGET_IP_THETA` is `.23`. SnapShot `P-0-4089.0.13` may still show `.22`; keypad/FKM is what RegisterSession hit. The write-IDN narrative below is the historical ticket.

**Site / project:** WT32-ETH01 gantry (Lawrence Tech / lab) — theta axis  
**Priority:** Blocks EtherNet/IP Class 1 originator FO to theta (TCP 44818)

---

## Ask

How do we set and activate the EtherNet/IP (FKM / master communication) IP address on this Multi-Ethernet HCS01 so CIP listens on TCP 44818?

Writing `P-0-4089.0.13` (alias `P-0-2315`) fails even in **parameter mode (PM)** with:

> Invalid operating data: IDN is not supported, invalid bit number or bit combination

Engineering-over-IP on X24/X25 works. We need the **fieldbus / CIP** IP, not engineering.

Please advise the correct procedure for **CSB02.1C-ET-EC + MPB-20V30** (keypad steps, IndraWorks dialog, DHCP/BootP, required FW, or known limitation).

---

## The current test setup

Lab gantry: An MCU as EtherNet/IP **originator** on W5500 at `192.168.1.10` already runs Class 1 Absolute FO to two Kinetix axes (X/Z). Third axis this HCS01 on the same subnet at engineering IP `192.168.1.22` (X24/X25). Firmware scanner is built for HCS01 assemblies **101/102** and retries TCP **44818** to `.22`; X/Z stay up when theta is unreachable.

**Working today:** ping/HTTP/Telnet and IndraWorks over engineering IP; Multi-Ethernet protocol = EtherNet/IP (`P-0-4089.0.1=3`, profile `0xFFFE`); motor/encoder commissioning on ERD04 is separate and does not open CIP.

**Stuck:** master-communication (FKM) IP `P-0-4089.0.13` remains `0.0.0.0` → CIP never listens → no RegisterSession / Class 1 FO. We tried IndraWorks EtherNet/IP tab (OM + PM), Service Tool `.par` load (direct `.0.13` and alias `P-0-2315`), patched full dump, keypad FKM-IP, and PC probe of 44818 — all leave `.0.13` unset and 44818 closed. ARP shows only the engineering MAC (`…B6`), not the FKM device MAC (`…B9`). Engineering path alone cannot replace Class 1.

---

## Hardware / firmware

| Item | Value |
|------|--------|
| Power section | `HCS01.1E-W0005-A-03-B-ET-EC-NN-NN-NN-FW` |
| Control section | `CSB02.1C-ET-EC-NN-NN-NN-FW` |
| Firmware | `FWA-INDRV*-MPB-20V30-D5-1-NNN-NN` |
| Motor (theta) | SCHUNK `ERD 04-40-D-H-N` (third-party sync, HIPERFACE) |
| Engineering tool | IndraWorks Ds (online to drive) |
| Service Tool web | `http://192.168.1.22/` — login `service` / `serviceWeb` |

---

## Network intent

| Role | Desired |
|------|---------|
| EtherNet/IP (CIP / Class 1) | **`192.168.1.22`** / `255.255.255.0` / gateway `0.0.0.0` |
| Master protocol | EtherNet/IP (`P-0-4089.0.1 = 3`) |
| Profile | `P-0-4084 = 0xFFFE` |
| Assemblies (planned) | MDT **101** (18 B), AT **102** (14 B); config CP **110** in UI |
| Originator | ESP32 + W5500 at `192.168.1.10` (daisy-chain peers OK on same subnet) |

---

## What works

- Ping `192.168.1.22`
- HTTP `:80` (Service Tool / SnapShot.par)
- Telnet `:23` (login prompt)
- IndraWorks online via **Engineering over IP (X24/X25)**
- `S-0-1020 = 192.168.1.22` (engineering IP) sticks across reboot
- `P-0-4089.0.1 = 3` (EtherNet/IP) sticks
- `P-0-4084 = 0xFFFE` sticks
- Multi-Ethernet protocol selection UI shows EtherNet/IP active
- Encoder feedback live in Easy startup (absolute position updates when shaft turned by hand)

---

## What fails (core issue)

| Check | Result |
|-------|--------|
| `P-0-4089.0.13` (Master communication IP) | Remains **`0.0.0.0`** after reboot |
| TCP **44818** (CIP RegisterSession) | **Connection refused / fail** (PC probe) |
| Write `P-0-4089.0.13 = 192.168.1.22` in IndraWorks EtherNet/IP tab **in PM** | Error: **IDN is not supported, invalid bit number or bit combination** (value shown as octets 192 / 168 / 1 / 22) |
| Load SERCOS-ASCII `.par` containing `P-0-4089.0.13/14/15` | Same “IDN is not supported…” on those IDNs |
| IndraWorks parameter search for `P-0-2315/2316/2317` | Redirects to alias `P-0-4089.0.13/14/15` (same write failure) |

### Related MAC evidence (from parameter dump)

| IDN | Role | MAC (last octet) |
|-----|------|------------------|
| Engineering (`S-0-1019` family) | Eng over IP | `…:B6` |
| `P-0-4089.0.10` | Master communication device | `…:B9` |
| `P-0-4089.0.11` | Port 1 | `…:B7` |
| `P-0-4089.0.12` | Port 2 | `…:B8` |

Fieldbus diagnostics on EtherNet/IP tab: **`NO IO: no IO communication after Bootup`** (expected until an originator connects - But we are unable to open CIP).

---

## Steps already tried

1. Set master communication to **Multi-Ethernet**, active protocol **EtherNet/IP**; reboot.
2. Set EtherNet/IP tab IP/mask/gateway in UI → write fails on `P-0-4089.0.13`.
3. Parameter mode (PM); retry EtherNet/IP IP write → same error.
4. Load minimal `.par` with `P-0-4089.0.13=192.168.1.22` (and `S-0-102x`) via Service Tool driveparamload → IDNs rejected as not supported.
5. Load patched full dump with only CIP IP changed → `P-0-4089.0.13` still rejected (other IDNs write-protected as expected for full dump).
6. Confirmed post-reboot via `http://192.168.1.22/SnapShot.par`: `.0.13` still `0.0.0.0`; TCP 44818 still closed.
7. Motor commissioning progressed separately (ERD04 data); Easy startup can hit AF when bus ready — **orthogonal** to CIP listen issue.


---

## Questions for Rexroth

1. On **CSB02.1C-ET-EC** using the **MPB-20V30** firmware , what is the **supported method** to set EtherNet/IP master-communication IP (`P-0-4089.0.13`)?
2. Is write of parameter `P-0-4089.0.13` over the **engineering channel** unsupported by design? If so, what channel/tool must be used?
3. Is a **firmware update** (e.g. newer MPB-20V3x / MPB-21) required for FKM IP configuration on this hardware?
4. After a valid FKM IP is active, should CIP **44818** port appear on the **same X24/X25** jacks as engineering, or only on a specific Multi-Ethernet port (Port1/Port2 / MAC `…B9`)?
5. Any known limitation when engineering IP and FKM IP are both intended to be `192.168.1.22`?

---


## Impact

Unable to utilize HCS01 driver over the existing EtherNet/IP Class 1 scanner stack (assemblies 101/102). X/Z Kinetix Forward-Open on the same originator are already working. 

