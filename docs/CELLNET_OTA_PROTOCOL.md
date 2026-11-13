# CellNet OTA Block Protocol (draft)

**Status:** Draft, 2026-11-13, for [N-2](OPEN_ITEMS.md#cell-transport). Nothing here is implemented yet. Three decisions are needed before work starts ([section 10](#10-decisions-needed)). Two adversarial reviews checked this draft on 2026-11-13; their findings are folded in.
**Scope:** firmware updates for the WT32 nodes over CellNet L2, sent by the supervisor. This replaces the TCP OTA on port 8032 as part of N-1, which makes CellNet the only transport in the cell.

Evidence labels follow [OPEN_ITEMS.md](OPEN_ITEMS.md#how-to-keep-this-list-live):
- **Measured** numbers come from the gantry on 2026-11-13.
- **Derived** numbers are arithmetic on measured ones.
- **Assumed** numbers are not checked; each one says how to check it.

## 1. What the measurements settle

| Input | Value | Label and source |
|-------|-------|------------------|
| Image | 843,360 bytes (gantry, Nov 3 build). Its last 4 KB block holds 3,680 bytes | Measured |
| TCP OTA today, whole update on the gantry (erase up front) | 6.4 and 6.6 s | Measured; N-2, receiver timing in `OK COMPLETE` (`src/gantry_ota.cpp`) |
| Erase of the image range up front, 64 KB blocks | 2.8 to 2.9 s | Measured, N-2 |
| Writing the whole image | 2.78 and 2.84 s; longest single write 15 and 59 ms | Measured, N-2 |
| Flash write of one 4 KB block | about 13.5 ms (2.78 s / 206 blocks) | Derived |
| Image check and boot-slot write (`esp_ota_end`, `esp_ota_set_boot_partition`) | 0.6 s | Measured, N-2 |
| CellNet receive, 60-byte frames, evenly paced | at most 0.16 % loss up to 30,000 frames/s; ceiling about 36,700 | Measured with `netstat`, N-2. The sender spaced frames evenly with a busy-wait, not in bursts |
| CellNet receive, 1,014-byte frames, evenly paced | at most 0.31 % loss up to 10,000 frames/s | Measured, N-2 |
| Where frames are lost | EMAC missed-frame counter: no free receive buffer | Measured, N-2 |
| Other CellNet traffic reaching the gantry | about 5 to 6 frames/s | Measured with `netstat` at idle (11 frames in 2 s) |
| EMAC receive buffers | 10 of 512 bytes. The FCS is stored in the buffer, so a 508-byte frame fills one | `idf/sdkconfig` `CONFIG_ETH_DMA_RX_BUFFER_NUM=10`, `CONFIG_ETH_DMA_BUFFER_SIZE=512`; ESP-IDF `esp_eth_mac_esp_dma.c:335` |
| Largest frame `parseFrame` accepts | 64 bytes, after ESP-IDF strips the FCS | `CellNetL2Framing.cpp:93` |
| Flash write window | up to 8 KB per cache-off window, so one 4 KB write is one window | `CONFIG_SPI_FLASH_WRITE_CHUNK_SIZE=8192`; ESP-IDF `esp_flash_api.c` (`esp_flash_write`) |
| Erase | yields to other tasks every 20 ms | `CONFIG_SPI_FLASH_YIELD_DURING_ERASE=y`, `CONFIG_SPI_FLASH_ERASE_YIELD_DURATION_MS=20` |
| Rollback | on; task-watchdog panic off | `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`; `CONFIG_ESP_TASK_WDT_PANIC` not set |

## 2. The floor, and why CellNet can at best match TCP

A 4 KB flash write disables the cache on both cores for its whole duration, about 13.5 ms. The receive task cannot run during it, and the block buffer must be in DRAM.
- The EMAC keeps filling its 10 receive buffers in hardware, and drops frames once they are full.
- So during a write, at most 10 buffers' worth of frames can wait.
- The erase is different: it yields every 20 ms, so other tasks can run and the node can keep signalling while it erases. The protocol does not depend on this, because the sender waits for `READY`.

The whole update can therefore be no shorter than erase + write + check: 2.8 + 2.8 + 0.6 = **6.2 s** (derived). TCP already reaches 6.4 to 6.6 s. **A CellNet OTA can match TCP, but it cannot meaningfully beat it.**

OTA-only mode (decided in N-2) also costs one extra reboot before the update.
- After the TCP flashes, the new app's clock started about 1.6 s after `OK COMPLETE` (derived from its uptime at the first console answer). 0.7 s of that is the receiver's own delay before `esp_restart`.
- The network was answering within 7 s of app start, but polling every 3.5 s gives no closer bound.
- So boot-to-link time is not known. Decision D3 asks for it to be measured from the supervisor.

On the 2026-11-05 rule ("larger frames and resends only if faster than TCP"):
- **Resends are not optional.** Frames were lost at every rate above 2,000 frames/s, so the protocol needs them to be correct.
- **Larger frames buy about 0.8 s** (section 8), but still do not beat TCP once the reboot is counted. This is decision D1.

## 3. Entering and leaving OTA-only mode

**Entry.** The supervisor sends `CELL_COMMAND 7 = REBOOT_TO_OTA` with `param_u16` = session. The gantry already takes `CELL_COMMAND` only from node 0x04 (`pick_scheduler.cpp`, `onCellCommand`). The node then does what today's TCP path does before an update (`src/gantry_ota.cpp`):
1. It refuses if the gantry is busy (move, home or pick), or if the running image is not yet marked valid (`esp_ota_get_state_partition`). A pending image cannot start an update: `esp_ota_begin` would refuse, and a reset would roll it back.
2. It disables the motors under control (`gantryOtaDisableMotors`), and restarts only once they are confirmed off. When Class 1 then drops at the reboot, the drives are already servo-off, as today.
3. It stores a flag in `__NOINIT_ATTR` DRAM, which ESP-IDF documents to survive a software restart (`memory-types.rst`, "noinit" DRAM). The flag is `{u32 magic; u16 session; u16 ~session}`. Content is random at power-on, so a single byte could read as set.
4. It calls `esp_restart`.

**At boot**, before anything else:
- Read the flag and clear it. Enter OTA mode only if the magic and complement match and `esp_reset_reason() == ESP_RST_SW`.
- Bench-test this: set the flag and restart 20 times, then power-cycle once. OTA mode must be entered exactly when expected.
- Clearing at once means a crash in OTA mode comes back up in normal mode.

**In OTA mode:**
- The node starts Ethernet, CellNet and the receiver only. It attaches CellNet as soon as the PHY link is up, without waiting for an IP address (`ETH_IP_WAIT_TIMEOUT_MS` is 1.5 s today). The EtherNet/IP scanner never starts, so the motors stay off.
- The gantry sends no heartbeat today; `sendHeartbeat` has no caller in `src/`. OTA mode sends a `HEARTBEAT` every 250 ms with `status_flags` bit 15 (`OTA_MODE`) set, and bit 14 (`ERASING`) set while erasing.
- **An absolute deadline starts at boot, before Ethernet init:** 30 s to the first valid `OTA_BEGIN`, then 2 s with no progress (section 6). Every error ends the same way: `esp_ota_abort`, an error status, and a reboot into the normal image.

**Accepted frames.** Only `OTA_BEGIN`, `OTA_DATA` and `OTA_END` from sender 0x04 carrying the stored session. Any other session gets `ERR_SESSION`, and the sender drops replies whose session does not match.
- The supervisor keeps the session as a counter, not a random number, so a stale `OTA_BEGIN` from an earlier attempt cannot capture the node.
- A `REBOOT_TO_OTA` with a new session, received while in OTA mode, aborts and restarts with the new session.

## 4. Messages

All frames carry CellNet's EtherType 0x88B5, version 1 and the 8-byte cell header (`include/cell_net_l2_protocol.h`). The cell header's `sequence` is not used by OTA. OTA frames are unicast between the supervisor and one node.

| Type | Name | Direction | Payload (little-endian, packed) | Frame |
|------|------|-----------|----------------------------------|-------|
| 0x05 | `OTA_BEGIN` | S → N | `session u16`, `block_size u16` (4096), `frag_data u16`, `reserved u16`, `image_size u32` | 34 B, padded to 60 |
| 0x06 | `OTA_DATA` | S → N | `session u16`, `block u16`, `frag u8`, `flags u8` (bit 0 `ACK_REQ`), `data_len u16`, `data[data_len]` | 30 B + data |
| 0x07 | `OTA_STATUS` | N → S | `session u16`, `block u16`, `status u8`, `reserved u8`, `missing[16]` (bit *i* set = fragment *i* missing) | 44 B, padded to 60 |
| 0x08 | `OTA_END` | S → N | `session u16`, `reserved u16`, `image_size u32` | 30 B, padded to 60 |
| 0x09 | `OTA_RESULT` | N → S | `session u16`, `status u8`, `reserved u8`, `erase_ms u16`, `stream_ms u16`, `write_ms u16`, `check_ms u16` | 34 B, padded to 60 |

**`OTA_STATUS` codes:** `READY` (erase done), `RECEIVED`, `MISSING` (bitmap valid), `ERR_SIZE`, `ERR_SESSION`, `ERR_BUSY`, `ERR_FLASH`, `ERR_VERIFY`, `ERR_TIMEOUT`.

**Everything else is derived from `OTA_BEGIN`**, and any frame that disagrees is rejected:
- blocks = ceil(`image_size` / 4096). The last block is short: 3,680 bytes for the 843,360-byte image.
- A block's fragment count is ceil(block length / `frag_data`). Fragment *i* carries min(`frag_data`, block length − *i* · `frag_data`) bytes, and `data_len` must equal that.
- `OTA_BEGIN` gets `ERR_SIZE` if `block_size` is not 4096, a block would need more than 128 fragments, `frag_data` exceeds the variant's maximum, or `image_size` exceeds the target slot.
- `OTA_END`'s `image_size` must equal `OTA_BEGIN`'s.
- `data_len` is explicit because Ethernet padding makes the frame length unreliable for a short fragment.

**Integrity:** the EMAC drops frames with a bad FCS, and `esp_ota_end` checks the whole image. No per-frame checksum is added.

## 5. Receiver model and flow control

**One task does everything.** It parses, assembles into **one** 4 KB DRAM block buffer, and writes inline. This is the OTA-only mode decided in N-2, where the update is the node's only task.

The node keeps a single counter, `next_block`, the next block to assemble. Frames are handled by how their block compares with it:
- **Earlier block:** reply `RECEIVED(block)`. Never store the data and never send `MISSING`. A lost acknowledgement therefore costs one reply, never a second write.
- **`next_block`:** store the fragment if it is new, ignore it if it is a duplicate.
- **Later block:** drop it.

Writes use `esp_ota_write_with_offset(handle, data, len, block * 4096)`. The range is already erased by the sized `esp_ota_begin`, and ESP-IDF allows offset writes then. So even a repeated block would land on its own address, not shift the image (`esp_ota_ops.c`).

**`ACK_REQ`.** The sender sets it on the last frame of every burst: the first send of a block, every resend set, and every timeout probe. The node answers **every** `ACK_REQ` frame with the status of that frame's block, even when the frame is a duplicate and even when the block is complete. This removes both livelocks: a lost `MISSING` and a resend lost again.

**Variant A: 64-byte frames (no parser size change).**
- 64 − 30 header bytes leaves 34 data bytes, so a full block is 121 fragments.
- 121 fragments cannot wait in 10 buffers during a write, so A is stop-and-wait: the node writes block *n*, **then** sends `RECEIVED n`, and only then does the sender start block *n*+1.

**Variant B: one-buffer frames of 486 bytes (parser cap 508 for `OTA_DATA`).**
- A full block is 9 fragments (8 × 456 + 448 data bytes), and nine fit in the 10 buffers.
- When block *n* is complete, the node sends `RECEIVED n`, **then** writes it.
- The sender, now allowed one block ahead, sends block *n*+1, which waits in the EMAC buffers during the write. The task picks it up as soon as the write returns.
- One buffer is left for other traffic, which runs at about 5 to 6 frames/s. Two such frames arriving during a write cost a fragment, and the bitmap recovers it. D2 asks whether to add margin.
- `parseFrame` needs a per-type maximum length: 508 for `OTA_DATA`, 64 for the rest.

**Sender pacing.** The receive measurements used evenly paced frames, and that is the only condition they cover. A line-rate burst of 64-byte frames (about 148,800 frames/s at 100 Mbit/s) would overrun the 10 buffers within 10 frames.
- The sender paces `OTA_DATA` at no more than 25,000 frames/s, a 40 µs gap, which is inside the measured loss-free range.
- Variant B's 9-frame burst fits the buffers even unpaced, but it is paced the same way for margin.

## 6. Timing, loss and the end of the update

**Begin.**
- `OTA_BEGIN` is idempotent per session. A repeat during the erase is dropped. A repeat after the erase gets `READY` again, with no second `esp_ota_begin`.
- The node resends `READY` every 250 ms until the first `OTA_DATA` arrives.
- The sender retries `OTA_BEGIN` every 1 s, and waits up to 2 s + 3.5 µs × `image_size` for `READY`. That is derived from the measured 2.8 s erase for 843,360 bytes, and gives about 8.9 s for a full 1,920 KB slot.

**Progress timer.** The node's 2 s abort counts from `READY`, and is reset only by progress: a newly completed block, or `OTA_END`. Frames that keep arriving without progress do not reset it.

**Sender timeouts.**
- 100 ms with no status after a burst, then a probe with `ACK_REQ`. 100 ms is assumed: it covers a 4 KB write (13.5 ms) and the longest single write measured after an up-front erase (59 ms). Check it against `write_ms` in `OTA_RESULT`.
- After 20 retries on one block, the sender aborts.

**End.**
- The node handles `OTA_END` only after the last block's write has returned and the bytes written equal `image_size`. If anything is short, it replies `OTA_STATUS` for `next_block` instead.
- It then runs `esp_ota_end` and `esp_ota_set_boot_partition`, and caches `OTA_RESULT`. It resends the cached result for any duplicate `OTA_END`, never calling `esp_ota_end` twice.
- It waits 300 ms before `esp_restart`, so the last frame leaves. The TCP path waits 700 ms.
- The sender waits at least 2 s for `OTA_RESULT`, since the check takes 0.6 s.
- **The update counts as done only when the node's first frame after the reboot shows the new slot** (`GANTRY_STATUS.active_slot`). A missing `OTA_RESULT` means "unknown", not "failed".

**Loss cost (derived, to be measured in OTA-only mode).**
- At 0.16 % loss, about 0.19 fragments are lost per variant-A block, about 40 per update. Each is resent after one `MISSING` round trip.
- Only a lost last fragment or a lost status costs a 100 ms timeout. That happens about 0.3 times per update in each direction (206 × 0.16 %), about 0.1 s in total at worst.

**Rollback.**
- Keep TCP OTA until a CellNet update has been proven end to end on each node. An image whose CellNet receiver is broken would otherwise be recoverable only over USB.
- Consider marking a new image valid only after a CellNet exchange with the supervisor, and turning on task-watchdog panic, so that an image that hangs early rolls back.

## 7. Sequence

```mermaid
sequenceDiagram
    participant S as Supervisor (Pi)
    participant N as Node (WT32)
    S->>N: CELL_COMMAND 7 REBOOT_TO_OTA (session)
    Note over N: refuse if busy or image pending; motors off; noinit flag; esp_restart
    N-->>S: HEARTBEAT OTA_MODE (every 250 ms)
    S->>N: OTA_BEGIN (session, size, block and fragment size)
    Note over N: esp_ota_begin(size) erases, about 2.8 s (heartbeat shows ERASING)
    N-->>S: OTA_STATUS READY (repeated every 250 ms until data)
    loop each 4 KB block
        S->>N: OTA_DATA fragments, last one with ACK_REQ
        Note over N: A: write block, then RECEIVED. B: RECEIVED, then write while block n+1 waits in the EMAC
        N-->>S: OTA_STATUS RECEIVED, or MISSING with bitmap
    end
    S->>N: OTA_END
    Note over N: after the last write: esp_ota_end, set boot slot
    N-->>S: OTA_RESULT (cached for duplicate OTA_END)
    Note over N: wait 300 ms, esp_restart
    N-->>S: first frame from the new image (active_slot)
```

## 8. Estimated whole update (derived, not measured)

Per-block cost:
- **Variant A:** about 4.8 ms to send 121 fragments at 25,000 frames/s, 13.5 ms to write, and an assumed 0.5 ms for the acknowledgement round trip. That is about 18.8 ms per block, so 206 blocks take about 3.9 s.
  - To check the 0.5 ms: on the supervisor, time from the last fragment sent to `RECEIVED` received, then subtract the node's `write_ms` divided by the block count.
- **Variant B:** 9 frames take about 0.4 ms at 25,000 frames/s and overlap the write, so a block costs about 13.5 to 14 ms. 206 blocks take 2.8 to 2.9 s.

| | Erase | Blocks | Check | On the node | Plus |
|---|---|---|---|---|---|
| TCP today (measured) | 2.8–2.9 s | 3.0–3.1 s | 0.6 s | **6.4–6.6 s** | none |
| CellNet A, 64-byte frames | 2.8 s | about 3.9 s | 0.6 s | **about 7.3 s** | reboot into OTA mode, loss (about 0.1 s) |
| CellNet B, 486-byte frames | 2.8 s | about 2.9 s | 0.6 s | **about 6.3 s** | reboot into OTA mode, loss |

## 9. Work, once decided

In this repository:
1. `include/cell_net_l2_protocol.h`: message types 0x05 to 0x09, their payloads, `CELL_COMMAND` id 7, and heartbeat bits 15 and 14.
2. `lib/CellNetL2`: build and parse the new types, with the derived-length checks of section 4 and, for variant B only, the per-type maximum length.
3. A pure-logic receiver state machine: session, `next_block`, fragment bitmap, `ACK_REQ` replies, progress timer, `OTA_END` rules. **Host-test it before wiring** (`AGENTS.md`), with the loss, duplicate and reorder sequences from the 2026-11-13 reviews as test cases.
4. `src/gantry_ota.cpp`: the CellNet receiver, reusing `gantryOtaStartStream` and `gantryOtaFinishStream`, with offset writes.
5. `CELL_COMMAND 7` handling: the busy and image-valid checks, `gantryOtaDisableMotors`, the noinit flag, then `esp_restart`.
6. `app_main`: the OTA-mode branch (flag check, link-up attach, heartbeat, deadline). It lives in the kept working-tree `main.cpp` (X-6).

Outside this repository:
- **Supervisor** ([ADSB_Sorter-supervisor](https://github.com/Mynderse-Research-Lab/ADSB_Sorter-supervisor)): a CellNet sender behind `/api/ota`, with a session counter, pacing and the retry rules. Keep TCP until CellNet OTA is proven (N-1 migrates one service at a time). Refresh its `vendor/cellnet` copy with `scripts/sync-cellnet.py` (X-3).
- **Conveyor:** one USB flash to install the receiver (C-2).

## 10. Decisions needed

| ID | Question | Options |
|----|----------|---------|
| D1 | Fragment size | **A, 64-byte frames:** about 7.3 s on the node, no parser size change, the simplest design, and within the 2026-11-05 rule. **B, 486-byte frames:** about 6.3 s, roughly equal to TCP, but needs per-type frame lengths and the one-block-ahead window |
| D2 | Variant B only: raise `CONFIG_ETH_DMA_RX_BUFFER_NUM` | Keep 10, with one spare buffer during a write. Or raise it to 20, using 5 KB more RAM in every mode, and set it in `idf/sdkconfig.defaults` |
| D3 | Measure before building? | From the supervisor: the time from `REBOOT_TO_OTA` to the node's first frame, which is the real cost of OTA-only mode. Frame loss in OTA-only mode at the paced rate |
