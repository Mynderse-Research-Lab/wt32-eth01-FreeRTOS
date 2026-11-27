# CellNet OTA Protocol

Firmware updates for the WT32 nodes over CellNet L2, sent by the supervisor ([N-2](OPEN_ITEMS.md#cell-transport)). It is the default OTA transport; TCP port 8032 stays as the fallback (`POST /api/ota?transport=tcp`).

| Part | Where |
|------|-------|
| Receiver state machine (pure logic, host-tested) | `lib/CellNetL2/src/CellNetOtaReceiver.cpp`, `test/host/test_cellnet_ota_receiver.cpp` |
| Gantry wiring: OTA-only mode, flash writes | `lib/GantryOta/src/gantry_cellnet_ota.cpp` |
| Sender (Linux) | `lib/CellNetL2/src/CellNetOtaSender.cpp`, `CellNetOtaSenderPhases.cpp`, `CellNetOtaSenderReport.cpp`, `CellNetOtaSenderInternal.h`; CLI `tools/cellnet_ota_send.cpp`; the supervisor's `POST /api/ota` |
| Message types and payloads | `include/cell_net_l2_protocol.h` |

## 1. Design basis

- **Flash writes stall reception.** A 4 KB flash write turns the cache off on both cores for about 13.5 ms, so no task can take frames out of the EMAC receive buffers during it. The protocol therefore bounds how many frames can be waiting (section 5.1).
- **OTA-only mode.** The node reboots into a mode where the update is its only work: no EtherNet/IP scanner and no pick scheduler, so the motors stay off (section 3).
- **Erase up front.** `esp_ota_begin` with the image size erases the whole range in 64 KB blocks before any data arrives, so each block is a plain offset write.
- **Fixed sizes.** 4 KB blocks of 486-byte frames (9 fragments a block), one frame per 512-byte receive buffer; 20 receive buffers (`CONFIG_ETH_DMA_RX_BUFFER_NUM=20` in `idf/sdkconfig.defaults`).

## 2. Performance

Measured on the gantry with the 889 KB image, through the supervisor:

| Phase | Time |
|-------|------|
| Command to the first OTA-mode heartbeat (reboot) | about 2.6 s |
| Erase | about 3.0 s |
| Stream (1,956 data frames) | about 2.6 s |
| Image check and boot-slot write | about 0.55 s |
| **Command to `OTA_RESULT`** | **8.3–8.8 s** |

Every run so far had no resends and **EMAC missed 0**.

## 3. Entering and leaving OTA-only mode

**Entry.** The supervisor sends `CELL_COMMAND 7 = REBOOT_TO_OTA` with `param_u16` = session. The gantry takes `CELL_COMMAND` only from node 0x04 (`pick_scheduler.cpp`, `onCellCommand`). The node then:
1. Refuses if the gantry is busy (move, home or pick), or if `esp_ota_get_state_partition` reports the running image in any state other than VALID or UNDEFINED (a pending image would be rolled back by the reset). A refusal is only logged; the sender sees no reply and times out waiting for the OTA-mode heartbeat.
2. Disables the motors under control (`gantryOtaDisableMotors`), and restarts only once they are confirmed off.
3. Stores `{u32 magic; u16 session; u16 ~session}` in `__NOINIT_ATTR` DRAM, which survives a software restart (ESP-IDF `memory-types.rst`, "noinit" DRAM).
4. Calls `esp_restart`.

**At boot**, before anything else, the node reads the flag and clears it. It enters OTA mode only if the magic and complement match and `esp_reset_reason() == ESP_RST_SW`. Content is random at power-on, so the complement guards against a stray match; clearing at once means a crash in OTA mode comes back up in normal mode.

**In OTA mode:**
- The node starts Ethernet (`EthernetLink`, with the static IP as in normal mode), CellNet and the receiver only. The EtherNet/IP scanner never starts, so the motors stay off.
- It sends a `HEARTBEAT` every 250 ms with `status_flags` bit 15 (`OTA_MODE`) set, and bit 14 (`ERASING`) set while erasing.
- **An absolute deadline starts at boot, before Ethernet init:** 30 s to the first valid `OTA_BEGIN`, then 2 s with no progress (section 6). Every error ends the same way: `esp_ota_abort`, an error status, and a reboot into the normal image.

**Accepted frames.** Only `OTA_BEGIN`, `OTA_DATA` and `OTA_END` from sender 0x04 carrying the stored session; every other type, `CELL_COMMAND` included, is ignored. A frame with another session gets `ERR_SESSION`, which carries the node's session; the sender treats it as fatal, and drops any other reply whose session does not match.
- The supervisor keeps the session as a persisted counter (`/var/lib/adsb-cell/ota_session`), not a random number, so a stale `OTA_BEGIN` from an earlier attempt cannot capture the node.
- A new attempt cannot interrupt OTA mode: it waits for the deadline reboot.

## 4. Messages

All frames carry CellNet's EtherType 0x88B5, version 1 and the 8-byte cell header (`include/cell_net_l2_protocol.h`). In `OTA_DATA`, the cell header's `sequence` byte numbers every frame the sender sends, resends and probes included; the window in section 5.1 is counted with it. OTA frames are unicast between the supervisor and the node's OTA address (section 5.1). The node learns the supervisor's MAC from `OTA_BEGIN`, so an `ERR_TIMEOUT` sent before any `OTA_BEGIN` goes out as broadcast.

| Type | Name | Direction | Payload (little-endian, packed) | Frame |
|------|------|-----------|----------------------------------|-------|
| 0x05 | `OTA_BEGIN` | S → N | `session u16`, `block_size u16` (4096), `frag_data u16`, `reserved u16`, `image_size u32` | 34 B, padded to 60 |
| 0x06 | `OTA_DATA` | S → N | `session u16`, `block u16`, `frag u8`, `flags u8` (bit 0 `ACK_REQ`), `data_len u16`, `data[data_len]` | 30 B + data |
| 0x07 | `OTA_STATUS` | N → S | `session u16`, `block u16`, `status u8`, `processed_seq u8` (the last `OTA_DATA` sequence the node has taken from its receive buffers), `missing[16]` (bit *i* set = fragment *i* missing) | 44 B, padded to 60 |
| 0x08 | `OTA_END` | S → N | `session u16`, `reserved u16`, `image_size u32` | 30 B, padded to 60 |
| 0x09 | `OTA_RESULT` | N → S | `session u16`, `status u8`, `reserved u8`, `erase_ms u16`, `stream_ms u16`, `write_ms u16`, `check_ms u16`, `emac_missed u16`, `emac_overflow u16` | 38 B, padded to 60 |

**`OTA_STATUS` codes** (`CellNetOtaReceiver.h`): 1 `READY` (erase done), 2 `RECEIVED`, 3 `MISSING` (bitmap valid), 4 `DONE` (image checked, `OTA_RESULT` follows), 5 `ERR_SIZE`, 6 `ERR_SESSION`, 7 `ERR_BUSY` (defined, never sent), 8 `ERR_FLASH`, 9 `ERR_VERIFY`, 10 `ERR_TIMEOUT`.

**Everything else is derived from `OTA_BEGIN`**, and any frame that disagrees is rejected:
- blocks = ceil(`image_size` / 4096); the last block is short.
- A block's fragment count is ceil(block length / `frag_data`). Fragment *i* carries min(`frag_data`, block length − *i* · `frag_data`) bytes, and `data_len` must equal that.
- `OTA_BEGIN` gets `ERR_SIZE` if `block_size` is not 4096, a block would need more than 128 fragments, or `frag_data` exceeds 478 (the 508-byte cap minus 30 header bytes; the sender uses 456). An `image_size` larger than the slot fails the erase and gets `ERR_FLASH`.
- `OTA_END`'s `image_size` must equal `OTA_BEGIN`'s.
- `data_len` is explicit because Ethernet padding makes the frame length unreliable for a short fragment.

**Frame length.** `parseFrame` allows up to 508 bytes for `OTA_DATA` and 64 for every other type, counted after ESP-IDF strips the FCS.

**Integrity:** the EMAC drops frames with a bad FCS, and `esp_ota_end` checks the whole image. No per-frame checksum is added.

## 5. Receiver model and flow control

**Tasks.** The `emac_rx` callback only copies each frame into a 24-deep queue. `otaTask` (priority 5, core 0) takes frames off it, parses, assembles into **one** 4 KB DRAM block buffer, and writes inline. `heartbeatTask` (priority 3) sends the heartbeats. Queue overflows are counted (`s_rxq_drops`) and logged, not reported in `OTA_RESULT`; the window of section 5.1 keeps the queue from filling.

The node keeps a single counter, `next_block`, the next block to assemble. Frames are handled by how their block compares with it:
- **Earlier block:** reply `RECEIVED(block)`. Never store the data and never send `MISSING`. A lost acknowledgement therefore costs one reply, never a second write.
- **`next_block`:** store the fragment if it is new, ignore it if it is a duplicate.
- **Later block:** drop it.

Writes use `esp_ota_write_with_offset(handle, data, len, block * 4096)`. The range is already erased by the sized `esp_ota_begin`, so even a repeated block lands on its own address and cannot shift the image.

**`ACK_REQ`.** The sender sets it on the last frame of every burst: the first send of a block, every resend set, and every timeout probe. The node answers **every** `ACK_REQ` frame: with the status of that frame's block for the current or an earlier block, duplicates and complete blocks included, and with `MISSING` for `next_block` for a later or out-of-range block or a wrong `data_len`. A lost `MISSING`, or a resend lost again, therefore cannot stall the transfer.

**One block ahead.** A full block is 9 fragments (8 × 456 + 448 data bytes). When block *n* is complete, the node sends `RECEIVED n`, **then** writes it. The sender may then send block *n*+1, which waits in the receive buffers during the write; the task picks it up as soon as the write returns.

### 5.1 Zero drops from full receive buffers

**Requirement:** no frame may ever be dropped because all receive buffers are full. Two rules bound every frame that can reach the buffers.

**1. Only the OTA sender can reach the buffers.**
- On entering OTA mode, the node switches the EMAC to a private OTA address: its own MAC with the locally administered bit set (first octet `| 0x02`), applied with `esp_eth_ioctl(ETH_CMD_S_MAC_ADDR)`.
- It turns broadcast reception off (`emac_ll_broadcast_frame_enable(false)`), and receive-all-multicast and promiscuous mode off (`ETH_CMD_S_ALL_MULTICAST`, `ETH_CMD_S_PROMISCUOUS`). With `receive_all` cleared, as ESP-IDF sets it, the MAC passes only frames that match the address filter.
- Nothing else in the cell knows the OTA address: with broadcast reception off, the node never sees an ARP request, so no host learns it. Broadcasts, other nodes' CellNet traffic and TCP retries are dropped by the MAC before they can take a buffer.
- The node's OTA-mode heartbeats are sent from the OTA address. That is how the sender learns where to send.

**2. The sender never has more frames unprocessed than the buffers hold.**
- Each `OTA_STATUS` carries `processed_seq`, the last `OTA_DATA` sequence `otaTask` has taken off its queue. A frame not yet processed is either in a receive buffer or in the queue, so the count below bounds the buffers too.
- The sender keeps (last sequence sent − `processed_seq`), modulo 256, at or below **W = 18**. Every `OTA_DATA` counts: first sends, resends and `ACK_REQ` probes.
- With 20 buffers, at most 18 frames can be waiting at any moment, whatever the node is doing, with 2 buffers spare. A block plus a full resend of the next block still fits inside W, so the window never slows the one-block-ahead pipeline.
- When the window is full, the sender waits for a status and sends **no** new frame. A probe is sent only while the window has room.

**Why this is a guarantee.** Frames reach the buffers only from the sender (rule 1), and the sender cannot have more than 18 frames that the node has not taken out (rule 2). 18 is less than 20, for any stall length. Two other losses remain possible, and the resend rules recover both: CRC errors on the wire, and an EMAC receive-FIFO overflow (a separate counter).

**Proof in every run.** `OTA_RESULT` reports the EMAC missed-frame count (buffers full) and the FIFO-overflow count, each read and cleared at `OTA_BEGIN`. **The missed count must be 0 in every update.**

**Sender pacing.** With both rules in place, the sender needs no pacing to protect the buffers; it sends at line rate.

## 6. Timing, loss and the end of the update

**Begin.**
- `OTA_BEGIN` is idempotent per session. A repeat that arrives during the erase waits in the queue; after the erase, it and any later repeat get `READY` again (`ERR_SIZE` if its parameters differ), with no second `esp_ota_begin`.
- The node resends `READY` every 250 ms until the first `OTA_DATA` arrives.
- The sender retries `OTA_BEGIN` every 1 s, and waits up to 2 s + 3.5 µs × `image_size` for `READY`: about 8.9 s for a full 1,920 KB slot.

**Progress timer.** The node's 2 s abort counts from `READY`, and is reset only by progress: a newly completed block, or `OTA_END`. Frames that keep arriving without progress do not reset it.

**Sender timeouts.**
- 100 ms with no status after a burst, then a probe with `ACK_REQ`, but only while the window has room (section 5.1). 100 ms covers a 4 KB write and the longest single write seen after an up-front erase (59 ms).
- 20 consecutive timeouts on one block abort the update; any useful status (a `processed_seq` advance, `RECEIVED`, or a current `MISSING`) resets the count. `MISSING` resend rounds are limited to 64 per block.
- Each lost fragment is resent after one `MISSING` round trip; only a lost last fragment or a lost status costs a 100 ms timeout.

**End.**
- The node handles `OTA_END` only after the last block's write has returned and the bytes written equal `image_size`. If anything is short, it replies `OTA_STATUS` for `next_block` instead.
- It then runs `esp_ota_end` and `esp_ota_set_boot_partition`, and caches `OTA_RESULT`. It resends the cached result for any duplicate `OTA_END`, never calling `esp_ota_end` twice.
- It waits 300 ms before `esp_restart`, so the last frame leaves.
- The sender resends `OTA_END` every 2 s and waits up to 10 s per round, for up to 5 rounds; a `MISSING` reply sends the block again. After `OTA_STATUS DONE` it waits 1 s more for `OTA_RESULT`. With no `OTA_RESULT` the outcome is "unknown" (exit 3), not "failed".
- **The update counts as done when the node's first `GANTRY_STATUS` after the reboot, within 30 s, shows the new slot** (`active_slot`). If the sender never saw the pre-update slot, that first status counts as success without the slot check.

**Rollback.** Rollback is on (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`). An image whose CellNet receiver is broken can still be replaced over the TCP route, or over USB.

## 7. Sequence

```mermaid
sequenceDiagram
    participant S as Supervisor (Pi)
    participant N as Node (WT32)
    S->>N: CELL_COMMAND 7 REBOOT_TO_OTA (session)
    Note over N: refuse if busy or image pending; motors off; noinit flag; esp_restart
    N-->>S: HEARTBEAT OTA_MODE (every 250 ms)
    S->>N: OTA_BEGIN (session, size, block and fragment size)
    Note over N: esp_ota_begin(size) erases (heartbeat shows ERASING)
    N-->>S: OTA_STATUS READY (repeated every 250 ms until data)
    loop each 4 KB block
        S->>N: OTA_DATA fragments, last one with ACK_REQ
        Note over N: RECEIVED, then write while block n+1 waits in the EMAC
        N-->>S: OTA_STATUS RECEIVED, or MISSING with bitmap
    end
    S->>N: OTA_END
    Note over N: after the last write: esp_ota_end, set boot slot
    N-->>S: OTA_RESULT (cached for duplicate OTA_END)
    Note over N: wait 300 ms, esp_restart
    N-->>S: first frame from the new image (active_slot)
```
