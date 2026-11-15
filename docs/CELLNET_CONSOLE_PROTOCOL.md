# CellNet Console Stream (draft)

**Status:** Draft, 2026-11-14, revision 2, for [N-1](OPEN_ITEMS.md#cell-transport). The gantry console is the second service to move from IP onto CellNet L2, after OTA ([CELLNET_OTA_PROTOCOL.md](CELLNET_OTA_PROTOCOL.md)).
- Nothing here is implemented yet.
- Revision 2 folds in an adversarial review of revision 1 against the code. Revision 1 assumed that only the transport changes. That is not true: the console's output routing and log sink must change too ([section 5](#5-the-gantry-side)).
- Decisions are needed before work starts ([section 10](#10-decisions-needed)).

**Scope:** the gantry console between the supervisor and the gantry. Today that is TCP port `2323`. The conveyor is out of scope (user, 2026-11-14).

Evidence labels follow [OPEN_ITEMS.md](OPEN_ITEMS.md#how-to-keep-this-list-live): **Measured**, **Derived** (arithmetic on measured numbers), **Assumed** (each says how to check it). File and line references are to the code on 2026-11-14.

## 1. The console today

### 1.1 Gantry: `src/gantry_net_console.cpp`

It serves one TCP client at a time: `listen(fd, 1)` with a serial accept loop. A second client waits in the backlog until the first one disconnects, so a live session is never taken over.
1. **Login.** A password login, which a recently seen IP can skip.
2. **Banner.** The log sink is attached first (`serveClient`, line 340), then the banner is sent (line 345). It ends in `"\r\n> "` and names `TCP 2323`.
3. **Line loop.**
   - Line assembly is written inline. `;`, CR and LF each end a line. Non-printable bytes are dropped. A line is truncated at 255 characters. An empty line produces no prompt.
   - `logout`, `exit` and `quit` are handled before the command parser: the console sends `Bye`, detaches the sink and closes (lines 361–368).
   - Every other line is echoed into the log as `[TCP RX] <line>` under the `NetConsole` tag. It is then run by `gantryConsoleProcessLine(cfg, line, replyToSocket, fd)` and ended by the prompt `\r\n> `.
4. **Sending.** `sendAll` (lines 83–124) behaves differently for the two kinds of caller:
   - **The console task** waits for `g_sendMutex` and socket space, but only up to `kConsoleSendWaitTicks` = 500 ms. A reply cut short there is counted as `waited_short` and shown as `PROMPT-SHORT` by `netcon`.
   - **Every other task** never waits: timeout 0, `MSG_DONTWAIT`. If the socket is busy, the line is dropped (`log_skipped`) or cut part-way (`log_short`).
5. **Diagnostics.** `netcon` reports these counters and the last 8 received lines, each with its receipt-to-prompt time.

### 1.2 Gantry: how output reaches the client (`src/gantry_test_console.cpp`)

**Command output does not travel through `reply_fn` while a sink is attached:**
- `consoleTeeVprintf` (lines 158–172) sends every ESP_LOG line at INFO or above to the single global sink `g_sessionFn` whenever one is set. That includes command output. It uses `g_replyFn` only when no sink is set.
- After the sink, it always writes the line to the boot-log ring and to UART0.
- Whether a line may wait is decided by task identity: `xTaskGetCurrentTaskHandle() == g_consoleTask`, the TCP task.

**Attach and detach** (lines 3488–3507):
- Both take `g_consoleCmdMutex` with `portMAX_DELAY`. `gantryConsoleProcessLine` holds that same mutex for a whole command. A command can run for minutes: `calibrate all` runs `runEipBringUpSequence` inline, with a timeout of `BRINGUP_SEQUENCE_TIMEOUT_MS` = 120 s.
- Detach clears the sink unconditionally. There is no owner check and no saved previous sink.

The UART console (`gantryTestConsoleTask`, line 3655) runs the same `gantryConsoleProcessLine`. Its output also goes to whichever session holds the sink.

### 1.3 Supervisor: `ADSB_Sorter-supervisor`, `src/gantry_status.cpp`

The session is opened on demand and then kept open until the gantry closes it or no prompt arrives. It is opened by any of three things:
- The reader loop, every 2 s while developer-page watchers exist.
- A `/api/console` command (`consoleExchange`).
- The status-poll fallback.

The code is written directly against the socket:
- `g_fd >= 0` means the session is alive.
- `readSome` uses `SO_RCVTIMEO` plus `recv`. `readerLoop` uses `poll`. `writeAll` uses `send(MSG_NOSIGNAL)`.
- The urgent `stop` path calls `writeAll` from another thread while `consoleExchange` holds `g_mu` (lines 529–543).
- `connectAuth` (lines 246–312) does one 200 ms read of the banner. If that read does not contain `authenticated`, it writes the password as a line.
- `consoleExchange` rejects `;`, non-printable characters and lines of 255 characters or more (lines 514–523), to match the gantry's line rules.
- `isPollReplyLine` hides lines tagged `GantryConsole:` or `NetConsole:` while a status poll is quiet (lines 202–206).
- `noteMotion` takes busy and motor state from log text such as `CTRL FLIP: ... busy -> 1` (lines 103–112).

Other users of the session:
- `pollGantryStatus` sends `status` about every 1 s when the L2 `GANTRY_STATUS` is older than 2 s (`supervisor_main.cpp:631–641`).
- `otaBlockReason` sends `status` before a Dual-OTA (lines 397–417).

Two PC tools also connect straight to `2323`: `tools/lan_debug_ui.py` and `tools/driver_config_ui.py`. Because of `listen(1)`, they reach it only while the supervisor is not holding it.

### 1.4 Measured baseline

**Measured 2026-11-14** through the supervisor's `/api/console` from the PC: each command three times, the same each time. The times include one PC → Pi HTTP round trip, assumed to be about 0.25 s.

| Command | Reply | Lines | Time |
|---|---|---|---|
| `stop` | 119 B | 3 | 0.26–0.29 s |
| `status` | 1,811 B | 26 | 0.40–0.41 s |
| `eiptiming` | 660 B | 10 | 0.29–0.31 s |
| `help` | 6,278 B | 66 | 0.76–0.79 s |
| `cfg export` | 7,256 B | 148 | 1.03–1.05 s |
| `bootlog` | 17,428 B | 160 | 1.74–1.75 s |

**Assumed, not established: what limits the output rate.**

The numbers do not show a single byte-rate limit:
- After subtracting the assumed 0.25 s, `bootlog` gives about 11.6 KB/s and `cfg export` about 9.3 KB/s.
- `help` gives about 12.3 KB/s and `eiptiming` about 16 KB/s. Both are above the UART's 11.52 KB/s line rate (115,200 baud).

UART0 is not obviously the limit either:
- `log_uart_async` defaults to 1 (`lib/AppConfig/src/AppConfig.cpp:523`).
- With it set, `main.cpp` installs a UART driver with a 16 KB TX ring, so most of a 17 KB reply would go into memory at once.

**How to settle it (D4):** read `netcon`'s per-command receipt-to-prompt time, which removes the HTTP hop. Run `bootlog` with `log_uart_async` at 0 and at 1.

**Sizing rule until then:** the stream must handle bursts of up to about 16 KB at memory speed, not only a steady 12 KB/s.

## 2. Approach

Carry the console's byte stream over CellNet as an ordered, reliable stream between the supervisor (node 0x04) and the gantry (0x03). The prompt contract (`"\r\n> "` ends every command) and the line rules (section 1.1) stay the same. Three things change on each side.

**Gantry:**
1. **Tasks.** A stream engine task and a CellNet command task, kept separate (section 5.1).
2. **Output routing and the sink.** Output is routed to the issuing session, and the log sink becomes owner-checked (section 5.3). This touches `gantry_test_console.cpp` and the TCP console as well.
3. **Line assembly.** One shared, host-tested line assembler, used by TCP, CellNet and UART.

**Supervisor:**
1. **A stream interface** in place of the socket (section 6).
2. **A separate CellNet open path** with no password.
3. **A stream engine thread,** so the receive window never depends on application reads.

There is no login on the CellNet console. This follows the 2026-11-05 decision to keep no security beyond the supervisor web application, which keeps its own login.

TCP `2323` stays during the migration (N-1 moves one service at a time). How long it stays is decision D3.

## 3. Messages

All new types are unicast between the supervisor and the gantry's **normal** MAC. Fields are little-endian and packed.

| Type | Name | Direction | Payload |
|---|---|---|---|
| 0x0A | `CON_OPEN` | S → G | `session u16` (≠ 0), `nonce u32`, `isn u16` (supervisor's initial sequence number), `flags u8` (bit 0 = live logs) |
| 0x0D | `CON_OPEN_ACK` | G → S | `session u16`, `nonce u32` (echoed), `boot_nonce u32` (random at each gantry boot), `isn u16` (gantry's), `status u8` (`OK`, `BUSY_COMMAND`) |
| 0x0B | `CON_DATA` | both | `session u16`, `seq u16`, `ack u16`, `window u16`, `len u16`, `data[len]` |
| 0x0C | `CON_CLOSE` | both | `session u16`, `reason u8`, `reserved u8` |

`CON_CLOSE` reasons: `NORMAL`, `LOGOUT`, `NO_SESSION`, `REPLACED`, `SILENCE`, `NO_PROGRESS`, `BAD_ACK`.

### 3.1 Opening a session

1. **The supervisor picks the session values.** It chooses a random `session` and a random `isn` each time it opens a session, and a random 32-bit `nonce`. Random values avoid a collision with a session the gantry still holds after the supervisor process or the Pi restarts.
2. **It resends `CON_OPEN`** every 200 ms until a matching `CON_OPEN_ACK` arrives, and sends no `CON_DATA` before that.
3. **The gantry answers.**
   - A `CON_OPEN` with a new (`session`, `nonce`) opens a new session and replaces any old one (section 3.2).
   - A `CON_OPEN` with the current (`session`, `nonce`) is a duplicate. The gantry sends the same `CON_OPEN_ACK` again and resets nothing.
4. **The gantry accepts `CON_OPEN` only from sender 0x04,** like `CELL_COMMAND`. This guards against stray frames; it is not security.
5. **The gantry boots with no session.**

### 3.2 Replacing a session

A replace is new behaviour: TCP never takes over a live session (section 1.1).

**When a new session replaces an old one, the gantry:**
1. Flushes the send ring.
2. Resets the line assembler.
3. Discards every received line that has not run yet, and logs how many it discarded.
4. Advances the **session epoch.**

**What happens to a command that is still running:**
- Writes the command task makes for an older epoch are discarded. Its leftover output and its final prompt therefore never reach the new session.
- `CON_OPEN_ACK` reports `BUSY_COMMAND`. The engine sends one line, `command '<x>' still running`, with no prompt.
- The command task sends the banner and its prompt when it next takes a line: at once if it is idle, otherwise when the old command ends.

**Test clients:** any client from 0x04 replaces the supervisor's session. During client tests the supervisor's CellNet console reader must be stopped (section 8).

### 3.3 Unknown sessions and reboots

- **The gantry answers `CON_DATA` or `CON_CLOSE` for an unknown session with `CON_CLOSE(NO_SESSION)`,** as OTA answers with `ERR_SESSION`. After a `cfg reboot` or an OTA, the supervisor learns at its first resend that the session is gone, rather than after 5 s of silence.
- **A changed `boot_nonce` also tells the supervisor the gantry has rebooted.**

### 3.4 Frame size

Frame size is decision D1. The header is 22 bytes for the cell header plus 10 bytes for `CON_DATA`.

| Frame | Data per frame |
|---|---|
| 64 bytes | 32 bytes |
| One-buffer frame of up to 508 bytes | up to 476 bytes |

**Both options need parser changes** (`CellNetL2Framing.cpp`):
- New `expectedFrameLength` entries.
- `ParsedFrame` members, with `data_len` checks and a data pointer like `OTA_DATA` (lines 101–149).
- A dispatch path in the node (section 6.2).
- The same changes in the supervisor's vendored copy (`vendor/cellnet`).

The large-frame option also widens the 508-byte cap, which is a ternary for `OTA_DATA` only (line 104), to cover `CON_DATA`.

## 4. Reliability and flow control

### 4.1 Sequence numbers

1. **Sequence numbers count bytes, as in TCP.** Each side starts at its random `isn`. Every comparison uses serial-number arithmetic (RFC 1982) on `u16`.
2. **Acks.** An ack is accepted only if `snd_una < ack <= snd_nxt`. Any other ack is ignored and counted. `BAD_ACK_LIMIT` consecutive bad acks close the session with `BAD_ACK`.
3. **Segments.** A segment is accepted if it overlaps `[rcv_nxt, rcv_nxt + rcv_wnd)`. Its already-received head is trimmed, so a resend may re-cut the data.
   - A segment wholly below `rcv_nxt` is acked again and dropped.
   - A segment that starts after `rcv_nxt` is dropped, since the sender resends in order.
4. **Window cap.** The advertised window is capped at 16,384 bytes, below half the sequence space. This keeps old and new data apart across the wrap. Derived: at about 12 KB/s the 16-bit space wraps every 5.5 s.

### 4.2 Sending and resending

- **Bytes in flight are capped at `MAX_IN_FLIGHT`,** proposed at 2 KB. That is about 8 times what 12 KB/s needs over a 20 ms ack interval. It limits a resend burst next to Class 1 to about 64 frames of 32 data bytes, or 5 frames of 476 bytes.
- **Resend:** after 100 ms with no ack, resend from `snd_una` (go-back-N, in bytes).
- **Zero window:** while the peer's window is 0 and data is pending, probe every 100 ms with a frame of length 0.

### 4.3 Acks

- **Delay:** an ack is sent no later than 20 ms after data arrives. It is delayed, never suppressed. It rides on outgoing data when there is any. Every frame carries `ack` and `window`.
- **Sent at once in three cases:**
  - When the gantry receives the end of a line, before the line runs.
  - When the supervisor receives a prompt.
  - When a receive window grows from below one segment to at least one segment (a window update).

### 4.4 Timers

| Timer | Rule | Purpose |
|---|---|---|
| Keepalive | Send an empty `CON_DATA` 1 s after the last frame this side sent, whatever is pending | The link stays visibly alive during long commands |
| Silence | Close (`SILENCE`) after 5 s with nothing received | Link loss |
| Progress | Close (`NO_PROGRESS`) when data is outstanding and `snd_una` has not advanced for 3 s | Stuck or desynced sessions, which keep exchanging keepalives for ever and so never trip the silence timer. The OTA design uses the same idea (its 2 s progress timer) |

### 4.5 What "acked" means, and session end

- **An ack means received, not executed.**
- **Unacked bytes are discarded when a session ends.** They are never resent in a new session.
- **When the supervisor loses a session during a command,** it reports "outcome unknown, check `status`" and does not retry on its own. A retried `cfg reboot` or `move` would otherwise run twice.
- **`logout`, `exit` and `quit`** make the gantry send `Bye`, wait for the send ring to drain (at most 1 s), then send `CON_CLOSE(LOGOUT)`. The supervisor reopens only on demand.

## 5. The gantry side

### 5.1 Tasks

The stream engine and the command executor must be separate tasks.

**Why:**
- A command can block its task for up to 120 s (section 1.2). In that time a combined task would send no acks and no keepalives, and the session would close mid-command.
- A reply larger than the ring would deadlock. The task waiting for ring space would be the same task that processes the acks that free it. `bootlog` (17 KB) is larger than the 8 KB ring.

**Engine task:**
- Owns the receive queue, session state, acks, resends, all timers and all transmits.
- Runs at a priority above the command task.
- Is the only task that frees ring space and the only task that calls `esp_eth_transmit` for the console.

**Receive path:** the EMAC receive hook only copies console frames into the engine's queue. It never runs console code.

**Command task:**
- Takes complete lines from the receive ring.
- Runs `gantryConsoleProcessLine`.
- Writes output and prompts into the send ring.

**Attach and detach of the log sink** never happen in the engine task. Both currently block behind `g_consoleCmdMutex` (section 1.2). Section 5.3 removes that lock from them.

### 5.2 Send ring (8 KB)

1. **Atomic writes.** A write reserves its whole length or writes nothing, so a line is never torn.
2. **Command output and prompts wait for space,** with a bound.
   - They wait while the session is alive and making progress.
   - The wait returns `false` at session close, at the progress timeout or at an epoch change. The rest of the reply is dropped and counted as `reply_short`.
   - The writer lock is held across the wait for the prompt, as TCP holds `g_sendMutex`. A log line therefore cannot land between `\r\n` and `> `. A split prompt would hide the prompt from the supervisor's `atPrompt` and leave the console reading busy for 180 s.
3. **Live log lines never wait.** If they do not fit whole, they are dropped and counted. A complete `N log lines dropped` line is inserted before the next accepted line, never between output and its prompt. This is decision D2.
4. **Never a reason to block.** The ring is never the reason a non-console task blocks. The UART leg still can block one: `consoleTeeVprintf` always writes to UART after the sink, and the 16 KB UART ring blocks when it is full (G-B12). The CellNet sink cannot change that.

### 5.3 Output routing and the log sink

This part changes code shared with TCP and UART.

1. **Route command output to the issuing session.**
   - `gantryConsoleProcessLine` records the running command's reply context: session, epoch, and may-wait = true.
   - The tee sends lines from the task running the command to that context.
   - All other lines are live logs and go to the live-log sinks without waiting.
   - This replaces both the single global sink for replies and the `g_consoleTask` identity test.
2. **Sinks become a small list,** one slot for TCP and one for CellNet.
   - Each attach returns an owner token. Detach clears a slot only if the token still owns it.
   - The TCP console's unconditional detach (lines 364 and 386) therefore no longer removes the CellNet sink.
   - Attach and detach use a short critical section, not `g_consoleCmdMutex`.
   - Sink contexts are static.
3. **Sinks only copy.** A sink only copies into its ring, under a short critical section. It never logs, never transmits, and never calls anything that can log.
   - The sink is called from any task that logs, including Class 1, EipLog and the EMAC receive task.
   - Logging from inside the sink would feed back into itself.
   - Stream diagnostics are counters, not log lines.
4. **Echo.** The CellNet echo keeps the `NetConsole` tag, as `[CN RX] <line>`, so the supervisor's `isPollReplyLine` still hides it.
5. **Banner.** The banner names the transport (`CellNet L2`), contains no login text, and is sent by the command task (section 3.2).

### 5.4 Line assembly

- **One shared function,** host-tested, with the rules of section 1.1. It replaces the three inline copies (`serveClient`, `recvLine`, the UART task).
- **Reset** on `CON_OPEN` and at session close (section 3.2).
- **A queued `stop` does not pre-empt a running command,** as over TCP today. The immediate path is `CELL_COMMAND` STOP (`kCmdStop` = 2, handled in `src/pick_scheduler.cpp:1927`).

### 5.5 Receive window

The gantry's receive ring holds at least the line being assembled plus one more 255-byte line: 512 bytes. A supervisor `stop` written during a long command then fits, as it fits in the TCP buffer today.

## 6. The supervisor side

### 6.1 Stream interface

A small stream interface replaces the raw socket: `open`, readable-with-timeout, `read`, thread-safe `write`, `close`, `alive`.

**Functions that change:** `connectAuth`, `readSome`, `writeAll`, `closeSock`, `readerLoop`, and the host/port reconnect checks in `consoleExchange` and `pollGantryStatus`.

**The CellNet open path** is separate from `connectAuth`:
1. `CON_OPEN`.
2. Wait for `CON_OPEN_ACK`.
3. Read until `atPrompt`, with a deadline of about 1 s.
4. Never send a password.

**A stream engine thread** owns acks, resends and timers. It has a large receive buffer, so its window never depends on how fast the reader or `consoleExchange` reads.

**The transport is one process-wide setting,** for example an environment variable like `GANTRY_HOST`. A per-request choice like `?transport=` for OTA does not fit, because the reader, the status poll and `/api/console` share one session.

### 6.2 Frames and node statistics

**Keep console frames out of the node statistics.** Today unknown types fail `parseFrame` and increase `rx_invalid_frames`. Each node also keeps one `u8` sequence counter per sender (`CellNetL2.cpp:208–226`).

**Proposal: one sequence counter per node.** Console frames use the node's own CellNet sequence counter, and are intercepted after the sequence check and before the type dispatch. `rx_sequence_drops` then stays valid loss evidence for both.

**Separately, the supervisor's node must skip its own outgoing frames.** It already sees them today (N-2 note: no `PACKET_OUTGOING` skip).

### 6.3 MAC and OTA

- **The console learns the gantry MAC only from `GANTRY_STATUS`.** It never sends to an address with the `0x02` bit set, which is the private OTA MAC.
- **Console reconnects pause while the supervisor's own OTA runs.** OTA's guarantee of no full-buffer drops depends on only the OTA sender reaching that address and on the W = 18 window.

### 6.4 Busy state

Busy and motor state should come from `GANTRY_STATUS` `motion_state` (`CellNetL2.h:83`), not from log text.
- Under D2, live log lines are dropped during large replies. The `CTRL FLIP` lines `noteMotion` relies on can be among them.
- TCP already loses such lines through `log_skipped`.

## 7. Estimates

- **Command round trip, excluding output: about 2 ms on the cell switch. Assumed.** Check: the round-trip time of an empty `CON_DATA` from the test client.
- **Output rate:** at least the TCP console's, with bursts up to about 16 KB (section 1.4).

  | D1 option | Frames per second at about 12 KB/s |
  |---|---|
  | 476-byte frames | about 26 |
  | 32-byte frames | about 375, plus acks every 20 ms |

  Both are below the 2,000 frames/s above which the N-2 sweeps saw loss, and far below the 6,000–8,000 frames/s of 1,014-byte frames at which CellNet traffic stalled Class 1 (G-B10).
- **Loss:** the N-2 sweeps were receive-rate floods.
  - They saw 0–0.16 % loss for 60-byte frames, up to 30,000 frames/s.
  - They saw 0–0.31 % for 1,014-byte frames.
  - They saw "some frames lost at every rate above 2,000 frames/s".
  - Loss at console rates was not seen.
  - The sweeps cover neither gantry transmit bursts nor loss at the Pi's raw socket. Both are measured in section 8.
- **Resend cost:** 100 ms for each loss, plus at most `MAX_IN_FLIGHT` resent.

## 8. Testing

### 8.1 Host tests, before any wiring

**Stream engine:**
- An ISN of 65,500 with a segment that crosses 0.
- An ack above `snd_nxt`, and an ack from an old session.
- A re-cut resend with partial overlap.
- A duplicate and a lost `CON_OPEN`; a lost `CON_OPEN_ACK`.
- Zero window with a lost window update.
- The progress and silence timers.
- `NO_SESSION` after a reboot.
- A command that blocks for 30 s with the session staying open.

**Send ring:**
- Atomic writes.
- A prompt wait interleaved with log writes; the prompt arrives whole.
- A reply longer than the ring; it completes.
- A wait that ends at close.
- The drop-marker placement.

**Line assembler:** the section 1.1 rules, and reset on a new session.

**Output routing:**
- A CellNet command while a TCP client holds a sink: the output goes only to CellNet.
- A TCP detach leaves the CellNet sink in place.

### 8.2 Diagnostics

- Extend `netcon` (or add a sibling command) with CellNet counters: resends, duplicates, bad acks, window stalls, log lines dropped, `reply_short`.
- Record each command's receive-to-prompt time.

### 8.3 Linux test client on the Pi

Like `tools/cellnet_ota_send`. The supervisor's CellNet console reader must be stopped during these tests (section 3.2). Checks:
1. **The six commands of section 1.4.** Compare against TCP after normalising timestamps and keeping only `GantryConsole` and `NetConsole` lines. Byte-for-byte equality is not possible: timestamps and live values differ, and live logs interleave.
2. **A forced loss test,** dropping every n-th frame.
3. **`bootlog` throughput.**
4. **Class 1 `eiptiming` and `gaptrace`** during a sustained `bootlog` loop, with and without forced loss. This measures the gantry's transmit bursts.

### 8.4 Then the supervisor

The supervisor's reader moves behind the transport setting.

## 9. Implementation order

1. **Line assembler, sink list and output routing,** shared with TCP. Host tests, then a TCP regression check on the gantry: the six commands and `netcon`.
2. **Stream engine and send ring** as pure logic, with host tests.
3. **Framing changes** in the firmware and the vendored supervisor copy.
4. **Gantry tasks and wiring;** the `netcon` counters.
5. **The Linux test client** (section 8.3).
6. **The supervisor stream interface** behind the transport setting.

## 10. Decisions needed

| ID | Question | Options |
|---|---|---|
| D1 | Frame size | **One-buffer frames, up to 508 bytes (476 data):** about 26 frames/s at full output. Needs the per-type length cap widened as well as the common parser changes. **64-byte frames (32 data):** about 375 frames/s plus acks. Needs only the common parser changes (section 3.4) |
| D2 | Live log lines when the send ring is full | **Drop whole lines and insert a "N dropped" line** (the G-B13 rule: logs never block). TCP today also drops them, silently and sometimes cut mid-line (section 1.1). Or let live logs wait, which can stall whatever task logged. In both cases the UART leg can still block a logging task (section 5.2) |
| D3 | How long TCP `2323` stays | Until the supervisor's CellNet console is the default; then remove it and move `lan_debug_ui.py` and `driver_config_ui.py` onto the supervisor's HTTP API. Or keep it for PC tools indefinitely, against the N-1 goal. While both exist, a PC tool on TCP can connect whenever the supervisor uses CellNet. With the section 5.3 sink list it gets its own sink and no longer takes the CellNet one |
| D4 | Measure first? | Use `netcon` to time `bootlog` with `log_uart_async` at 0 and at 1, before sizing the ring and `MAX_IN_FLIGHT` (section 1.4) |
| D5 | The supervisor's status fallback | `pollGantryStatus` is the IP fallback for when `GANTRY_STATUS` is stale. If the console runs over CellNet, that fallback uses the link that just failed. Keep the fallback on TCP (D3 then keeps `2323` for it), or drop it and show "gantry status unknown" when CellNet is down. Dual-OTA gating (`otaBlockReason`) is then also blocked while CellNet is down |
| D6 | Prompt counter | Optional. Show a per-line counter in the prompt, for example `\r\n[17]> `, so the supervisor knows which command a prompt ends. This changes the prompt contract ([LOW_LEVEL §9](LOW_LEVEL_GANTRY_CONTROL.md#9-console-contract)) for TCP too. Or keep `\r\n> ` and rely on the session rules of section 4.5 |
