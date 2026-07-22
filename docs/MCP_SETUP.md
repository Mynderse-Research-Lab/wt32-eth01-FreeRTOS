# MCP Server Setup for WT32-ETH01 Firmware Development

This guide walks through installing and configuring six MCP (Model Context Protocol) servers that bridge Cursor IDE with hardware development tools.

## Prerequisites

Install these on your Windows development machine before proceeding:

### 1. Rust Toolchain

Serial and debugger MCPs are Rust binaries.

```powershell
winget install Rustlang.Rustup
# OR download from https://rustup.rs
```

Verify:

```powershell
rustc --version
cargo --version
```

**Important — MSVC linker:** Rust on Windows defaults to the MSVC toolchain, which requires the Visual Studio Build Tools to provide `link.exe`. If `cargo install` fails with `linker 'link.exe' not found`, either:

- Install Visual Studio Build Tools (several GB):
  ```powershell
  winget install Microsoft.VisualStudio.2022.BuildTools --override "--add Microsoft.VisualStudio.Workload.VCTools --includeRecommended --passive"
  ```

- Or switch Rust to the GNU toolchain (no Visual Studio required):
  ```powershell
  rustup toolchain install stable-x86_64-pc-windows-gnu
  rustup default stable-x86_64-pc-windows-gnu
  ```

### 2. uv (Python Package Runner)

MQTT and memory-bank MCPs use uvx for zero-install execution.

```powershell
winget install --id=astral-sh.uv
```

Verify:

```powershell
uv --version
```

### 3. Node.js LTS

GitHub and datasheet MCPs use npx.

```powershell
winget install OpenJS.NodeJS.LTS
```

Verify:

```powershell
node --version
npm --version
```

### 4. CP210x USB-UART Driver

WT32-ETH01 uses a CP2104 USB-to-serial chip. If Windows does not auto-install the driver:

- Download from [Silicon Labs CP210x Universal Driver](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
- Verify in Device Manager under **Ports (COM & LPT)** — note the COM port number (e.g., `COM3`)

---

## MCP Server Installation

### 1. serial-mcp-server

Read and write the ESP32 serial monitor directly from Cursor.

```powershell
cargo install serial-mcp-server
```

Verify:

```powershell
serial-mcp-server --help
```

No credentials required. The server auto-detects available COM ports.

**Usage in Cursor:** Ask Cursor to list COM ports, open a specific port at a baud rate, send commands, or stream output. Example: "Connect to COM3 at 115200 baud and show me the ESP32 boot log."

---

### 2. mcp2mqtt

Subscribe to and publish MQTT topics (e.g., `gantry/obs`, `conveyor/state`) without leaving Cursor.

No manual install needed — `uvx` pulls and runs it automatically.

**Environment Variables (required):**

Set these before launching Cursor or in your system environment:

```powershell
$env:MQTT_BROKER_HOST = "192.168.1.100"   # your broker IP
$env:MQTT_BROKER_PORT = "1883"             # default MQTT port
```

To make persistent, add them via **System Properties > Environment Variables**.

**Usage in Cursor:** Ask Cursor to subscribe to `gantry/#` or publish a test message. Example: "Subscribe to gantry/obs and show me the last 5 messages."

---

### 3. mcp-memory-bank

Persists project context across Cursor sessions — pin assignments, milestone status, architecture decisions.

No manual install needed — `uvx` pulls and runs it from GitHub automatically.

**Usage in Cursor:** The memory bank builds automatically as you work. At the start of each session, Cursor reads the stored context. Example: "What pin is the X-axis step signal on?" or "What was the last milestone we completed?"

---

### 4. sheetsdata-mcp

Instant datasheet lookups for component specs, pinouts, and absolute maximum ratings.

No manual install needed — `npx` pulls and runs it automatically.

**Usage in Cursor:** Ask Cursor to look up component specs. Example: "What is the max VCC for the TXS0108E level shifter?" or "Show me the pinout of the WT32-ETH01 module."

---

### 5. embedded-debugger-mcp

Flash firmware, set breakpoints, and read CPU registers via a debug probe.

```powershell
cargo install embedded-debugger-mcp
```

Verify:

```powershell
embedded-debugger-mcp --help
```

**Caveat for ESP32:**

This MCP uses probe-rs under the hood, which primarily targets ARM Cortex-M chips. ESP32 (Xtensa LX6/LX7) support requires:

| Approach | Setup |
|----------|-------|
| ESP32-S2/S3 with built-in USB-JTAG | May work with probe-rs experimental Xtensa support |
| ESP32 (original) via external JTAG probe | Requires an FT2232H-based probe + OpenOCD bridge |
| Alternative | Use `idf.py monitor` and `idf.py flash` directly in a Cursor terminal instead |

**Usage in Cursor (when compatible probe is available):** "Flash the current firmware to the ESP32 and halt at main." or "Read the value of register R0 after the crash."

---

### 6. github-mcp-server

Manage branches, PRs, and issues for your repository without leaving Cursor.

No manual install needed — `npx` pulls and runs it automatically.

**Environment Variable (required):**

Create a GitHub Personal Access Token (classic) at [github.com/settings/tokens](https://github.com/settings/tokens) with scopes: `repo`, `read:org`, `workflow`.

```powershell
$env:GITHUB_PERSONAL_ACCESS_TOKEN = "ghp_your_token_here"
```

To make persistent, add via **System Properties > Environment Variables**.

**Usage in Cursor:** Ask Cursor to create a PR, list open issues, or review a diff. Example: "Create a PR for the current branch with a summary of changes."

---

## Configuration File

The MCP server definitions live at `.cursor/mcp.json` in the project root. Each server can be individually disabled by setting `"disabled": true` if you do not need it for a session.

## Environment Variables Reference

Copy `.env.example` to `.env` and fill in your values, or set them as system environment variables.

| Variable | Server | Required | Example |
|----------|--------|----------|---------|
| `GITHUB_PERSONAL_ACCESS_TOKEN` | github-mcp-server | Yes | `ghp_abc123...` |
| `MQTT_BROKER_HOST` | mcp2mqtt | Yes | `192.168.1.100` |
| `MQTT_BROKER_PORT` | mcp2mqtt | Yes | `1883` |

## Verification Checklist

After completing all installations, restart Cursor and verify each MCP:

- [ ] **serial:** "List available COM ports"
- [ ] **mcp2mqtt:** "Subscribe to gantry/# and show one message"
- [ ] **mcp-memory-bank:** "What is stored in the project memory bank?"
- [ ] **sheetsdata:** "Look up the WT32-ETH01 pinout"
- [ ] **embedded-debugger:** "List connected debug probes" (requires probe hardware)
- [ ] **github:** "List open issues in this repo"

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `cargo: command not found` | Rust toolchain not installed or not on PATH. Restart terminal after `rustup` install. |
| `uvx: command not found` | uv not on PATH. Restart terminal or add `%USERPROFILE%\.local\bin` to PATH. |
| `npx: command not found` | Node.js not on PATH. Restart terminal. |
| Serial port not found | Check Device Manager for CP210x under COM ports. Install driver if missing. |
| MCP server fails to start in Cursor | Check Cursor's MCP logs: **View > Output > Model Context Protocol**. |
| `GITHUB_PERSONAL_ACCESS_TOKEN` not recognized | Ensure the variable is set at system level, not just in the current PowerShell session. |
