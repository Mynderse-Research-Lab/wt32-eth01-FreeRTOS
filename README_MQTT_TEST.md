# Gantry MQTT Test Script

Python script for testing the WT32-ETH01 Gantry control system via MQTT.

## Installation

1. Install Python 3.6 or higher
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
   
   Or install directly:
   ```bash
   pip install paho-mqtt
   ```

## Usage

### Automatic Test Sequence (Default)

Run the script with default settings (connects to 192.168.2.1:1883):

```bash
python test_gantry_mqtt.py
```

The script will:
1. Connect to the MQTT broker
2. Wait for the WT32-ETH01 device to connect (detects "online" message)
3. Run a sequence of test commands
4. Monitor position updates

### Custom MQTT Broker

Specify a different broker:

```bash
python test_gantry_mqtt.py --host 192.168.1.100 --port 1883
```

### Interactive Mode

For manual command entry:

```bash
python test_gantry_mqtt.py --interactive
```

In interactive mode:
- Enter commands manually in the format: `x:VALUE,y:VALUE,t:VALUE,s:VALUE`
- Special commands:
  - `home` - Send homing command
  - `cal` - Send calibration command
  - `status` - Check current position
  - `quit` or `exit` - Exit program

## Command Format

Commands are sent in the format expected by `main.cpp`:

```
x:VALUE,y:VALUE,t:VALUE,s:VALUE,h:FLAG,c:FLAG
```

Where:
- `x`: X position (mm) - integer
- `y`: Y position (mm) - integer  
- `t`: Theta angle (degrees) - integer
- `s`: Speed (pulses per second) - integer
- `h`: Home flag (1 = execute homing, 0 = no homing)
- `c`: Calibrate flag (1 = execute calibration, 0 = no calibration)

Examples:
```
x:1000,y:200,t:90,s:5000,h:0,c:0    # Move to position
x:0,y:0,t:0,s:0,h:1,c:0              # Execute homing
x:0,y:0,t:0,s:0,h:0,c:1              # Execute calibration
```

## MQTT Topics

The script subscribes to:
- `lab/outbox` - Device status (expects "online" message when device connects)
- `lab/gantry` - Position updates (format: "X:VALUE Y:VALUE Th:VALUE")

The script publishes to:
- `lab/gantry/cmd` - Command topic (sends movement commands)

## Example Output

```
🔌 Connecting to MQTT broker at 192.168.2.1:1883...
✓ Connected to MQTT broker at 192.168.2.1:1883
✓ Subscribed to: lab/outbox, lab/gantry

⏳ Waiting for WT32-ETH01 to connect (timeout: 30s)...
   Listening for 'online' message on lab/outbox

============================================================
✓ WT32-ETH01 device is ONLINE!
============================================================

📨 [OUTBOX] online
📍 [POSITION] X:0 Y:0 Th:0

============================================================
Starting Gantry Test Sequence
============================================================

────────────────────────────────────────────────────────────
Test 1/4
────────────────────────────────────────────────────────────
📤 Sending command: x:0,y:0,t:0,s:0
   Description: Status check (no movement)
✓ Command sent successfully

👂 Monitoring for 2.0 seconds...
📍 [POSITION] X:0 Y:0 Th:0

...
```

## Troubleshooting

### Connection Failed

**Problem**: Cannot connect to MQTT broker

**Solutions**:
1. Verify MQTT broker is running
2. Check broker IP address and port
3. Verify network connectivity (ping test)
4. Check firewall settings

### Device Not Connecting

**Problem**: Script waits but device never sends "online" message

**Solutions**:
1. Verify WT32-ETH01 is powered on
2. Check Ethernet connection
3. Verify device IP address matches network configuration
4. Check serial output on WT32 for connection status
5. Verify MQTT broker is accessible from device

### No Position Updates

**Problem**: Commands sent but no position updates received

**Solutions**:
1. Verify device is connected (check for "online" message)
2. Check serial output on WT32 for command processing
3. Verify device is processing commands (check logs)
4. Ensure gantry task is running

## Requirements

- Python 3.6+
- paho-mqtt library
- Network access to MQTT broker
- WT32-ETH01 device powered and connected

## Notes

- The script waits up to 30 seconds for the device to connect
- Commands use QoS 1 (at least once delivery)
- Position updates are received every 5 seconds (as configured in main.cpp)
- The script monitors messages continuously while waiting/completing tests
