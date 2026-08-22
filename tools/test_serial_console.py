import sys
import time
import serial

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

def main():
    port = "COM9"
    baud = 115200
    print(f"Connecting to {port} @ {baud}...")
    try:
        ser = serial.Serial(port, baud, timeout=0.5, rtscts=False, dsrdtr=False)
        ser.dtr = False
        ser.rts = False
    except Exception as e:
        print(f"Failed to open {port}: {e}")
        return 1

    # Flush input buffer
    ser.reset_input_buffer()
    # Send empty newline to prompt console
    ser.write(b"\r\n")
    time.sleep(1.0)

    # Read whatever is pending
    out = b""
    while ser.in_waiting:
        out += ser.read(ser.in_waiting)
        time.sleep(0.05)
    print("Initial banner/prompt:")
    print(out.decode("latin1", errors="replace"))

    commands = [
        "help",
        "status",
        "limits"
    ]

    for cmd in commands:
        print(f"\n==========================================")
        print(f">>> COMMAND: {cmd}")
        print(f"==========================================")
        ser.write((cmd + "\r\n").encode("latin1"))
        time.sleep(1.2)
        resp = b""
        while ser.in_waiting:
            resp += ser.read(ser.in_waiting)
            time.sleep(0.05)
        print(resp.decode("latin1", errors="replace").strip())

    # Now launch test_cycle
    print(f"\n==========================================")
    print(f">>> COMMAND: test_cycle (Live Holistic Validation)")
    print(f"==========================================")
    ser.write(b"test_cycle\r\n")

    # Monitor output for full multi-stage cycle execution (up to 150 seconds)
    start_time = time.time()
    while time.time() - start_time < 150.0:
        if ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            text = chunk.decode("latin1", errors="replace")
            sys.stdout.write(text)
            sys.stdout.flush()
            if "HOLISTIC TEST CYCLE COMPLETE: PASS" in text or "test_cycle FAIL" in text:
                time.sleep(2.0)
                if ser.in_waiting:
                    rest = ser.read(ser.in_waiting).decode("latin1", errors="replace")
                    sys.stdout.write(rest)
                    sys.stdout.flush()
                break
        time.sleep(0.1)

    ser.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
