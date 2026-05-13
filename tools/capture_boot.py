"""Reset ESP32 via DTR/RTS and capture serial output for N seconds."""
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
SECS = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0
OUT = sys.argv[4] if len(sys.argv) > 4 else "boot_serial.log"

s = serial.Serial(PORT, BAUD, timeout=0.05)
# ESP32 reset sequence: pull EN low (RTS), keep BOOT high (DTR), then release.
s.dtr = False
s.rts = True
time.sleep(0.1)
s.rts = False
time.sleep(0.05)

deadline = time.time() + SECS
total = 0
with open(OUT, "wb") as f:
    while time.time() < deadline:
        data = s.read(8192)
        if data:
            f.write(data)
            f.flush()
            total += len(data)
s.close()
print(f"captured {total} bytes -> {OUT}")
