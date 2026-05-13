"""Use esptool's hard reset sequence, then listen to the serial output.
This matches what `idf.py monitor` does internally."""
import sys, time, serial
PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
SECS = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0
OUT  = sys.argv[4] if len(sys.argv) > 4 else "boot.log"

s = serial.Serial(PORT, BAUD, timeout=0.05)
# esptool HardReset: DTR=False; RTS=True; sleep 100ms; DTR=False; RTS=False
s.setDTR(False); s.setRTS(True);  time.sleep(0.1)
s.setDTR(False); s.setRTS(False); time.sleep(0.05)
# Discard any garbage from the reset
s.reset_input_buffer()

deadline = time.time() + SECS
total = 0
with open(OUT, "wb") as f:
    while time.time() < deadline:
        data = s.read(8192)
        if data:
            f.write(data); f.flush(); total += len(data)
s.close()
print(f"captured {total} bytes -> {OUT}")
