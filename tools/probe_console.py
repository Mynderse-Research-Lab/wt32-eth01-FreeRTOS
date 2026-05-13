"""Open serial, send console commands one per line, capture replies."""
import sys, time, serial
PORT = sys.argv[1] if len(sys.argv) > 1 else "COM6"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
OUT  = sys.argv[3] if len(sys.argv) > 3 else "probe.log"

CMDS = [
    "mcp_dump a",
    "mcp_dump b",
    "limits",
]

s = serial.Serial(PORT, BAUD, timeout=0.05)
s.reset_input_buffer()
out = open(OUT, "wb")

def drain(secs):
    deadline = time.time() + secs
    total = 0
    while time.time() < deadline:
        d = s.read(8192)
        if d:
            out.write(d); out.flush(); total += len(d)
    return total

drain(1.0)

for cmd in CMDS:
    marker = f"\n===== CMD: {cmd} =====\n".encode()
    out.write(marker); out.flush()
    s.write((cmd + "\r\n").encode())
    s.flush()
    drain(2.0)

out.close()
s.close()
print("done ->", OUT)
