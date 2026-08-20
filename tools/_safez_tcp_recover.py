import socket, time, re, sys
HOST,PORT,PW="192.168.1.100",2323,"LTU_1932"
def strip(s): return re.sub(r"\x1b\[[0-9;]*[A-Za-z]","",s)
def drain(s,t=0.3):
  time.sleep(t); o=b""
  try:
    while True: o+=s.recv(4096)
  except socket.timeout: pass
  return strip(o.decode("utf-8","replace"))
def cmd(s,line,w=1.0):
  print(">>>",line); s.sendall((line+"\n").encode()); time.sleep(w); out=drain(s,0.2)
  print("\n".join(out.replace("\r","").splitlines()[-12:])); return out

# wait TCP + Class1
deadline=time.time()+120
s=None
while time.time()<deadline:
  try:
    s=socket.create_connection((HOST,PORT),timeout=3); s.settimeout(1.0); break
  except OSError:
    time.sleep(2)
if not s: sys.exit("no TCP")
b=drain(s,0.5)
if "Password:" in b:
  s.sendall((PW+"\n").encode()); print(drain(s,1)[-200:])
else:
  print(b[-200:])
cmd(s,"alarmreset",1)
cmd(s,"enable",3)
# poll eiptiming / status for online
ok=False
for i in range(30):
  out=cmd(s,"eiptiming",1.2)
  if re.search(r"n=\s*[1-9]|GO|exchange", out, re.I) and "n=0" not in out.split("exchange")[-1][:40] if "exchange" in out else True:
    # also try faults
    st=cmd(s,"status",0.8)
    if "Motor Enabled: Yes" in st:
      # check if move works with tiny Z-only
      m=cmd(s,"move 35 25 0",1.5)
      time.sleep(2)
      st2=cmd(s,"status",0.8)
      if "[PATH] Z arm failed" not in m and "Z Position:" in st2:
        # if still at 30 and path failed, class1 still dead
        zm=re.search(r"Z Position:\s*([-+0-9.]+)", st2)
        if zm and abs(float(zm.group(1))-25)<3:
          print("CLASS1_MOTION_OK"); ok=True; break
        if "[PATH] Z arm failed" in m or "Z arm failed" in drain(s,0.5):
          print("still no Z motion")
        else:
          # wait idle
          for _ in range(40):
            st2=cmd(s,"status",0.5)
            if "Busy: No" in st2: break
          zm=re.search(r"Z Position:\s*([-+0-9.]+)", st2)
          print("Z after move attempt", zm.group(1) if zm else "?")
          if zm and abs(float(zm.group(1))-25)<3:
            print("CLASS1_MOTION_OK"); ok=True; break
  print(f"poll {i}: waiting Class1...")
  time.sleep(2)
if not ok:
  cmd(s,"faults",1); cmd(s,"disable",1); print("CLASS1_DOWN"); sys.exit(3)
# Interlock at high Z — only if motion works
cmd(s,"move 35 90 0",1)
for _ in range(60):
  st=cmd(s,"status",0.5)
  if "Busy: No" in st: break
st=cmd(s,"status",0.8)
zm=re.search(r"Z Position:\s*([-+0-9.]+)", st)
print("Z for interlock", zm.group(1) if zm else "?")
hx=cmd(s,"home x",1.5)
print("home x result markers:", "blocked" in hx.lower(), "above SAFE_Z" in hx, "INTERLOCK" in hx)
# lower into band
cmd(s,"move 35 10 0",1)
for _ in range(60):
  st=cmd(s,"status",0.5)
  if "Busy: No" in st: break
# X jog in band
cmd(s,"move 60 10 0",1)
for _ in range(80):
  st=cmd(s,"status",0.5)
  if "Busy: No" in st: break
st=cmd(s,"status",0.8)
xm=re.search(r"X Position:\s*([-+0-9.]+)", st)
zm=re.search(r"Z Position:\s*([-+0-9.]+)", st)
print(f"final X={xm.group(1) if xm else '?'} Z={zm.group(1) if zm else '?'}")
cmd(s,"move 35 30 0",1)
for _ in range(60):
  st=cmd(s,"status",0.5)
  if "Busy: No" in st: break
cmd(s,"disable",1)
cmd(s,"logout",0.3)
s.close()
print("LIVE_TCP_OK")
