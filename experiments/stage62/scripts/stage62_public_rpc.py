"""Read-only encoding of the existing unmodified PX4 CLI Unix protocol.
Protocol provenance: PX4 SDK platforms/posix/src/px4/common/px4_daemon/{client,sock_protocol,server}.cpp.
No uORB memory access, publication, estimation, control, or SDK changes.
"""
import base64,socket,time
from types import SimpleNamespace
ALLOWED={'sensor_accel','sensor_gyro','vehicle_imu'}

def read_public(topic,timeout=.25,socket_path='/tmp/px4-sock-1'):
 if topic not in ALLOWED:raise ValueError('Only three read-only listener topics are allowed')
 begin=time.monotonic();deadline=begin+timeout;reply=bytearray()
 def remaining():
  value=deadline-time.monotonic()
  if value<=0:raise TimeoutError('Complete native CLI reply deadline')
  return value
 with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
  client.settimeout(remaining());client.connect(socket_path)
  client.settimeout(remaining());client.sendall(('listener '+topic).encode('ascii')+b'\x00')
  while True:
   client.settimeout(remaining());block=client.recv(4096)
   if not block:break
   reply.extend(block)
   if len(reply)>32768:raise RuntimeError('Native CLI reply byte cap')
 if len(reply)<2 or reply[-2]!=0 or b'\x00' in reply[:-2]:raise RuntimeError('Malformed native CLI terminal status; no imputation')
 stdout=reply[:-2].decode('utf8',errors='strict')
 return SimpleNamespace(stdout=stdout,stderr='',returncode=int(reply[-1]),raw_reply_base64=base64.b64encode(reply).decode(),elapsed_s=time.monotonic()-begin)
