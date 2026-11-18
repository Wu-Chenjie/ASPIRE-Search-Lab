"""Read-only size inventory. Only named source-owned atomic temporaries may vanish."""
from pathlib import Path
import stat
VOLATILE=frozenset(['observer-status.tmp','first-imu.tmp','camera-ready.tmp'])
def measure(root):
    capture=8192;other=0;gone=[]
    for p in root.rglob('*') if root.exists() else []:
        if p.name=='health.tmp':continue # identical original exclusion
        try:s=p.lstat() # obtain one size snapshot, no second lookup after discovery
        except FileNotFoundError:
            relative=str(p.relative_to(root))
            if relative not in VOLATILE:raise
            gone.append(relative);continue
        if not stat.S_ISREG(s.st_mode):continue
        if 'map-evidence' in p.parts:capture+=s.st_size
        else:other+=s.st_size
    return dict(capture_bytes=capture,other_bytes=other,volatile_temporaries_renamed=gone)
