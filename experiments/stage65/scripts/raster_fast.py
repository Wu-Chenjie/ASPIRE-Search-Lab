from pathlib import Path
import ctypes,numpy as np
lib=ctypes.CDLL(str(Path('/Volumes/ColimaRuntime/runtime/research-stage63/eval-lib/raster.dylib')))
pointer=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
lib.raster_depth.argtypes=[pointer,ctypes.c_int,*([ctypes.c_double]*4),ctypes.c_int,ctypes.c_int,pointer]
lib.raster_depth.restype=None

def raster(mesh,k,t,shape):
 cam=np.ascontiguousarray((mesh-t[:3,3])@t[:3,:3],dtype=np.float64);k=np.asarray(k).reshape(3,3);out=np.empty(shape,np.float64)
 lib.raster_depth(cam,len(cam),k[0,0],k[1,1],k[0,2],k[1,2],*shape,out)
 return out
