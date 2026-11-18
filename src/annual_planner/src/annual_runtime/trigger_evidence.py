"""Opt-in passive ring evidence. Never read by planning, mapping or safety policy."""
from collections import deque
from pathlib import Path
import io,json,os,queue,threading,time,uuid
import numpy as np
from .control_observation import freeze

MIB=1024*1024

class EvidenceRing:
    def __init__(self,directory,drone,role,*,pre_records=12,post_records=2,
                 memory_bytes=8*MIB,queue_bytes=8*MIB,max_record_bytes=2*MIB,
                 max_disk_bytes=32*MIB,max_triggers=2,pre_seconds=4.):
        self.drone=int(drone);self.role=role;self.session=os.environ.get('RACER_OBSERVATION_SESSION','unspecified')[:96]
        self.folder=Path(directory)/('drone-%s-%s'%(drone,role));self.folder.mkdir(parents=True,exist_ok=False)
        self.pre_records=pre_records;self.post_records=post_records;self.memory_limit=memory_bytes
        self.queue_limit=queue_bytes;self.record_limit=max_record_bytes;self.disk_limit=max_disk_bytes
        self.max_triggers=max_triggers;self.pre_seconds=pre_seconds
        self.ring=deque();self.ring_bytes=0;self.queued_bytes=0;self.memory_peak=0
        self.queue=queue.Queue(maxsize=64);self.lock=threading.Lock();self.closed=False
        self.records=0;self.written=0;self.disk_bytes=0;self.evicted=0;self.oversize=0
        self.queue_dropped=0;self.disk_dropped=0;self.write_errors=0;self.trigger_dropped=0
        self.windows=[];self.seen=set();self.pending_window=None;self.capture_max_wall_s=0.;self.hook_max_wall_s=0.
        self.health_sequence=0
        self.thread=threading.Thread(target=self._writer,daemon=True);self.thread.start()

    def record(self,kind,sim_s,metadata,arrays=None):
        if self.closed:return
        started=time.monotonic()
        try:
            sim_s=float(sim_s)
            if not np.isfinite(sim_s):raise ValueError('nonfinite evidence time')
            meta=json.dumps(freeze(dict(kind=kind,sim_s=sim_s,wall_ns=time.monotonic_ns(),**metadata)),separators=(',',':'),allow_nan=False)
            if len(meta.encode())>65536:self.oversize+=1;return
            values={};size=len(meta.encode())
            for k,v in (arrays or {}).items():
                if not isinstance(k,str) or not k.isidentifier() or len(k)>64:raise ValueError('invalid evidence array name')
                a=np.asarray(v)
                if a.dtype.hasobject:raise ValueError('object evidence array')
                size+=a.nbytes
                if size>self.record_limit:self.oversize+=1;return
                a=a.copy();a.flags.writeable=False;values[k]=a
            if size>self.record_limit:self.oversize+=1;return
            frame=(meta,values,size,sim_s)
            self.records+=1;self.ring.append(frame);self.ring_bytes+=size
            while self.ring and (len(self.ring)>self.pre_records or self.ring_bytes>self.memory_limit or sim_s-self.ring[0][3]>self.pre_seconds):
                self.ring_bytes-=self.ring.popleft()[2];self.evicted+=1
            self.memory_peak=max(self.memory_peak,self.ring_bytes)
            w=self.pending_window
            if w is not None and w['post_captured']<self.post_records:
                if self._enqueue(frame,w,'post'):w['post_captured']+=1
                if w['post_captured']==self.post_records:self.pending_window=None
        except (ValueError,TypeError,OSError):self.write_errors+=1
        finally:self.capture_max_wall_s=max(self.capture_max_wall_s,time.monotonic()-started)

    def _enqueue(self,frame,window,phase):
        with self.lock:
            if self.queued_bytes+frame[2]>self.queue_limit:self.queue_dropped+=1;return False
            self.queued_bytes+=frame[2]
        try:
            self.queue.put_nowait((frame,window['index'],phase,window['trigger']))
            return True
        except queue.Full:
            with self.lock:self.queued_bytes-=frame[2]
            self.queue_dropped+=1;return False

    def trigger(self,reason,sim_s,*,trigger_id=None,details=None):
        if self.closed:return None
        try:
            ident=trigger_id or uuid.uuid4().hex
            if not isinstance(ident,str) or not 0<len(ident)<=128 or ident in self.seen:return None
            if len(self.windows)>=self.max_triggers:self.trigger_dropped+=1;return None
            sim_s=float(sim_s)
            if not np.isfinite(sim_s):raise ValueError('nonfinite trigger')
            trigger=dict(schema=1,kind='map_evidence_trigger',id=ident,drone=self.drone,session=self.session,
                         sim_s=sim_s,reason=str(reason)[:256],details=freeze(details or {}))
            if len(json.dumps(trigger).encode())>16384:self.oversize+=1;return None
            w=dict(index=len(self.windows)+1,trigger=trigger,pre_captured=len(self.ring),post_requested=self.post_records,post_captured=0)
            self.seen.add(ident)
            self.windows.append(w)
            for frame in self.ring:self._enqueue(frame,w,'pre')
            self.pending_window=w
            return trigger
        except (ValueError,TypeError,KeyError):self.write_errors+=1;return None

    def receive_trigger(self,data):
        try:
            if len(data.encode())>16384:self.oversize+=1;return
            packet = json.loads(data)
            if not isinstance(packet, dict):
                raise ValueError('Evidence trigger must be a JSON object')
            if (packet.get('schema') != 1 or packet.get('kind') != 'map_evidence_trigger' or
                    packet.get('drone') != self.drone or packet.get('session') != self.session):
                return
            self.trigger(packet['reason'], packet['sim_s'], trigger_id=packet['id'], details=packet.get('details'))
        except (ValueError, TypeError, KeyError, OverflowError, RecursionError):
            self.write_errors += 1

    def _writer(self):
        last_health=-float('inf')
        while True:
            if time.monotonic()-last_health>=.5:
                self._write_health();last_health=time.monotonic()
            try:item=self.queue.get(timeout=.25)
            except queue.Empty:continue
            if item is None:self.queue.task_done();return
            frame,window,phase,trigger=item
            try:
                meta,arrays,size,stamp=frame
                payload=dict(arrays);payload['metadata_json']=np.frombuffer(json.dumps(dict(record=json.loads(meta),window=window,phase=phase,trigger=trigger),separators=(',',':')).encode(),dtype=np.uint8)
                output=io.BytesIO();np.savez_compressed(output,**payload);value=output.getvalue()
                if self.disk_bytes+len(value)>self.disk_limit:self.disk_dropped+=1;continue
                path=self.folder/('%02d-%05d-%s.npz'%(window,self.written+1,phase))
                with path.open('xb') as f:f.write(value)
                self.disk_bytes+=len(value);self.written+=1
            except (OSError,ValueError):self.write_errors+=1
            finally:
                with self.lock:self.queued_bytes-=frame[2]
                self.queue.task_done()

    def _write_health(self):
        try:
            self.health_sequence+=1
            health=dict(schema=1,role=self.role,drone=self.drone,session=self.session,
                wall_ns=time.monotonic_ns(),sequence=self.health_sequence,closed=self.closed,
                records=self.records,written=self.written,queued_bytes=self.queued_bytes,
                pending=self.queue.qsize(),disk_bytes=self.disk_bytes,disk_limit=self.disk_limit,
                oversize=self.oversize,queue_dropped=self.queue_dropped,disk_dropped=self.disk_dropped,
                write_errors=self.write_errors,trigger_dropped=self.trigger_dropped,
                capture_max_wall_s=self.capture_max_wall_s,hook_max_wall_s=self.hook_max_wall_s)
            data=json.dumps(health,separators=(',',':'),allow_nan=False)
            if len(data.encode())>4096:raise ValueError('Health record exceeds 4096 bytes')
            temp=self.folder/'health.tmp';temp.write_text(data+'\n');temp.replace(self.folder/'health.json')
        except (OSError,ValueError):self.write_errors+=1

    def close(self):
        if self.closed:return
        if not self.windows and self.ring:self.trigger('shutdown_tail_no_trigger',self.ring[-1][3])
        self.closed=True
        try:self.queue.put(None,timeout=2.)
        except queue.Full:self.write_errors+=1
        self.thread.join(timeout=5.)
        status=dict(schema=1,role=self.role,drone=self.drone,session=self.session,records=self.records,written=self.written,
                    disk_bytes=self.disk_bytes,disk_limit=self.disk_limit,memory_limit=self.memory_limit,queue_limit=self.queue_limit,
                    memory_peak=self.memory_peak,ring_bytes=self.ring_bytes,queued_bytes=self.queued_bytes,capture_max_wall_s=self.capture_max_wall_s,hook_max_wall_s=self.hook_max_wall_s,
                    evicted=self.evicted,oversize=self.oversize,queue_dropped=self.queue_dropped,disk_dropped=self.disk_dropped,
                    write_errors=self.write_errors,trigger_dropped=self.trigger_dropped,windows=self.windows,
                    writer_alive=self.thread.is_alive(),pending=self.queue.qsize(),closed=True)
        try:(self.folder/'status.json').write_text(json.dumps(status,separators=(',',':'))+'\n')
        except OSError:self.write_errors+=1


def optional_ring(drone,role,**options):
    directory=os.environ.get('RACER_MAP_EVIDENCE_DIR')
    if not directory:return None
    try:
        path=Path(directory).resolve();root=Path(__file__).resolve().parents[4]
        if path.is_relative_to(root):return None
        return EvidenceRing(path,drone,role,**options)
    except (OSError,ValueError):return None
