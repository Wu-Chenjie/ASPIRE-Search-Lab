"""Timing diagnostics and fail-closed clock continuity for a local flight session."""
import math


def finite(value):
    return float(value) if math.isfinite(value) else None


class HeartbeatDiagnostics:
    def __init__(self, trace):
        self.trace=trace;self.inputs={};self.previous_clock=None;self.previous_heartbeat=None
        self.last_failures=None;self.last_log=-math.inf;self.last_snapshot={}

    def input(self,kind,stamp,now,wall):
        old=self.inputs.get(kind)
        # A retried pending position or duplicate sample is not a new receipt.
        if old and stamp<=old['stamp_s']:return
        record=dict(stamp_s=float(stamp),received_sim_s=now,received_wall_s=wall,
                    interval_sim_s=stamp-old['stamp_s'] if old else None,
                    interval_wall_s=wall-old['received_wall_s'] if old else None)
        self.inputs[kind]=record
        self.trace.event('input_receive',input=kind,**record)

    def clock(self,now,wall):
        previous=self.previous_clock;self.previous_clock=(now,wall)
        if previous is None:return None
        ds,dw=now-previous[0],wall-previous[1]
        # A backwards ROS clock invalidates map/sample epochs. A forward jump
        # beyond the existing 0.5 s estimator age budget is also not continuous
        # progress when it outpaces elapsed wall time by this margin.
        if ds < -1e-6 or ds>max(.5,2*max(dw,0.)+.1):
            return dict(sim_delta_s=ds,wall_delta_s=dw)
        return None

    def gate(self,checks,ages,now,wall):
        failures=tuple(k for k,v in checks.items() if not v)
        self.last_snapshot=dict(failed_gates=list(failures),ages={k:finite(v) for k,v in ages.items()},
            inputs={k:dict(v) for k,v in self.inputs.items()},sim_s=now)
        if failures!=self.last_failures or wall-self.last_log>=.2:
            self.trace.event('heartbeat_gate',**self.last_snapshot)
            self.last_log=wall;self.last_failures=failures
        return not failures

    def published(self,now,wall):
        old=self.previous_heartbeat;self.previous_heartbeat=(now,wall)
        self.trace.event('heartbeat_interval',sim_s=now,interval_sim_s=now-old[0] if old else None,
                         interval_wall_s=wall-old[1] if old else None)
