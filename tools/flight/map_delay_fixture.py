"""One bounded interruption of an owned SITL mapper, for safety validation only.

No target/obstacle truth or flight commands. The real mapper is resumed before
cleanup, and its messages retain their original timestamps. Not a search policy.
"""
import json
import math
import signal
import time
from pathlib import Path


class MeasuredMapDelay:
    def __init__(self, mapper, folder, hold_sim=1.8, max_hold_wall=15.):
        self.mapper = mapper
        self.hold_sim, self.max_hold_wall = hold_sim, max_hold_wall
        self.state = 'waiting'
        self.stopped = False
        self.brake_seen = False
        self.start_sim = self.start_wall = None
        self.file = (Path(folder) / 'measured-map-delay.jsonl').open('x', buffering=1)

    def event(self, event, sim, **data):
        self.file.write(json.dumps(dict(event=event, sim_s=sim,
            wall_ns=time.monotonic_ns(), **data)) + '\n')

    def resume(self, sim, reason):
        if self.stopped:
            if self.mapper.poll() is None:
                self.mapper.send_signal(signal.SIGCONT)
                self.event('mapper_resumed', sim, pid=self.mapper.pid, reason=reason)
            self.stopped = False

    def step(self, sim, flight, execution):
        wall = time.monotonic()
        if self.state == 'waiting':
            speed = math.sqrt(sum(float(v)**2 for v in execution.get('reference_velocity', [])))
            remaining = execution.get('trajectory_duration', 0.) - execution.get('trajectory_time', 0.)
            if (execution.get('ready') and not execution.get('arrived') and execution.get('token')
                    and execution.get('stopping_reserve') and remaining >= 3. and speed > .1
                    and flight.get('armed') and flight.get('offboard')):
                if self.mapper.poll() is not None:
                    raise RuntimeError('Owned mapper exited before map-delay fixture')
                self.mapper.send_signal(signal.SIGSTOP)
                self.stopped = True
                self.start_sim, self.start_wall = sim, wall
                self.state = 'active'
                self.event('mapper_paused', sim, pid=self.mapper.pid, token=execution['token'],
                           reference_speed=speed, remaining_reference_s=remaining,
                           hold_sim_s=self.hold_sim, max_hold_wall_s=self.max_hold_wall)
        if self.state == 'active':
            if execution.get('safety_revision') and not self.brake_seen:
                self.brake_seen = True
                self.event('certified_brake_reported', sim,
                           token=execution.get('token'), revision=execution['safety_revision'])
            if wall-self.start_wall >= self.max_hold_wall:
                self.resume(sim, 'wall_watchdog')
                raise RuntimeError('Map-delay fixture wall watchdog exceeded')
            if sim-self.start_sim >= self.hold_sim:
                self.resume(sim, 'bounded_delay_complete')
                self.state = 'complete'

    def validate(self):
        if self.state != 'complete' or not self.brake_seen:
            raise RuntimeError('Map-delay fixture did not complete with a reported certified brake')

    def close(self, sim):
        self.resume(sim, 'cleanup')
        self.file.close()
