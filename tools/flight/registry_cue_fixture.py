"""Explicit empty-SITL interface fixture; no detector, map or actuator changes."""
import math
class ChurningCueFixture:
    def __init__(self):
        self.start=None;self.last=-math.inf;self.sequences={};self.sent=0;self.drop_steps=0
    def step(self,now,ready):
        if not ready or not math.isfinite(now) or now<0:return None
        if self.start is None:self.start=now
        elapsed=now-self.start
        if 8<=elapsed<15:self.drop_steps+=1;return None
        if now-self.last<.5:return None
        ident=int(elapsed//3)+1
        if ident>64:return None
        self.last=now;self.sent+=1;sequence=self.sequences.get(ident,0)+1;self.sequences[ident]=sequence
        row=dict(robot='uav0',session='explicit-empty-cue-fixture',counter=str(ident),
            sequence=str(sequence),revision='0',first_s=self.start+3*(ident-1),capture_s=now,
            frame='map',frame_version='empty-fixture-local-v1',shared_aligned=False,
            visual_state='person_candidate',localization_state='unknown',
            has_position=False,has_bearing=True,confidence=.95,
            ox=0.,oy=-2.,oz=1.8,dx=1.,dy=0.,dz=0.,half_angle_rad=.15,
            camera_half_fov_rad=.637,support_captures=5,support_first_s=max(0.,now-.8))
        return dict(schema=1,kind='observations',raw_tracks_total=0,observations=[row])
    def report(self):
        return dict(explicit_synthetic_observation_fixture=True,start_s=self.start,sent=self.sent,
            source_ids=len(self.sequences),drop_steps=self.drop_steps,
            purpose='Repeated new IDs, fixed camera-ray neighborhood, no position; dropout8..15s. Fake support is interface input, not actual RGB evidence.',
            safety_mapping_actuators_changed=False)
