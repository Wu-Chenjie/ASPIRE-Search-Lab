"""Measured startup and tracking observations for the SITL harness."""
import math


def paced_wall_deadline(sim_seconds, real_time_factor, default_wall_seconds):
    # Gazebo Harmonic runs without pacing below 1e-9. Keep the existing wall
    # deadline unless configured slow pacing needs more time, plus setup slack.
    if 1e-9 <= real_time_factor < 1.:
        return max(default_wall_seconds, sim_seconds / real_time_factor + 60.)
    return default_wall_seconds


class BootstrapTrackingWatchdog:
    def __init__(self):self.since=None

    def observe(self,execution):
        t=float(execution.get('time',0.));error=float(execution.get('tracking_error',0.))
        bad=(not execution.get('ready',False) and execution.get('reason')=='initial_yaw_scan'
             and (not math.isfinite(error) or error>.65))
        if not bad:self.since=None;return None
        if self.since is None or t<self.since:self.since=t
        if t-self.since>=3.:
            return f'Measured bootstrap tracking deviation persisted 3 s: {error:.3f} m at {t:.3f} s'
        return None


class StartupDeadline:
    # Harness timeouts only; never creates odometry or alters arming guards.
    def __init__(self,odom_wall=120.,odom_sim=30.,ready_wall=180.,ready_sim=60.,*,real_time_factor=1.):
        self.odom_wall=paced_wall_deadline(odom_sim,real_time_factor,odom_wall)
        self.ready_wall=paced_wall_deadline(ready_sim,real_time_factor,ready_wall)
        self.odom_sim=odom_sim;self.ready_sim=ready_sim
        self.has_odometry=False;self.ready=False;self.odom_at=None;self.ready_at=None
        self.initial_z=None;self.takeoff_observed=False;self.executor_ready=False
    def observe(self,sim,wall,odometry,flight,execution):
        stamp=(odometry or {}).get('stamp')
        fresh=stamp is not None and math.isfinite(stamp) and -.1<=sim-stamp<=.5
        if fresh and flight.get('origin') is not None and flight.get('origin_calibration')=='calibrated':
            if not self.has_odometry:self.odom_at=dict(sim_s=sim,wall_s=wall)
            self.has_odometry=True
        xyz=(odometry or {}).get('xyz')
        if fresh and xyz is not None and len(xyz)==3 and all(math.isfinite(v) for v in xyz):
            if self.initial_z is None:self.initial_z=float(xyz[2])
            if flight.get('armed') and flight.get('offboard') and xyz[2]-self.initial_z>=.3:self.takeoff_observed=True
        self.executor_ready=bool(execution.get('ready'))
        # Full three-height yaw scanning may legitimately outlast a preflight
        # wall deadline in slow SITL. Measured takeoff ends the startup wait;
        # remaining bootstrap tracking/execution guards continue unchanged.
        if self.has_odometry and (self.takeoff_observed or self.executor_ready) and flight.get('armed') and flight.get('offboard'):
            if not self.ready:self.ready_at=dict(sim_s=sim,wall_s=wall)
            self.ready=True
        if not self.has_odometry and (wall>=self.odom_wall or sim>=self.odom_sim):
            return 'Startup odometry deadline exceeded before validated origin/estimated odometry'
        if not self.ready and (wall>=self.ready_wall or sim>=self.ready_sim):
            return 'Startup flight-ready deadline exceeded before measured ready/armed/offboard'
        return None
    def report(self):
        return dict(has_odometry=self.has_odometry,ready=self.ready,takeoff_observed=self.takeoff_observed,executor_ready=self.executor_ready,
                    ready_definition='fresh_estimated_departure_0.3m_or_executor_ready_while_armed_offboard',odom_at=self.odom_at,ready_at=self.ready_at,
                    odom_wall_s=self.odom_wall,odom_sim_s=self.odom_sim,ready_wall_s=self.ready_wall,ready_sim_s=self.ready_sim)
