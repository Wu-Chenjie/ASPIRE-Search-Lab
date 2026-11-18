"""Default-disabled moving handoff adapter; original safety owns all motion."""
import time
import numpy as np
from core.planning.execution_safety import ExecutionCorridor, TRACKING_MARGIN
from core.planning.handoff import validate_safe_handoff, check_handoff_budget


class GuardedC2Executor:
    def _c2_setup(self, enabled):
        self.moving_c2_enabled = enabled
        if enabled and (not self.safety_enabled or self.static_cancellations is None):
            raise ValueError('Moving C2 requires original safety and a current session')

    def _c2_check(self, packet, curve, now, started, *, phase, next_time=None):
        if not self.moving_c2_enabled or packet.get('static_task') is None:
            raise ValueError('Moving C2 disabled or unbound task')
        if not self._static_command_session_valid(packet) or not self._static_task_admit(packet):
            raise ValueError('Moving C2 cancelled or stale session')
        boundary = packet.get('handoff', {})
        if curve is None or not isinstance(boundary, dict):
            raise ValueError('Moving C2 requires a curve and boundary identity')
        if (self.curve is None or self.arrived or self.paused or self.done or self.safety_fault or
                boundary.get('from_token') != self.token or boundary.get('from_epoch') != self.epoch):
            raise ValueError('Moving C2 predecessor changed or unavailable')
        age = now-self.stamps.get(self.id, -np.inf)
        if not 0 <= age <= .5 or np.linalg.norm(self.positions[self.id]-self.ref) > TRACKING_MARGIN:
            raise ValueError('Moving C2 odometry stale or tracking tube exceeded')
        self.safety_replica.map.rebuild()
        extra = {}
        if phase == 'transition':
            receipt = packet.get('c2_receipt')
            if not isinstance(receipt, dict):
                raise ValueError('Moving C2 missing admission certificate')
            next_reference = curve.sample(next_time-boundary['trajectory_time'])[0]
            if np.linalg.norm(next_reference-self.positions[self.id]) > TRACKING_MARGIN:
                raise ValueError('Moving C2 next reference exceeds tracking tube')
            extra.update(new_progress=next_time-boundary['trajectory_time'],
                         reference_step=next_time-self.curve_time,
                         expected_new=receipt['new_curve_sha256'])
        return validate_safe_handoff(
            self.curve, curve, boundary['trajectory_time'], self.curve_time,
            expected_old=boundary.get('old_curve_sha256'),
            old_safety_revision=1 if self.safety_revision else 0,
            now=now, native_stamp=self.native_safety.stamp, mapper_stamp=self.safety_stamp,
            corridor=ExecutionCorridor(self.safety_replica.map, self.native_safety),
            started=started, phase=phase, clock=time.monotonic, **extra)

    def _c2_budget(self, now, started):
        elapsed = check_handoff_budget(
            now, self.native_safety.stamp, self.safety_stamp, started, time.monotonic)
        if now - self.stamps.get(self.id, -np.inf) + elapsed > .5:
            raise ValueError('handoff_odometry_stale')

    def _c2_reject_candidate(self, packet, reason):
        if self.static_cancellations is not None and packet.get('static_task') is not None:
            try:
                self.static_cancellations.reject_prepared(packet['static_task'], packet['token'])
            except ValueError:
                # Invalid identities have no ledger entry to retire.
                pass
        self.reject_activation(packet, reason)

    def _c2_admit(self, packet, curve, started):
        if self.pending is not None:
            if self.pending['token'] == packet['token']:
                return True
            self._c2_reject_candidate(packet, 'Moving C2 already has a pending command')
            return False
        now = self.get_clock().now().nanoseconds*1e-9
        try:
            receipt = self._c2_check(packet, curve, now, started, phase='admission')
            self.save_safety_certificate('c2_admission', curve, 0., token=packet['token'])
            self._c2_budget(now, started)
        except (ValueError, KeyError, TypeError) as error:
            # Admission runs in its own callback. Recheck the predecessor on
            # the next control tick before advancing after a slow candidate.
            self.safety_dirty = True
            self._c2_reject_candidate(packet, str(error))
            return False
        self.pending = dict(packet, curve=curve, errors=receipt['continuity'], c2_receipt=receipt)
        return True

    def _c2_transition(self, packet, next_time, now, safety_cycle_started):
        try:
            receipt=self._c2_check(packet, packet['curve'], now, safety_cycle_started,
                                   phase='transition', next_time=next_time)
            self.save_safety_certificate('c2_transition', packet['curve'],
                                        next_time-packet['handoff']['trajectory_time'], token=packet['token'])
            self._c2_budget(now, safety_cycle_started)
        except (ValueError, KeyError, TypeError) as error:
            self.pending = None
            rejection = str(error)
            # A failed geometry check can itself overrun before reaching its
            # next budget check. The predecessor cannot advance on that tick.
            try:
                self._c2_budget(now, safety_cycle_started)
            except ValueError as expired:
                rejection = str(expired)
                self._defer_execution_check()
            self._c2_reject_candidate(packet, rejection)
            if self.safety_hold_valid:
                try:
                    self._c2_budget(now, safety_cycle_started)
                except ValueError:
                    self._defer_execution_check()
            return False
        packet['c2_transition_receipt']=dict(receipt,authorized_transition_passed=True,observation_complete=False)
        return True

    def _c2_applied(self, packet, now):
        self.current_static_handoff = False
        if self.static_task is not None:
            self.static_cancel_ack=self.static_cancellations.supersede(
                self.static_task,packet['handoff']['from_token'],packet['token'],
                packet['c2_transition_receipt'])
        self._static_task_activated(packet)
        self.terminal_hold_since = None
        self.terminal_hold_status = None
        self.observation_failure = None
        self.safety_revision = None
        self.safety_stopping = False
        self.safety_stop_deadline = None
        self.safety_hold_valid = True
        self.activation_rejected = None
        self.last_safety_check = now
