"""Executor adapter for explicit, default-disabled static-task cancellation."""
from core.planning.static_cancellation import StaticCancellation


class StaticCancellationExecutor:
    def _static_command_session_valid(self, packet):
        # A larger epoch from a previous process must not take over this run.
        # Session protection is opt-in alongside the existing cancellation mode.
        return (not self.authorization_session or
                packet.get('authorization_session') == self.authorization_session)

    def _static_cancel_setup(self, session):
        self.authorization_session = session
        self.static_cancellations = None
        self.static_task = None
        self.static_cancel_ack = None
        if session and self.safety_enabled:
            self.static_cancellations = StaticCancellation(session)

    def _static_cancel_command(self, packet):
        if 'static_cancel' not in packet:
            return False
        try:
            if self.static_cancellations is None:
                raise ValueError('Static cancellation disabled')
            self.static_cancel_ack = self.static_cancellations.request(packet['static_cancel'])
            pending = self.pending
            if pending and pending.get('static_task') == packet['static_cancel']['task']:
                self.pending = None
            if self._static_cancel_waiting():
                # No stop(), path mutation, reference reset or completion credit.
                self.observation_proof = None
                self.hold_since = None
        except ValueError as error:
            self.static_cancel_ack = dict(version=1, terminal=False, state='rejected',
                                          reason=str(error))
        self.report()
        return True

    def _static_task_admit(self, packet):
        if 'static_task' not in packet:
            return True
        try:
            return (self.static_cancellations is not None and
                    self.static_cancellations.admit(packet['static_task'], packet.get('token')))
        except ValueError:
            return False

    def _static_task_activated(self, packet):
        self.static_task = packet.get('static_task')
        if self.static_task is not None:
            self.static_cancellations.activated(self.static_task, packet['token'])

    def _static_cancel_waiting(self):
        return bool(self.static_cancellations is not None and
                    self.static_cancellations.waiting(self.static_task, self.token))

    def _static_cancel_retire(self, now, geometry):
        # Called only after the original fresh-map, geometry, timing and evidence
        # checks, inside that serialized safety callback. Keep the certified
        # endpoint curve/reference so ordinary arrived-hold checks continue.
        proof = dict(geometry, checked_at=now, original_safety_checks_passed=True)
        self.static_cancel_ack = self.static_cancellations.retire(
            self.static_task, self.token, proof)
        self.arrived = True
        self.observation_proof = None
        self.hold_since = None
        self.observation_failure = None
        self.reason = 'static_cancel_retired_at_safe_rest'
