"""Serialized authorization-side bridge for opt-in static cancellations."""
import math
from core.planning.static_cancellation import StaticTask, PlannerCancellation


def _task_from_wire(data):
    if len(data) != 6 or any(type(v) not in (int, float) or
            not 0 <= v < 2**53 or int(v) != v for v in data):
        raise ValueError('Invalid exact cancellation wire integers')
    version, session, task, proposal, revision, cancel_revision = map(int, data)
    if version != 1 or cancel_revision != 1 or session <= 0:
        raise ValueError('Invalid cancellation wire version/session')
    packet = dict(session=str(session), task=task, proposal=proposal, map_revision=revision)
    return packet, StaticTask.parse(packet)


def task_from_wire(data):
    return _task_from_wire(data)[0]


def task_wire(task):
    key = StaticTask.parse(task)
    session = int(key.session) if key.session.isdecimal() else 0
    if not 0 < session < 2**53:
        raise ValueError('Native cancellation session must be an exact positive integer')
    return [1, session, key.task, key.proposal, key.map_revision, 1]


class StaticCancelGateway:
    def _static_gateway_setup(self, session):
        self.static_cancel_session = session
        self.static_tasks = {}
        self.static_requests = {}
        self.static_sent = {}
        self.static_handoffs = {}
        self.static_receipts = {}
        self.static_retired_tokens = set()
        self.static_ack_stamp = -math.inf
        if session:
            task_wire(dict(session=session, task=1, proposal=0, map_revision=1))
            if not self.safety_enabled or not self.config.get('stop_and_plan'):
                raise ValueError('Cancellation requires original safety and stop-and-plan')

    def _static_register(self, key):
        if not self.static_cancel_session or key.session != self.static_cancel_session:
            raise ValueError('Static cancellation session mismatch/disabled')
        if self.static_tasks.setdefault(key.proposal, key) != key:
            raise ValueError('Static proposal version changed')
        return key

    def _static_before_propose(self, row):
        packet = row.get('static_task')
        if packet is None:
            # A late cancellation may precede even its boundary metadata.
            return row['traj_id'] not in self.static_requests
        try:
            key = self._static_register(StaticTask.parse(packet))
            return key.proposal == row['traj_id'] and key.proposal not in self.static_requests
        except ValueError:
            return False

    def _static_attach_pending(self, row):
        if row.get('static_task') is not None:
            self.pending['static_task'] = dict(row['static_task'])

    def _static_command_sent(self, packet):
        if packet.get('static_task') is not None and packet.get('trajectory') is not None:
            key = self._static_register(StaticTask.parse(packet['static_task']))
            self.static_sent[key.proposal] = packet['token']
            if packet.get('handoff'):
                self.static_handoffs[packet['token']] = packet['handoff']['from_token']

    def _static_cancel_request(self, data):
        packet, key = _task_from_wire(data)
        self._static_register(key)
        if key.proposal in self.static_receipts:
            self._static_send_ack(packet, self.static_receipts[key.proposal])
            return
        request = self.static_requests.get(key.proposal)
        if request is None:
            self.static_requests[key.proposal] = request = PlannerCancellation(packet)
        command = request.request()
        pending = self.pending if self.pending and self.pending['native_id'] == key.proposal else None
        active = self.intent is not None and self.intent['native_id'] == key.proposal
        # A cancellation can arrive before static metadata. A matching active
        # or committed proposal can therefore be absent from the sent ledger.
        if active or key.proposal in self.static_sent or (pending and pending['committed']):
            # Original executor owns both already sent and activated commands.
            # Do not use the generic cancel(), which resets a motion reference.
            self.publish(self.command_pub, dict(static_cancel=command))
            return
        if pending:
            self.pending = None
        # No command was sent. Keep the tombstone for queued/late proposals.
        self.last_rejection = key.proposal
        ack = dict(version=1, task=packet, revision=1, state='revoked_before_start',
                   terminal=True, executor_token=None, proof=None)
        if request.acknowledge(ack):
            self.static_receipts[key.proposal] = 2
            self._static_send_ack(packet, 2)

    def _static_execution_ack(self, execution):
        ack = execution.get('static_cancel_ack')
        if not isinstance(ack, dict):
            return
        try:
            key = self._static_register(StaticTask.parse(ack.get('task')))
        except ValueError:
            return
        request = self.static_requests.get(key.proposal)
        stamp = execution.get('time')
        if (request is None or type(stamp) not in (int, float)
                or stamp < self.static_ack_stamp or not 0 <= self.now()-stamp <= .5):
            return
        if (type(ack.get('version')) is not int or ack['version'] != 1 or
                type(ack.get('revision')) is not int or ack['revision'] != 1):
            return
        sent_token = self.static_sent.get(key.proposal)
        if ack.get('executor_token') is not None and ack['executor_token'] != sent_token:
            return
        self.static_ack_stamp = stamp
        # A terminal active-task receipt needs current safe-hold readiness.
        state = ack.get('state')
        proof = ack.get('proof')
        if state in ('retired_at_safe_rest', 'superseded_by_c2') and (
                execution.get('ready') is not True or execution.get('safety_fault')
                or execution.get('flight_session_fault') or not isinstance(proof, dict)
                or type(proof.get('checked_at')) not in (int, float)
                or not 0 <= proof['checked_at'] <= stamp):
            return
        if state == 'superseded_by_c2' and (
                sent_token is None or ack.get('executor_token') != sent_token or
                self.static_handoffs.get(proof.get('successor_token')) != sent_token):
            return
        if not request.acknowledge(ack):
            if state == 'waiting_safe_rest':
                self._static_send_ack(ack['task'], 1)
            return
        if self.pending and self.pending['native_id'] == key.proposal:
            self.pending = None
        if self.intent and self.intent['native_id'] == key.proposal:
            self.intent = None
        self.last_rejection = key.proposal
        status = {'retired_at_safe_rest': 3, 'superseded_by_c2': 4, 'revoked_before_start': 2}[state]
        if ack.get('executor_token'):
            self.static_retired_tokens.add(ack['executor_token'])
        self.static_receipts[key.proposal] = status
        self._static_send_ack(ack['task'], status)
