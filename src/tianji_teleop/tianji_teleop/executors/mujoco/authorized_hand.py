"""Opt-in official hand direct execution for new simulation sessions only.

Legacy MujocoExecutor semantics are unchanged. Tracking requires the bound
producer and fresh coordinator authorization. A fault/stale session holds
actual qpos; explicit returning resets this kinematic simulator to hand zero.
That instantaneous simulation reset is not a physical-device trajectory.
"""
from threading import RLock

from .node import MujocoExecutor, _sample_payload
from ...protocol.messages import HandJointCommand, ProtocolError


class AuthorizedHandMujoco(MujocoExecutor):
    def __init__(self, *, hand_producer_id, hand_producer_instance_id, **kwargs):
        for value in (hand_producer_id, hand_producer_instance_id):
            if not isinstance(value, str) or not value.strip():
                raise ValueError('explicit hand producer binding required')
        self._hand_authority = (hand_producer_id, hand_producer_instance_id)
        self._hand_authority_lock = RLock()
        self._last_hand_rejection = None
        super().__init__(**kwargs)

    @property
    def last_hand_rejection(self):
        with self._hand_authority_lock:
            return self._last_hand_rejection

    def _authorized_phase(self, now):
        state = self._session_state
        if (not self._healthy or self._safety_locked or state is None or
                not 0 <= now - state.timestamp_ns <= self.command_timeout_ns):
            return None
        return state.state

    def validate_bilateral_home_rearm(self, execution_epoch):
        if (self._bilateral_session is None or self._bilateral_faulted or
                type(execution_epoch) is not int or execution_epoch >= 2**63 or
                execution_epoch != self._bilateral_session['execution_epoch'] + 1 or
                self._authorized_phase(self.clock()) != 'idle' or
                self.arm_state.position_rad != list(self.robot.home_all) or
                any(not self.hand_config.at_zero(self.hand_state(side).position_rad) for side in self.hand_sides)):
            raise ValueError('simulator rearm requires healthy idle Home and next epoch')

    def rearm_bilateral_at_home(self, execution_epoch):
        with self._bilateral_lock:
            with self._hand_authority_lock:
                self.validate_bilateral_home_rearm(execution_epoch)
                self._bilateral_session['execution_epoch'] = execution_epoch
                self._pending_arm.clear()
                self._pending_hand.clear()

    def on_hand_command(self, value):
        with self._hand_authority_lock:
            try:
                command = HandJointCommand.from_dict(
                    value.to_dict() if isinstance(value, HandJointCommand) else _sample_payload(value))
                now = self.clock()
                if ((command.producer, command.publisher_instance_id) != self._hand_authority or
                        command.router_zid != self.router_zid or
                        not 0 <= now - command.timestamp_ns <= self.command_timeout_ns or
                        self._authorized_phase(now) != 'teleop'):
                    state = self._session_state
                    self._last_hand_rejection = (
                        f'hand authority/freshness rejected: side={command.side}, sequence={command.sequence}, '
                        f'command_age_ns={now - command.timestamp_ns}, '
                        f'session_age_ns={now - state.timestamp_ns if state else None}, '
                        f'phase={state.state if state else None}, healthy={self._healthy}, '
                        f'safety_locked={self._safety_locked}, timeout_ns={self.command_timeout_ns}')
                    return False
                accepted = super().on_hand_command(command)
                self._last_hand_rejection = None if accepted else self._last_error
                return accepted
            except (ProtocolError, ValueError, TypeError) as exc:
                self._last_hand_rejection = f'invalid hand message: {exc}'
                return False

    def _tick(self, *, now_ns=None):
        with self._hand_authority_lock:
            now = self.clock() if now_ns is None else now_ns
            phase = self._authorized_phase(now)
            if phase != 'teleop':
                self._pending_hand.clear()
            if phase == 'returning':
                for addresses in self._hand_addresses.values():
                    for address, value in zip(addresses.values(), self.hand_config.zero_position_rad):
                        self._qpos[address] = value
            return super()._tick(now_ns=now)
