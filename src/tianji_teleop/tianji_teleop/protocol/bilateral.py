"""Additive paired-arm wire contract; old single-arm messages stay unchanged."""
from copy import deepcopy
from dataclasses import dataclass

from .messages import ArmJointProposal, ArmJointCommand, ProtocolError

ARM_BILATERAL_PROPOSAL = 'tianji/proposal/arm/bilateral'
ARM_BILATERAL_RECEIPT = 'tianji/coordinator/arm/bilateral_receipt'
ARM_BILATERAL_COMMAND = 'tianji/command/arm/bilateral'


@dataclass(frozen=True)
class ArmBilateralProposal:
    run_id: str
    execution_epoch: int
    tick_id: int
    left: ArmJointProposal
    right: ArmJointProposal

    def __post_init__(self):
        if not isinstance(self.run_id, str) or not self.run_id.strip():
            raise ProtocolError('bilateral run_id must be nonempty')
        for name in ('execution_epoch', 'tick_id'):
            value = getattr(self, name)
            if type(value) is not int or not 0 < value < 2**63:
                raise ProtocolError(f'bilateral {name} must be positive int64')
        for side in ('left', 'right'):
            value = getattr(self, side)
            if not isinstance(value, ArmJointProposal) or value.side != side:
                raise ProtocolError('bilateral sides must be left and right')
            # Snapshot even when a local caller supplies an already parsed object.
            object.__setattr__(self, side, ArmJointProposal.from_dict(deepcopy(value.to_dict())))
        for name in ('sequence', 'timestamp_ns', 'target_sequence', 'producer',
                     'publisher_instance_id', 'router_zid'):
            if getattr(self.left, name) != getattr(self.right, name):
                raise ProtocolError(f'bilateral arms have different {name}')
        if self.left.sequence != self.tick_id:
            raise ProtocolError('bilateral proposal sequence must equal native tick')

    def to_dict(self):
        return dict(schema_version=1, kind='arm_bilateral_proposal', run_id=self.run_id,
                    execution_epoch=self.execution_epoch, tick_id=self.tick_id,
                    left=deepcopy(self.left.to_dict()), right=deepcopy(self.right.to_dict()))

    @classmethod
    def from_dict(cls, value):
        fields = {'schema_version', 'kind', 'run_id', 'execution_epoch', 'tick_id', 'left', 'right'}
        if not isinstance(value, dict) or set(value) != fields:
            raise ProtocolError('invalid bilateral proposal fields')
        if type(value['schema_version']) is not int or value['schema_version'] != 1:
            raise ProtocolError('unsupported bilateral proposal schema')
        if value['kind'] != 'arm_bilateral_proposal':
            raise ProtocolError('invalid bilateral proposal kind')
        return cls(value['run_id'], value['execution_epoch'], value['tick_id'],
                   ArmJointProposal.from_dict(value['left']), ArmJointProposal.from_dict(value['right']))


@dataclass(frozen=True)
class ArmBilateralCommand:
    """Atomic final-command envelope. Idle/Home has no reference tick.

    Command sequence belongs to the coordinator; reference_tick_id belongs
    to the native solver and is deliberately a separate association field.
    """
    run_id: str
    execution_epoch: int
    reference_tick_id: int | None
    left: ArmJointCommand
    right: ArmJointCommand

    def __post_init__(self):
        if not isinstance(self.run_id, str) or not self.run_id.strip():
            raise ProtocolError('bilateral command run_id required')
        if type(self.execution_epoch) is not int or not 0 < self.execution_epoch < 2**63:
            raise ProtocolError('bilateral command execution_epoch must be positive int64')
        if self.reference_tick_id is not None and (type(self.reference_tick_id) is not int or
                                                   not 0 < self.reference_tick_id < 2**63):
            raise ProtocolError('reference tick must be positive int64 or null')
        for side in ('left', 'right'):
            command = getattr(self, side)
            if not isinstance(command, ArmJointCommand) or command.side != side or command.producer != 'coordinator':
                raise ProtocolError('paired commands require canonical coordinator sides')
            object.__setattr__(self, side, ArmJointCommand.from_dict(deepcopy(command.to_dict())))
        for name in ('sequence', 'timestamp_ns', 'mode', 'proposal_sequence', 'target_sequence',
                     'publisher_instance_id', 'router_zid'):
            if getattr(self.left, name) != getattr(self.right, name):
                raise ProtocolError(f'paired commands have different {name}')
        if self.left.mode == 'teleop' and self.reference_tick_id != self.left.proposal_sequence:
            raise ProtocolError('teleop command does not match reference tick')
        if self.left.mode != 'teleop' and self.reference_tick_id is not None:
            raise ProtocolError('non-teleop commands must not claim a native tick')

    def to_dict(self):
        return dict(schema_version=1, kind='arm_bilateral_command', run_id=self.run_id,
            execution_epoch=self.execution_epoch, reference_tick_id=self.reference_tick_id,
            left=deepcopy(self.left.to_dict()), right=deepcopy(self.right.to_dict()))

    @classmethod
    def from_dict(cls, value):
        fields = {'schema_version', 'kind', 'run_id', 'execution_epoch', 'reference_tick_id', 'left', 'right'}
        if (not isinstance(value, dict) or set(value) != fields or type(value['schema_version']) is not int or
                value['schema_version'] != 1 or value['kind'] != 'arm_bilateral_command'):
            raise ProtocolError('invalid paired command schema')
        return cls(value['run_id'], value['execution_epoch'], value['reference_tick_id'],
                   ArmJointCommand.from_dict(value['left']), ArmJointCommand.from_dict(value['right']))
