"""Canonical Zenoh key expressions for the Tianji teleoperation protocol."""

SESSION_INTENT = "tianji/session/intent"
SESSION_STATE = "tianji/session/state"
SOURCE_STATUS = "tianji/source/status"

ARM_TARGET = "tianji/target/arm/{side}"
HAND_TARGET = "tianji/target/hand/{side}"

PRODUCER_STATUS = "tianji/producer/status"
ARM_PROPOSAL = "tianji/proposal/arm/{side}"
ARM_SOLVED_POSE = "tianji/producer/arm/{side}/solved_pose"

COORDINATOR_STATUS = "tianji/coordinator/status"
AT_HOME = "tianji/coordinator/at_home"
RETURN_COMPLETE = "tianji/coordinator/return_complete"
ARM_COMMAND = "tianji/command/arm/{side}"

HAND_COMMAND = "tianji/command/hand/{side}"
ARM_STATE = "tianji/state/arm"
HAND_STATE = "tianji/state/hand/{side}"
EXECUTOR_STATUS = "tianji/executor/status"
HAND_EXECUTOR_STATUS = "tianji/executor/hand/{side}/status"

SAFETY_STOP = "tianji/safety/stop"
SAFETY_ACK = "tianji/safety/ack/{executor_id}"

RAW_PICO_CONTROLLER = "tianji/raw/pico_controller"
RAW_MOCAP_LIVE = "tianji/raw/mocap_live"
RAW_H5_REPLAY = "tianji/raw/h5_replay"
FRAME0_HAND_SKELETON = "tianji/diagnostics/h5/frame0_hand_skeleton"

MOCAP_ALIGNED_HANDS = "mocap/aligned/hands"
MOCAP_HANDS_FRAME = "mocap/hands/frame"
MOCAP_RIGID_BODY_NAMES = "mocap/rigid_body_names"


def arm_target(side: str) -> str:
    return ARM_TARGET.format(side=side)


def hand_target(side: str) -> str:
    return HAND_TARGET.format(side=side)


def arm_proposal(side: str) -> str:
    return ARM_PROPOSAL.format(side=side)


def arm_solved_pose(side: str) -> str:
    return ARM_SOLVED_POSE.format(side=side)


def arm_command(side: str) -> str:
    return ARM_COMMAND.format(side=side)


def hand_command(side: str) -> str:
    return HAND_COMMAND.format(side=side)


def hand_state(side: str) -> str:
    return HAND_STATE.format(side=side)


def hand_executor_status(side: str) -> str:
    return HAND_EXECUTOR_STATUS.format(side=side)


def safety_ack(executor_id: str) -> str:
    return SAFETY_ACK.format(executor_id=executor_id)
