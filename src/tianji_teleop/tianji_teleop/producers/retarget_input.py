"""Explicit per-source input adapters for the shared retarget owner thread."""
from dataclasses import dataclass

from ..hand_tracking.models import PicoRawFrame
from ..hand_tracking.reference_manus_process import ManusCallback


@dataclass(frozen=True)
class RetargetInput:
    payload: object
    sequence: int
    received_timestamp_ns: int
    receiver_instance_id: str


def manus_callback_input(row):
    if not isinstance(row, ManusCallback):
        raise TypeError('Manus loop requires an actual ManusCallback')
    return RetargetInput(list(row.points), row.sequence, row.received_timestamp_ns, row.receiver_instance_id)


def pico_frame_input(frame):
    if not isinstance(frame, PicoRawFrame):
        raise TypeError('PICO loop requires an actual PicoRawFrame')
    return RetargetInput(frame, frame.receiver_frame_sequence + 1,
                         frame.received_timestamp_ns, frame.receiver_instance_id)
