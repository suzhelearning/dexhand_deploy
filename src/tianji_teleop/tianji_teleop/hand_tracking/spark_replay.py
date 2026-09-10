"""Offline deterministic receive-clock scheduling for SPARK conformance tests."""
from dataclasses import dataclass
from typing import Iterable, Iterator

from .reference_tjvr_receiver import ReceivedTjvrFrame, ReferenceTjvrReceiver
from .tjvr_trace import TjvrRecord


@dataclass(frozen=True)
class ReplayTick:
    tick_id: int
    now_ns: int
    sample: ReceivedTjvrFrame | None


def iter_reference_ticks(records: Iterable[TjvrRecord], *, period_ns: int = 5_000_000,
                         tail_ns: int = 250_000_000,
                         receiver: ReferenceTjvrReceiver | None = None) -> Iterator[ReplayTick]:
    """Fixed phase: first control tick is receive origin, arrivals <= tick first.

    No interpolation, FIFO control consumption, source-time sorting or real
    sleeping. Every arrival goes through the gate BEFORE latest-only sampling.
    This is a stated deterministic test schedule, not measured Viewer timing.
    """
    if type(period_ns) is not int or period_ns <= 0 or type(tail_ns) is not int or tail_ns < 0:
        raise ValueError('period must be a positive integer; tail must be nonnegative')
    source = receiver if receiver is not None else ReferenceTjvrReceiver('offline-replay', .15, .6)
    origin_ns = 1_000_000_000
    iterator = iter(records)
    def next_record(previous_time):
        item = next(iterator, None)
        if item is not None and (type(item.relative_receive_ns) is not int or
                                  item.relative_receive_ns < previous_time):
            raise ValueError('invalid/negative/backward TJVR receive time')
        return item
    record = next_record(0)
    if record is None:
        raise ValueError('empty TJVR trace')
    relative_ns = 0
    last_arrival_ns = 0
    tick_id = 0
    while record is not None or relative_ns <= last_arrival_ns + tail_ns:
        while record is not None and record.relative_receive_ns <= relative_ns:
            time_ns = record.relative_receive_ns
            if type(time_ns) is not int or time_ns < last_arrival_ns:
                raise ValueError('negative/backward TJVR receive time')
            source.ingest(record.packet, origin_ns + time_ns)
            last_arrival_ns = time_ns
            record = next_record(time_ns)
        tick_id += 1
        yield ReplayTick(tick_id, origin_ns + relative_ns, source.try_read_latest())
        relative_ns += period_ns


def encode_tick(tick: ReplayTick) -> str:
    if tick.sample is None:
        suffix = '0 0 0 -'
    else:
        sample = tick.sample
        f = sample.observation.frame
        suffix = (f'{f.received_timestamp_ns} {sample.resynchronization_generation} '
                  f'{int(sample.stream_discontinuity)} {f.raw_packet.hex()}')
    return f'TJSC1 {tick.tick_id} {tick.now_ns} {suffix}\n'
