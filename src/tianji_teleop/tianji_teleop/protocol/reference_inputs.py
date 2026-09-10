"""Raw reference input envelopes, independent of stream acceptance/authority."""
from dataclasses import dataclass

from ..hand_tracking.reference_tjvr import ReferenceTjvrFrame
from ..hand_tracking.reference_tjvr_receiver import ReceivedTjvrFrame

RAW_REFERENCE_TJVR = 'tianji/raw/tjvr_upper_limb'


@dataclass(frozen=True)
class ReferenceTjvrRaw:
    observation: ReferenceTjvrFrame
    router_zid: str

    def __post_init__(self):
        if not isinstance(self.observation, ReferenceTjvrFrame):
            raise ValueError('reference raw observation required')
        if not isinstance(self.router_zid, str) or not self.router_zid.strip():
            raise ValueError('raw router identity required')

    def to_dict(self):
        row = ReceivedTjvrFrame(self.observation).to_dict()
        del row['stream_discontinuity']
        del row['resynchronization_generation']
        row.update(kind='tjvr_upper_limb_raw', router_zid=self.router_zid)
        return row

    @classmethod
    def from_dict(cls, value):
        fields = {'schema_version', 'kind', 'router_zid', 'raw_packet_base64',
                  'received_timestamp_ns', 'receiver_instance_id', 'receiver_frame_sequence'}
        if not isinstance(value, dict) or set(value) != fields or value.get('kind') != 'tjvr_upper_limb_raw':
            raise ValueError('invalid reference raw envelope')
        row = dict(value)
        router = row.pop('router_zid')
        row.update(kind='tjvr_upper_limb_observation', stream_discontinuity=False,
                   resynchronization_generation=0)
        return cls(ReceivedTjvrFrame.from_dict(row).observation, router)
