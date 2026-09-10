"""Managed PICO hand resources; no receiver, router or actuator ownership.

The enclosing simulation launcher supplies an already verified router session
and the complete expected authority inventory. Absence during ordered startup
is not authorization: ordinary fresh SessionState and input gates still apply.
Connection generation is explicit; reconnect never restarts algorithm state.
"""
from contextlib import ExitStack
from pathlib import Path

from ..coordination.live_domain_guard import LiveDomainGuard
from ..zenoh_util import declare_component_liveliness
from .hand_retarget import OfficialHandClient
from .pico_hand_service import PicoHandService
from .pico_official_hand import PicoOfficialHandBackend, PicoOfficialHandProducer


class PicoHandComponent:
    def __init__(self, session, *, root, publisher_instance_id, router_zid,
                 coordinator_instance_id, receiver_instance_id, connection_generation,
                 expected_tokens, required_capability, client_factory=OfficialHandClient,
                 audit_processed_inputs=False):
        if required_capability != 'simulation':
            raise ValueError('PICO hand component requires simulation capability')
        for identity in (publisher_instance_id, router_zid, coordinator_instance_id, receiver_instance_id):
            if not isinstance(identity, str) or not identity.strip() or '/' in identity:
                raise ValueError('explicit single-component PICO identities required')
        if type(connection_generation) is not int or not 0 <= connection_generation < 2**63:
            raise ValueError('explicit PICO connection generation required')
        own = f'tj/live/producer/hand/official_wuji_hand2/{publisher_instance_id}'
        coordinator = f'tj/live/coordinator/arm/arm/{coordinator_instance_id}'
        self._guard = LiveDomainGuard(expected_tokens)
        if not {own, coordinator} <= self._guard.expected:
            raise ValueError('expected authorities must include this producer and bound coordinator')
        self._stack = ExitStack()
        self._closed = False
        self._service = None
        root = Path(root)
        try:
            import zenoh
            live = session.liveliness()
            subscription = live.declare_subscriber('tj/live/**',
                lambda sample: self._guard.observe(str(sample.key_expr),
                    present=sample.kind == zenoh.SampleKind.PUT), history=True)
            self._stack.callback(subscription.undeclare)
            for reply in live.get('tj/live/**', timeout=.5):
                if not reply.ok:
                    raise RuntimeError('PICO hand authority inventory failed')
                self._guard.observe(str(reply.result.key_expr), present=True)
            if self._guard.failure:
                raise RuntimeError(self._guard.failure)
            clients = {}
            for side in ('left', 'right'):
                client = client_factory(python=root / 'tools/wuji_hand_native/.pixi/envs/default/bin/python',
                    script=root / 'scripts/wuji_hand_worker.py', single_hand_side=side, startup_handshake=True)
                self._stack.callback(client.close)
                clients[side] = client
            if self._guard.failure:
                raise RuntimeError(self._guard.failure)
            backend = PicoOfficialHandBackend(clients, receiver_instance_id=receiver_instance_id,
                                               connection_generation=connection_generation)
            self._stack.callback(backend.close)
            producer = PicoOfficialHandProducer(backend, publisher_instance_id=publisher_instance_id,
                router_zid=router_zid, coordinator_instance_id=coordinator_instance_id,
                receiver_instance_id=receiver_instance_id, freshness_ns=200_000_000)
            token = declare_component_liveliness(session, role='producer/hand',
                logical_id=producer.producer_id, instance_id=publisher_instance_id)
            if token is None:
                raise RuntimeError('PICO hand component requires liveliness support')
            self._stack.callback(token.undeclare)
            self._service = PicoHandService(session, producer, required_capability=required_capability,
                                            authority_guard=self._guard,
                                            audit_processed_inputs=audit_processed_inputs)
            # LIFO: stop publication/thread before withdrawing own token, then
            # close workers and finally remove authority supervision.
            self._stack.callback(self._service.close)
        except BaseException:
            self.close()
            raise

    @property
    def failure(self):
        return self._guard.failure or (self._service.failure if self._service else None)

    def tick(self):
        if self._closed:
            raise RuntimeError('PICO hand component closed')
        return self._service.tick()

    def close(self):
        if not self._closed:
            self._closed = True
            self._stack.close()
