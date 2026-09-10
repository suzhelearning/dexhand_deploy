#!/usr/bin/env python3
"""Managed official PICO hand producer; receiver and executors are external."""
from contextlib import ExitStack
import json
import os
from pathlib import Path
import signal
import sys
from threading import Event

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tianji_teleop'))


def bindings(environment):
    if (environment.get('TIANJI_PICO_HAND_MANAGED') != '1' or
            environment.get('TIANJI_REQUIRED_CAPABILITY') != 'simulation'):
        raise ValueError('use managed pico2_hands_sim simulation launcher')
    names = {'publisher_instance_id': 'TIANJI_HAND_PRODUCER_INSTANCE_ID',
             'router_zid': 'TIANJI_ROUTER_ZID',
             'coordinator_instance_id': 'TIANJI_COORDINATOR_INSTANCE_ID',
             'receiver_instance_id': 'TIANJI_OBSERVATION_PUBLISHER_INSTANCE_ID'}
    result = {key: environment.get(name, '') for key, name in names.items()}
    if any(not value.strip() or '/' in value for value in result.values()):
        raise ValueError('explicit PICO hand component identities required')
    authorities = json.loads(environment.get('TIANJI_AUTHORITIES', '{}'))
    from tianji_teleop.coordination.arm_command_coordinator import ArmCommandCoordinator
    authorities = ArmCommandCoordinator._validate_authorities(authorities)
    tokens = set()
    roles = {'source': 'source', 'producer_arm': 'producer/arm', 'producer_hand': 'producer/hand',
             'executor_arm': 'executor/arm', 'executor_hand': 'executor/hand', 'coordinator_arm': 'coordinator/arm'}
    for role, token_role in roles.items():
        entry = authorities[role]
        rows = entry.values() if 'logical_id' not in entry else [entry]
        for row in rows:
            if row.get('enabled', True):
                if row['router_zid'] != result['router_zid']:
                    raise ValueError('PICO authorities belong to another router')
                tokens.add(f"tj/live/{token_role}/{row['logical_id']}/{row['publisher_instance_id']}")
    # This receiver is started fresh by the same launcher; its first successful
    # connection is generation 1. A reconnect is a fault, not an implicit reset.
    return dict(result, expected_tokens=tokens, connection_generation=1, required_capability='simulation')


def main():
    kwargs = bindings(os.environ)
    from tianji_teleop.zenoh_util import open_session, require_single_router
    from tianji_teleop.producers.pico_hand_component import PicoHandComponent
    stop = Event()
    with ExitStack() as stack:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous = signal.signal(sig, lambda *_: stop.set())
            stack.callback(signal.signal, sig, previous)
        session = open_session()
        stack.callback(session.close)
        require_single_router(session, kwargs['router_zid'])
        component = PicoHandComponent(session, root=ROOT, audit_processed_inputs=True, **kwargs)
        stack.callback(component.close)
        print('pico_hand_component_ready', flush=True)
        while not stop.is_set():
            status = component.tick()
            if not status.healthy:
                print(status.error, file=sys.stderr, flush=True)
                return 1
            stop.wait(.02)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
