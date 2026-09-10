import unittest
import json
import time

import numpy as np

from tianji_teleop.hand_tracking.xr_input import (
    XrButtonBinding,
    XrControllerState,
    XrFrame,
)
from tianji_teleop.hand_tracking.xr_operator import (
    XrControllerOperatorPublisher,
    bind_from_environment,
    decode_xr_operator_observation,
    encode_xr_operator_observation,
)
from tianji_teleop.protocol import topics


def _frame(sequence, received, *, grip=0.0, trigger=0.0):
    controller = {
        "left": XrControllerState("left", [0, 0, 0, 0, 0, 0, 1], True, True, 0, 0, [0, 0]),
        "right": XrControllerState("right", [0, 0, 0, 0, 0, 0, 1], True, True, trigger, grip, [0, 0]),
    }
    return XrFrame(sequence, sequence + 10, received, [0, 0, 0, 0, 0, 0, 1], controller, ())


class XrOperatorBindingTest(unittest.TestCase):
    def publisher(self, epoch=3):
        return XrControllerOperatorPublisher(
            source_instance_id="xr-observation",
            epoch=epoch,
            start=XrButtonBinding("right", "grip", 0.8),
            home=XrButtonBinding("left", "grip", 0.8),
            clutch=XrButtonBinding("right", "trigger", 0.5),
        )

    def test_publisher_emits_explicit_start_home_and_clutch_edges_as_observations(self):
        observations = self.publisher().observations(_frame(4, 100, grip=0.9, trigger=0.7))
        self.assertEqual([item.action for item in observations], [
            "start_request", "home_request", "clutch_press", "clutch_release",
        ])
        values = {item.action: item for item in observations}
        self.assertTrue(values["start_request"].pressed)
        self.assertFalse(values["home_request"].pressed)
        self.assertTrue(values["clutch_press"].pressed)
        self.assertFalse(values["clutch_release"].pressed)
        self.assertEqual(values["start_request"].sequence, 4)
        self.assertEqual(values["start_request"].epoch, 3)

    def test_observation_wire_round_trip_is_strict_and_identity_bound(self):
        value = self.publisher().observations(_frame(0, 100))[0]
        payload = encode_xr_operator_observation(
            value, router_zid="router", publisher_instance_id="xr-observation"
        )
        decoded = decode_xr_operator_observation(payload)
        self.assertEqual(decoded, value)
        with self.assertRaises(ValueError):
            decode_xr_operator_observation(dict(payload, router_zid="other-router"),
                                            expected_router_zid="router")

    def test_invalid_controller_is_reported_unavailable_not_as_a_pressed_command(self):
        frame = _frame(0, 100)
        controllers = dict(frame.controllers)
        controllers["right"] = XrControllerState(
            "right", None, False, False, 1.0, 1.0, np.zeros(2)
        )
        invalid = XrFrame(frame.sequence, frame.source_timestamp_ns, frame.received_timestamp_ns,
                          frame.hmd_pose, controllers, ())
        values = {item.action: item for item in self.publisher().observations(invalid)}
        self.assertFalse(values["start_request"].available)
        self.assertFalse(values["start_request"].pressed)
        self.assertEqual(values["start_request"].confidence, 0.0)

    def test_managed_factory_routes_controller_edges_to_target_session(self):
        class Session:
            def __init__(self):
                self.callbacks = {}

            def declare_subscriber(self, topic, callback):
                self.callbacks[topic] = callback
                session = self

                class Handle:
                    def undeclare(self):
                        session.callbacks.pop(topic, None)

                return Handle()

        class Node:
            phase = "armed"

            def __init__(self):
                self.calls = []

            def request_start(self, **kwargs):
                self.calls.append(("start", kwargs["reason"]))
                return True

            def request_return(self, reason):
                self.calls.append(("return", reason))
                return True

            def set_clutch(self, side, pressed):
                self.calls.append(("clutch", side, pressed))
                return True

        session = Session()
        node = Node()
        environment = {
            "TIANJI_XR_OPERATOR_INPUT": "controller",
            "TIANJI_REQUIRED_CAPABILITY": "simulation",
            "TIANJI_REQUIRED_OBSERVATION_PROFILE": "manus",
            "TIANJI_ROUTER_ZID": "router",
            "TIANJI_COMPONENT_INSTANCE_ID": "target",
            "TIANJI_OBSERVATION_PUBLISHER_INSTANCE_ID": "xr-observation",
            "TIANJI_RESOLVED_DUAL_SESSION": json.dumps({
                "profile": "vr_manus_xr_sim",
                "config": {
                    "input_mode": "vr_manus",
                    "arm_input": "xr_tracker",
                    "operator_input": "controller",
                },
            }),
        }
        binding = bind_from_environment(environment, session=session, node=node,
                                       config={
                                           "operator_config": {
                                               "start": {"side": "right", "control": "grip", "threshold": 0.8},
                                               "home": {"side": "left", "control": "grip", "threshold": 0.8},
                                               "clutch": {"side": "right", "control": "trigger", "threshold": 0.5},
                                               "stable_ns": 0,
                                           }
                                       })
        self.addCleanup(binding.close)
        publisher = self.publisher(epoch=1)
        for sequence in range(9):
            now = time.monotonic_ns()
            for payload in publisher.payloads(_frame(sequence, now), router_zid="router"):
                session.callbacks[topics.XR_OPERATOR_OBSERVATION](payload)
        for sequence in range(9, 19):
            now = time.monotonic_ns()
            for payload in publisher.payloads(_frame(sequence, now, grip=0.9), router_zid="router"):
                session.callbacks[topics.XR_OPERATOR_OBSERVATION](payload)
        self.assertIn(("start", "xr_controller_start"), node.calls)

        for sequence in range(19, 28):
            now = time.monotonic_ns()
            for payload in publisher.payloads(_frame(sequence, now, grip=0.9, trigger=0.7), router_zid="router"):
                session.callbacks[topics.XR_OPERATOR_OBSERVATION](payload)
        self.assertIn(("clutch", "right", True), node.calls)

    def test_managed_binding_accepts_the_next_xr_connection_generation(self):
        class Session:
            def __init__(self):
                self.callbacks = {}

            def declare_subscriber(self, topic, callback):
                self.callbacks[topic] = callback
                return type("Handle", (), {"undeclare": lambda handle: None})()

        class Node:
            phase = "armed"

            def __init__(self):
                self.calls = []

            def request_start(self, **kwargs):
                self.calls.append(("start", kwargs["reason"]))

            def request_return(self, reason):
                self.calls.append(("return", reason))

            def set_clutch(self, side, pressed):
                self.calls.append(("clutch", side, pressed))

        session, node = Session(), Node()
        environment = {
            "TIANJI_XR_OPERATOR_INPUT": "controller",
            "TIANJI_REQUIRED_CAPABILITY": "simulation",
            "TIANJI_REQUIRED_OBSERVATION_PROFILE": "manus",
            "TIANJI_ROUTER_ZID": "router",
            "TIANJI_COMPONENT_INSTANCE_ID": "target",
            "TIANJI_OBSERVATION_PUBLISHER_INSTANCE_ID": "xr-observation",
            "TIANJI_RESOLVED_DUAL_SESSION": json.dumps({
                "profile": "vr_manus_xr_sim",
                "config": {
                    "input_mode": "vr_manus",
                    "arm_input": "xr_tracker",
                    "operator_input": "controller",
                },
            }),
        }
        binding = bind_from_environment(
            environment,
            session=session,
            node=node,
            config={
                "operator_config": {
                    "start": {"side": "right", "control": "grip", "threshold": 0.8},
                    "home": {"side": "left", "control": "grip", "threshold": 0.8},
                    "clutch": {"side": "right", "control": "trigger", "threshold": 0.5},
                    "stable_ns": 0,
                }
            },
        )
        self.addCleanup(binding.close)
        publisher = self.publisher(epoch=2)
        callback = session.callbacks[topics.XR_OPERATOR_OBSERVATION]
        now = time.monotonic_ns()
        for payload in publisher.payloads(_frame(0, now, grip=0.0), router_zid="router"):
            callback(payload)
        for payload in publisher.payloads(_frame(1, now + 1, grip=0.9), router_zid="router"):
            callback(payload)
        self.assertIn(("start", "xr_controller_start"), node.calls)

    def test_xr_controller_binding_is_rejected_for_pico_profile(self):
        environment = {
            "TIANJI_XR_OPERATOR_INPUT": "controller",
            "TIANJI_REQUIRED_CAPABILITY": "simulation",
            "TIANJI_REQUIRED_OBSERVATION_PROFILE": "pico",
        }
        with self.assertRaisesRegex(ValueError, "XR/Manus"):
            bind_from_environment(environment, session=None, node=None, config={})


if __name__ == "__main__":
    unittest.main()
