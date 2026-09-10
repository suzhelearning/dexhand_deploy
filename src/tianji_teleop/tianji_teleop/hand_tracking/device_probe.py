"""Bounded receive-only liveness counters, never a control readiness gate."""


class InputProbe:
    def __init__(self):
        self.sides = {side: dict(sequence=None, first_ns=None, last_ns=None,
            new_frames=0, max_gap_ns=0) for side in ('left', 'right')}

    def observe(self, side, sequence, received_ns):
        row = self.sides[side]
        if row['sequence'] is not None and sequence <= row['sequence']:
            return
        if row['last_ns'] is not None:
            row['max_gap_ns'] = max(row['max_gap_ns'], received_ns - row['last_ns'])
        else:
            row['first_ns'] = received_ns
        row.update(sequence=sequence, last_ns=received_ns, new_frames=row['new_frames'] + 1)

    def report(self, now_ns, *, minimum_frames=10):
        sides = {}
        for side, row in self.sides.items():
            age = None if row['last_ns'] is None else now_ns - row['last_ns']
            sides[side] = dict(new_frames=row['new_frames'],
                fresh=age is not None and 0 <= age <= 200_000_000,
                last_valid_age_ms=None if age is None else age / 1e6,
                max_valid_gap_ms=row['max_gap_ns'] / 1e6)
        return dict(passed=all(row['fresh'] and row['new_frames'] >= minimum_frames for row in sides.values()),
            scope='bilateral_input_liveness_only', sides=sides, minimum_frames=minimum_frames,
            freshness_ms=200, robot_commands_enabled=False, hardware_acceptance_complete=False,
            limitations=['does not verify calibration, gesture accuracy, SDK identity or model mapping',
                'not a production authorization gate; stop probe before starting a teleop session'])


class XrInputProbe:
    """Receive-only liveness report for the complete XR input surface.

    The XR route needs more than two arm pose streams: the HMD is the
    reference device, both controllers carry start/Home/clutch state, and the
    configured wrist/forearm Tracker serials must be present.  This class
    records only that data was received and fresh.  It deliberately has no
    router, command, IK, or retargeting dependency.
    """

    _FRESHNESS_NS = 200_000_000

    def __init__(self, required_tracker_serials, *, minimum_trackers=1):
        serials = tuple(str(item).strip() for item in required_tracker_serials)
        if any(not item for item in serials) or len(set(serials)) != len(serials):
            raise ValueError('required_tracker_serials must contain distinct non-empty serials')
        if type(minimum_trackers) is not int or minimum_trackers < 0:
            raise ValueError('minimum_trackers must be a non-negative integer')
        self.required_tracker_serials = serials
        self.minimum_trackers = minimum_trackers
        self.frames = 0
        self._last_frame_key = None
        self._latest_valid_tracker_count = 0
        self._signals = {
            name: dict(first_ns=None, last_ns=None, valid_frames=0)
            for name in ('hmd', 'controller_left', 'controller_right')
        }
        self._signals.update({
            f'tracker:{serial}': dict(first_ns=None, last_ns=None, valid_frames=0)
            for serial in serials
        })
        self._arm = InputProbe()

    @staticmethod
    def _serial_matches(actual, configured):
        actual = str(actual).strip()
        configured = str(configured).strip()
        return bool(actual and configured and (actual == configured or actual.endswith(configured)))

    def _observe_signal(self, name, valid, received_ns):
        row = self._signals[name]
        if not valid:
            return
        if row['last_ns'] is None:
            row['first_ns'] = received_ns
        row.update(last_ns=received_ns, valid_frames=row['valid_frames'] + 1)

    def observe(self, frame, binding):
        """Observe one :class:`XrFrame`; return False for duplicate frames."""
        from .xr_input import XrBindingConfig, XrFrame

        if not isinstance(frame, XrFrame) or not isinstance(binding, XrBindingConfig):
            raise TypeError('frame and binding are required')
        frame_key = (frame.connection_generation, frame.sequence)
        if self._last_frame_key is not None and frame_key <= self._last_frame_key:
            return False
        self._last_frame_key = frame_key
        self.frames += 1
        received_ns = frame.received_timestamp_ns
        self._observe_signal('hmd', frame.hmd_pose is not None, received_ns)
        for side in ('left', 'right'):
            controller = frame.controller(side)
            self._observe_signal(
                f'controller_{side}',
                bool(controller.available and controller.valid and controller.pose is not None),
                received_ns,
            )

        valid_tracker_count = 0
        for tracker in frame.trackers:
            if tracker.valid and tracker.pose is not None:
                valid_tracker_count += 1
            for serial in self.required_tracker_serials:
                if self._serial_matches(tracker.serial_number, serial):
                    self._observe_signal(
                        f'tracker:{serial}',
                        bool(tracker.valid and tracker.pose is not None),
                        received_ns,
                    )
        self._latest_valid_tracker_count = valid_tracker_count

        for side in ('left', 'right'):
            if binding.pose_for_arm(frame, side) is not None:
                self._arm.observe(side, self.frames, received_ns)
        return True

    def _signal_report(self, row, now_ns, minimum_frames):
        age = None if row['last_ns'] is None else now_ns - row['last_ns']
        return dict(
            valid_frames=row['valid_frames'],
            fresh=age is not None and 0 <= age <= self._FRESHNESS_NS,
            last_valid_age_ms=None if age is None else age / 1e6,
            minimum_frames=minimum_frames,
        )

    def report(self, now_ns, *, minimum_frames=10):
        if type(minimum_frames) is not int or minimum_frames < 1:
            raise ValueError('minimum_frames must be a positive integer')
        hmd = self._signal_report(self._signals['hmd'], now_ns, minimum_frames)
        controllers = {
            side: self._signal_report(self._signals[f'controller_{side}'], now_ns, minimum_frames)
            for side in ('left', 'right')
        }
        tracker_rows = {
            serial: self._signal_report(self._signals[f'tracker:{serial}'], now_ns, minimum_frames)
            for serial in self.required_tracker_serials
        }
        arm = self._arm.report(now_ns, minimum_frames=minimum_frames)
        required_fresh_count = sum(
            row['fresh'] and row['valid_frames'] >= minimum_frames
            for row in tracker_rows.values()
        )
        device_ok = (
            self.frames >= minimum_frames
            and hmd['fresh'] and hmd['valid_frames'] >= minimum_frames
            and all(row['fresh'] and row['valid_frames'] >= minimum_frames for row in controllers.values())
            and required_fresh_count == len(tracker_rows)
            and self._latest_valid_tracker_count >= self.minimum_trackers
            and arm['passed']
        )
        return dict(
            passed=bool(device_ok),
            scope='xr_input_liveness_only',
            frames=self.frames,
            hmd=hmd,
            controllers=controllers,
            trackers=dict(
                required_count=len(tracker_rows),
                fresh_count=required_fresh_count,
                latest_valid_count=self._latest_valid_tracker_count,
                minimum_trackers=self.minimum_trackers,
                required=tracker_rows,
            ),
            arm=arm,
            minimum_frames=minimum_frames,
            freshness_ms=self._FRESHNESS_NS / 1e6,
            robot_commands_enabled=False,
            hardware_acceptance_complete=False,
            limitations=[
                'does not verify calibration, SDK version, Manus glove data or model mapping',
                'does not execute start/Home/clutch and is not a production authorization gate',
                'stop the probe before starting a teleoperation session',
            ],
        )
