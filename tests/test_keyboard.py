import os
import struct
import unittest

from tianji_teleop.sources.common.keyboard import _EvdevKeyState


class EvdevKeyStateTests(unittest.TestCase):
    def test_tracks_enter_press_and_release(self) -> None:
        read_fd, write_fd = os.pipe()
        try:
            state = _EvdevKeyState(
                ("Return",),
                event_paths=(f"/proc/self/fd/{read_fd}",),
            )
            event = struct.Struct("@llHHi")
            self.assertFalse(state.is_pressed())
            os.write(write_fd, event.pack(0, 0, 1, 28, 1))
            self.assertTrue(state.is_pressed())
            os.write(write_fd, event.pack(0, 0, 1, 28, 0))
            self.assertFalse(state.is_pressed())
            state.close()
        finally:
            os.close(read_fd)
            os.close(write_fd)


if __name__ == "__main__":
    unittest.main()
