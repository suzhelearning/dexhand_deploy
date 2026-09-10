import unittest
import glfw

from tianji_teleop.producers.spark import live_runner


class DualLiveKeysTest(unittest.TestCase):
    def test_terminal_bytes_and_glfw_key_codes_have_same_actions(self):
        self.assertTrue(hasattr(live_runner, 'decode_key'))
        for byte, glfw_key, expected in ((ord('s'), glfw.KEY_S, 's'),
                                        (ord('h'), glfw.KEY_H, 'h'),
                                        (ord('r'), glfw.KEY_R, 'r'),
                                        (ord('q'), glfw.KEY_Q, 'q')):
            self.assertEqual(live_runner.decode_key(byte), expected)
            self.assertEqual(live_runner.decode_key(glfw_key), expected)
        for value in (glfw.KEY_F1, -1, True, ord('a')):
            self.assertIsNone(live_runner.decode_key(value))
