import os
import subprocess
import sys
import unittest


class NativeManusProcessTest(unittest.TestCase):
    def test_reader_shutdown_does_not_kill_driver_before_owner_cleanup(self):
        from tianji_teleop.hand_tracking.native_manus_process import NativeManusProcess
        import time
        process = NativeManusProcess(command=[sys.executable, '-c',
            'import os,signal,time;signal.signal(signal.SIGPIPE,signal.SIG_DFL);'
            '\nwhile True: os.write(1,b"frame\\n");time.sleep(.001)'])
        try:
            fd = process.fileno()
            reader = subprocess.Popen([sys.executable, '-c',
                'import os,sys;os.read(int(sys.argv[1]),6)', str(fd)], pass_fds=(fd,))
            try:
                process.mark_transferred()
                self.assertEqual(reader.wait(timeout=5), 0)
                time.sleep(.1)
                self.assertIsNone(process.failure)
            finally:
                if reader.poll() is None:
                    reader.kill()
                reader.wait()
        finally:
            process.close()
        with self.assertRaises(OSError):
            os.fstat(fd)

    def test_stdout_handoff_and_owned_cleanup(self):
        from tianji_teleop.hand_tracking.native_manus_process import NativeManusProcess
        process=NativeManusProcess(command=[sys.executable,'-c',
            'import os,time;os.write(1,b"SDK ready\\n");time.sleep(60)'])
        try:
            fd=process.fileno()
            reader=subprocess.Popen([sys.executable,'-c',
                'import os,sys;sys.stdout.buffer.write(os.read(int(sys.argv[1]),100))',str(fd)],
                pass_fds=(fd,),stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            try:
                process.mark_transferred()
                with self.assertRaises(ValueError):process.fileno()
                with self.assertRaises(ValueError):process.mark_transferred()
                out,err=reader.communicate(timeout=5)
                self.assertEqual(reader.returncode,0,err)
                self.assertEqual(out,b'SDK ready\n')
                self.assertIsNone(process.failure)
            finally:
                if reader.poll() is None:reader.kill()
                reader.communicate()
        finally:
            process.close();process.close()
        self.assertIsNotNone(process.returncode)

    def test_early_driver_failure_visible(self):
        from tianji_teleop.hand_tracking.native_manus_process import NativeManusProcess
        import time
        process=NativeManusProcess(command=[sys.executable,'-c','raise SystemExit(7)'])
        try:
            end=time.monotonic()+5
            while process.failure is None and time.monotonic()<end:time.sleep(.01)
            self.assertIn('7',process.failure)
        finally:process.close()

    def test_failed_launch_and_untransferred_cleanup(self):
        from tianji_teleop.hand_tracking.native_manus_process import NativeManusProcess
        with self.assertRaises(ValueError):NativeManusProcess(command=[])
        with self.assertRaises(FileNotFoundError):NativeManusProcess(command=['/nonexistent/rawviz'])
        process=NativeManusProcess(command=[sys.executable,'-c','import time;time.sleep(60)'])
        fd=process.fileno()
        process.close()
        with self.assertRaises(OSError):os.fstat(fd)
