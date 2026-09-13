import os
from pathlib import Path
import signal
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
COMMON = ROOT / 'scripts/common.sh'


class GuardFailureRecoveryTest(unittest.TestCase):
    def run_shell(self, directory, code):
        return subprocess.run(['bash', '-c', 'source "$1"\n' + code, '_', str(COMMON)],
            env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=directory),
            capture_output=True, text=True, timeout=10, start_new_session=True)

    def test_administration_failure_rejects_conditional_acquire(self):
        with tempfile.TemporaryDirectory() as directory:
            result = self.run_shell(directory, '''
_lock_guard_administration() { return 1; }
if acquire_teleop_guard pico_vr_manus_sim; then exit 99; fi
[[ "$_TELEOP_GUARD_HELD" == false ]]
''')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertFalse((Path(directory) / 'guards/pico_vr_manus_sim/owner').exists())

    def test_detached_token_process_recovered_by_new_owner(self):
        with tempfile.TemporaryDirectory() as directory:
            token = 'review-' + Path(directory).name
            child = subprocess.Popen(['sleep', '30'], start_new_session=True,
                env=dict(os.environ, TIANJI_EMBEDDED_PICO_PROCESS_TOKEN=token))
            try:
                # A dead supervisor left its persisted token, but no direct PGID.
                result = self.run_shell(directory, '''
acquire_teleop_guard pico_vr_manus_sim
printf '%s\\n' "$2" > "$TELEOP_GUARD_DIR/process_tokens"
'''.replace('"$2"', '"' + token + '"'))
                self.assertEqual(result.returncode, 0, result.stderr)
                recovered = self.run_shell(directory, '''
acquire_teleop_guard pico2_hands_sim pico_vr_manus_sim
release_teleop_guard
''')
                self.assertEqual(recovered.returncode, 0, recovered.stderr)
                self.assertIsNotNone(child.poll(), 'detached descendant survived recovery')
            finally:
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)

    def test_owner_record_write_failure_rejects_acquire(self):
        with tempfile.TemporaryDirectory() as directory:
            result = self.run_shell(directory, '''
mkdir() {
  command mkdir "$@" || return
  if [[ "$*" == *guards/pico_vr_manus_sim ]]; then
    command mkdir "$TELEOP_GUARD_DIR/owner"
  fi
}
if acquire_teleop_guard pico_vr_manus_sim; then exit 99; fi
[[ "$_TELEOP_GUARD_HELD" == false && -z "$_TELEOP_TAKEOVER_FD" ]]
''')
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_sigkill_supervisor_recovers_detached_descendant(self):
        with tempfile.TemporaryDirectory() as directory:
            owner = subprocess.Popen(['bash', '-c', '''
source "$1"
acquire_teleop_guard pico_vr_manus_sim
register_teleop_process_token "$2"
setsid env TIANJI_EMBEDDED_PICO_PROCESS_TOKEN="$2" sleep 30 &
printf '%s\\n' "$!"
wait
''', '_', str(COMMON), 'sigkill-' + Path(directory).name],
                env=dict(os.environ, TIANJI_TELEOP_RUNTIME_DIR=directory),
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                start_new_session=True)
            child = None
            try:
                child = int(owner.stdout.readline())
                # Wait for exec to publish the inherited token before SIGKILL.
                import time
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    if b'TIANJI_EMBEDDED_PICO_PROCESS_TOKEN=' in Path(f'/proc/{child}/environ').read_bytes():
                        break
                    time.sleep(.01)
                else:
                    self.fail('descendant did not exec')
                owner.kill()
                owner.wait(timeout=5)
                result = self.run_shell(directory, '''
acquire_teleop_guard pico2_hands_sim pico_vr_manus_sim
release_teleop_guard
''')
                self.assertEqual(result.returncode, 0, result.stderr)
                stat = Path(f'/proc/{child}/stat')
                self.assertTrue(not stat.exists() or stat.read_text().split()[2] == 'Z')
                self.assertFalse((Path(directory) / 'guards/pico_vr_manus_sim').exists())
            finally:
                if owner.poll() is None:
                    owner.kill()
                owner.wait(timeout=5)
                if child is not None:
                    try:
                        os.killpg(child, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                owner.stdout.close()
                owner.stderr.close()

    def test_start_child_waits_for_independent_group(self):
        source = (ROOT / 'scripts/run_embedded_pico_vr_session.sh').read_text()
        function = source[source.index('start_child() {'):source.index('\nstop_child() {')]
        with tempfile.TemporaryDirectory() as directory:
            result = self.run_shell(directory, '''
declare -A child_process_token child_pid child_pgid child_start_ticks
runtime_env=(); embedded_process_run_token=test; log_dir="$TELEOP_RUNTIME_DIR"
acquire_teleop_guard pico_vr_manus_sim
fail() { exit 5; }
process_group_for() {
  if [[ ! -e "$log_dir/queried" ]]; then
    touch "$log_dir/queried"; printf '2\\n'
  else printf '%s\\n' "$1"; fi
}
register_teleop_process_group() { [[ "$1" == "${child_pid[test]}" ]]; }
''' + function + '''
start_child test sleep 2
kill "${child_pid[test]}"; wait "${child_pid[test]}" || true
''')
            self.assertEqual(result.returncode, 0, result.stderr)
