"""Cold-path rawviz owner: transfer stdout to the native receiver, never parse it.

The caller passes fileno() in the gateway's pass_fds, then calls
mark_transferred() only after successful spawn. The gateway owns the inherited
reader; this owner retains a non-consuming read handle until process cleanup.
This prevents SIGPIPE when native ingress stops before output draining finishes.
No SDK, reconnect or driver changes.
"""
import subprocess


class NativeManusProcess:
    def __init__(self, *, command, cwd=None, env=None):
        if (not isinstance(command, (list, tuple)) or not command or
                any(not isinstance(arg, str) or not arg or '\0' in arg for arg in command)):
            raise ValueError('rawviz requires an explicit nonempty argument vector')
        self._closed = False
        self._transferred = False
        self._process = subprocess.Popen(command, cwd=cwd, env=env,
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, bufsize=0)

    def fileno(self):
        if self._closed or self._transferred:
            raise ValueError('rawviz stdout is closed or already transferred')
        return self._process.stdout.fileno()

    def mark_transferred(self):
        self.fileno()  # Single-use, after successful gateway spawn.
        # Never read here. Keep the pipe alive until we deliberately stop rawviz;
        # closing the native reader during drain must not kill it with SIGPIPE.
        self._transferred = True

    @property
    def returncode(self):
        return self._process.poll()

    @property
    def failure(self):
        code = self.returncode
        if code is not None and not self._closed:
            return f'rawviz exited ({code}); explicit restart required'
        return None

    def close(self):
        if self._closed:
            return
        try:
            if self._process.poll() is None:
                self._process.terminate()
                try:
                    self._process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    self._process.wait(timeout=2)
        finally:
            self._process.stdout.close()
        self._closed = True
