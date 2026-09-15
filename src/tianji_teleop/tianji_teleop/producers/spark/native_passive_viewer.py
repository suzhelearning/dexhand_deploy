"""Join the passive renderer before interpreter/GLFW teardown (native route only).

MuJoCo's public passive handle closes asynchronously and hides its daemon thread.
Use the installed launch primitive to retain thread ownership without changing
the SDK globally. Keep this small compatibility seam covered by a real GL test.
"""
from contextlib import contextmanager
from queue import Queue
from threading import Thread
import sys


@contextmanager
def joined_passive_viewer(model, data, *, key_callback=None):
    import mujoco.viewer
    if sys.platform == 'darwin':
        raise RuntimeError('joined native-route passive viewer requires Linux; use the supported viewer backend')
    ready = Queue()
    errors = []

    def render():
        try:
            mujoco.viewer._launch_internal(
                model, data, run_physics_thread=False, handle_return=ready,
                key_callback=key_callback, show_left_ui=True, show_right_ui=True)
        except BaseException as exc:
            errors.append(exc)
            ready.put(exc)

    thread = Thread(target=render, name='native-route-passive-viewer', daemon=False)
    thread.start()
    handle = ready.get()
    if isinstance(handle, BaseException):
        thread.join()
        raise handle
    try:
        yield handle
    finally:
        try:
            handle.close()
        finally:
            thread.join(timeout=10)
        if thread.is_alive():
            raise RuntimeError('passive viewer renderer did not finish destruction')
        if errors:
            raise RuntimeError('passive viewer renderer failed') from errors[0]
