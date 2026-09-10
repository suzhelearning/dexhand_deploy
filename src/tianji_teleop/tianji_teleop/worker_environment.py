"""Keep pinned worker imports/DSOs independent of the main inference runtime."""
import os


def isolated_worker_environment():
    # Each worker uses an absolute executable, pinned loader paths and explicit
    # package roots. Never modify the parent process's inference environment.
    excluded = {'PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'LD_PRELOAD'}
    return {key: value for key, value in os.environ.items() if key not in excluded}
