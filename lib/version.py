"""Single source of truth for the PiLiDAR version.

Read from ``version.txt`` at the repo root (the file the release process bumps),
so the CLI, captures and logs all report the same number.
"""
import os

_VERSION_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "version.txt")


def _read_version():
    try:
        with open(_VERSION_FILE) as f:
            return f.read().strip() or "0.0.0"
    except OSError:
        return "0.0.0"


__version__ = _read_version()
