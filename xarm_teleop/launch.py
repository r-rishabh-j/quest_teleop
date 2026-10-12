"""Use mjpython on macOS, including uv-managed Python's shared-library path."""
import os
from pathlib import Path
import sys


def server():
    if sys.platform == "darwin":
        executable = Path(sys.executable).parent / "mjpython"
        if not executable.is_file():
            raise SystemExit("mjpython is missing; run uv sync")
        env = os.environ.copy()
        library_path = str(Path(sys.base_prefix) / "lib")
        env["DYLD_LIBRARY_PATH"] = library_path + (":" + env["DYLD_LIBRARY_PATH"] if env.get("DYLD_LIBRARY_PATH") else "")
        os.execve(str(executable), [str(executable), "-m", "xarm_teleop.server", *sys.argv[1:]], env)
    else:
        from .server import main
        main()
