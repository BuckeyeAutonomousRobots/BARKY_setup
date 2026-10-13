"""Find the VS Code PlatformIO installation without machine-specific project paths."""
from pathlib import Path
import os
import shutil
import subprocess
import sys


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in {'build', 'upload', 'monitor'}:
        raise SystemExit('Usage: python pio_task.py build|upload|monitor')
    executable = shutil.which('pio') or shutil.which('platformio')
    if executable is None:
        core = Path(os.environ.get('PLATFORMIO_CORE_DIR', str(Path.home() / '.platformio')))
        candidate = core / 'penv' / ('Scripts/platformio.exe' if os.name == 'nt' else 'bin/platformio')
        if candidate.is_file():
            executable = str(candidate)
    if executable is None:
        raise SystemExit('PlatformIO not found. Install PlatformIO IDE in this VS Code window, '
                         'wait for setup, then use PlatformIO: Build or its Core CLI terminal.')
    action = sys.argv[1]
    arguments = {'build': ['run'], 'upload': ['run', '--target', 'upload'],
                 'monitor': ['device', 'monitor']}[action]
    try:
        return subprocess.call([executable, *arguments], cwd=Path(__file__).resolve().parents[1])
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
