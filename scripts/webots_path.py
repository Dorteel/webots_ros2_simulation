#!/usr/bin/env python3
"""Read-only Webots check; never invokes the driver's automatic installer."""
import os
from pathlib import Path
import re
import sys


def resolve_webots():
    from webots_ros2_driver.utils import get_webots_home, WebotsVersion
    home = get_webots_home()
    if not home:
        raise RuntimeError('Webots not found. Run ./install-webots.sh or set ROS2_WEBOTS_HOME / WEBOTS_HOME.')
    home = Path(home)
    executable = home / 'webots'
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise RuntimeError(f'Webots executable is missing or not executable: {executable}')
    version = WebotsVersion.from_path(str(home))
    if not version or not re.match(r'^R2025a(?:\s|$)', str(version)):
        raise RuntimeError(f'Unsupported Webots {version} at {home}; this repository targets R2025a (including revisions).')
    if version < WebotsVersion.minimum():
        raise RuntimeError(f'The installed ROS driver requires {WebotsVersion.minimum()}, newer than this Webots. Use a compatible Jazzy driver.')
    os.environ['WEBOTS_HOME'] = str(home)
    return executable, str(version)


if __name__ == '__main__':
    try:
        executable, version = resolve_webots()
        print(f'Webots {version}: {executable}')
    except (ImportError, RuntimeError, ValueError, IndexError) as error:
        print(f'Webots prerequisite unavailable: {error}', file=sys.stderr)
        sys.exit(2)
