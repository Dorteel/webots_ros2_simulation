#!/usr/bin/env bash
# Official Cyberbotics APT method, restricted to the repository's R2025a release.
# This script deliberately has no unattended-confirmation flag.
set -eo pipefail
if [[ ${1:-} == --help ]]; then
    echo 'Usage: ./install-webots.sh (Ubuntu 24.04 amd64; interactive INSTALL confirmation)'
    exit 0
fi
[[ $# == 0 ]] || { echo 'Unexpected arguments; see --help.' >&2; exit 2; }
fail() { echo "$*" >&2; exit 2; }
source /etc/os-release
[[ $ID == ubuntu && $VERSION_ID == 24.04 && $(dpkg --print-architecture) == amd64 ]] ||
    fail 'Supported platform: Ubuntu 24.04 amd64 only.'
[[ ${ROS_DISTRO:-jazzy} == jazzy ]] || fail 'Use ROS 2 Jazzy, in a clean shell.'
[[ -r /opt/ros/jazzy/setup.bash ]] || fail 'Install ROS 2 Jazzy before running this script.'
source /opt/ros/jazzy/setup.bash
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
installed_home=
for directory in "${ROS2_WEBOTS_HOME:-}" "${WEBOTS_HOME:-}" /usr/local/webots /usr/share/webots; do
    if [[ -x $directory/webots && -r $directory/resources/version.txt ]]; then
        installed_home=$directory
        break
    fi
done
if [[ -n $installed_home ]]; then
    echo "Existing Webots: $(cat "$installed_home/resources/version.txt") at $installed_home"
fi
compatible=false
if [[ -n $installed_home ]] && grep -Eq '^R2025a([[:space:]]|$)' "$installed_home/resources/version.txt"; then
    compatible=true
fi
bridge_installed=false
if [[ $(dpkg-query -W -f='${Status}' ros-jazzy-webots-ros2 2>/dev/null || true) == 'install ok installed' ]]; then
    bridge_installed=true
fi
report() {
    python3 "$ROOT/scripts/webots_path.py" || return 2
    dpkg-query -W -f='${Package} ${Version}\n' webots ros-jazzy-webots-ros2 2>/dev/null || true
    python3 - <<'PY'
from ament_index_python.packages import get_package_prefix
for package in ('webots_ros2_driver', 'webots_ros2_control'):
    print(f'{package}: {get_package_prefix(package)}')
PY
}
if $compatible && $bridge_installed; then
    echo 'Compatible Webots and ROS bridge already installed; no APT changes needed.'
    report
    exit
fi
echo 'This will use sudo to install prerequisites, configure a Cyberbotics signed-by'
echo 'APT source if Webots is needed, update APT, and install Webots R2025a plus'
echo 'ros-jazzy-webots-ros2. Existing unrelated APT configurations will not be edited.'
read -r -p 'Type INSTALL to authorize these system changes: ' answer || exit 2
[[ $answer == INSTALL ]] || { echo 'Cancelled; no system changes made.'; exit 2; }
sudo apt-get update -o APT::Update::Error-Mode=any || fail 'APT update failed. Check network and existing repository configuration; retry when available.'
if ! $compatible; then
    # Refuse conflicting/duplicate sources rather than changing someone else's file.
    python3 - <<'PY' || exit 2
from pathlib import Path
own = Path('/etc/apt/sources.list.d/tiago-cyberbotics.list')
files = [Path('/etc/apt/sources.list')]
folder = Path('/etc/apt/sources.list.d')
files += list(folder.glob('*.list')) + list(folder.glob('*.sources'))
for path in files:
    if path != own and path.exists():
        for line in path.read_text().splitlines():
            if not line.lstrip().startswith('#') and 'cyberbotics.com/debian' in line:
                raise SystemExit(f'Existing Cyberbotics source in {path}. Review/consolidate it manually before retrying; no source was overwritten.')
PY
    sudo apt-get install ca-certificates curl gnupg || fail 'Could not install HTTPS/key tools; check APT/network.'
    temporary=$(mktemp -d)
    trap 'rm -rf -- "$temporary"' EXIT
    curl --fail --show-error --location --proto '=https' --proto-redir '=https' \
        --connect-timeout 15 --max-time 60 --retry 2 \
        https://cyberbotics.com/Cyberbotics.asc -o "$temporary/Cyberbotics.asc" ||
        fail 'Cyberbotics signing key download failed. Check network/proxy; no Cyberbotics configuration was written.'
    gpg --batch --show-keys "$temporary/Cyberbotics.asc" >/dev/null || fail 'Downloaded signing key is invalid.'
    printf '%s\n' 'deb [arch=amd64 signed-by=/etc/apt/keyrings/tiago-cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/' > "$temporary/tiago-cyberbotics.list"
    for item in /etc/apt/keyrings/tiago-cyberbotics.asc /etc/apt/sources.list.d/tiago-cyberbotics.list; do
        original="$temporary/$(basename "$item")"
        [[ $item != *.asc ]] || original="$temporary/Cyberbotics.asc"
        if [[ -e $item ]] && ! cmp -s "$item" "$original"; then
            fail "Refusing to replace differing existing file: $item. Review it manually."
        fi
    done
    sudo install -d -m 0755 /etc/apt/keyrings
    sudo install -m 0644 "$temporary/Cyberbotics.asc" /etc/apt/keyrings/tiago-cyberbotics.asc
    sudo install -m 0644 "$temporary/tiago-cyberbotics.list" /etc/apt/sources.list.d/tiago-cyberbotics.list
    sudo apt-get update -o APT::Update::Error-Mode=any || fail 'Cyberbotics APT update failed; check network/signature errors and retry.'
    # Never silently move to a newer release or downgrade an existing newer one.
    version=$(apt-cache madison webots | awk '$3 ~ /^(R)?2025a([+~.:-]|$)/ {print $3; exit}')
    [[ -n $version ]] || fail 'No Webots R2025a package is available from APT. Use the official R2025a .deb/manual method in README.md.'
    sudo apt-get install "webots=$version" || fail 'Webots install failed; check APT errors (no automatic downgrade is allowed).'
fi
sudo apt-get install ros-jazzy-webots-ros2 || fail 'ROS bridge installation failed. Check the official ROS 2 Jazzy APT source and network.'
report || fail 'Installation verification failed. Check stale ROS2_WEBOTS_HOME/WEBOTS_HOME overrides and the installed driver version.'
