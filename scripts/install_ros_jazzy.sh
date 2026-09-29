#!/usr/bin/env bash
# Ubuntu 24.04 only. Run in your own terminal; sudo asks for Ubuntu authentication.
# Uses the official ROS apt-source release, pinned and SHA256 checked.
# Default: sensor permissions + ROS Jazzy desktop/recording tools. No full OS upgrade.
set -Eeuo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:---install}"
if [[ "$mode" != --install && "$mode" != --sensors-only && "$mode" != --check ]]; then
  echo 'Usage: bash scripts/install_ros_jazzy.sh [--check|--install|--sensors-only]' >&2
  exit 2
fi
. /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 || "$(dpkg --print-architecture)" != amd64 ]]; then
  echo 'This installer is for Ubuntu 24.04 amd64 only.' >&2
  exit 2
fi
ros_deb="$task_dir/.setup/ros2-apt-source_1.2.0.noble_all.deb"
ros_sha='0804d9b13db770eb87019be414cd78378835228ad5fa801fc88758596dd8f7e5'
if [[ "$mode" != --sensors-only ]]; then
  if [[ ! -f "$ros_deb" ]]; then
    echo 'Missing verified ROS repository package in .setup. Ask to prepare it again.' >&2
    exit 2
  fi
  python3 - "$ros_deb" "$ros_sha" <<'PY'
import hashlib, pathlib, sys
if hashlib.sha256(pathlib.Path(sys.argv[1]).read_bytes()).hexdigest() != sys.argv[2]:
    raise SystemExit('ROS repository package checksum mismatch; stopping.')
PY
fi
if [[ "$mode" == --check ]]; then
  echo 'Verified: Ubuntu 24.04 amd64; pinned official ROS repository package.'
  echo 'Changes: add user to dialout/video/plugdev; limited D455 and u-blox USB rules; current-device ACL.'
  echo 'Packages: official ROS apt source, ros-jazzy-desktop, ros-jazzy-rosbag2, ros-dev-tools,'
  echo '          python3-venv, python3-pip, python3-serial, v4l-utils, acl.'
  echo 'No motor commands, firmware upload, OS upgrade, or shell startup edits.'
  exit 0
fi
if (( EUID != 0 )); then
  exec sudo bash "$task_dir/scripts/install_ros_jazzy.sh" "$mode"
fi
task_user="${SUDO_USER:-}"
if [[ -z "$task_user" || "$task_user" == root ]] || ! id "$task_user" >/dev/null 2>&1; then
  echo 'Run this script from your normal Ubuntu user terminal, without switching to a root login.' >&2
  exit 2
fi
if ! command -v setfacl >/dev/null; then
  apt-get update
  apt-get install -y acl
fi
usermod -a -G dialout,video,plugdev "$task_user"
rules_path=/etc/udev/rules.d/60-hlfma-sensors.rules
rules_tmp="$(mktemp)"
trap 'rm -f -- "$rules_tmp"' EXIT
cat > "$rules_tmp" <<'RULES'
# HL FMA: D455 (PID from official librealsense rules), u-blox GNSS receiver.
# Group/active-session access only. No blanket world-writable USB or tty permissions.
SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", MODE="0660", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="1546", ATTR{idProduct}=="01a9", MODE="0660", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0660", GROUP="dialout", TAG+="uaccess"
RULES
if [[ -e "$rules_path" ]] && ! cmp -s "$rules_tmp" "$rules_path"; then
  echo "Existing custom rules differ at $rules_path; stopping without overwriting." >&2
  exit 2
fi
install -m 0644 "$rules_tmp" "$rules_path"
udevadm control --reload-rules
# Grant this already-running user session access to only the connected target sensors.
# No device is opened and no serial data is sent here.
python3 - "$task_user" <<'PY'
import pathlib, subprocess, sys
user = sys.argv[1]
allowed = {('8086', '0b5c'), ('1546', '01a9')}
def read(path):
    try: return path.read_text().strip()
    except OSError: return ''
def grant(path):
    if path.exists():
        subprocess.run(['setfacl', '-m', f'u:{user}:rw-', str(path)], check=True)
        print(f'Sensor access ready: {path}', flush=True)
for dev in pathlib.Path('/sys/bus/usb/devices').iterdir():
    if (read(dev/'idVendor'), read(dev/'idProduct')) in allowed:
        bus, number = read(dev/'busnum'), read(dev/'devnum')
        if bus.isdecimal() and number.isdecimal():
            grant(pathlib.Path(f'/dev/bus/usb/{int(bus):03d}/{int(number):03d}'))
for tty in pathlib.Path('/sys/class/tty').glob('ttyACM*'):
    for ancestor in (tty.resolve(), *tty.resolve().parents):
        if (read(ancestor/'idVendor'), read(ancestor/'idProduct')) == ('1546','01a9'):
            grant(pathlib.Path('/dev')/tty.name)
            break
PY
if [[ "$mode" == --sensors-only ]]; then
  echo 'Sensor access prepared. Reconnect sensors if needed; new group membership applies after login.'
  exit 0
fi
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
if [[ "$(dpkg-query -W -f='${db:Status-Status}' ros2-apt-source 2>/dev/null || true)" != installed ]]; then
  dpkg -i "$ros_deb"
fi
apt-get update
apt-get install -y ros-jazzy-desktop ros-jazzy-rosbag2 ros-dev-tools \
  python3-venv python3-pip python3-serial v4l-utils acl
test -r /opt/ros/jazzy/setup.bash
echo 'ROS Jazzy packages installed. Next: source /opt/ros/jazzy/setup.bash'
echo 'Actual sensor recording and ROS message tests still need to be run.'

