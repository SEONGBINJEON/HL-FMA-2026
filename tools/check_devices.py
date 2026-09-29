#!/usr/bin/env python3
"""Read Linux device metadata without opening serial ports or capturing images.

Uses only the Python standard library. By default, prints JSON to stdout.
--summary prints a short Korean view; --output always saves the full JSON report.
An output path is created exclusively: an existing report is never overwritten.
"""

import argparse
from datetime import datetime, timezone
import grp
import json
import os
from pathlib import Path
import platform
import shutil
import stat
import sys


def read_text(path):
    try:
        return Path(path).read_text(encoding="utf-8", errors="replace").strip()
    except (OSError, UnicodeError):
        return None


def children(path):
    try:
        return sorted(Path(path).iterdir(), key=lambda item: item.name)
    except OSError:
        return []


def permissions(path):
    """Inspect metadata/access only; never open the device."""
    try:
        info = Path(path).stat()
    except OSError as exc:
        return {"present": False, "error": type(exc).__name__}
    try:
        group = grp.getgrgid(info.st_gid).gr_name
    except KeyError:
        group = str(info.st_gid)
    return {
        "present": True,
        "character_device": stat.S_ISCHR(info.st_mode),
        "mode": stat.filemode(info.st_mode),
        "group": group,
        "readable_by_current_process": os.access(path, os.R_OK),
        "writable_by_current_process": os.access(path, os.W_OK),
    }


def usb_metadata(path):
    vendor_id = read_text(path / "idVendor")
    product_id = read_text(path / "idProduct")
    if vendor_id is None or product_id is None:
        return None
    manufacturer = read_text(path / "manufacturer")
    product = read_text(path / "product")
    label = " ".join(value for value in (manufacturer, product) if value).lower()
    hints = []
    # These are literal product-string matches, not a VID/PID model database.
    if "realsense" in label:
        hints.append("product_string_mentions_realsense")
    if "u-blox" in label or "ublox" in label:
        hints.append("product_string_mentions_u_blox")
    return {
        "sysfs_device": path.name,
        "vendor_id": vendor_id.lower(),
        "product_id": product_id.lower(),
        "manufacturer": manufacturer,
        "product": product,
        "serial": read_text(path / "serial"),
        "bus_number": read_text(path / "busnum"),
        "device_number": read_text(path / "devnum"),
        "speed_mbps": read_text(path / "speed"),
        "identification_hints": hints,
    }


def usb_parent(path):
    try:
        resolved = path.resolve(strict=True)
    except (OSError, RuntimeError):
        return None
    for parent in (resolved, *resolved.parents):
        metadata = usb_metadata(parent)
        if metadata is not None:
            return metadata["sysfs_device"]
    return None


def device_inventory(sys_root=Path("/sys"), dev_root=Path("/dev")):
    usb_root = sys_root / "bus/usb/devices"
    video_root = sys_root / "class/video4linux"
    tty_root = sys_root / "class/tty"
    usb = [item for path in children(usb_root)
           if (item := usb_metadata(path)) is not None]

    stable_paths = []
    for directory in ("serial/by-id", "serial/by-path"):
        for link in children(dev_root / directory):
            try:
                target = link.resolve(strict=False)
            except (OSError, RuntimeError):
                target = None
            stable_paths.append({
                "path": str(link),
                "target": str(target) if target is not None else None,
                "permissions": permissions(link),
            })

    # Include USB serial devices and conventional onboard UART names; do not
    # claim every tty is a GNSS receiver. Also include custom stable-link targets.
    tty_names = {
        path.name for path in children(tty_root)
        if path.name.startswith(("ttyACM", "ttyUSB", "ttyAMA", "ttyS"))
    }
    tty_names.update(Path(item["target"]).name for item in stable_paths
                     if item["target"] is not None)
    serial_ports = []
    for name in sorted(tty_names):
        node = dev_root / name
        serial_ports.append({
            "device": str(node),
            "usb_parent": usb_parent(tty_root / name),
            "stable_paths": [item["path"] for item in stable_paths
                             if item["target"] == str(node)],
            "permissions": permissions(node),
        })

    video_nodes = [{
        "device": str(dev_root / path.name),
        "name": read_text(path / "name"),
        "index": read_text(path / "index"),
        "usb_parent": usb_parent(path),
        "permissions": permissions(dev_root / path.name),
    } for path in children(video_root) if path.name.startswith("video")]

    return {
        "scan_paths": {"usb": str(usb_root), "tty": str(tty_root),
                       "video4linux": str(video_root)},
        "sysfs_available": {
            "usb": usb_root.is_dir(), "tty": tty_root.is_dir(),
            "video4linux": video_root.is_dir(),
        },
        "usb_devices": usb,
        "serial_ports": serial_ports,
        "serial_stable_paths": stable_paths,
        "video_nodes": video_nodes,
    }


def ros_inventory(ros_root=Path("/opt/ros"), executable_lookup=shutil.which):
    installations = []
    for path in children(ros_root):
        setup = [str(path / name) for name in ("setup.bash", "setup.sh")
                 if (path / name).is_file()]
        if setup:
            installations.append({"directory": str(path), "setup_files": setup})
    executables = {name: executable_lookup(name)
                   for name in ("ros2", "roscore", "rosbag")}
    if any(executables.values()):
        status = "commands_found_on_path"
    elif installations:
        status = "setup_files_found_commands_not_on_path"
    else:
        status = "not_detected_in_checked_locations"
    return {
        "status": status,
        "installations": installations,
        "executables_on_path": executables,
        "runtime_working": "unknown_not_executed",
        "scope": "Only /opt/ros setup files and command paths; custom installs may be missed.",
    }


def os_inventory(os_release=Path("/etc/os-release")):
    fields = {}
    for line in (read_text(os_release) or "").splitlines():
        key, separator, value = line.partition("=")
        if separator and key in {"ID", "VERSION_ID", "PRETTY_NAME"}:
            fields[key.lower()] = value.strip().strip('"').strip("'")
    return {"system": platform.system(), "kernel_release": platform.release(),
            "distribution": fields}


def current_groups():
    groups = []
    for group_id in sorted(set(os.getgroups()) | {os.getegid()}):
        try:
            name = grp.getgrgid(group_id).gr_name
        except KeyError:
            name = str(group_id)
        groups.append(name)
    return {"active_groups": groups, "dialout_active": "dialout" in groups,
            "note": "Active process groups only; a new login may be needed after group changes."}


def collect_report():
    usage = shutil.disk_usage(Path.cwd())
    return {
        "schema_version": 1,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "inspection_mode": "metadata_only_no_serial_open_no_image_capture",
        "os": os_inventory(),
        "architecture": platform.machine(),
        "python_version": platform.python_version(),
        "disk": {"scope": "filesystem_containing_current_working_directory",
                 "total_bytes": usage.total, "used_bytes": usage.used,
                 "free_bytes": usage.free},
        "groups": current_groups(),
        "ros": ros_inventory(),
        "devices": device_inventory(),
        "limitations": [
            "USB strings are reported as supplied by devices; no model is inferred from an unverified VID/PID.",
            "V4L nodes may be built-in MIPI processors, metadata nodes, or multiple nodes for one camera; they are not a camera count.",
            "No V4L node is assumed to be a RealSense D455.",
            "Access flags do not prove a device is functional, available, configured, or unused by another process.",
            "No GNSS fix, RTK state, camera image, or motor function is tested.",
        ],
    }


def format_summary(report):
    """A compact view of observed metadata, without inferring sensor health."""
    def label(value, fallback="미제공"):
        if value is None or value == "":
            return fallback
        return " ".join("".join(c if c.isprintable() else " " for c in str(value)).split())

    def access(info, key):
        if not info.get("present", False):
            return "경로 없음/접근 미확인"
        value = info.get(key)
        return "가능" if value is True else "불가" if value is False else "미확인"

    inventory = report.get("devices", {})
    usb = inventory.get("usb_devices", [])
    usb_by_id = {item["sysfs_device"]: item for item in usb}
    serial = inventory.get("serial_ports", [])
    video = inventory.get("video_nodes", [])
    usb_serial = [item for item in serial
                  if item.get("usb_parent") and item.get("permissions", {}).get("present")]
    usb_video = [item for item in video if item.get("usb_parent")]
    non_usb_video = sum(not item.get("usb_parent") for item in video)
    ros = report.get("ros", {})
    distribution = report.get("os", {}).get("distribution", {})
    lines = ["장치 연결 요약 | " + label(report.get("timestamp_utc")),
             label(distribution.get("pretty_name"), "OS 미확인") + " | " +
             label(report.get("architecture"), "구조 미확인")]
    free_bytes = report.get("disk", {}).get("free_bytes")
    if free_bytes is not None:
        lines.append(f"현재 작업 폴더의 디스크 여유: {free_bytes / 1024 ** 3:.1f} GiB")
    groups = report.get("groups", {})
    dialout = groups.get("dialout_active")
    lines.append("현재 실행 환경의 dialout 그룹: " +
                 ("활성" if dialout is True else "비활성" if dialout is False else "미확인") +
                 " (아래 포트 접근 여부는 ACL 권한도 반영)")

    lines.extend(["", f"USB 장치: {len(usb)}개 (허브·내장 장치 포함)"])
    for item in usb:
        name = label(item.get("product"), "제품명 미제공")
        manufacturer = item.get("manufacturer")
        if manufacturer and str(manufacturer).lower() not in name.lower():
            name += " / " + label(manufacturer)
        lines.append("- " + label(item.get("sysfs_device")) + " | " + name + " | " +
                     label(item.get("vendor_id")) + ":" + label(item.get("product_id")) +
                     " | USB 연결속도 " + label(item.get("speed_mbps"), "미확인") + " Mbps")
    if not usb:
        lines.append("- 이번 메타데이터 검사에서 USB 장치를 확인하지 못했습니다.")

    lines.extend(["", f"USB에 연결된 시리얼 포트: {len(usb_serial)}개"])
    for item in usb_serial:
        info = item.get("permissions", {})
        product = usb_by_id.get(item["usb_parent"], {}).get("product")
        lines.append("- " + label(item.get("device")) + " | " + label(product, "제품명 미확인") +
                     " | 읽기 " + access(info, "readable_by_current_process") +
                     ", 쓰기 " + access(info, "writable_by_current_process"))
        paths = item.get("stable_paths", [])
        lines.extend("  고정 경로: " + label(path) for path in paths)
        if not paths:
            lines.append("  고정 경로: 미검출")
    if not usb_serial:
        lines.append("- USB 부모와 실제 장치 경로가 모두 확인된 포트가 없습니다.")
    omitted_serial = len(serial) - len(usb_serial)
    if omitted_serial:
        lines.append(f"  그 외 시리얼 노드 {omitted_serial}개는 전체 JSON에만 표시합니다.")

    lines.extend(["", f"USB 연결 영상 노드: {len(usb_video)}개 (노드 수는 카메라 대수가 아님)"])
    for item in usb_video:
        product = usb_by_id.get(item["usb_parent"], {}).get("product")
        lines.append("- " + label(item.get("device")) + " | " + label(item.get("name"), "노드 이름 미제공") +
                     " | USB 제품 " + label(product, "미확인") + " | 읽기 " +
                     access(item.get("permissions", {}), "readable_by_current_process"))
    if not usb_video:
        lines.append("- 이번 검사에서 USB 부모에 연결된 V4L 영상 노드를 찾지 못했습니다.")
    lines.append(f"USB 부모 미확인 영상 노드: {non_usb_video}개")
    lines.append("  내장 MIPI·영상 처리·메타데이터 노드 등이 포함될 수 있습니다. 이 개수로 카메라 모델이나 D455 연결을 판단하지 않습니다.")

    lines.extend(["", "ROS 설치 흔적"])
    setup_files = [path for item in ros.get("installations", [])
                   for path in item.get("setup_files", [])]
    lines.extend("- 설정 파일: " + label(path) for path in setup_files)
    if not setup_files:
        lines.append("- /opt/ros 설정 파일 미검출 (다른 경로의 설치는 미확인)")
    commands = {name: path for name, path in ros.get("executables_on_path", {}).items() if path}
    lines.extend("- 현재 찾을 수 있는 명령: " + label(name) + " → " + label(path)
                 for name, path in commands.items())
    if not commands:
        lines.append("- 현재 실행 경로에서 ROS 명령 미검출")
    lines.append("- ROS 실행·통신 정상 여부: 미확인 (실행하지 않음)")
    unavailable = [name for name, available in inventory.get("sysfs_available", {}).items() if not available]
    if unavailable:
        lines.append("검사 경로 미확인: " + ", ".join(unavailable))
    lines.extend(["", "미검출은 고장이나 기능 실패 판정이 아닙니다. 포트 열기·영상 촬영·GPS/RTK·모터 동작 검사는 하지 않았습니다."])
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        help="New JSON report path (existing files are refused).")
    parser.add_argument("--summary", action="store_true",
                        help="짧은 한국어 요약 출력. --output과 함께 쓰면 파일에는 전체 JSON 저장")
    args = parser.parse_args(argv)
    try:
        report = collect_report()
        payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(payload)
            print("Saved device metadata report: " + str(args.output))
        if args.summary:
            sys.stdout.write(format_summary(report))
        elif args.output is None:
            sys.stdout.write(payload)
    except (OSError, ValueError) as exc:
        print("Device report failed: " + str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
