#!/usr/bin/env python3
"""scripts/check_host.py — host version manifest + drift/isolation checker. Built by HOST-01.

Compares the live host inventory (OS, kernel, NVIDIA driver, CUDA toolkits, ROS/gz packages, the
crackvision conda env, the claude CLI, bwrap) against config/host_versions.json, and asserts that
ROS (system py3.10), the crackvision conda env (py3.11) and local inference stay isolated from each
other.

Stdlib only — this script is invoked directly as `/usr/bin/python3 -sE scripts/check_host.py`, never
through ./env.sh, so it must not import anything from the crackvision package or third-party
packages. It only ever reads state; `--record` is the one flag that writes, and it writes only the
manifest file named by `--manifest`.
"""

from __future__ import annotations

import argparse
import json
import platform
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

EXIT_OK = 0
EXIT_RUNTIME = 1
EXIT_USAGE = 2
EXIT_PRECONDITION = 3

SCHEMA_VERSION = 1

# Fields whose drift is reported but does not fail the run — the host is expected to drift here
# (kernel updates land via unattended-upgrades; the claude CLI self-updates).
WARN_ONLY_FIELDS = {"kernel", "claude_cli"}

ROS_PACKAGES = [
    "ros-humble-desktop",
    "ros-humble-moveit",
    "ros-humble-ros2-control",
    "ros-humble-realsense2-camera",
    "ros-humble-librealsense2",
]

CRACKVISION_PACKAGES = ["torch", "nnunetv2", "pyrealsense2", "numpy"]

REPO_ROOT = Path(__file__).resolve().parents[1]


def _run(cmd: list[str], **kwargs) -> subprocess.CompletedProcess | None:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=30, **kwargs)
    except (OSError, subprocess.SubprocessError):
        return None


# --- live collectors (real system calls; exercised manually, not by the unit test suite) -----------


def read_os_release() -> dict[str, str]:
    path = Path("/etc/os-release")
    data: dict[str, str] = {}
    if not path.exists():
        return data
    for line in path.read_text().splitlines():
        if "=" in line and not line.startswith("#"):
            key, _, value = line.partition("=")
            data[key.strip()] = value.strip().strip('"')
    return data


def dpkg_version(pkg: str) -> str | None:
    result = _run(["/usr/bin/dpkg-query", "-W", "-f=${Version}", pkg])
    if result is None or result.returncode != 0:
        return None
    value = result.stdout.strip()
    return value or None


def nvidia_driver_version() -> str | None:
    exe = shutil.which("nvidia-smi")
    if exe is None:
        return None
    result = _run([exe, "--query-gpu=driver_version", "--format=csv,noheader"])
    if result is None or result.returncode != 0:
        return None
    lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    return lines[0] if lines else None


def cuda_toolkits() -> list[str]:
    base = Path("/usr/local")
    versions = []
    if base.exists():
        for entry in sorted(base.glob("cuda-*")):
            if entry.is_dir() and not entry.is_symlink():
                versions.append(entry.name[len("cuda-") :])
    return versions


def gz_sim_version() -> str | None:
    exe = shutil.which("gz")
    if exe is None:
        return None
    result = _run([exe, "sim", "--version"])
    if result is None:
        return None
    text = (result.stdout or result.stderr).strip()
    return text.splitlines()[0] if text else None


def ros_distro() -> str | None:
    result = _run(
        ["bash", "-lc", "set +u; source /opt/ros/humble/setup.bash 2>/dev/null; echo \"$ROS_DISTRO\""]
    )
    if result is None or result.returncode != 0:
        return None
    value = result.stdout.strip()
    return value or None


def crackvision_versions(root: Path) -> dict[str, str | None]:
    code = (
        "import json, sys, importlib.metadata as m\n"
        "out = {'python': '%d.%d.%d' % sys.version_info[:3]}\n"
        f"for pkg in {CRACKVISION_PACKAGES!r}:\n"
        "    try:\n"
        "        out[pkg] = m.version(pkg)\n"
        "    except m.PackageNotFoundError:\n"
        "        out[pkg] = None\n"
        "print(json.dumps(out))\n"
    )
    result = _run([str(root / "env.sh"), "python", "-c", code], cwd=str(root))
    if result is None or result.returncode != 0:
        return {}
    try:
        return json.loads(result.stdout.strip().splitlines()[-1])
    except (json.JSONDecodeError, IndexError):
        return {}


def claude_version() -> str | None:
    exe = shutil.which("claude")
    if exe is None:
        return None
    result = _run([exe, "--version"])
    if result is None or result.returncode != 0:
        return None
    text = (result.stdout or result.stderr).strip()
    return text.splitlines()[0] if text else None


def bwrap_version() -> str | None:
    exe = shutil.which("bwrap")
    if exe is None:
        return None
    result = _run([exe, "--version"])
    if result is None or result.returncode != 0:
        return None
    text = result.stdout.strip()
    return text.splitlines()[0] if text else None


def collect_live_inventory(root: Path) -> dict:
    os_release = read_os_release()
    return {
        "os": {
            "id": os_release.get("ID", ""),
            "version_id": os_release.get("VERSION_ID", ""),
            "pretty_name": os_release.get("PRETTY_NAME", ""),
        },
        "kernel": platform.release(),
        "nvidia_driver": nvidia_driver_version(),
        "cuda_toolkits": cuda_toolkits(),
        "ros": {
            "distro": ros_distro(),
            "packages": {pkg: dpkg_version(pkg) for pkg in ROS_PACKAGES},
        },
        "gz_sim": gz_sim_version(),
        "crackvision": crackvision_versions(root),
        "claude_cli": claude_version(),
        "bwrap": bwrap_version(),
    }


# --- isolation checks (real subprocesses; exercised manually, not by the unit test suite) ----------


@dataclass
class IsolationResult:
    name: str
    ok: bool
    detail: str


def check_crackvision_no_ros(root: Path) -> IsolationResult:
    result = _run(
        [str(root / "env.sh"), "python", "-c", "import sys; print(any('/opt/ros' in p for p in sys.path))"],
        cwd=str(root),
    )
    if result is None or result.returncode != 0:
        return IsolationResult("crackvision_no_ros_on_syspath", False, "could not run ./env.sh python")
    lines = result.stdout.strip().splitlines()
    ok = bool(lines) and lines[-1] == "False"
    return IsolationResult("crackvision_no_ros_on_syspath", ok, result.stdout.strip() or "(no output)")


def check_ros_python_imports() -> IsolationResult:
    result = _run(
        [
            "bash",
            "-lc",
            "set +u; source /opt/ros/humble/setup.bash 2>/dev/null; "
            "/usr/bin/python3 -c 'import rclpy, moveit_msgs'",
        ]
    )
    ok = result is not None and result.returncode == 0
    if ok:
        detail = "rclpy, moveit_msgs import OK"
    else:
        detail = (result.stderr.strip() if result and result.stderr else "subprocess failed")
    return IsolationResult("ros_python_imports_rclpy_moveit", ok, detail)


def _site_packages_entries(paths: list[str]) -> set[str]:
    return {p for p in paths if "site-packages" in p or "dist-packages" in p}


def check_no_shared_site_packages(root: Path) -> IsolationResult:
    cv_result = _run(
        [str(root / "env.sh"), "python", "-c", "import sys, json; print(json.dumps(sys.path))"], cwd=str(root)
    )
    ros_result = _run(
        [
            "bash",
            "-lc",
            "set +u; source /opt/ros/humble/setup.bash 2>/dev/null; "
            "/usr/bin/python3 -c 'import sys, json; print(json.dumps(sys.path))'",
        ]
    )
    if cv_result is None or cv_result.returncode != 0 or ros_result is None or ros_result.returncode != 0:
        return IsolationResult(
            "no_shared_site_packages", False, "could not enumerate sys.path for one or both interpreters"
        )
    try:
        cv_paths = _site_packages_entries(json.loads(cv_result.stdout.strip().splitlines()[-1]))
        ros_paths = _site_packages_entries(json.loads(ros_result.stdout.strip().splitlines()[-1]))
    except (json.JSONDecodeError, IndexError):
        return IsolationResult("no_shared_site_packages", False, "could not parse sys.path JSON")
    shared = cv_paths & ros_paths
    ok = not shared
    detail = "(disjoint)" if ok else f"shared: {sorted(shared)}"
    return IsolationResult("no_shared_site_packages", ok, detail)


def run_isolation_checks(root: Path) -> list[IsolationResult]:
    return [
        check_crackvision_no_ros(root),
        check_ros_python_imports(),
        check_no_shared_site_packages(root),
    ]


# --- comparison logic (pure; this is what the unit test suite exercises with fake inventories) -----


@dataclass
class FieldDiff:
    field: str
    recorded: object
    live: object
    warn_only: bool

    @property
    def matches(self) -> bool:
        return self.recorded == self.live


def _flatten(prefix: str, value: object) -> dict[str, object]:
    if isinstance(value, dict):
        out: dict[str, object] = {}
        for key, sub_value in value.items():
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            out.update(_flatten(child_prefix, sub_value))
        return out
    return {prefix: value}


def compare_manifest(manifest: dict, live: dict) -> list[FieldDiff]:
    """Pure diff between a recorded manifest's `versions` block and a live inventory dict.

    Both inputs are plain (possibly nested) dicts of JSON-serialisable scalars/lists — no I/O here,
    which is what lets the unit tests exercise this with fake inventories.
    """
    recorded_flat = _flatten("", manifest.get("versions", {}))
    live_flat = _flatten("", live)
    keys = sorted(set(recorded_flat) | set(live_flat))
    diffs = []
    for key in keys:
        top_field = key.split(".", 1)[0]
        diffs.append(
            FieldDiff(
                field=key,
                recorded=recorded_flat.get(key),
                live=live_flat.get(key),
                warn_only=top_field in WARN_ONLY_FIELDS,
            )
        )
    return diffs


# --- reporting ---------------------------------------------------------------------------------


def _print_diff_table(diffs: list[FieldDiff]) -> None:
    field_width = max(len("FIELD"), *(len(d.field) for d in diffs)) if diffs else len("FIELD")
    print(f"{'FIELD':<{field_width}}  {'STATUS':<11}  {'RECORDED':<30}  LIVE")
    for diff in diffs:
        if diff.matches:
            status = "MATCH"
        elif diff.warn_only:
            status = "WARN-DRIFT"
        else:
            status = "DRIFT"
        print(f"{diff.field:<{field_width}}  {status:<11}  {str(diff.recorded):<30}  {diff.live}")


def _print_isolation_table(results: list[IsolationResult]) -> None:
    name_width = max(len("CHECK"), *(len(r.name) for r in results)) if results else len("CHECK")
    print(f"{'CHECK':<{name_width}}  STATUS  DETAIL")
    for result in results:
        status = "OK" if result.ok else "FAIL"
        print(f"{result.name:<{name_width}}  {status:<6}  {result.detail}")


# --- CLI -----------------------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--manifest", type=Path, default=REPO_ROOT / "config" / "host_versions.json", help="manifest path"
    )
    parser.add_argument(
        "--record", action="store_true", help="overwrite the manifest with the live host inventory and exit"
    )
    parser.add_argument("--json", action="store_true", help="emit a machine-readable JSON report")
    parser.add_argument("--root", type=Path, default=REPO_ROOT, help="project root (locates ./env.sh)")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.record:
        live = collect_live_inventory(args.root)
        payload = {
            "schema_version": SCHEMA_VERSION,
            "recorded_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "versions": live,
        }
        args.manifest.parent.mkdir(parents=True, exist_ok=True)
        args.manifest.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        print(f"recorded live host inventory to {args.manifest}")
        return EXIT_OK

    if not args.manifest.exists():
        print(f"manifest not found: {args.manifest} (run with --record first)", file=sys.stderr)
        return EXIT_PRECONDITION

    try:
        manifest = json.loads(args.manifest.read_text())
    except json.JSONDecodeError as exc:
        print(f"manifest is not valid JSON: {exc}", file=sys.stderr)
        return EXIT_USAGE

    if not isinstance(manifest, dict):
        print("manifest must be a JSON object", file=sys.stderr)
        return EXIT_USAGE

    live = collect_live_inventory(args.root)
    diffs = compare_manifest(manifest, live)
    isolation = run_isolation_checks(args.root)

    hard_failures = [d for d in diffs if not d.matches and not d.warn_only]
    isolation_failures = [r for r in isolation if not r.ok]
    ok = not hard_failures and not isolation_failures

    if args.json:
        report = {
            "status": "ok" if ok else "drift",
            "diffs": [{**asdict(d), "matches": d.matches} for d in diffs],
            "isolation": [asdict(r) for r in isolation],
        }
        print(json.dumps(report, indent=2))
    else:
        _print_diff_table(diffs)
        print()
        _print_isolation_table(isolation)
        print()
        print("OK — host matches config/host_versions.json, isolation intact" if ok else "DRIFT DETECTED")

    return EXIT_OK if ok else EXIT_RUNTIME


if __name__ == "__main__":
    sys.exit(main())
