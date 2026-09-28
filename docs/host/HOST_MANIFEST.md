# HOST_MANIFEST.md — host version pinning and drift checking (HOST-01)

**Status:** NORMATIVE. `config/host_versions.json` is the pinned record; `scripts/check_host.py` is
the automated check that proves the live host still matches it and that ROS, perception (the
`crackvision` conda env) and local inference stay isolated from each other.

## 1. Why this exists

REQ-HOST-1 asks for a reproducible host setup with pinned, recorded versions and an automated drift
check. REQ-HOST-2 asks for automated proof that ROS (system Python 3.10), the `crackvision` conda
env (Python 3.11) and local inference never mix interpreters or site-packages — this is the same
leak documented as F-1/F-2/F-3 in `docs/MACHINE_STATE.md` §4, just checked continuously instead of
being a one-time inspection.

## 2. Running it

```bash
/usr/bin/python3 -sE scripts/check_host.py --manifest config/host_versions.json          # compare (default)
/usr/bin/python3 -sE scripts/check_host.py --manifest config/host_versions.json --json    # machine-readable
/usr/bin/python3 -sE scripts/check_host.py --manifest config/host_versions.json --record  # rewrite the manifest
```

`check_host.py` is run with `/usr/bin/python3 -sE` (system Python, stdlib-only, no site dir, no
inherited `PYTHONPATH`/env) — **not** through `./env.sh` — because it has to inspect both stacks
from outside either of them. It never installs, downloads, `sudo`s, or writes anything except the
manifest file, and only when `--record` is passed.

Exit codes follow `docs/INTERFACES.md` §0.2: `0` on match, `1` on drift or an isolation failure,
`2` on a malformed manifest, `3` when the manifest file is missing.

## 3. What is pinned, and where each value comes from

| Manifest field | Live source | Drift treatment |
|---|---|---|
| `os.{id,version_id,pretty_name}` | `/etc/os-release` | hard fail |
| `kernel` | `platform.release()` | **warn-only** — kernel updates land via unattended-upgrades and don't affect this project |
| `nvidia_driver` | `nvidia-smi --query-gpu=driver_version` | hard fail (`null` if `nvidia-smi` can't reach a driver, e.g. inside a sandbox with no GPU device node — that is itself the live state to compare against) |
| `cuda_toolkits` | sorted list of `/usr/local/cuda-*` directory names | hard fail |
| `ros.distro` | `$ROS_DISTRO` after sourcing `/opt/ros/humble/setup.bash` | hard fail |
| `ros.packages.*` | `dpkg-query -W` for `ros-humble-{desktop,moveit,ros2-control,realsense2-camera,librealsense2}` | hard fail |
| `gz_sim` | `gz sim --version` | hard fail |
| `crackvision.{python,torch,nnunetv2,pyrealsense2,numpy}` | `./env.sh python -c '...importlib.metadata...'` | hard fail |
| `claude_cli` | `claude --version` | **warn-only** — the CLI self-updates |
| `bwrap` | `bwrap --version` | hard fail |

Fields present on only one side (manifest or live) are also drift, unless the field is warn-only.

## 4. Isolation checks (REQ-HOST-2)

Run every time, independent of the manifest comparison:

1. `crackvision_no_ros_on_syspath` — `./env.sh python -c "any('/opt/ros' in p for p in sys.path)"` must
   be `False`.
2. `ros_python_imports_rclpy_moveit` — `/usr/bin/python3` after sourcing Humble must `import rclpy,
   moveit_msgs` cleanly.
3. `no_shared_site_packages` — the `site-packages`/`dist-packages` entries on each interpreter's
   `sys.path` must be disjoint sets.

Any isolation failure makes the whole run exit `1`, same as a hard version drift.

## 5. Updating the pin deliberately

When you intentionally upgrade something (e.g. a new `ros-humble-moveit` release, a `torch` bump in
`crackvision`), re-record after the upgrade and commit the new manifest in the same change:

```bash
/usr/bin/python3 -sE scripts/check_host.py --manifest config/host_versions.json --record
git diff config/host_versions.json   # review exactly what moved before committing
```

Never hand-edit `config/host_versions.json` to make a drift warning go away — `--record` is the only
sanctioned way to move the pin, and it should always be a reviewed, intentional diff.

## 6. Unit tests

`tests/test_check_host.py` exercises `compare_manifest()` (pure — no I/O) with hand-built
manifest/live dicts, and `main()` with `collect_live_inventory`/`run_isolation_checks` monkeypatched.
It never shells out to `dpkg`, `nvidia-smi`, `./env.sh`, or ROS — the real collectors and isolation
checks are exercised by actually running the script (§2), not by the automated suite.
