"""tests/test_integration_smoke.py — end-to-end seams + repo-hygiene guards. Built by TC-014.

Tests the contracts *between* components (TC-007/009/010/013's own test files each cover their own
module in isolation) plus the invariants the architecture depends on: the frame invariant end to
end, the naming round-trip, no hard-coded machine paths, the deferred-decision scope guards
(adr/008, adr/009, adr/010), the CLI surface, and clean marker-based skipping. No GPU, no camera, no
downloaded model is ever required; `crackvision.inference` (the one component that shells out to
nnU-Net) is never invoked here — predictions are written directly, per this card's scope.
"""

from __future__ import annotations

import importlib
import json
import re
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from crackvision.config import Config
from crackvision.logging_setup import EXIT_OK
from crackvision.naming import (
    comparison_path,
    derive_case_id,
    mask_path,
    nnunet_input_path,
    overlay_path,
    prediction_path,
    skeleton_overlay_path,
    skeleton_path,
    skeleton_stats_path,
)
from crackvision.prepare_inputs import main as prepare_inputs_main
from crackvision.skeleton import main as skeleton_main
from crackvision.visualize import main as visualize_main

REPO_ROOT = Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------------------
# A/B/C. End-to-end with a stub predictor + the frame invariant + naming round-trip
# ---------------------------------------------------------------------------


def _write_rgb_jpeg(path: Path, size: tuple[int, int]) -> None:
    width, height = size
    rng = np.random.default_rng(1)
    arr = rng.integers(0, 255, size=(height, width, 3), dtype=np.uint8)
    Image.fromarray(arr, mode="RGB").save(path, format="JPEG")


def _write_rgba_png(path: Path, size: tuple[int, int]) -> None:
    Image.new("RGBA", size, (10, 20, 30, 128)).save(path, format="PNG")


def _write_grayscale_png(path: Path, size: tuple[int, int]) -> None:
    rng = np.random.default_rng(2)
    arr = rng.integers(0, 255, size=size[::-1], dtype=np.uint8)
    Image.fromarray(arr, mode="L").save(path, format="PNG")


def _write_stub_prediction(path: Path, width: int, height: int) -> None:
    """A `{0,1}` prediction with a small crack-shaped blob, deterministic per case size."""
    pred = np.zeros((height, width), dtype=np.uint8)
    r0, r1 = height // 4, height // 4 + max(1, height // 10)
    c0, c1 = width // 4, 3 * width // 4
    pred[r0:r1, c0:c1] = 1
    Image.fromarray(pred, mode="L").save(path)


@pytest.fixture
def e2e_project(tmp_project: Config) -> Config:
    """RGB JPEG, RGBA PNG, grayscale PNG and a non-square fixture, run through the real chain."""
    in_dir = tmp_project.paths["input_originals"]
    _write_rgb_jpeg(in_dir / "rgb_case.jpg", (64, 48))
    _write_rgba_png(in_dir / "rgba_case.png", (30, 20))
    _write_grayscale_png(in_dir / "gray_case.png", (22, 14))
    _write_rgb_jpeg(in_dir / "wide_case.jpg", (640, 360))  # non-square: catches transpose bugs

    rc = prepare_inputs_main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    case_map = json.loads((tmp_project.root / "data" / "case_map.json").read_text(encoding="utf-8"))
    for case in case_map["cases"]:
        pred_path = tmp_project.root / prediction_path(case["case_id"])
        _write_stub_prediction(pred_path, case["width"], case["height"])

    rc = visualize_main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK
    rc = skeleton_main(["--root", str(tmp_project.root)])
    assert rc == EXIT_OK

    return tmp_project


def test_end_to_end_stub_predictor_produces_every_expected_file(e2e_project: Config) -> None:
    case_map = json.loads((e2e_project.root / "data" / "case_map.json").read_text(encoding="utf-8"))
    assert {c["source_name"] for c in case_map["cases"]} == {
        "rgb_case.jpg", "rgba_case.png", "gray_case.png", "wide_case.jpg",
    }

    for case in case_map["cases"]:
        case_id = case["case_id"]
        for path in (
            e2e_project.root / nnunet_input_path(case_id),
            e2e_project.root / prediction_path(case_id),
            e2e_project.root / mask_path(case_id),
            e2e_project.root / overlay_path(case_id),
            e2e_project.root / comparison_path(case_id),
            e2e_project.root / skeleton_path(case_id),
            e2e_project.root / skeleton_overlay_path(case_id),
            e2e_project.root / skeleton_stats_path(case_id),
        ):
            assert path.is_file(), f"missing expected output for case {case_id}: {path}"
            assert path.stat().st_size > 0, f"expected output is empty for case {case_id}: {path}"


def test_frame_invariant_end_to_end_including_non_square(e2e_project: Config) -> None:
    case_map = json.loads((e2e_project.root / "data" / "case_map.json").read_text(encoding="utf-8"))
    non_square_seen = False

    for case in case_map["cases"]:
        case_id = case["case_id"]
        original_path = e2e_project.root / case["source_path"]
        with Image.open(original_path) as img:
            original_size = img.size  # (width, height), post-EXIF is fine here (no EXIF in fixtures)

        sizes = {"original": original_size}
        for label, path in (
            ("nnunet_input", nnunet_input_path(case_id)),
            ("prediction", prediction_path(case_id)),
            ("mask", mask_path(case_id)),
            ("overlay", overlay_path(case_id)),
            ("skeleton", skeleton_path(case_id)),
        ):
            with Image.open(e2e_project.root / path) as img:
                sizes[label] = img.size

        assert len(set(sizes.values())) == 1, f"frame invariant violated for case {case_id}: {sizes}"
        if original_size[0] != original_size[1]:
            non_square_seen = True

    assert non_square_seen, "fixture set must include a non-square image to catch transpose bugs"


def test_naming_round_trip_for_every_case(e2e_project: Config) -> None:
    case_map = json.loads((e2e_project.root / "data" / "case_map.json").read_text(encoding="utf-8"))
    for case in case_map["cases"]:
        source_path = e2e_project.root / case["source_path"]
        assert derive_case_id(source_path) == case["case_id"]

        for path in (
            nnunet_input_path(case["case_id"]),
            prediction_path(case["case_id"]),
            mask_path(case["case_id"]),
            overlay_path(case["case_id"]),
            comparison_path(case["case_id"]),
            skeleton_path(case["case_id"]),
            skeleton_overlay_path(case["case_id"]),
            skeleton_stats_path(case["case_id"]),
        ):
            assert (e2e_project.root / path).is_file(), f"naming helper points at a missing file: {path}"


# ---------------------------------------------------------------------------
# D. No hard-coded machine paths (RISKS.md R-15)
# ---------------------------------------------------------------------------

_SCAN_DIRS = ("src", "scripts", "tools", "tests")
_SCAN_FILES = ("env.sh", "run_test.sh")


def _files_to_scan() -> list[Path]:
    files: list[Path] = []
    for d in _SCAN_DIRS:
        directory = REPO_ROOT / d
        if directory.is_dir():
            files.extend(p for p in directory.rglob("*.py") if "__pycache__" not in p.parts)
            files.extend(directory.rglob("*.sh"))
    for name in _SCAN_FILES:
        f = REPO_ROOT / name
        if f.is_file():
            files.append(f)
    return files


def test_no_hard_coded_home_paths() -> None:
    # Built from parts (never spelled out whole here) so a manual grep for the literal pattern
    # over tests/ does not flag this guard's own source as an offender.
    marker = "/" + "home" + "/"
    offenders = []
    for path in _files_to_scan():
        text = path.read_text(encoding="utf-8", errors="replace")
        if marker in text:
            offenders.append(path.relative_to(REPO_ROOT).as_posix())
    assert not offenders, f"hard-coded home path(s) found in: {offenders}"


# ---------------------------------------------------------------------------
# E. Scope guards — keep the deferred decisions deferred
# ---------------------------------------------------------------------------


def test_skeleton_module_has_no_graph_traversal_code() -> None:
    """adr/008: skeleton.py's endpoint is a one-pixel raster, nothing more."""
    text = (REPO_ROOT / "src" / "crackvision" / "skeleton.py").read_text(encoding="utf-8")
    forbidden = re.compile(r"networkx|skan|traverse|ordered_path|prune|spur|longest_path|polyline|spline")
    match = forbidden.search(text)
    assert match is None, f"skeleton.py contains out-of-scope traversal code: {match.group(0)!r}"


def test_no_source_file_imports_ros_or_moveit() -> None:
    """adr/009: this phase's perception code never imports the robotics stack."""
    forbidden_modules = ("rclpy", "rospy", "moveit", "sensor_msgs")
    forbidden_re = re.compile(
        r"^\s*(?:import\s+(" + "|".join(forbidden_modules) + r")\b|from\s+(" + "|".join(forbidden_modules) + r")\b)",
        re.MULTILINE,
    )
    offenders = []
    for directory in ("src",):
        for path in (REPO_ROOT / directory).rglob("*.py"):
            text = path.read_text(encoding="utf-8", errors="replace")
            if forbidden_re.search(text):
                offenders.append(path.relative_to(REPO_ROOT).as_posix())
    assert not offenders, f"source file(s) import the ROS/MoveIt stack: {offenders}"


def test_no_source_file_calls_nnunet_training_entry_points() -> None:
    """adr/010: nnUNetv2_train / nnUNetv2_plan_and_preprocess are never invoked, only named in prose.

    Matched as a quoted string literal (how a real `subprocess` invocation would spell the
    executable name) rather than a bare substring, so prose like inference.py's own module
    docstring ("...nnUNetv2_train is never invoked...") does not trip a false positive.
    """
    forbidden = re.compile(r"['\"]nnUNetv2_train['\"]|['\"]nnUNetv2_plan_and_preprocess['\"]")
    offenders = []
    for path in (REPO_ROOT / "src").rglob("*.py"):
        if forbidden.search(path.read_text(encoding="utf-8", errors="replace")):
            offenders.append(path.relative_to(REPO_ROOT).as_posix())
    assert not offenders, f"source file(s) invoke an nnU-Net training entry point: {offenders}"


def test_no_source_file_uses_weights_only_false() -> None:
    """TC-005 safety rule: checkpoints are only ever loaded with weights_only=True."""
    offenders = []
    for directory in ("src", "scripts", "tools"):
        d = REPO_ROOT / directory
        if not d.is_dir():
            continue
        for path in d.rglob("*.py"):
            if "weights_only=False" in path.read_text(encoding="utf-8", errors="replace"):
                offenders.append(path.relative_to(REPO_ROOT).as_posix())
    assert not offenders, f"weights_only=False found in: {offenders}"


def test_no_source_file_uses_shell_true() -> None:
    offenders = []
    for directory in ("src", "scripts", "tools"):
        d = REPO_ROOT / directory
        if not d.is_dir():
            continue
        for path in d.rglob("*.py"):
            if "shell=True" in path.read_text(encoding="utf-8", errors="replace"):
                offenders.append(path.relative_to(REPO_ROOT).as_posix())
    assert not offenders, f"shell=True found in: {offenders}"


def test_no_source_file_kills_processes_or_resets_the_gpu() -> None:
    """AGENT_INSTRUCTIONS.md rule 13: never kill a process or reset the shared GPU from code."""
    forbidden = re.compile(r"os\.system\(|subprocess\.call\(\s*[\"']kill|pkill|nvidia-smi\s+--gpu-reset")
    offenders = []
    for directory in ("src", "scripts", "tools"):
        d = REPO_ROOT / directory
        if not d.is_dir():
            continue
        for path in d.rglob("*.py"):
            if forbidden.search(path.read_text(encoding="utf-8", errors="replace")):
                offenders.append(path.relative_to(REPO_ROOT).as_posix())
    assert not offenders, f"process-kill / gpu-reset call found in: {offenders}"


# ---------------------------------------------------------------------------
# F. CLI surface: --help exits 0; --dry-run and --verbose are accepted
# ---------------------------------------------------------------------------

# The `crackvision.*` library CLIs and `scripts/*.py` operational scripts that build their own
# argparse parser via `crackvision.config.add_common_args` (docs/INTERFACES.md §0.3): every one of
# them must support `--help`, `--dry-run` and `--verbose`. `scripts/check_host.py` is a documented
# exception (its own docstring: stdlib-only, invoked directly, never via ./env.sh, predates the
# crackvision CLI contract) and `scripts/capture_d405.py` is a thin wrapper with no parser of its
# own (it forwards straight to `crackvision.realsense_capture.main`, exercised below).
CRACKVISION_CLI_MODULES = ("prepare_inputs", "inference", "visualize", "skeleton", "paths", "realsense_capture")
SCRIPT_CLI_FILES = (
    "check_env.py",
    "check_realsense.py",
    "fetch_model.py",
    "verify_model.py",
    "smoke_test.py",
    "summarize_run.py",
)


def _load_script_module(filename: str):
    import sys
    import importlib.util

    path = REPO_ROOT / "scripts" / filename
    name = f"crackvision._cli_surface_{path.stem}"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    # A handful of these scripts use @dataclass, which looks up sys.modules[cls.__module__] during
    # class creation — the module must be registered before exec_module runs (mirrors
    # crackvision.realsense_capture._load_depth_png_helpers's own workaround for the same issue).
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _assert_help_exits_0(build_parser) -> None:
    with pytest.raises(SystemExit) as exc_info:
        build_parser().parse_args(["--help"])
    assert exc_info.value.code == 0


def _assert_accepts_dry_run_and_verbose(build_parser) -> None:
    namespace = build_parser().parse_args(["--dry-run", "--verbose"])
    assert namespace.dry_run is True
    assert namespace.verbose is True


@pytest.mark.parametrize("module_name", CRACKVISION_CLI_MODULES)
def test_crackvision_cli_help_and_common_flags(module_name: str) -> None:
    module = importlib.import_module(f"crackvision.{module_name}")
    _assert_help_exits_0(module.build_parser)
    _assert_accepts_dry_run_and_verbose(module.build_parser)


@pytest.mark.parametrize("filename", SCRIPT_CLI_FILES)
def test_script_cli_help_and_common_flags(filename: str) -> None:
    module = _load_script_module(filename)
    _assert_help_exits_0(module.build_parser)
    _assert_accepts_dry_run_and_verbose(module.build_parser)


def test_capture_d405_wrapper_shares_realsense_capture_parser() -> None:
    """scripts/capture_d405.py has no parser of its own; `main(["--help"])` must still exit 0."""
    module = _load_script_module("capture_d405.py")
    with pytest.raises(SystemExit) as exc_info:
        module.main(["--help"])
    assert exc_info.value.code == 0


def test_check_host_documents_its_own_exemption_from_the_common_flags_contract() -> None:
    """scripts/check_host.py is explicitly stdlib-only and invoked outside ./env.sh; --help still works."""
    module = _load_script_module("check_host.py")
    with pytest.raises(SystemExit) as exc_info:
        module.build_parser().parse_args(["--help"])
    assert exc_info.value.code == 0


# ---------------------------------------------------------------------------
# G. Markers behave: gpu / camera / model tests skip cleanly with the resource absent
# ---------------------------------------------------------------------------


@pytest.mark.gpu
def test_gpu_marked_test_skips_cleanly_without_cuda() -> None:
    torch = pytest.importorskip("torch")
    if not torch.cuda.is_available():
        pytest.skip("no CUDA GPU available")
    assert torch.cuda.device_count() >= 1


@pytest.mark.model
def test_model_marked_test_skips_cleanly_without_a_downloaded_checkpoint() -> None:
    checkpoint = (
        REPO_ROOT
        / "models"
        / "opencrack-nnunet"
        / "Dataset501_OpenCrack"
        / "nnUNetTrainer__nnUNetPlans__2d"
        / "fold_0"
        / "checkpoint_ep0500.pth"
    )
    if not checkpoint.is_file():
        pytest.skip("model checkpoint not downloaded (run scripts/fetch_model.py)")
    assert checkpoint.stat().st_size > 1_000_000  # a real checkpoint, not a placeholder


def test_markers_are_registered(pytestconfig: pytest.Config) -> None:
    """`-m 'not gpu and not camera and not model'` must be a valid selector (markers are registered)."""
    registered = {line.split(":", 1)[0].strip() for line in pytestconfig.getini("markers")}
    assert {"gpu", "camera", "model"} <= registered
