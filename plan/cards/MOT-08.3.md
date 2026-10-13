# MOT-08.3 — crackvision.trajectory_preview CLI: deterministic headless PNG render (joints, tool path + crack + scene, tool speed) and manifest

```json card
{
  "kind": "impl",
  "depends_on": [
    "MOT-08.2"
  ],
  "requirements": [
    "REQ-MOT-2",
    "REQ-OPS-1"
  ],
  "scope": {
    "write": [
      "src/crackvision/trajectory_preview.py",
      "tests/test_trajectory_preview.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#18",
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#10",
    "src/crackvision/preview_core.py",
    "src/crackvision/visualize_paths.py",
    "config/scene/scene.yaml",
    "config/motion/execution.yaml"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_trajectory_preview.py -q -p no:cacheprovider",
        "timeout_s": 900,
        "expect_exit": 0
      },
      {
        "id": "help",
        "cmd": "bash -c 'env -u DISPLAY ./env.sh python -m crackvision.trajectory_preview --help'",
        "timeout_s": 60,
        "expect_exit": 0
      },
      {
        "id": "independence",
        "cmd": "bash -c '! grep -nE \"^\\s*(from|import)\\s+(crackvision_motion|rclpy)\" src/crackvision/trajectory_preview.py && ! grep -n \"plt.show\" src/crackvision/trajectory_preview.py'",
        "timeout_s": 30,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "The CLI accepts the §0.3 common flags plus --trajectory, --plan, --retime-report, --check-report, --limits, --end-effector-config, --scene-config, --execution-config, --urdf and --out-dir, with the §18 defaults. Exit codes are 0/1/2/3 per §0.2 and §18. --dry-run binds inputs and prints the summary without writing anything.",
      "joints.png shows q, finite-difference v and finite-difference a per joint against dashed limit lines, with leg spans shaded by kind. toolpath.png shows the tool-tip path in XY/XZ/YZ projections coloured by leg kind, start/end markers, scene box outlines from scene.yaml, and the paths3d crack polylines when the sha-verified file exists (otherwise a manifest warning). toolspeed.png shows unscaled and preview-scaled tool speed with each trace leg's v_c band.",
      "A failing check report renders a red 'CHECK FAILED' banner on every PNG and sets all_pass false. The run still exits 0.",
      "Rendering the same inputs twice gives identical PNG sha256 values (tested). Runs with DISPLAY unset because matplotlib is forced to Agg before pyplot is imported.",
      "Writes only under --out-dir, default data/previews/{case}_{sha8}/. Tests write only to tmp_path."
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "python-cli-render",
    "local_ok": false
  },
  "track": "simulation",
  "id": "MOT-08.3",
  "title": "crackvision.trajectory_preview CLI: deterministic headless PNG render (joints, tool path + crack + scene, tool speed) and manifest",
  "parent": "MOT-08",
  "outcome": "`./env.sh python -m crackvision.trajectory_preview` writes data/previews/{case}_{sha8}/ with joints.png, toolpath.png, toolspeed.png and a valid preview.json. It works headless and renders byte-identically when re-run."
}
```

Implement src/crackvision/trajectory_preview.py: the CLI plus matplotlib rendering on top of crackvision.preview_core (§18).

- Call `matplotlib.use("Agg")` before importing pyplot. Set `MPLCONFIGDIR` to a writable temp dir if the default is not writable (the host's ~/.config/matplotlib is not). Never show a window.
- Follow the CLI conventions in crackvision.trajectory_check: `add_common_args`, `setup_logging`, `RunSummary` and the exit constants from crackvision.logging_setup.
- Determinism: fixed figure size and DPI, `savefig(..., metadata={"Software": None})` (or whatever §18 prescribes), no timestamps drawn in images, sorted iteration.
- Scene boxes: read config/scene/scene.yaml (`objects[]` with `shape: box`, `dimensions_m`, `pose.position_m`, `pose.rpy_rad`). Draw the projected outline of the rotated box corners. Skip non-box shapes and record a warning for each.
- Crack polylines: when the trajectory's `source.paths3d` is non-null and the file at that path (resolved against --root) has the matching sha, draw each component's robot-frame points (§10). Otherwise add a manifest warning. View-move trajectories have no paths3d, so draw only the path and the scene.
- Write the PNGs first, hash them, then write preview.json last via `preview_core.build_manifest`. Validate it before writing; if validation fails, that is exit 1 and the directory is removed.

Tests: build a small synthetic case in tmp_path (reuse the preview_core test helpers' approach) and run `main([...])` in-process. Assert:
- the exit code;
- the three PNGs exist and are non-empty;
- the manifest validates and its artefact shas match the files;
- a second run gives identical PNG shas;
- `--dry-run` writes nothing;
- a check-report sha mismatch gives exit 3 and no directory;
- a failing check report gives exit 0 with all_pass false.
