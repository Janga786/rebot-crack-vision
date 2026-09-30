# GEOM-08.4.R1 — Repair GEOM-08.4: §9.4 image-dimension consistency refusal rule is never enforced

```json card
{
  "kind": "repair",
  "requirements": [
    "REQ-GEOM-2",
    "REQ-CAM-2"
  ],
  "scope": {
    "write": [
      "src/crackvision/capture_record.py",
      "tests/test_capture_record.py"
    ]
  },
  "inputs": [
    "docs/INTERFACES.md#0",
    "docs/INTERFACES.md#3.10",
    "docs/INTERFACES.md#8.4",
    "docs/INTERFACES.md#9",
    "src/crackvision/kinematics.py",
    "src/crackvision/geometry.py",
    "src/crackvision/config.py",
    "src/crackvision/logging_setup.py",
    "src/crackvision/paths.py"
  ],
  "acceptance": {
    "checks": [
      {
        "id": "unit",
        "cmd": "./env.sh pytest tests/test_capture_record.py -q -p no:cacheprovider",
        "timeout_s": 300,
        "expect_exit": 0
      },
      {
        "id": "cli-help",
        "cmd": "./env.sh python -m crackvision.capture_record --help",
        "timeout_s": 60,
        "expect_exit": 0
      },
      {
        "id": "suite",
        "cmd": "./env.sh pytest tests/ -q -p no:cacheprovider",
        "timeout_s": 1200,
        "expect_exit": 0
      }
    ],
    "criteria": [
      "load_capture_record (or a validate-time check invoked by the CLI) compares record.image_hw against the actual colour image dimensions, the aligned depth PNG dimensions, the mask dimensions, and paths.json's image_height/image_width when those artefacts are available for the case, raising CaptureRecordError with a rule-identifying reason text on any mismatch, with a test that constructs each mismatch and asserts the refusal.",
      "Add a test that writes a case_map.json with downscaled: true for the target case_id and asserts load_capture_record raises CaptureRecordError with a reason mentioning the downscale/§0.5 guarantee, and a test that sets kinematic_model.sha256 to a wrong hex string and asserts refusal with a reason mentioning the sha mismatch.",
      "All acceptance criteria of GEOM-08.4 still hold at the new revision"
    ]
  },
  "routing": {
    "complexity": 2,
    "ambiguity": 2,
    "context": 3,
    "consequence": 3,
    "task_class": "impl-library",
    "local_ok": false
  },
  "track": "calibration",
  "priority": 95,
  "repairs": "GEOM-08.4",
  "findings": [
    "60cee26367521d66",
    "92d9d42c0ff85788"
  ],
  "id": "GEOM-08.4.R1",
  "title": "Repair GEOM-08.4: §9.4 image-dimension consistency refusal rule is never enforced",
  "parent": "GEOM-08",
  "outcome": "Resolve the review findings on GEOM-08.4 while every acceptance criterion of GEOM-08.4 still holds."
}
```

Repair work generated deterministically from review attempt `00182-GEOM-08.4-review` of `GEOM-08.4`.
Original card: `plan/cards/GEOM-08.4.md` — its acceptance criteria must still hold.

## Finding 1 [major] §9.4 image-dimension consistency refusal rule is never enforced
- Evidence: src/crackvision/capture_record.py's load_capture_record never compares doc['image']['height']/['width'] against the colour image, the aligned depth PNG, the mask, or paths.json's image_height/image_width — grep for 'paths.json' or 'mask' in the file returns nothing outside build_record's own dict construction. load_depth performs a shape check but is a separate function never called from load_capture_record, so it is not part of record validation.
- Consequence: A capture record whose declared image.height/width silently disagrees with the actual colour image, mask, or paths.json (the exact corruption §0.5's frame invariant exists to prevent) will load and validate successfully instead of being refused, and can be fed into downstream lifting despite violating the frame invariant this contract is meant to guard.
- Affected: src/crackvision/capture_record.py, tests/test_capture_record.py
- Acceptance condition: load_capture_record (or a validate-time check invoked by the CLI) compares record.image_hw against the actual colour image dimensions, the aligned depth PNG dimensions, the mask dimensions, and paths.json's image_height/image_width when those artefacts are available for the case, raising CaptureRecordError with a rule-identifying reason text on any mismatch, with a test that constructs each mismatch and asserts the refusal.

## Finding 2 [major] case_map.json downscaled and kinematic_model.sha256-mismatch refusal rules ship with zero tests
- Evidence: capture_record.py implements both checks (the case_map 'downscaled' lookup and the chain.urdf_sha256 vs kinematic_model.sha256 comparison), but tests/test_capture_record.py has no test exercising either path — confirmed by grepping the test file for case_map/downscaled/kinematic_model.sha256, which only turns up an unrelated build_record kwarg name. The implementer's own notes_for_reviewer explicitly concedes this gap.
- Consequence: The card's own acceptance criterion 'Every §9 refusal rule has a test that triggers it and checks the reason text' is unmet for two named rules, so a future regression in either check (e.g. an off-by-something in the sha comparison, or a schema-shape change in case_map.json) would go undetected by the test suite.
- Affected: tests/test_capture_record.py
- Acceptance condition: Add a test that writes a case_map.json with downscaled: true for the target case_id and asserts load_capture_record raises CaptureRecordError with a reason mentioning the downscale/§0.5 guarantee, and a test that sets kinematic_model.sha256 to a wrong hex string and asserts refusal with a reason mentioning the sha mismatch.
