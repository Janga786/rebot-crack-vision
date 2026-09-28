#!/usr/bin/env bash
# run_test.sh — the one-command workflow: data/input_originals/ -> data/comparisons/. Built by TC-011.
#
# Orchestrates prepare_inputs -> inference -> visualize -> skeleton -> summarize_run, stopping at the
# first stage that fails. This script only orchestrates: it never changes a stage's behaviour, and it
# never touches the camera (docs/INTERFACES.md §3.11).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DEVICE=""
SKIP_SKELETON=0
CLEAN=0

usage() {
    cat <<'EOF'
Usage: run_test.sh [--device cuda|cpu] [--skip-skeleton] [--clean] [--help]

Runs the whole crack-vision chain on data/input_originals/ and prints a summary table.

  --device cuda|cpu   inference device, passed to crackvision.inference (default: config/project.yaml)
  --skip-skeleton      skip the skeletonization stage (stage 5); summary shows "-" for its columns
  --clean               remove existing data/nnunet_input/*.png before converting inputs
  --help, -h            show this message and exit 0

Stages: check_env -> prepare_inputs -> inference -> visualize -> skeleton -> summarize_run.
Put images in data/input_originals/ first, then run this script, then open data/comparisons/.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --device)
            DEVICE="${2:-}"
            if [ -z "$DEVICE" ]; then
                echo "run_test.sh: --device requires an argument (cuda|cpu)" >&2
                exit 2
            fi
            shift 2
            ;;
        --device=*)
            DEVICE="${1#--device=}"
            shift
            ;;
        --skip-skeleton)
            SKIP_SKELETON=1
            shift
            ;;
        --clean)
            CLEAN=1
            shift
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "run_test.sh: unrecognized argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [ -n "$DEVICE" ] && [ "$DEVICE" != "cuda" ] && [ "$DEVICE" != "cpu" ]; then
    echo "run_test.sh: --device must be cuda or cpu, got: $DEVICE" >&2
    exit 2
fi

friendly_msg() {
    case "$1" in
        "check_env")
            echo "Environment not ready — see the FAIL rows above."
            ;;
        "prepare_inputs")
            echo "No input images. Put .jpg/.png files in data/input_originals/ and re-run."
            ;;
        *)
            echo "see the log above for details."
            ;;
    esac
}

run_stage() {
    local name="$1"
    shift
    echo
    echo "── $name ──"
    set +e
    "$@"
    local rc=$?
    set -e
    if [ $rc -eq 3 ]; then
        echo
        echo "STOPPED: $name — $(friendly_msg "$name")"
        exit 3
    fi
    if [ $rc -ne 0 ]; then
        echo
        echo "FAILED: $name (exit $rc)"
        exit $rc
    fi
}

START_TIME=$(date +%s)

run_stage "check_env" "$ROOT/env.sh" python "$ROOT/scripts/check_env.py"

PREPARE_ARGS=()
if [ "$CLEAN" -eq 1 ]; then
    PREPARE_ARGS+=(--clean)
fi
run_stage "prepare_inputs" "$ROOT/env.sh" python -m crackvision.prepare_inputs "${PREPARE_ARGS[@]}"

INFERENCE_ARGS=()
if [ -n "$DEVICE" ]; then
    INFERENCE_ARGS+=(--device "$DEVICE")
fi
run_stage "inference" "$ROOT/env.sh" python -m crackvision.inference "${INFERENCE_ARGS[@]}"

run_stage "visualize" "$ROOT/env.sh" python -m crackvision.visualize

if [ "$SKIP_SKELETON" -eq 0 ]; then
    run_stage "skeleton" "$ROOT/env.sh" python -m crackvision.skeleton
else
    echo
    echo "── skeleton ── (skipped: --skip-skeleton)"
fi

run_stage "summarize_run" "$ROOT/env.sh" python "$ROOT/scripts/summarize_run.py"

END_TIME=$(date +%s)
echo
echo "Total elapsed: $((END_TIME - START_TIME))s"
