#!/usr/bin/env bash
# scripts/rebuild_env.sh — recreate the crackvision conda env from its lock files. Built by HOST-02.
#
# Never touches the existing 'crackvision' env: it always creates/updates a NEW env
# (default name 'crackvision-rebuild', override with --name). Use --dry-run to print
# the exact commands without running anything.
#
# Usage:
#   bash scripts/rebuild_env.sh --dry-run [--name NAME]
#   bash scripts/rebuild_env.sh [--name NAME]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

NEW_ENV_NAME="crackvision-rebuild"
DRY_RUN=0

while [ "$#" -gt 0 ]; do
    case "$1" in
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        --name)
            NEW_ENV_NAME="$2"
            shift 2
            ;;
        --root)
            ROOT_DIR="$2"
            shift 2
            ;;
        -h|--help)
            grep '^#' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "rebuild_env.sh: unknown argument '$1'" >&2
            exit 2
            ;;
    esac
done

if [ "$NEW_ENV_NAME" = "crackvision" ]; then
    echo "rebuild_env.sh: refusing to target the existing 'crackvision' env; pick a --name" >&2
    exit 2
fi

CONDA_LOCK="$ROOT_DIR/requirements/crackvision.conda-explicit.txt"
PIP_LOCK="$ROOT_DIR/requirements/crackvision.pip-freeze.txt"

for f in "$CONDA_LOCK" "$PIP_LOCK"; do
    if [ ! -f "$f" ]; then
        echo "rebuild_env.sh: missing lock file $f" >&2
        exit 3
    fi
done

CONDA_BIN="$(command -v conda || true)"
if [ -z "$CONDA_BIN" ]; then
    for candidate in "$HOME/miniconda3/bin/conda" "$HOME/anaconda3/bin/conda"; do
        if [ -x "$candidate" ]; then
            CONDA_BIN="$candidate"
            break
        fi
    done
fi
if [ -z "$CONDA_BIN" ] && [ "$DRY_RUN" -eq 0 ]; then
    echo "rebuild_env.sh: conda executable not found" >&2
    exit 3
fi
CONDA_BIN="${CONDA_BIN:-conda}"

CREATE_CMD=("$CONDA_BIN" create --name "$NEW_ENV_NAME" --file "$CONDA_LOCK" --yes)
PIP_CMD=("$CONDA_BIN" run -n "$NEW_ENV_NAME" python -m pip install -r "$PIP_LOCK")

echo "# Commands to recreate the crackvision environment as '$NEW_ENV_NAME'"
echo "# (the existing 'crackvision' env is never modified by this script)"
printf '%q ' "${CREATE_CMD[@]}"; echo
printf '%q ' "${PIP_CMD[@]}"; echo

if [ "$DRY_RUN" -eq 1 ]; then
    echo "# --dry-run: no commands were executed."
    exit 0
fi

"${CREATE_CMD[@]}"
"${PIP_CMD[@]}"

echo "rebuild_env.sh: env '$NEW_ENV_NAME' created. Verify with:"
echo "  conda run -n $NEW_ENV_NAME python -m pip check"
