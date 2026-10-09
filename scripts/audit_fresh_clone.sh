#!/usr/bin/env bash
# scripts/audit_fresh_clone.sh — fresh-clone + from-lock reproducibility audit. Built by HOST-06.1.
#
# Two independent, non-destructive halves:
#
#   1. Path-relocatability: materialises a full working copy of HEAD at a path OUTSIDE this repo
#      (via `git worktree add`, falling back to `git clone --no-hardlinks` if worktree metadata
#      can't be written — e.g. a read-only .git under some sandboxes) and re-runs the fast
#      reproducibility surface from there. The already-fetched model weights under models/ and the
#      already-built ~/opt/llama.cpp are reused via a symlink/fixed path, never re-downloaded or
#      rebuilt — that reproducibility is HOST-02's / LLM-02's job, already proven.
#   2. Lock sufficiency: creates a NEW, disposable conda env (`crackvision-audit`) from nothing but
#      the committed lock files (requirements/crackvision.conda-explicit.txt +
#      requirements/crackvision.pip-freeze.txt) and confirms it installs cleanly.
#
# Both the relocated copy and the conda env are removed in a trap that fires on success AND
# failure. This script never passes --record to check_host.py, never writes
# config/host_versions.json, and never touches the real `crackvision` conda env. It regenerates
# config/llm_runtime.json / docs/llm/RUNTIME.md only inside the relocated copy, never in this repo.
#
# Usage:
#   ./scripts/audit_fresh_clone.sh
#
# Exit codes (docs/INTERFACES.md §0.2): 0 success, 1 any check/step failed, 2 usage error.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

if [ "$#" -gt 0 ]; then
    echo "audit_fresh_clone.sh: takes no arguments" >&2
    exit 2
fi

CONDA_SH="$HOME/miniconda3/etc/profile.d/conda.sh"
AUDIT_ENV_NAME="crackvision-audit"
RELOCATED_DIR=""
RELOCATED_IS_WORKTREE=0
AUDIT_ENV_CREATED=0
OVERALL_STATUS=0

log() { printf '[audit_fresh_clone] %s\n' "$*"; }

cleanup() {
    local rc=$?
    if [ "$AUDIT_ENV_CREATED" -eq 1 ]; then
        log "removing temporary conda env '$AUDIT_ENV_NAME'"
        set +u
        source "$CONDA_SH"
        set -u
        conda env remove --name "$AUDIT_ENV_NAME" -y >/dev/null 2>&1 || true
    fi
    if [ -n "$RELOCATED_DIR" ] && [ -d "$RELOCATED_DIR" ]; then
        if [ "$RELOCATED_IS_WORKTREE" -eq 1 ]; then
            log "removing git worktree at $RELOCATED_DIR"
            git -C "$ROOT_DIR" worktree remove --force "$RELOCATED_DIR" >/dev/null 2>&1 || rm -rf "$RELOCATED_DIR"
        else
            log "removing relocated clone at $RELOCATED_DIR"
            rm -rf "$RELOCATED_DIR"
        fi
    fi
    exit "$rc"
}
trap cleanup EXIT

run_step() {
    # run_step <label> <command...>
    local label="$1"
    shift
    log "RUN: $label"
    if "$@"; then
        log "PASS: $label (exit 0)"
    else
        local rc=$?
        log "FAIL: $label (exit $rc)"
        OVERALL_STATUS=1
    fi
}

# ---------------------------------------------------------------------------
# Half 1: path-relocatability
# ---------------------------------------------------------------------------
RELOCATED_DIR="$(mktemp -d /tmp/crackvision-audit-relocated.XXXXXX)"
rmdir "$RELOCATED_DIR"

log "attempting: git worktree add $RELOCATED_DIR HEAD"
if git -C "$ROOT_DIR" worktree add "$RELOCATED_DIR" HEAD >/dev/null 2>&1; then
    RELOCATED_IS_WORKTREE=1
    log "using git worktree at $RELOCATED_DIR"
else
    log "git worktree add failed (likely a read-only .git in this sandbox) — falling back to git clone --no-hardlinks"
    if git clone --no-hardlinks --quiet "$ROOT_DIR" "$RELOCATED_DIR"; then
        log "using git clone at $RELOCATED_DIR"
    else
        log "FAIL: both git worktree add and git clone failed"
        OVERALL_STATUS=1
        RELOCATED_DIR=""
    fi
fi

if [ -n "$RELOCATED_DIR" ] && [ -d "$RELOCATED_DIR" ]; then
    # models/ and nnunet/{raw,preprocessed,results} are untracked (git-ignored) so neither
    # `worktree add` nor `clone` populates them. Reuse the already-fetched weights via a symlink
    # instead of re-downloading; verify_model.py itself (re-)creates the nnunet/results symlink.
    rm -rf "$RELOCATED_DIR/models"
    ln -s "$ROOT_DIR/models" "$RELOCATED_DIR/models"

    cd "$RELOCATED_DIR"
    run_step "relocated: check_env.py"                 ./env.sh python scripts/check_env.py
    run_step "relocated: verify_model.py"               ./env.sh python scripts/verify_model.py --check-hashes
    run_step "relocated: smoke_test.py"                 ./env.sh python scripts/smoke_test.py --device cpu
    run_step "relocated: pytest"                        ./env.sh pytest tests/ -v
    run_step "relocated: check_host.py"                 /usr/bin/python3 -sE scripts/check_host.py --manifest config/host_versions.json
    run_step "relocated: verify_lock.py"                ./env.sh python scripts/verify_lock.py
    run_step "relocated: build_llama_cpp.sh (no-op path)" scripts/llm/build_llama_cpp.sh
    cd "$ROOT_DIR"
fi

# ---------------------------------------------------------------------------
# Half 2: lock sufficiency via a disposable conda env built only from the lock files
# ---------------------------------------------------------------------------
CONDA_LOCK="$ROOT_DIR/requirements/crackvision.conda-explicit.txt"
PIP_LOCK="$ROOT_DIR/requirements/crackvision.pip-freeze.txt"

set +u
source "$CONDA_SH"
set -u

if conda env list | grep -q "^$AUDIT_ENV_NAME "; then
    log "FAIL: conda env '$AUDIT_ENV_NAME' already exists — refusing to touch it"
    OVERALL_STATUS=1
else
    log "RUN: conda create --name $AUDIT_ENV_NAME --file $CONDA_LOCK"
    if conda create --name "$AUDIT_ENV_NAME" --file "$CONDA_LOCK" -y; then
        AUDIT_ENV_CREATED=1
        log "PASS: conda create from conda-explicit lock"
        # --extra-index-url is required: pip-freeze.txt pins torch==*+cu126, a build that only
        # exists on download.pytorch.org, not on PyPI (docs/RUNBOOK.md step 3 does the same for
        # the live env). A bare `pip install -r` of this lock file alone 404s on torch and fails
        # the whole install — a from-scratch-only failure mode `verify_lock.py` can never see.
        # --no-input plus a dedicated, neutral --cwd avoid pip's interactive vcs prompt that fires
        # when the editable `-e git+...#egg=crackvision` line's default `./src/crackvision` clone
        # target collides with this repo's own `src/` directory.
        PIP_WORKDIR="$(mktemp -d /tmp/crackvision-audit-pip.XXXXXX)"
        log "RUN: conda run -n $AUDIT_ENV_NAME pip install --no-input --extra-index-url https://download.pytorch.org/whl/cu126 -r $PIP_LOCK (cwd=$PIP_WORKDIR)"
        if conda run -n "$AUDIT_ENV_NAME" --cwd "$PIP_WORKDIR" python -m pip install --no-input \
            --extra-index-url https://download.pytorch.org/whl/cu126 -r "$PIP_LOCK"; then
            log "PASS: pip install from pip-freeze lock"
        else
            log "FAIL: pip install from pip-freeze lock"
            OVERALL_STATUS=1
        fi
        rm -rf "$PIP_WORKDIR"
    else
        log "FAIL: conda create from conda-explicit lock"
        OVERALL_STATUS=1
    fi
fi

if [ "$OVERALL_STATUS" -eq 0 ]; then
    log "AUDIT OK — path-relocatability and lock sufficiency both confirmed"
else
    log "AUDIT FAILED — see FAIL lines above"
fi

exit "$OVERALL_STATUS"
