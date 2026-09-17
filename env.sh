#!/usr/bin/env bash
# env.sh — scrubbing launcher for the crackvision conda environment. Built by TC-002.
#
# ROS Humble is unconditionally sourced from ~/.bashrc and exports a Python 3.10
# PYTHONPATH that leaks into any interpreter, including this project's Python 3.11
# env (verified — docs/MACHINE_STATE.md §4, adr/007). A plain `conda activate crackvision`
# is not sufficient; this script scrubs the inherited environment before activating.
#
# Usage:
#   source env.sh                          # activate in the current shell
#   ./env.sh python -m crackvision.foo     # run one command in a scrubbed env

_crackvision_env_setup() {
    # Resolve the project root from this script's own location so the project stays
    # relocatable — no hard-coded absolute user path.
    local script_path
    if [ -n "${BASH_SOURCE:-}" ]; then
        script_path="${BASH_SOURCE[0]}"
    else
        script_path="$0"
    fi
    local script_dir
    script_dir="$(cd "$(dirname "$script_path")" && pwd)"
    export CRACKVISION_ROOT="$script_dir"

    # F-1: ROS 3.10 PYTHONPATH leak.
    unset PYTHONPATH

    # F-2: stale CUDA 11.8 / ROS entries on LD_LIBRARY_PATH can shadow the torch
    # wheel's bundled CUDA runtime. Filter them out; keep everything else.
    if [ -n "${LD_LIBRARY_PATH:-}" ]; then
        local _ld_new=""
        local _ld_entry
        local IFS=:
        for _ld_entry in $LD_LIBRARY_PATH; do
            case "$_ld_entry" in
                */opt/ros*|*/usr/local/cuda-11.8*)
                    ;;
                *)
                    if [ -z "$_ld_new" ]; then
                        _ld_new="$_ld_entry"
                    else
                        _ld_new="$_ld_new:$_ld_entry"
                    fi
                    ;;
            esac
        done
        unset IFS
        export LD_LIBRARY_PATH="$_ld_new"
    fi

    # F-3 defence in depth: ~/.local user-site pollution.
    unset PYTHONHOME
    export PYTHONNOUSERSITE=1

    # Activate the crackvision conda env. The conda hook references unset
    # variables internally, so relax `set -u` around it if it is active.
    local _had_nounset=0
    case "$-" in *u*) _had_nounset=1 ;; esac
    set +u
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
    conda activate crackvision
    local _activate_status=$?
    if [ "$_had_nounset" -eq 1 ]; then
        set -u
    fi
    if [ $_activate_status -ne 0 ]; then
        echo "env.sh: failed to activate conda env 'crackvision'. Has it been created? See task_cards/TC-002-python-environment.md." >&2
        return 1
    fi

    # Project-local nnU-Net roots (adr/004).
    export PYTHONNOUSERSITE=1
    export nnUNet_raw="$CRACKVISION_ROOT/nnunet/raw"
    export nnUNet_preprocessed="$CRACKVISION_ROOT/nnunet/preprocessed"
    export nnUNet_results="$CRACKVISION_ROOT/nnunet/results"
    export nnUNet_compile=f

    return 0
}

_crackvision_env_setup
_crackvision_env_status=$?

if [ $_crackvision_env_status -ne 0 ]; then
    unset -f _crackvision_env_setup
    unset _crackvision_env_status
    return 1 2>/dev/null || exit 1
fi
unset -f _crackvision_env_setup

if [ "$#" -gt 0 ]; then
    exec "$@"
fi
unset _crackvision_env_status
