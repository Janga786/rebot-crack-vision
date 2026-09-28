#!/usr/bin/env bash
# scripts/ros/env_ros.sh — scrubbing launcher for the crackvision ROS 2 overlay (MOT-02).
#
# Mirrors env.sh (TC-002 / adr/007): a shell that already has the crackvision conda env
# active leaks a conda PYTHONPATH/PATH into any interpreter it spawns, including ROS 2's
# system python3. This script strips that out, then sources ROS Humble and the
# ~/rebot_ws install/ underlay (read-only — this script never writes to ~/rebot_ws).
#
# Usage:
#   source scripts/ros/env_ros.sh                    # activate in the current shell
#   ./scripts/ros/env_ros.sh ros2 launch ...          # run one command in the scrubbed env

_crackvision_ros_env_setup() {
    local script_path
    if [ -n "${BASH_SOURCE:-}" ]; then
        script_path="${BASH_SOURCE[0]}"
    else
        script_path="$0"
    fi
    local script_dir
    script_dir="$(cd "$(dirname "$script_path")/../.." && pwd)"
    export CRACKVISION_ROOT="$script_dir"

    # Conda vars / conda PYTHONPATH would shadow ROS Humble's system python3 site-packages.
    unset PYTHONPATH
    unset PYTHONHOME
    unset CONDA_PREFIX
    unset CONDA_DEFAULT_ENV
    unset CONDA_SHLVL
    unset CONDA_PYTHON_EXE
    export PYTHONNOUSERSITE=1

    if [ -n "${PATH:-}" ]; then
        local _path_new=""
        local _path_entry
        local IFS=:
        for _path_entry in $PATH; do
            case "$_path_entry" in
                */miniconda3*|*/envs/crackvision*)
                    ;;
                *)
                    if [ -z "$_path_new" ]; then
                        _path_new="$_path_entry"
                    else
                        _path_new="$_path_new:$_path_entry"
                    fi
                    ;;
            esac
        done
        unset IFS
        export PATH="$_path_new"
    fi

    # ROS Humble's setup.bash and ~/rebot_ws's underlay both reference variables that are
    # unset in a fresh shell; relax `set -u` around them, same as env.sh does for conda.
    local _had_nounset=0
    case "$-" in *u*) _had_nounset=1 ;; esac
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    if [ -f "$HOME/rebot_ws/install/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "$HOME/rebot_ws/install/setup.bash"
    else
        echo "env_ros.sh: $HOME/rebot_ws/install/setup.bash not found — has the underlay been built?" >&2
        if [ "$_had_nounset" -eq 1 ]; then set -u; fi
        return 1
    fi
    if [ "$_had_nounset" -eq 1 ]; then
        set -u
    fi

    export ROS_LOCALHOST_ONLY=1

    # ~/.ros can be read-only under sandboxing; keep ROS's own log dir inside this repo's
    # git-ignored overlay area instead of the default ~/.ros/log.
    export ROS_LOG_DIR="$CRACKVISION_ROOT/ros2_ws/log/ros_home"
    mkdir -p "$ROS_LOG_DIR"

    # trac_ik_kinematics_plugin (built into the underlay) dlopen's libnlopt.so.0, which is
    # not on the default system loader path; pick it up from a user-local build if present.
    if [ -d "$HOME/opt/nlopt/lib" ]; then
        export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$HOME/opt/nlopt/lib"
    fi

    return 0
}

_crackvision_ros_env_setup
_crackvision_ros_env_status=$?

if [ $_crackvision_ros_env_status -ne 0 ]; then
    unset -f _crackvision_ros_env_setup
    unset _crackvision_ros_env_status
    return 1 2>/dev/null || exit 1
fi
unset -f _crackvision_ros_env_setup

if [ "$#" -gt 0 ]; then
    exec "$@"
fi
unset _crackvision_ros_env_status
