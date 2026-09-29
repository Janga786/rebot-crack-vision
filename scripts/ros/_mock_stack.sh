#!/usr/bin/env bash
# scripts/ros/_mock_stack.sh — sourced helper factoring out the launch/pgid/cleanup pattern
# shared by test_mock_plan.sh (MOT-02) and test_reachability.sh (MOT-04.4).
#
# Usage (from a script that has already `set -m` for job control and sourced env_ros.sh +
# the overlay's install/setup.bash):
#   source scripts/ros/_mock_stack.sh
#   start_mock_stack "$LOG_DIR/some_launch.log"   # backgrounds the launch, installs the trap
#   ...                                            # run CLIs against the mock stack
#   stop_mock_stack                                # SIGINT the group, wait, SIGKILL fallback
#
# `start_mock_stack` requires the caller's shell to already have `set -m` (job control) so the
# backgrounded launch gets its own process group, separate from the caller script's — without
# that, signalling "the launch's process group" during cleanup would also signal the caller.

_MOCK_STACK_PID=""
_MOCK_STACK_PGID=""

start_mock_stack() {
    local launch_log="$1"
    ros2 launch crackvision_motion mock_planning.launch.py > "$launch_log" 2>&1 &
    _MOCK_STACK_PID=$!
    sleep 1
    _MOCK_STACK_PGID=$(ps -o pgid= -p "$_MOCK_STACK_PID" 2>/dev/null | tr -d ' ')
    if [ -z "$_MOCK_STACK_PGID" ]; then
        echo "_mock_stack.sh: launch process $_MOCK_STACK_PID exited immediately; log follows:" >&2
        cat "$launch_log" >&2
        return 1
    fi

    trap stop_mock_stack EXIT INT TERM

    sleep 3
    if ! kill -0 "$_MOCK_STACK_PID" 2>/dev/null; then
        echo "_mock_stack.sh: launch process died on startup; log follows:" >&2
        cat "$launch_log" >&2
        return 1
    fi
    echo "_mock_stack.sh: launch pid=$_MOCK_STACK_PID pgid=$_MOCK_STACK_PGID up"
    return 0
}

stop_mock_stack() {
    if [ -n "$_MOCK_STACK_PGID" ]; then
        kill -INT -- "-$_MOCK_STACK_PGID" 2>/dev/null || true
        for _ in $(seq 1 10); do
            if ! kill -0 -- "-$_MOCK_STACK_PGID" 2>/dev/null; then
                _MOCK_STACK_PGID=""
                return 0
            fi
            sleep 1
        done
        if kill -0 -- "-$_MOCK_STACK_PGID" 2>/dev/null; then
            echo "_mock_stack.sh: process group $_MOCK_STACK_PGID still alive after SIGINT, sending SIGKILL" >&2
            kill -KILL -- "-$_MOCK_STACK_PGID" 2>/dev/null || true
            sleep 1
        fi
        _MOCK_STACK_PGID=""
    fi
}
