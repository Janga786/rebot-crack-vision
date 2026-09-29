#!/usr/bin/env bash
# scripts/llm/build_llama_cpp.sh — pinned, reproducible llama.cpp (CUDA, sm_86) build in user
# space. Built by LLM-02.
#
# Clones ggml-org/llama.cpp into ~/opt/llama.cpp, checks out a pinned release tag + commit,
# configures CMake against /usr/local/cuda-12.8's nvcc for the RTX 3090 (sm_86) and builds
# llama-server. No sudo anywhere. Re-running is a fast no-op once the pinned commit is already
# built (pass --force to rebuild, e.g. after a toolchain change). Regenerates
# config/llm_runtime.json and docs/llm/RUNTIME.md from the actual build/tool output on every run
# so those two files never describe a build that wasn't just verified.
set -euo pipefail

# --- Pin --------------------------------------------------------------------
LLAMA_TAG="v0.5.0"
LLAMA_COMMIT="7fe450e19305b828c199d602c23a8337aaa1f03b"
LLAMA_REPO_URL="https://github.com/ggml-org/llama.cpp.git"

# --- Toolchain ----------------------------------------------------------------
CUDA_HOME="/usr/local/cuda-12.8"
NVCC="$CUDA_HOME/bin/nvcc"
CUDA_ARCH="86"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

LLAMA_DIR="${LLAMA_CPP_HOME:-$HOME/opt/llama.cpp}"
BUILD_DIR="$LLAMA_DIR/build"
BIN="$BUILD_DIR/bin/llama-server"
JOBS="${JOBS:-$(nproc)}"

RUNTIME_JSON="$REPO_ROOT/config/llm_runtime.json"
RUNTIME_DOC="$REPO_ROOT/docs/llm/RUNTIME.md"

FORCE=0
for arg in "$@"; do
    case "$arg" in
        --force) FORCE=1 ;;
        *)
            echo "usage: $0 [--force]" >&2
            exit 2
            ;;
    esac
done

if [ ! -x "$NVCC" ]; then
    echo "build_llama_cpp.sh: $NVCC not found — expected the CUDA 12.8 toolkit at $CUDA_HOME" >&2
    exit 3
fi

mkdir -p "$(dirname "$LLAMA_DIR")"

# --- Fetch / checkout the pinned commit --------------------------------------
if [ ! -d "$LLAMA_DIR/.git" ]; then
    git clone "$LLAMA_REPO_URL" "$LLAMA_DIR"
fi

cd "$LLAMA_DIR"
git fetch --force origin "refs/tags/${LLAMA_TAG}:refs/tags/${LLAMA_TAG}"
git checkout --detach "$LLAMA_COMMIT"

ACTUAL_COMMIT="$(git rev-parse HEAD)"
if [ "$ACTUAL_COMMIT" != "$LLAMA_COMMIT" ]; then
    echo "build_llama_cpp.sh: checked out $ACTUAL_COMMIT, expected pinned $LLAMA_COMMIT" >&2
    exit 1
fi

SHORT_COMMIT="${LLAMA_COMMIT:0:9}"

# --- Skip a rebuild if the pinned commit is already built --------------------
NEED_BUILD=1
if [ "$FORCE" -eq 0 ] && [ -x "$BIN" ] && "$BIN" --version 2>&1 | grep -q "$SHORT_COMMIT"; then
    NEED_BUILD=0
    echo "build_llama_cpp.sh: $BIN already built at pinned commit ${SHORT_COMMIT}; skipping build (use --force to rebuild)"
fi

if [ "$NEED_BUILD" -eq 1 ]; then
    cmake -S "$LLAMA_DIR" -B "$BUILD_DIR" \
        -DCMAKE_BUILD_TYPE=Release \
        -DGGML_CUDA=ON \
        -DCMAKE_CUDA_COMPILER="$NVCC" \
        -DCMAKE_CUDA_ARCHITECTURES="$CUDA_ARCH" \
        -DLLAMA_CURL=ON

    cmake --build "$BUILD_DIR" --config Release --target llama-server -j "$JOBS"
fi

if ! "$BIN" --version 2>&1 | grep -q "$SHORT_COMMIT"; then
    echo "build_llama_cpp.sh: built binary at $BIN does not report pinned commit ${SHORT_COMMIT}" >&2
    exit 1
fi

VERSION_OUTPUT="$("$BIN" --version 2>&1)"
LIST_DEVICES_OUTPUT="$("$BIN" --list-devices 2>&1)"
CUDA_VERSION="$("$NVCC" --version | sed -n 's/.*release \([0-9.]*\),.*/\1/p')"
BUILD_DATE_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

if ! echo "$LIST_DEVICES_OUTPUT" | grep -qi "3090"; then
    echo "build_llama_cpp.sh: --list-devices did not list the RTX 3090:" >&2
    echo "$LIST_DEVICES_OUTPUT" >&2
    exit 1
fi

mkdir -p "$(dirname "$RUNTIME_JSON")" "$(dirname "$RUNTIME_DOC")"

python3 - "$RUNTIME_JSON" "$LLAMA_TAG" "$LLAMA_COMMIT" "$CUDA_HOME" "$CUDA_VERSION" "$CUDA_ARCH" "$BUILD_DATE_UTC" "$LLAMA_DIR" "$BIN" "$JOBS" <<'PYEOF'
import json
import sys

(path, tag, commit, cuda_home, cuda_version, cuda_arch, build_date,
 llama_dir, bin_path, jobs) = sys.argv[1:11]

data = {
    "component": "llama.cpp",
    "repo_url": "https://github.com/ggml-org/llama.cpp.git",
    "tag": tag,
    "commit": commit,
    "build_date_utc": build_date,
    "install_dir": llama_dir,
    "binary": bin_path,
    "cmake_flags": {
        "CMAKE_BUILD_TYPE": "Release",
        "GGML_CUDA": "ON",
        "CMAKE_CUDA_COMPILER": f"{cuda_home}/bin/nvcc",
        "CMAKE_CUDA_ARCHITECTURES": cuda_arch,
        "LLAMA_CURL": "ON",
    },
    "cuda_version": cuda_version,
    "cuda_home": cuda_home,
    "build_jobs": int(jobs),
    "gpu_target": "NVIDIA GeForce RTX 3090 (sm_86)",
}

with open(path, "w") as f:
    json.dump(data, f, indent=2, sort_keys=False)
    f.write("\n")
PYEOF

cat > "$RUNTIME_DOC" <<EOF
# LLM runtime — pinned llama.cpp build

Built by \`scripts/llm/build_llama_cpp.sh\` (LLM-02). No sudo; entirely in \`~/opt\`.

## Pin

| | |
|---|---|
| Repo | https://github.com/ggml-org/llama.cpp |
| Tag | \`${LLAMA_TAG}\` |
| Commit | \`${LLAMA_COMMIT}\` |
| Install dir | \`${LLAMA_DIR}\` |
| Binary | \`${BIN}\` |
| Built (UTC) | ${BUILD_DATE_UTC} |
| CUDA toolkit | ${CUDA_VERSION} (\`${CUDA_HOME}\`) |
| CMake CUDA arch | ${CUDA_ARCH} (sm_86, RTX 3090) |

## CMake configuration

\`\`\`
cmake -S ${LLAMA_DIR} -B ${LLAMA_DIR}/build \\
    -DCMAKE_BUILD_TYPE=Release \\
    -DGGML_CUDA=ON \\
    -DCMAKE_CUDA_COMPILER=${CUDA_HOME}/bin/nvcc \\
    -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCH} \\
    -DLLAMA_CURL=ON
cmake --build ${LLAMA_DIR}/build --config Release --target llama-server -j <nproc>
\`\`\`

## Rebuilding

\`\`\`
scripts/llm/build_llama_cpp.sh            # no-op if the pinned commit is already built
scripts/llm/build_llama_cpp.sh --force    # force a clean rebuild at the pinned commit
\`\`\`

The script re-checks out \`${LLAMA_COMMIT}\` on every run and refuses to proceed (exit 1) if the
working tree lands on a different commit, so the build is reproducible: same tag/commit in, same
binary out. This file and \`config/llm_runtime.json\` are regenerated from the just-built binary's
own \`--version\`/\`--list-devices\` output on every run, so they always describe the build that was
actually verified, not a stale record.

## \`llama-server --version\`

\`\`\`
${VERSION_OUTPUT}
\`\`\`

## \`llama-server --list-devices\`

Captured on this host (RTX 3090, driver/CUDA per \`config/host_versions.json\`):

\`\`\`
${LIST_DEVICES_OUTPUT}
\`\`\`

## Isolation (REQ-HOST-2)

This build is consumed only by the local-inference runtime (\`~/claude-auto/claude_auto/localcoder.py\`,
per \`config/llm_candidates.json\`), started on demand and never co-resident with nnU-Net or MoveIt
GPU use. It does not touch the \`crackvision\` conda env (py3.11) or the system ROS py3.10
interpreter — it is a standalone C++/CUDA binary invoked directly, not through \`./env.sh\`.
EOF

echo "build_llama_cpp.sh: OK — $BIN"
echo "build_llama_cpp.sh: wrote $RUNTIME_JSON"
echo "build_llama_cpp.sh: wrote $RUNTIME_DOC"
