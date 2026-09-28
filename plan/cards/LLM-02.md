# LLM-02 — Build pinned llama.cpp (CUDA, sm_86) in user space

```json card
{
  "kind": "impl",
  "depends_on": [
    "HOST-01"
  ],
  "requirements": [
    "REQ-LLM-2",
    "REQ-HOST-2"
  ],
  "scope": {
    "write": [
      "scripts/llm/build_llama_cpp.sh",
      "config/llm_runtime.json",
      "docs/llm/RUNTIME.md"
    ]
  },
  "acceptance": {
    "checks": [
      {
        "id": "version",
        "cmd": "~/opt/llama.cpp/build/bin/llama-server --version"
      },
      {
        "id": "json",
        "cmd": "python3 -m json.tool config/llm_runtime.json >/dev/null"
      }
    ],
    "criteria": [
      "Build script is idempotent, pins a release tag + commit, uses /usr/local/cuda-12.8/bin/nvcc and CMAKE_CUDA_ARCHITECTURES=86",
      "config/llm_runtime.json records tag, commit, cmake flags, CUDA version, build date",
      "llama-server lists the RTX 3090 as a CUDA device (--list-devices output captured in RUNTIME.md)"
    ]
  },
  "routing": {
    "complexity": 3,
    "ambiguity": 2,
    "context": 2,
    "consequence": 2
  },
  "resources": [
    "gpu"
  ],
  "track": "automation",
  "timeout_min": {
    "impl": 120
  },
  "priority": 45,
  "extra_rw": [
    "~/opt/llama.cpp"
  ],
  "id": "LLM-02",
  "title": "Build pinned llama.cpp (CUDA, sm_86) in user space",
  "parent": "L-LLM",
  "outcome": "A reproducible, pinned llama-server build exists at ~/opt/llama.cpp without sudo.",
  "gpu_min_free_mb": 1500
}
```


