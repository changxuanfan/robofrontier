# GPT-6 Luna CPU benchmark

This branch starts autonomous adaptive benchmark jobs through the Codex CLI app-server; no supervising LLM session is required. The model is `gpt-6-luna`, effort `high`, via OpenAI Responses and `OPENAI_API_KEY`. `OPENAI_API_KEY` is an API credential; ChatGPT subscription login is not used by this provider. Frozen tasks keep the R14 release of the parent Codex branch. A 600-second run is an engineering test, not the standard full task limit.

## Linux native

Install Python 3.12, Node.js and Codex CLI, plus Mesa software rendering (`libosmesa6` on Debian/Ubuntu). Create the Python environment and install this project; for example:

```sh
python3.12 -m venv .venv
.venv/bin/pip install .
cp .env.example .env
chmod 600 .env
# Edit .env locally and set OPENAI_API_KEY; never commit it.
```

Restore the matching policy-assets archive with `.venv/bin/python -m scripts.benchmark_handoff restore /path/to/policy-assets.tar.gz`. The archive must match this task release. On shared clusters place the checkout, assets and runs on the appropriate large filesystem. The wrapper forces OSMesa, Mesa software rendering, and an empty `CUDA_VISIBLE_DEVICES`; before starting a task it renders a frame and checks `GL_RENDERER` is a known software rasterizer. MuJoCo physics itself runs on CPU. The probe fails closed if CPU rendering cannot be verified.

```sh
.venv/bin/python -m scripts.luna_cpu doctor
.venv/bin/python -m scripts.luna_cpu preflight
.venv/bin/python -m scripts.luna_cpu submit --output runs/policy/luna-l1-001 --level 1 --min-arms --wall-limit-s 600 --parallel 4
```

`preflight` and `submit` make billable API calls. Preflight checks image input, function calling, manual compaction and fork, but does not prove robot-task performance. A changed CLI version or model configuration requires another preflight. `submit` prepares and runs four independent tasks concurrently in the foreground. The deadline applies to each task after START. Pick a fresh output directory; inspect running jobs in another terminal:

```sh
.venv/bin/python -m scripts.luna_cpu status runs/policy/luna-l1-001
.venv/bin/python -m scripts.luna_cpu usage runs/policy/luna-l1-001
.venv/bin/python -m scripts.luna_cpu performance runs/policy/luna-l1-001
.venv/bin/python -m scripts.luna_cpu stop runs/policy/luna-l1-001
```

Usage reads local Codex session traces and deduplicates response IDs, including forked roles. It reports missing trace files and is not a billing statement. `stop` requests graceful termination. For detached work use a job scheduler, tmux, or systemd; command completion or remote connection loss must not be confused with task completion.

Performance records owner-side wall time for physics steps, rendering/capture and dispatch, and unions action and model-turn intervals across roles. These intervals overlap: action time is not additional to model turn time; dispatch includes capture. The model-turn interval includes tool waits and is not a measurement of API network or inference latency. Owner counters begin when the physical owner starts, slightly before START, and are sampled every five seconds. Usage TPM is calculated from completed response timestamps and may differ from the provider's rate-limit accounting.

## Mac with strict CPU rendering

Stock MuJoCo does not support its OSMesa backend on macOS. Run a Linux container on the Mac to use the same software renderer; native macOS rendering is not certified CPU-only. The following requires a running Docker engine. Build from this checkout (the image does not contain credentials or run assets):

```sh
docker build -f docker/luna-cpu.Dockerfile -t robofrontier-luna-cpu .
docker run --rm -v "$PWD:/workspace" -w /workspace robofrontier-luna-cpu doctor
docker run --rm -v "$PWD:/workspace" -w /workspace -v "$PWD/.codex-luna:/root/.codex" robofrontier-luna-cpu preflight
docker run --rm -v "$PWD:/workspace" -w /workspace -v "$PWD/.codex-luna:/root/.codex" robofrontier-luna-cpu submit --output runs/policy/luna-l1-001 --level 1 --min-arms --wall-limit-s 600 --parallel 4
docker run --rm -v "$PWD:/workspace" -w /workspace -v "$PWD/.codex-luna:/root/.codex" robofrontier-luna-cpu usage runs/policy/luna-l1-001
```

The `.codex-luna` mount preserves Codex session traces between commands; keep it private and out of Git. The working directory bind mount supplies `.env`, assets and results. Container performance, Codex version, API access and CPU rendering still need validation on the actual machine. Rendering on software can be substantially slower; parallelism should be sized to the host.
