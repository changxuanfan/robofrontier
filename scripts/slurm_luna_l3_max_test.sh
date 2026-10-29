#!/usr/bin/env bash
# Engineering test only. Supply site-specific partition, CPU, memory and time to sbatch.
set -euo pipefail
cd "${SLURM_SUBMIT_DIR:?Submit this script with sbatch from the checkout root}"
export PATH="$PWD/.codex-tools/node_modules/.bin:$PATH"
mkdir -p runs/tmp
export TMPDIR="$PWD/runs/tmp" PYTHONUNBUFFERED=1
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}" OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
batch="${RF_BATCH_OUTPUT:-runs/policy/luna-l3-max-test-${SLURM_JOB_ID:?}}"
echo "Engineering test: Luna high, four Level 3 maximum-arm tasks, parallel 4, 3600 seconds per task; batch=$batch"
echo "Preflight and task execution use billable API calls."
.venv/bin/python -m scripts.luna_cpu setup-assets
.venv/bin/python -m scripts.luna_cpu doctor
.venv/bin/python -m scripts.luna_cpu preflight --output "$batch"
.venv/bin/python -m scripts.luna_cpu l3-max-test --output "$batch"
