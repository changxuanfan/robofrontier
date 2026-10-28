#!/usr/bin/env bash
# Submit with explicit site-specific --partition, --cpus-per-task, --mem and --time.
set -euo pipefail
cd "${SLURM_SUBMIT_DIR:?Submit this script with sbatch from the checkout root}"
export PATH="$PWD/.codex-tools/node_modules/.bin:$PATH"
mkdir -p runs/tmp
export TMPDIR="$PWD/runs/tmp"
batch="${RF_BATCH_OUTPUT:-runs/policy/luna-slurm-${SLURM_JOB_ID:?}}"
wall_limit="${RF_WALL_LIMIT_S:-86400}"
parallel="${RF_PARALLEL:-36}"
export PYTHONUNBUFFERED=1
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
echo "Luna high CPU batch=$batch parallel=$parallel per_task_wall_s=$wall_limit"
echo "Preflight and task execution use billable API calls."
.venv/bin/python -m scripts.luna_cpu setup-assets
.venv/bin/python -m scripts.luna_cpu doctor
.venv/bin/python -m scripts.luna_cpu preflight --output "$batch"
.venv/bin/python -m scripts.luna_cpu submit --output "$batch" --wall-limit-s "$wall_limit" --parallel "$parallel"
