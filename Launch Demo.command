#!/bin/zsh
set -e
cd "${0:A:h}"
if [[ ! -f vendor/panda/SOURCE.json ]]; then
  uv run python scripts/fetch_robot.py
fi
exec uv run robofrontier
