# Claude harness: development status

This branch is under development. It does **not** start scored tasks or perform
model inference. Provider profiles are candidates, not compatibility claims.
The existing Codex benchmark remains separate.

After `uv sync --locked --extra oracle`, use:

```sh
.venv/bin/python -m scripts.claude_benchmark models
.venv/bin/python -m scripts.claude_benchmark doctor --models haiku opus kimi qwen glm deepseek
.venv/bin/python -m scripts.claude_benchmark plan --models kimi --level 1 --min-arms --parallel 4
.venv/bin/python -m scripts.claude_benchmark prepare-draft --models kimi --level 1 --min-arms --output runs/engineering/claude-draft
.venv/bin/python -m scripts.claude_offline_probe
```

Real precheck (spends money; needs a credential and an explicit cap):

```sh
.venv/bin/python -m scripts.claude_precheck --profile haiku --max-budget-usd 0.5
.venv/bin/python -m scripts.claude_precheck --profile haiku --max-budget-usd 0.5 --confirm-spend
```

Without `--confirm-spend` it only prints the plan. With it, it makes three
calls on one explicit session: a text reply, a robot tool call whose returned
image must be identified, and a resumed turn. The receipt is `receipt.json`
in the output directory; it records CLI version, requested and reported model,
cost and per-check results, never key values. A profile passes only if all
three checks pass within the cap. Bench runs use `opus`, not the Haiku
precheck profile.

The doctor checks locally only. It reports whether credentials are present,
not whether they work. If a sandbox denies the provider host or writes under
`runs/`, allow it in your own environment rather than bypassing the sandbox.
For local credentials copy `.env.claude.example` to
`.env` with permissions 600; existing environment variables take priority.
Do not commit credentials. The Anthropic-compatible Qwen endpoint must match
your actual account and region. Live preflight status:

- `haiku` (`claude-haiku-5-5`, CLI 2.1.312) passed on 2026-10-28 with a total
  cost of $0.000541 under a $0.5 cap. Its receipt is local at
  `runs/engineering/claude-precheck/haiku-20261028-194259/receipt.json`
  (`runs/` is gitignored). This covers only that profile, CLI version and cap.
- `opus` and the other providers have not been preflighted.

`robofrontier.claude.robot_mcp` is an experimental role-bound MCP server. Its
offline probe checks Claude CLI tool discovery, simulated tool-image forwarding and
fork history against local stub servers. It does not validate real provider image
handling, trusted session provenance, dynamic delegation or production physical runs.
Do not launch it against an actual scored world. The draft prepare command freezes inputs for engineering inspection only; it is
not a scored or runnable batch. The runner, real-provider preflight, status and
stop functions for Claude remain to be implemented and qualified; the Codex-to-Claude
component map and verified CLI facts are in [claude-codex-parity.md](claude-codex-parity.md).
Mosaic remains the released R14 scene in this branch.
