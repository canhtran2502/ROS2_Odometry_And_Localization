# 13 - AI Development Workflow (Claude Code Feature Pipeline)

## A. Objective

This document describes the automated feature workflow configured in `.claude/`.
A feature request goes through architecture planning, implementation and independent
testing, and the implement/test loop repeats until every acceptance criterion passes.
Each stage is committed on the feature branch so the history records the progress.

---

## B. Pipeline Overview

```
 /feature-pipeline "<feature>"
          |
          v
 +------------------+    plan.md (AC1..ACn, test plan)
 | architect (Opus) | -------------------------------+
 +------------------+                                |
                                                     v
          +-------------------+   code + build   +-------------------+
   +----> | coder (Sonnet)    | ---------------> | tester (Sonnet)   |
   |      +-------------------+                  +-------------------+
   |                                                     |
   |            test-report.md (FAIL items)              |  VERDICT
   +-----------------------------------------------------+
                FAIL and iteration < max          PASS -> done
                                                  stuck / max -> blocked (ask user)
```

The main Claude Code session is the **orchestrator**. Subagents cannot spawn other
subagents, so the loop runs in the main session and only delegates the work.

---

## C. Concepts

| Concept | Meaning | Where it is used |
|---|---|---|
| Harness | The runtime around the model: agent loop, tools, permissions, hooks, context | Claude Code, configured by `.claude/` |
| Agent loop | Model reasons, calls a tool, reads the result, repeats until done | Every subagent |
| Subagent | Agent with its own context window, model and tool set. It returns only a summary | `.claude/agents/*.md` |
| Skill | Packaged instructions loaded on demand (`/name`), i.e. progressive disclosure | `.claude/skills/*/SKILL.md` |
| Hook | Script run by the harness (not the model) on lifecycle events. It is deterministic | `.claude/hooks/*.sh` |
| Orchestrator-worker | One agent routes tasks to specialised workers | Main session to architect/coder/tester |
| Evaluator-optimizer | One agent produces, another grades, then repeat | coder and tester loop |
| Context engineering | Choosing what each agent sees; files act as external memory | `plan.md`, `test-report.md`, `state.json` |
| Model routing | Strong model for hard reasoning, fast model for iteration | Opus plans, Sonnet codes and tests |

---

## D. Components

### 1. Subagents (`.claude/agents/`)
| Agent | Model | Role | Writes |
|---|---|---|---|
| `architect` | opus | Designs architecture, numbered acceptance criteria, test plan | `plan.md` only |
| `coder` | sonnet | Implements the plan or fixes FAIL items; builds with colcon | product code, docs |
| `tester` | sonnet | Writes tests per AC, runs `colcon test`, gives a verdict | tests, `test-report.md` |

The tester never edits product code, and the coder never edits tests. This separation
keeps the evaluation honest.

### 2. Skills (`.claude/skills/`)
| Skill | Stage | Commit |
|---|---|---|
| `/feature-plan <description>` | 1. architecture | `feat(<slug>): plan architecture` |
| `/feature-implement <slug>` | 2. implement iteration N | `feat(<slug>): implement iteration N` |
| `/feature-test <slug>` | 3. test iteration N | `test(<slug>): iteration N - PASS/FAIL (x/y AC)` |
| `/feature-pipeline <description or slug> [--auto]` | all, with the loop | per stage plus `chore(<slug>): pipeline done` |

The loop stops on PASS, when it reaches `max_iterations` (default 5), or when the same AC
fails with the same root cause twice. The last two cases set the state to `blocked` and
ask the user.

### 3. Hooks (`.claude/settings.json`)
| Event | Script | Effect |
|---|---|---|
| PreToolUse `Edit\|Write` | `guard_paths.sh` | Blocks edits in `build/`, `install/`, `log/` and absolute paths in `*.launch.py` |
| PreToolUse `Bash` | `guard_git.sh` | Blocks push/merge to `main`, AI attribution or `--author` in commits, staging colcon output |
| PostToolUse `Edit\|Write` | `post_edit_lint.sh` | flake8 errors (py), `xmllint` (xacro/urdf/xml), YAML and JSON parse. Errors are fed back to the model |
| SessionStart | `session_status.sh` | Injects the in-progress pipelines into the new session's context |
| SubagentStop | `subagent_log.sh` | Appends a timeline line to `progress.log` |

Exit code 2 from a hook blocks the action, and its stderr is shown to the model.

### 4. Feature state (`.claude/features/<slug>/`)
| File | Content |
|---|---|
| `plan.md` | Architecture, file list, acceptance criteria, test plan |
| `state.json` | `stage`, `iteration`, `max_iterations`, verdict `history` (managed by `scripts/state.sh`) |
| `test-report.md` | Latest per-AC results and root causes |
| `progress.log` | Timeline written by the SubagentStop hook |

---

## E. Usage

```bash
# Start Claude Code in the workspace on the feature branch
cd bumperbot_ws && git switch auto-explore-slam && claude

# Full pipeline (pauses once for plan approval)
/feature-pipeline Autonomous frontier exploration that builds a map with slam_toolbox

# Fully autonomous run
/feature-pipeline <description> --auto

# Single stages, or resume after a pause
/feature-plan <description>
/feature-implement auto-explore-slam
/feature-test auto-explore-slam
/feature-pipeline auto-explore-slam

# Inspect configuration inside Claude Code
/agents     /hooks     /skills
```

Progress is visible with `git log --oneline` and `cat .claude/features/<slug>/state.json`.
Nothing is pushed automatically.
