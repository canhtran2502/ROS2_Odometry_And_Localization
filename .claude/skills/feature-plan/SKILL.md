---
name: feature-plan
description: Stage 1 of the feature pipeline. Creates a feature workspace under .claude/features/<slug>/, has the architect subagent (Opus) write plan.md with numbered acceptance criteria, and commits it. Use when starting a new BumperBot feature.
argument-hint: "<feature description>"
---

# Stage 1: Plan (architecture)

Feature request: $ARGUMENTS

1. Derive a short kebab-case slug from the request (e.g. `auto-explore-slam`).
   If `.claude/features/<slug>/plan.md` already exists, stop and ask whether to re-plan.
2. Initialise state:
   `.claude/skills/feature-pipeline/scripts/state.sh init <slug> "<title>"`
3. Delegate to the **architect** subagent (Agent tool, `subagent_type: architect`) with:
   - the full feature request,
   - the slug and the output path `.claude/features/<slug>/plan.md`,
   - a reminder that acceptance criteria must be testable headless.
4. Read `plan.md` and check:
   - every AC has a matching entry in the Test plan;
   - no rule in `CLAUDE.md` is violated.
   If either check fails, send the architect back once with the specific gaps.
5. Show the user the AC list and the Files table (short). Unless running inside
   `/feature-pipeline` in autonomous mode, wait for approval or edits.
6. Set the stage, then commit:
   `state.sh stage <slug> implement`, then
   `git add .claude/features/<slug> && git commit -m "feat(<slug>): plan architecture"`.
   Commit messages carry no AI attribution (the guard_git hook enforces this).
