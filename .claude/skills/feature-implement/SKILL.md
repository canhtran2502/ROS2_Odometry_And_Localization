---
name: feature-implement
description: Stage 2 of the feature pipeline. Has the coder subagent (Sonnet) implement plan.md, or fix the failures in test-report.md, builds with colcon, and commits the iteration. Use after feature-plan or after a failed feature-test.
argument-hint: "<slug>"
---

# Stage 2: Implement

Slug: $ARGUMENTS (if empty, read `.claude/features/.active`).

1. Check that `.claude/features/<slug>/plan.md` exists. If it doesn't, run `/feature-plan` first.
2. Increment the iteration: `N=$(.claude/skills/feature-pipeline/scripts/state.sh next <slug>)`
   and `state.sh stage <slug> implement`.
3. Delegate to the **coder** subagent (`subagent_type: coder`). Its prompt contains:
   - the paths to `plan.md` and, if present, `test-report.md`;
   - the iteration number N;
   - for N > 1: "Fix only the FAIL items in test-report.md; do not refactor unrelated code."
4. Verify the coder's claim yourself; don't trust the summary alone:
   `source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select <pkgs>`.
   If the build fails, send the error back to the coder (max 2 retries).
5. Review `git diff --stat`:
   - no files changed outside the plan's Files table without a reason;
   - no test files edited by the coder.
6. Commit:
   `git add -A src doc README.md .claude/features/<slug> && git commit -m "feat(<slug>): implement iteration N"`.
   Never add `build/`, `install/` or `log/`. No AI attribution in the message.
