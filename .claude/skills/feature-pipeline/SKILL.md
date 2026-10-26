---
name: feature-pipeline
description: Orchestrates the full automated feature workflow for BumperBot - plan with the architect (Opus), then loop implement (coder, Sonnet) and test (tester, Sonnet) until every acceptance criterion passes or the iteration limit is hit, then one adversarial review round by the reviewer (Opus) whose findings are fixed and re-tested. Commits each stage. Use when the user wants a feature built end to end, or to resume an in-progress feature by slug.
argument-hint: "<feature description | slug> [--review]"
---

# Feature pipeline (orchestrator)

Input: $ARGUMENTS

You are the **orchestrator**. The subagents do the work; you route tasks, verify results,
keep state and decide when to stop. Subagents cannot spawn subagents, so the loop runs here.
State helper: `.claude/skills/feature-pipeline/scripts/state.sh`.

## 0. Resume or start
- If the input matches an existing `.claude/features/<slug>/state.json`, resume from its `stage`.
- Otherwise run the **feature-plan** skill with the description in autonomous mode: post a
  short AC summary and continue straight into the loop without waiting for approval.
  Only with `--review` pause once for the user to approve the plan.

## 1. Loop (evaluator-optimizer)
```
while true:
    run feature-implement <slug>      # coder writes code, build verified
    run feature-test <slug>           # tester evaluates, verdict recorded
    if verdict == PASS (all AC):      break -> step 1b
    if iteration >= max_iterations:   state.sh stage <slug> blocked; break -> step 3
    if the same AC failed with the same root cause in the last 2 iterations:
        state.sh stage <slug> blocked; break -> step 3   # stuck, needs a human
```
Between iterations, tell the user one line, e.g. `iteration 2: FAIL 3/5 (AC2, AC4) -> fixing`.

## 1b. Opus review round (once per feature)
Runs after the first PASS, before Done. The tester only checks the written AC, so this round
looks for defects the tests cannot see (races, failure paths, C++/Python parity, safety).
1. Base commit = the commit before the plan commit (`git log --oneline` for the
   `feat(<slug>): plan architecture` commit, then its parent).
2. Delegate to the **reviewer** subagent (`subagent_type: reviewer`, Opus) with the slug, the
   base commit and the output path `.claude/features/<slug>/review.md`. Cross-check the high
   findings yourself against the code before acting on them.
3. Commit `docs(<slug>): opus review`. Then `state.sh extend <slug> 3` (the review round gets
   its own iteration budget).
4. Fix list = every high and medium finding that is not `hardware-only`, plus cheap doc
   corrections. Findings you defer (hardware-only, low) are listed in the final summary.
   If the fix list is empty, skip to step 2.
5. Run **feature-implement** with the fix list as the "FAIL items" (the coder fixes only
   those; it never edits tests), then **feature-test** with the instruction to add a test for
   every fix (the tester never edits product code). Loop as in step 1 until PASS.
6. Do not run the reviewer a second time on the same feature; a remaining high finding after
   the extended budget goes to step 3 (blocked).

## 2. Done
- `state.sh stage <slug> done`.
- Make sure README.md and doc/*.md describe the new behavior (a CLAUDE.md rule). If they
  don't, send the coder back once to update them and amend nothing: make a new commit
  `docs(<slug>): update documentation`.
- Commit `chore(<slug>): pipeline done`, staging only `.claude/features/<slug>`.
- Summary to the user: iterations used, AC table, review findings fixed and deferred, commits (`git log --oneline` since the plan commit).
  Do not push unless the user asks.

## 3. Blocked
- Commit the state, then present the user with: the failing AC, the root cause from the last
  `test-report.md`, what was tried, and 1-2 concrete options (adjust AC, change design,
  provide hardware/sim info). Wait for their decision.

## Guardrails
- Never edit tests to make them pass, and never relax an AC without the user's consent.
- The hooks enforce: no edits in build/install/log, no absolute launch paths, no push/merge
  to main, no AI attribution in commits, lint after each edit.
