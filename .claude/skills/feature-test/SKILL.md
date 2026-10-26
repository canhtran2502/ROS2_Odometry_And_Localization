---
name: feature-test
description: Stage 3 of the feature pipeline. Has the tester subagent (Sonnet) write and run tests for every acceptance criterion, produces test-report.md with a PASS/FAIL verdict, and commits it. Use after feature-implement.
argument-hint: "<slug>"
---

# Stage 3: Test (independent evaluation)

Slug: $ARGUMENTS (if empty, read `.claude/features/.active`).

1. Run `state.sh stage <slug> test` (script at `.claude/skills/feature-pipeline/scripts/state.sh`)
   and read the iteration N with `state.sh get <slug> .iteration`.
2. Delegate to the **tester** subagent (`subagent_type: tester`) with:
   - the paths to `plan.md` and the iteration N;
   - the output path `.claude/features/<slug>/test-report.md`;
   - the instruction "evaluate only; never modify product code".
3. Parse the first line of the reply, `VERDICT: PASS|FAIL x/y`. Cross-check it against
   `colcon test-result --verbose`; trust the command output over the summary.
4. Check that the tester did not touch product code:
   `git diff --name-only` must only show test files and the report.
   If product files changed, revert them with `git checkout -- <file>` and record a FAIL.
5. Record the result and commit:
   `state.sh record <slug> <PASS|FAIL> <x> <y>`, then
   `git add -A src .claude/features/<slug> && git commit -m "test(<slug>): iteration N - <PASS|FAIL> (x/y AC)"`.
6. Report the verdict and the failing AC titles to the caller.
