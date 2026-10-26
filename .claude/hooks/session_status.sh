#!/usr/bin/env bash
# SessionStart hook. Stdout is injected into the model's context, so a new session
# knows which feature pipeline is in progress and where to resume.
set -uo pipefail

root="${CLAUDE_PROJECT_DIR:-$(pwd)}"
shopt -s nullglob
found=0
for state in "$root"/.claude/features/*/state.json; do
  stage=$(jq -r '.stage' "$state")
  [ "$stage" = "done" ] && continue
  [ $found -eq 0 ] && echo "Feature pipelines in progress:"
  found=1
  jq -r '"- \(.slug): stage=\(.stage), iteration=\(.iteration)/\(.max_iterations), last verdict=\((.history | last | .verdict) // "none")"' "$state"
done
[ $found -eq 1 ] && echo "Resume with /feature-pipeline <slug> (or the single-stage skills)."
exit 0
