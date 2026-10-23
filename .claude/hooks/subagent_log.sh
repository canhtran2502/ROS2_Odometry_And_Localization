#!/usr/bin/env bash
# SubagentStop hook. Appends one line per finished subagent to the active feature's
# progress.log, giving a timeline of the pipeline independent of the model.
set -uo pipefail

input=$(cat)
root="${CLAUDE_PROJECT_DIR:-$(pwd)}"
active_file="$root/.claude/features/.active"
[ -f "$active_file" ] || exit 0
slug=$(cat "$active_file")
dir="$root/.claude/features/$slug"
[ -d "$dir" ] || exit 0

agent=$(jq -r '.agent_type // .subagent_type // "subagent"' <<<"$input")
stage=$(jq -r '.stage // "?"' "$dir/state.json" 2>/dev/null || echo "?")
iter=$(jq -r '.iteration // 0' "$dir/state.json" 2>/dev/null || echo 0)
echo "$(date -Iseconds) agent=$agent stage=$stage iteration=$iter" >>"$dir/progress.log"
exit 0
