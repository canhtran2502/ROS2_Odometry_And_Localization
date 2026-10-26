#!/usr/bin/env bash
# Deterministic state management for the feature pipeline.
# Usage:
#   state.sh init   <slug> <title> [max_iterations]
#   state.sh stage  <slug> <plan|implement|test|done|blocked>
#   state.sh next   <slug>                       # iteration += 1, prints new value
#   state.sh record <slug> <PASS|FAIL> <passed> <total>
#   state.sh get    <slug> [jq filter]
set -euo pipefail

root="${CLAUDE_PROJECT_DIR:-$(git rev-parse --show-toplevel)}"
cmd="$1"; slug="$2"; shift 2
dir="$root/.claude/features/$slug"
state="$dir/state.json"

update() { # jq filter, args...
  local tmp; tmp=$(mktemp)
  jq "$@" "$state" >"$tmp" && mv "$tmp" "$state"
}

case "$cmd" in
  init)
    mkdir -p "$dir"
    if [ ! -f "$state" ]; then
      jq -n --arg s "$slug" --arg t "$1" --argjson m "${2:-5}" \
        '{slug:$s, title:$t, stage:"plan", iteration:0, max_iterations:$m, history:[]}' >"$state"
    fi
    echo "$slug" >"$root/.claude/features/.active"
    cat "$state" ;;
  stage)
    update --arg v "$1" '.stage = $v' ;;
  next)
    update '.iteration += 1'
    jq -r '.iteration' "$state" ;;
  record)
    update --arg v "$1" --argjson p "$2" --argjson t "$3" \
      '.history += [{iteration:.iteration, verdict:$v, passed:$p, total:$t, at:(now|todate)}]' ;;
  get)
    jq -r "${1:-.}" "$state" ;;
  *)
    echo "unknown command: $cmd" >&2; exit 1 ;;
esac
