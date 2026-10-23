#!/usr/bin/env bash
# PreToolUse hook for Edit|Write.
# Blocks edits to colcon output dirs and absolute paths in launch files.
# Exit 2 = block the tool call; stderr is sent back to the model.
set -euo pipefail

input=$(cat)
file=$(jq -r '.tool_input.file_path // empty' <<<"$input")
[ -z "$file" ] && exit 0

root="${CLAUDE_PROJECT_DIR:-$(pwd)}"
rel="${file#"$root"/}"

case "$rel" in
  build/*|install/*|log/*)
    echo "Blocked: '$rel' is colcon output. Edit sources in src/ and rebuild instead." >&2
    exit 2 ;;
esac

if [[ "$rel" == *.launch.py ]]; then
  content=$(jq -r '(.tool_input.content // "") + (.tool_input.new_string // "")' <<<"$input")
  if grep -Eq "['\"]/(home|root|Users)/" <<<"$content"; then
    echo "Blocked: absolute path in launch file '$rel'. Use get_package_share_directory or launch substitutions (CLAUDE.md rule)." >&2
    exit 2
  fi
fi
exit 0
