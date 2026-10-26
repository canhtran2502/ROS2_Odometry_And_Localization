#!/usr/bin/env bash
# PostToolUse hook for Edit|Write.
# Fast syntax check of the edited file. On failure, exit 2 so the model sees the
# error and fixes it in the same turn.
set -uo pipefail

input=$(cat)
file=$(jq -r '.tool_input.file_path // empty' <<<"$input")
[ -z "$file" ] || [ ! -f "$file" ] && exit 0

out=""
case "$file" in
  *.py)
    # Only real errors (syntax, undefined names), not style of pre-existing code.
    out=$(python3 -m flake8 --select=E9,F63,F7,F82 "$file" 2>&1) ;;
  *.xacro|*.urdf|*.xml|*.sdf|*.launch)
    out=$(xmllint --noout "$file" 2>&1) ;;
  *.yaml|*.yml)
    out=$(python3 -c 'import sys, yaml; list(yaml.safe_load_all(open(sys.argv[1])))' "$file" 2>&1) ;;
  *.json)
    out=$(jq empty "$file" 2>&1) ;;
  *) exit 0 ;;
esac

if [ -n "$out" ]; then
  echo "Lint error in $file:" >&2
  echo "$out" | head -20 >&2
  exit 2
fi
exit 0
