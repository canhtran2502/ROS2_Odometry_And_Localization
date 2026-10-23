#!/usr/bin/env bash
# PreToolUse hook for Bash.
# Enforces the git rules: never push/merge into main, no AI attribution in commits,
# never stage colcon output.
set -euo pipefail

input=$(cat)
cmd=$(jq -r '.tool_input.command // empty' <<<"$input")
[ -z "$cmd" ] && exit 0
grep -q 'git' <<<"$cmd" || exit 0

block() { echo "Blocked by guard_git: $1" >&2; exit 2; }

root="${CLAUDE_PROJECT_DIR:-$(pwd)}"
branch=$(git -C "$root" branch --show-current 2>/dev/null || true)

# Push targeting main (explicit refspec, or plain push while on main).
if grep -Eq 'git[[:space:]]+push' <<<"$cmd"; then
  if grep -Eq 'git[[:space:]]+push.*([[:space:]:]|^)main([[:space:]]|$)' <<<"$cmd"; then
    block "pushing to 'main' is reserved for the user."
  fi
  if [ "$branch" = "main" ]; then
    block "current branch is 'main'; the user pushes main."
  fi
fi

# Merge into main.
if grep -Eq 'git[[:space:]]+merge' <<<"$cmd"; then
  if [ "$branch" = "main" ] || grep -Eq '(checkout|switch)[[:space:]]+main' <<<"$cmd"; then
    block "merging into 'main' is reserved for the user."
  fi
fi

# Commits must not carry AI attribution; author stays the local git identity.
if grep -Eq 'git[[:space:]]+commit' <<<"$cmd"; then
  if grep -Eiq 'co-authored-by:.*(claude|anthropic)|generated with.*claude|noreply@anthropic' <<<"$cmd"; then
    block "commit message contains AI attribution; remove the trailer/footer."
  fi
  if grep -Eq -- '--author|user\.(name|email)' <<<"$cmd"; then
    block "do not override the author; use the machine's git identity."
  fi
fi

# Never stage colcon output.
if grep -Eq 'git[[:space:]]+add.*(^|[[:space:]])(\./)?(build|install|log)(/|[[:space:]]|$)' <<<"$cmd"; then
  block "build/, install/ and log/ must never be committed."
fi
exit 0
