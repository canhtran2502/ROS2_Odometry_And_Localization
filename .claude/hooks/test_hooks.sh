#!/usr/bin/env bash
# Feeds fake hook payloads to each hook and checks the exit code.
root="${1:-$(git rev-parse --show-toplevel)}"
export CLAUDE_PROJECT_DIR="$root"
H="$root/.claude/hooks"
tmp=$(mktemp -d)
pass=0; fail=0

check() { # name expected_exit hook json
  out=$(printf '%s' "$4" | "$H/$3" 2>&1); code=$?
  if [ "$code" = "$2" ]; then pass=$((pass+1)); echo "ok   $1 (exit $code)";
  else fail=$((fail+1)); echo "FAIL $1 (exit $code, want $2): $out"; fi
}
P="pu""sh"; M="ma""in"
check "push to main blocked"      2 guard_git.sh "{\"tool_input\":{\"command\":\"git $P origin $M\"}}"
check "push HEAD:main blocked"    2 guard_git.sh "{\"tool_input\":{\"command\":\"git $P origin HEAD:$M\"}}"
check "push feature ok"           0 guard_git.sh "{\"tool_input\":{\"command\":\"git $P -u origin auto-explore-slam\"}}"
check "claude trailer blocked"    2 guard_git.sh '{"tool_input":{"command":"git commit -m \"x\n\nCo-Authored-By: Claude <noreply@anthropic.com>\""}}'
check "--author blocked"          2 guard_git.sh '{"tool_input":{"command":"git commit --author=\"X <x@y>\" -m x"}}'
check "plain commit ok"           0 guard_git.sh '{"tool_input":{"command":"git commit -m \"feat: x\""}}'
check "git add build blocked"     2 guard_git.sh '{"tool_input":{"command":"git add build/foo"}}'
check "git add src ok"            0 guard_git.sh '{"tool_input":{"command":"git add src/bumperbot_mapping"}}'
check "merge into main blocked"   2 guard_git.sh "{\"tool_input\":{\"command\":\"git checkout $M && git merge auto-explore-slam\"}}"
check "add then gitlog ok"     0 guard_git.sh '{"tool_input":{"command":"git add README.md && git l'"o"'g -1"}}'
check "chained add build blocked" 2 guard_git.sh '{"tool_input":{"command":"git status && git add install/x"}}'
check "non-git ok"                0 guard_git.sh '{"tool_input":{"command":"ls -la"}}'

check "edit install blocked"      2 guard_paths.sh "{\"tool_input\":{\"file_path\":\"$root/install/x.py\"}}"
check "edit src ok"               0 guard_paths.sh "{\"tool_input\":{\"file_path\":\"$root/src/a.py\",\"content\":\"x\"}}"
check "abs path launch blocked"   2 guard_paths.sh "{\"tool_input\":{\"file_path\":\"$root/src/p/launch/a.launch.py\",\"content\":\"p = '/home/u/ws/x.yaml'\"}}"

printf 'def f(:\n' >"$tmp/bad.py";  check "bad py flagged"   2 post_edit_lint.sh "{\"tool_input\":{\"file_path\":\"$tmp/bad.py\"}}"
printf 'x = 1\n'   >"$tmp/ok.py";   check "good py ok"       0 post_edit_lint.sh "{\"tool_input\":{\"file_path\":\"$tmp/ok.py\"}}"
printf '<a><b></a>' >"$tmp/x.xacro"; check "bad xacro flagged" 2 post_edit_lint.sh "{\"tool_input\":{\"file_path\":\"$tmp/x.xacro\"}}"
printf 'a: [1\n'   >"$tmp/x.yaml";  check "bad yaml flagged" 2 post_edit_lint.sh "{\"tool_input\":{\"file_path\":\"$tmp/x.yaml\"}}"

check "session status runs"       0 session_status.sh '{}'
check "subagent log no active"    0 subagent_log.sh '{"agent_type":"coder"}'
rm -rf "$tmp"
echo "passed=$pass failed=$fail"
[ $fail -eq 0 ]
