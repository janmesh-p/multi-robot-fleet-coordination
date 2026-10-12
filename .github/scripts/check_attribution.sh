#!/usr/bin/env bash
# Fails when a commit message, or the pull request description in $PR_BODY, carries an
# attribution line: a Co-Authored-By trailer, a "<Name>-Session: <url>" link, a bare session link
# (a URL whose path has "/session_"), or a "Generated with [...]" footer. See CONTRIBUTING.md.
#
#   check_attribution.sh <base sha> <head sha>
#
# Checks base..head. When base is empty, all zeros (a new branch), or unknown, it checks the
# commits on head that are not on origin/main, or head alone if there are none.
set -euo pipefail

pattern='^[[:space:]]*co-authored-by:|^[[:space:]]*[a-z]+-session:[[:space:]]*https?://|https?://[^[:space:]]*/session_[[:alnum:]]|generated with \['
base="${1:-}"
head="${2:-HEAD}"

if [[ -n "$base" && ! "$base" =~ ^0+$ ]] && git cat-file -e "${base}^{commit}" 2>/dev/null; then
  commits=$(git rev-list "${base}..${head}")
else
  commits=""
  if git rev-parse --verify --quiet origin/main >/dev/null; then
    commits=$(git rev-list "origin/main..${head}")
  fi
  if [[ -z "$commits" ]]; then
    commits=$(git rev-parse "$head")
  fi
fi

status=0
checked=0
for commit in $commits; do
  checked=$((checked + 1))
  if git log -1 --format=%B "$commit" | grep -qiE "$pattern"; then
    echo "::error::commit $(git log -1 --format='%h %s' "$commit") has an attribution line:"
    git log -1 --format=%B "$commit" | grep -iE "$pattern" | sed 's/^/    /'
    status=1
  fi
done

if [[ -n "${PR_BODY:-}" ]] && printf '%s\n' "$PR_BODY" | grep -qiE "$pattern"; then
  echo "::error::the pull request description has an attribution line:"
  printf '%s\n' "$PR_BODY" | grep -iE "$pattern" | sed 's/^/    /'
  status=1
fi

echo "checked $checked commit(s)"
exit "$status"
