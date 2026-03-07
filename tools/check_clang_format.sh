#!/bin/bash
# Detect clang-format violations.
# Run via: bazel test //:format_check  or  bazel test //...
# tags = ["local"]: sandboxing disabled; uses realpath to locate workspace root.
set -euo pipefail

# Resolve symlink and derive workspace root from tools/ parent directory
SCRIPT_REAL="$(realpath "${BASH_SOURCE[0]}")"
WORKSPACE="$(cd "$(dirname "${SCRIPT_REAL}")/.." && pwd)"

if [[ ! -f "${WORKSPACE}/MODULE.bazel" ]]; then
  echo "Error: workspace root not found (MODULE.bazel missing: ${WORKSPACE})" >&2
  exit 1
fi

cd "${WORKSPACE}"

ERRORS=0
while IFS= read -r -d '' f; do
  if ! clang-format --dry-run --Werror "${f}" 2>/dev/null; then
    echo "  violation: ${f}"
    ERRORS=$((ERRORS + 1))
  fi
done < <(find . \( -name '*.cpp' -o -name '*.hpp' \) -not -path './bazel-*' -print0)

if [[ "${ERRORS}" -eq 0 ]]; then
  echo "clang-format check OK (no violations)"
else
  echo ""
  echo "clang-format violations: ${ERRORS} file(s)"
  echo "  To fix: bazel run //:format"
  exit 1
fi
