#!/bin/bash
# Apply clang-format to all C++ source and header files.
# Run via: bazel run //:format  ($BUILD_WORKSPACE_DIRECTORY is required)
set -euo pipefail

if [[ -z "${BUILD_WORKSPACE_DIRECTORY:-}" ]]; then
  echo "Error: \$BUILD_WORKSPACE_DIRECTORY is not set." >&2
  echo "       Run as: bazel run //:format" >&2
  exit 1
fi

cd "${BUILD_WORKSPACE_DIRECTORY}"

find . \( -name '*.cpp' -o -name '*.hpp' \) \
  -not -path './bazel-*' \
  -exec clang-format -i {} +

echo "clang-format applied."
