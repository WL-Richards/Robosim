#!/usr/bin/env bash
#
# Lint runner for robosim.
#
# Usage:
#   scripts/lint.sh                # check format + tidy + custom rules
#   scripts/lint.sh --fix          # apply clang-format -i to changed files
#   scripts/lint.sh --all          # run on every tracked C++ file, not
#                                  # just files changed vs origin/main
#
# Exits non-zero on any violation. Used by CI.
#
# Custom rules enforced here (until a real clang-tidy check exists):
#   - No std::chrono::system_clock or steady_clock in src/ (sim core).
#     Time access goes through the sim-time interface.

set -euo pipefail

ROOT_DIR="$(git rev-parse --show-toplevel)"
cd "$ROOT_DIR"

MODE_FIX=0
MODE_ALL=0
for arg in "$@"; do
  case "$arg" in
    --fix) MODE_FIX=1 ;;
    --all) MODE_ALL=1 ;;
    -h|--help)
      sed -n '3,16p' "$0"
      exit 0
      ;;
    *) echo "unknown arg: $arg" >&2; exit 2 ;;
  esac
done

# --- File set -------------------------------------------------------------

if [[ $MODE_ALL -eq 1 ]]; then
  mapfile -t FILES < <(git ls-files \
    'src/**/*.cpp' 'src/**/*.h' 'tests/**/*.cpp' 'tests/**/*.h')
else
  # Files changed vs origin/main, restricted to C++ sources we own.
  BASE_REF="${ROBOSIM_LINT_BASE:-origin/main}"
  if ! git rev-parse --verify --quiet "$BASE_REF" >/dev/null; then
    BASE_REF=HEAD
  fi
  mapfile -t FILES < <(git diff --name-only --diff-filter=ACMR "$BASE_REF" -- \
    'src/*.cpp' 'src/*.h' 'src/**/*.cpp' 'src/**/*.h' \
    'tests/*.cpp' 'tests/*.h' 'tests/**/*.cpp' 'tests/**/*.h')
fi

if [[ ${#FILES[@]} -eq 0 ]]; then
  echo "lint: no C++ files to check"
  exit 0
fi

echo "lint: checking ${#FILES[@]} file(s)"

# --- clang-format ---------------------------------------------------------

if ! command -v clang-format >/dev/null; then
  echo "lint: clang-format not found in PATH" >&2
  exit 3
fi

if [[ $MODE_FIX -eq 1 ]]; then
  clang-format -i "${FILES[@]}"
  echo "lint: clang-format applied"
else
  clang-format --dry-run --Werror "${FILES[@]}"
  echo "lint: clang-format clean"
fi

# --- Custom rule: banned clocks in sim core -------------------------------
#
# src/ is the sim-core surface. Banned identifiers may not appear in it.
# Tests are allowed to reference them (e.g. to write a test that asserts
# they don't get used at runtime).
#
# Exception: src/viz/ is interactive tooling, not sim core. The
# determinism rules (banned clocks / banned RNG) explicitly do not
# apply there — see .claude/skills/visualizer.md "Determinism
# exception." Wall-clock for animation, frame pacing, and GLFW input
# timestamps is acceptable.

BANNED_PATTERN='\bstd::chrono::(system_clock|steady_clock)\b'
SIM_CORE_FILES=()
for f in "${FILES[@]}"; do
  case "$f" in
    src/viz/*) ;;  # determinism exception — skip
    src/*) SIM_CORE_FILES+=("$f") ;;
  esac
done

if [[ ${#SIM_CORE_FILES[@]} -gt 0 ]]; then
  if grep -nE "$BANNED_PATTERN" "${SIM_CORE_FILES[@]}" >/dev/null 2>&1; then
    echo "lint: BANNED clock found in sim core (use sim-time interface instead):" >&2
    grep -nE "$BANNED_PATTERN" "${SIM_CORE_FILES[@]}" >&2
    exit 4
  fi
  echo "lint: banned-clock check clean"
fi

# --- clang-tidy -----------------------------------------------------------
#
# Runs against compile_commands.json from build/. Requires a configured
# build dir.

if ! command -v clang-tidy >/dev/null; then
  echo "lint: clang-tidy not found in PATH" >&2
  exit 3
fi

BUILD_DIR="${ROBOSIM_BUILD_DIR:-build}"
if [[ ! -f "$BUILD_DIR/compile_commands.json" ]]; then
  echo "lint: $BUILD_DIR/compile_commands.json missing — run cmake -B $BUILD_DIR first" >&2
  exit 5
fi

# Tidy only files with TUs (drop pure headers; clang-tidy will warn).
TIDY_FILES=()
for f in "${FILES[@]}"; do
  case "$f" in
    *.cpp) TIDY_FILES+=("$f") ;;
  esac
done

if [[ ${#TIDY_FILES[@]} -gt 0 ]]; then
  clang-tidy -p "$BUILD_DIR" --warnings-as-errors='*' "${TIDY_FILES[@]}"
  echo "lint: clang-tidy clean"
fi

echo "lint: ok"
