#!/bin/bash
source /opt/ros/humble/setup.bash

# ==============================================================================
# Copyright 2026 vladubase
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

# ==============================================================================
# format_code.bash
# Purpose: Brings the workspace into the state that run_ci_checks.bash accepts.
#   1. Auto-formatting: uncrustify (C++), black (Python), trailing whitespace (CMake).
#   2. Verification: runs exactly the ament linters of the CI test stage (ctest label
#      "linter", configured per package in CMakeLists.txt, so the settings have a single
#      source of truth) and lists every remaining finding as one line.
#      Findings listed there have no automatic fixer (cpplint, lint_cmake, flake8 rules
#      beyond black, copyright, xmllint, cppcheck) and must be fixed by hand.
# Exit code: 0 = formatted and all CI linters pass; 1 = a tool failed or findings remain.
# Unit tests (gtest) are not part of this script; run_ci_checks.bash runs them.
# ==============================================================================

# Same cppcheck policy as run_ci_checks.bash: cppcheck 2.x runs, it is never skipped
export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

# Line limit for Python formatting; must match max-line-length in configs/setup.cfg
readonly PYTHON_LINE_LENGTH=120

# Define colors for output clarity
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Prints an error and stops the script; no tool failure is reported as success
fail() {
    echo -e "${RED}[ERROR] $1${NC}" >&2
    exit 1
}

echo -e "${BLUE}[INFO] Starting Code Formatting sequence...${NC}"

# 0. Pre-flight Check
if [ ! -d "src" ]; then
    echo "Please run this script from the root of your ROS 2 workspace."
    fail "Directory 'src' not found."
fi

# ------------------------------------------------------------------------------
# 1. Auto-formatting
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[1/3] Auto-formatting code to match standards...${NC}"

# C++ Formatting (uncrustify)
command -v ament_uncrustify &> /dev/null || fail "ament_uncrustify not found."

# The exit code of --reformat reports the divergence found before rewriting, so it is
# ignored here; a second run without --reformat verifies that the result is accepted.
ament_uncrustify -c configs/uncrustify.cfg --reformat src/ > /dev/null 2>&1
if ! uncrustify_report="$(ament_uncrustify -c configs/uncrustify.cfg src/ 2>&1)"; then
    echo "${uncrustify_report}" | grep -E "divergence in file|rror" >&2
    fail "uncrustify could not bring all C++ files into the configured style (see above)."
fi
echo -e "${GREEN}[OK] C++ formatted (ament_uncrustify).${NC}"

# Python Formatting (Black)
python3 -m black --version &> /dev/null || fail "Black formatter not found. Run 'pip install black' locally."
if ! python3 -m black --quiet --line-length "${PYTHON_LINE_LENGTH}" src/ scripts/; then
    fail "Black failed (a Python file probably contains a syntax error)."
fi
echo -e "${GREEN}[OK] Python formatted (Black).${NC}"

# CMake: strip trailing whitespace (lint_cmake has no automatic fixer for other rules)
find src configs \( -name "CMakeLists.txt" -o -name "*.cmake" \) -exec sed -i 's/[[:space:]]*$//' {} +
echo -e "${GREEN}[OK] CMake trailing whitespace removed.${NC}"

# ------------------------------------------------------------------------------
# 2. Build tree for the linter tests
# ------------------------------------------------------------------------------
# The linter tests are registered when CMake configures a package, so they need a
# configured build tree. An incremental build also picks up changed CMakeLists.txt.
echo -e "${YELLOW}[2/3] Updating the build tree for the CI linters...${NC}"
if ! colcon build --symlink-install > /dev/null 2>&1; then
    fail "colcon build failed; the CI linters need a configured build tree. Run 'make ci' for details."
fi
echo -e "${GREEN}[OK] Build tree up to date.${NC}"

# ------------------------------------------------------------------------------
# 3. Verification with the CI linters
# ------------------------------------------------------------------------------
echo -e "${YELLOW}[3/3] Running the CI linters (ctest label 'linter')...${NC}"
colcon test --ctest-args -L linter > /dev/null 2>&1

# One line per finding: linter, file:line, message, [rule]. The ament linters store a
# finding as a test case named "rule (location)" or "location", with the description in
# the failure message. Locations are shown relative to the workspace (src/<package>/...).
if ! python3 - << 'PYTHON'
import glob
import re
import sys
import xml.etree.ElementTree as ElementTree

LINTERS = ('uncrustify', 'cpplint', 'cppcheck', 'lint_cmake', 'flake8', 'xmllint', 'copyright')

def workspace_path(location, package):
    # Absolute paths are cut to src/...; package-relative paths get the package prefix
    if '/src/' in location:
        return 'src/' + location.split('/src/', 1)[1]
    return f'src/{package}/' + re.sub(r'^\./', '', location)

findings = []
for result in sorted(glob.glob('build/*/test_results/*/*.xunit.xml')):
    linter = result.rsplit('/', 1)[-1].split('.')[0]
    if linter not in LINTERS:
        continue
    package = result.split('/')[1]
    for case in ElementTree.parse(result).getroot().iter('testcase'):
        for failure in case.iter('failure'):
            name = case.get('name', '?')
            match = re.match(r'^(.*?) \((.+)\)$', name)
            rule, location = (match.group(1), match.group(2)) if match else ('', name)
            message = (failure.get('message') or failure.text or 'failed').strip()
            message = message.splitlines()[0].rstrip(':') if message else 'failed'
            line = f'{linter:<11} {workspace_path(location, package)}  {message}'
            findings.append(line + (f'  [{rule}]' if rule else ''))

for finding in dict.fromkeys(findings):
    print(finding)
sys.exit(1 if findings else 0)
PYTHON
then
    fail "The findings above have no automatic fixer; fix them by hand, then run this script again."
fi

echo -e "${GREEN}[SUCCESS] All files formatted and all CI linters pass.${NC}"
