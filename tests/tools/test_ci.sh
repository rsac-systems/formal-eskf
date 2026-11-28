#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
FIXTURE="$(mktemp -d)"
trap 'rm -rf -- "${FIXTURE}"' EXIT
mkdir -p "${FIXTURE}/scripts" "${FIXTURE}/tests/tools" "${FIXTURE}/bin"
cp "${REPO_DIR}/scripts/verify_ci.sh" "${FIXTURE}/scripts/"

make_stub()
{
    local FILE="$1" STEP="$2"
    printf '#!/usr/bin/env bash\n' > "${FILE}"
    # Expand the fixture variables only when the generated stub runs.
    # shellcheck disable=SC2016
    printf 'printf "%%s\\n" "%s $*" >> "${TRACE}"\n' "${STEP}" >> "${FILE}"
    # shellcheck disable=SC2016
    printf 'if [[ "${FAIL_STEP:-}" == "%s" ]]; then exit 17; fi\n' "${STEP}" >> "${FILE}"
    chmod +x "${FILE}"
}

make_stub "${FIXTURE}/scripts/format_cpp.sh" format
make_stub "${FIXTURE}/scripts/verify_cppcheck.sh" cppcheck
make_stub "${FIXTURE}/scripts/verify_tests.py" test
make_stub "${FIXTURE}/scripts/verify_asan.sh" asan
make_stub "${FIXTURE}/bin/shellcheck" scripts
make_stub "${FIXTURE}/bin/python3" python
make_stub "${FIXTURE}/bin/cmake" build
for TEST in ci esbmc agent_review; do
    make_stub "${FIXTURE}/tests/tools/test_${TEST}.sh" offline
done

export PATH="${FIXTURE}/bin:${PATH}" TRACE="${FIXTURE}/trace"
RUNNER="${FIXTURE}/scripts/verify_ci.sh"
for STEP in format scripts cppcheck build test asan; do
    : > "${TRACE}"
    RESULT=0
    FAIL_STEP="${STEP}" "${RUNNER}" all > "${FIXTURE}/result" 2>&1 || RESULT=$?
    [[ "${RESULT}" == 17 ]] || { printf 'failed exit propagation: %s\n' "${STEP}" >&2; exit 1; }
    [[ "$(tail -n 1 "${TRACE}")" == "${STEP} "* ]] || {
        printf 'execution continued after failure: %s\n' "${STEP}" >&2; exit 1;
    }
    if grep -q "Pass.*${STEP}" "${FIXTURE}/result"; then
        printf 'false pass after failure: %s\n' "${STEP}" >&2
        exit 1
    fi
done

: > "${TRACE}"
FAIL_STEP='' "${RUNNER}" all > "${FIXTURE}/result"
[[ "$(grep -c 'Pass' "${FIXTURE}/result")" == 6 ]]
grep -q -- 'format --check' "${TRACE}"
grep -q -- '-DCMAKE_CXX_FLAGS=-Werror' "${TRACE}"
grep -q -- '--no-build --ctest' "${TRACE}"

: > "${TRACE}"
CXX=clang++-14 BUILD_TYPE=Debug QUAT_APPROX=ON RESET_APPROX=ON MAG_TILT=OFF CODEGEN=OFF \
    BUILD_JOBS=3 "${RUNNER}" build > "${FIXTURE}/result"
for OPTION in CMAKE_CXX_COMPILER=clang++-14 CMAKE_BUILD_TYPE=Debug ESKF_QUAT_APPROX=ON \
              ESKF_RESET_APPROX=ON ESKF_MAG_TILT=OFF ESKF_CODEGEN=OFF; do
    grep -q -- "-D${OPTION}" "${TRACE}"
done
grep -q -- '--parallel 3' "${TRACE}"
RESULT=0
BUILD_JOBS=0 "${RUNNER}" build > /dev/null 2>&1 || RESULT=$?
[[ "${RESULT}" == 2 ]]
RESULT=0
"${RUNNER}" unknown > /dev/null 2>&1 || RESULT=$?
[[ "${RESULT}" == 2 ]]
printf 'CI runner: pass (fail-fast, status propagation, and shared check commands)\n'
