#!/usr/bin/env bash

set -euo pipefail

REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_DIR}/build/ci}"
BUILD_JOBS="${BUILD_JOBS:-2}"

usage()
{
    printf 'Usage: %s [all|format|scripts|cppcheck|build|test|asan]\n' "${0##*/}"
    printf 'Run the same non-proof checks locally and in GitHub CI (default: all).\n'
    printf 'all stops at the first failure; it does not run Lean or ESBMC.\n'
    printf 'Build profile: CXX=g++-11, BUILD_TYPE=Release, QUAT_APPROX=OFF,\n'
    printf 'RESET_APPROX=OFF, MAG_TILT=ON, CODEGEN=ON; override via environment.\n'
    printf 'BUILD_DIR selects the native build; BUILD_JOBS defaults to 2.\n'
    printf 'ASan uses two test jobs by default; override CTEST_PARALLEL_LEVEL.\n'
    printf 'One invocation validates one native profile, not the whole CI matrix.\n'
}

check_scripts()
{
    shellcheck scripts/*.sh tests/tools/*.sh
    python3 -m compileall -q examples
    ./tests/tools/test_ci.sh
    ./tests/tools/test_esbmc.sh
    ./tests/tools/test_agent_review.sh
    python3 tests/tools/test_cpp_inventory.py
    python3 tests/tools/test_call_graph.py
}

build()
{
    [[ "${BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]] || {
        printf 'error: BUILD_JOBS must be a positive integer\n' >&2
        return 2
    }
    cmake -S "${REPO_DIR}" -B "${BUILD_DIR}" \
        -DCMAKE_CXX_COMPILER="${CXX:-g++-11}" \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE:-Release}" \
        -DCMAKE_CXX_FLAGS=-Werror \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DBUILD_TESTING=ON \
        -DFORMAL_ESKF_BUILD_EIGEN_BACKEND=ON \
        -DFORMAL_ESKF_BUILD_EXAMPLES=ON \
        -DESKF_QUAT_APPROX="${QUAT_APPROX:-OFF}" \
        -DESKF_RESET_APPROX="${RESET_APPROX:-OFF}" \
        -DESKF_MAG_TILT="${MAG_TILT:-ON}" \
        -DESKF_CODEGEN="${CODEGEN:-ON}" \
        -DESKF_ZUPT=ON
    cmake --build "${BUILD_DIR}" --parallel "${BUILD_JOBS}"
}

run_check()
{
    local CHECK="$1"
    printf '\n==> %s\n' "${CHECK}"
    case "${CHECK}" in
        format) ./scripts/format_cpp.sh --check ;;
        scripts) check_scripts ;;
        cppcheck) ./scripts/verify_cppcheck.sh ;;
        build) build ;;
        test) ./scripts/verify_tests.py --build-dir "${BUILD_DIR}" --no-build --ctest ;;
        asan) CXX="${CXX:-g++-11}" BUILD_JOBS="${BUILD_JOBS}" \
              CTEST_PARALLEL_LEVEL="${CTEST_PARALLEL_LEVEL:-2}" ./scripts/verify_asan.sh ;;
    esac
    printf '\e[32;01m[Pass]\e[0m %s\n' "${CHECK}"
}

main()
{
    if (($# > 1)); then
        usage >&2
        exit 2
    fi
    local CHECK="${1:-all}"
    case "${CHECK}" in
        --help|-h) usage; return ;;
        all|format|scripts|cppcheck|build|test|asan) ;;
        *) usage >&2; exit 2 ;;
    esac
    cd -- "${REPO_DIR}"
    if [[ "${CHECK}" == all ]]; then
        for CHECK in format scripts cppcheck build test asan; do
            run_check "${CHECK}"
        done
    else
        run_check "${CHECK}"
    fi
}

trap 'RESULT=$?; if ((RESULT != 0)); then printf "\e[31;01m[Fail]\e[0m CI checks (exit %d)\n" "${RESULT}" >&2; fi' EXIT
main "$@"
