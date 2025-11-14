#!/bin/bash

build_type=Release
add_performance_test=false
timeout=40

help_and_exit () {
    echo Usage: "$0" [options]
    echo options:
    echo "   -h, --help  this help"
    echo "   -b, --build_type <Release|Debug> (default: "$build_type")"
    echo "   -p, --add_performance_test (requires Release build type)"
    echo "   -t, --timeout <SECONDS> (default: "$timeout")"
    exit -1
}

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -h|--help) help_and_exit ;;
        -b|--build_type) build_type="$2"; shift ;;
        -p|--add_performance_test) add_performance_test=true ;;
        -t|--timeout) timeout="$2"; shift ;;
        *) echo "Unknown parameter passed: $1"; exit -1 ;;
    esac
    shift
done

echo "build_type: $build_type, timeout: $timeout, add_performance_test: $add_performance_test"

# Run from esmini root ddirectory: ./scripts/run_unittests.sh

exit_with_msg() {
    echo $1
    exit -1
}

export workingDir=$(pwd)

export LSAN_OPTIONS="print_suppressions=false:suppressions="${workingDir}"/scripts/LSAN.supp"
export ASAN_OPTIONS="detect_invalid_pointer_pairs=1:strict_string_checks=1:detect_stack_use_after_return=1:check_initialization_order=1:strict_init_order=1:fast_unwind_on_malloc=0:suppressions="${workingDir}"/scripts/ASAN.supp"

export UNIT_TEST_FOLDER=${workingDir}/build/EnvironmentSimulator/Unittest
export SMOKE_TEST_FOLDER=${workingDir}/test

if [[ "$OSTYPE" == "msys" ]]; then
    export PATH=${PATH}";../Libraries/esminiLib/$build_type;../Libraries/esminiRMLib/$build_type"
    export EXE_FOLDER="./$build_type"
    export PYTHON="python"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    export LD_LIBRARY_PATH=${workingDir}"/externals/OSI/linux/lib-dyn"
    export path="../../../bin"
    export EXE_FOLDER="."
    export PYTHON="python3"
    export ASAN_OPTIONS="$ASAN_OPTIONS:detect_leaks=1"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    export LD_LIBRARY_PATH=${workingDir}"/externals/OSI/mac/lib-dyn"
    export path="../../../bin"
    export EXE_FOLDER="."
    export PYTHON="python3"
else
    echo "Unsupported OS: " $OSTYPE
fi

if [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "linux-gnu"* ]]; then

    cd $UNIT_TEST_FOLDER

    echo $'\n'Run unit tests:

    echo $'\n'OperatingSystem_test:
    if ! ${EXE_FOLDER}/OperatingSystem_test --disable_stdout; then
        exit_with_msg "OperatingSystem_test failed"
    fi

    echo $'\n'CommonMini_test:
    if ! ${EXE_FOLDER}/CommonMini_test --disable_stdout; then
        exit_with_msg "CommonMini_test failed"
    fi

    echo $'\n'RoadManager_test:
    if ! ${EXE_FOLDER}/RoadManager_test --disable_stdout; then
        exit_with_msg "RoadManager_test failed"
    fi

    echo $'\n'ScenarioPlayer_test:
    if ! ${EXE_FOLDER}/ScenarioPlayer_test --disable_stdout; then
        exit_with_msg "ScenarioPlayer_test failed"
    fi

    echo $'\n'ScenarioEngineDll_test:
    if ! ${EXE_FOLDER}/ScenarioEngineDll_test --disable_stdout; then
        exit_with_msg "ScenarioEngineDll_test failed"
    fi

    echo $'\n'ScenarioEngine_test:
    if ! ${EXE_FOLDER}/ScenarioEngine_test --disable_stdout; then
        exit_with_msg "ScenarioEngine_test failed"
    fi

    ls -al *.tga *.ppm

    echo $'\n'RoadManagerDll_test:
    if ! ${EXE_FOLDER}/RoadManagerDll_test --disable_stdout; then
        exit_with_msg "RoadManagerDll_test failed"
    fi

    echo $'\n'FollowRoute_test:
    if ! ${EXE_FOLDER}/FollowRoute_test --disable_stdout; then
        exit_with_msg "FollowRoute_test failed"
    fi

    echo $'\n'FollowRouteController_test:
    if ! ${EXE_FOLDER}/FollowRouteController_test --disable_stdout; then
        exit_with_msg "FollowRouteController_test failed"
    fi

    echo $'\n'OSCGlobalAction_test:
    if ! ${EXE_FOLDER}/OSCGlobalAction_test --disable_stdout; then
        exit_with_msg "OSCGlobalAction_test failed"
    fi
fi

cd $SMOKE_TEST_FOLDER

echo $'\n'Run smoke tests:

if ! ${PYTHON} smoke_test.py "-t $timeout"; then
    exit_with_msg "smoke test failed"
fi

echo $'\n'Run ALKS test suite:

if ! ${PYTHON} alks_suite.py -t $timeout; then
    exit_with_msg "alks_suite test failed"
fi

echo $'\n'Run NCAP test suite:

if ! ${PYTHON} ncap_suite.py -t $timeout; then
    exit_with_msg "ncap_suite test failed"
fi

if  [[ "$add_performance_test" == true ]] && [[ "$build_type" == "Release" ]]; then
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        echo $'\n'Run performance test:
        if ! ${PYTHON} performance_test.py "-t $timeout" "--disable_plot"; then
            exit_with_msg "performance test failed"
        fi
    fi
fi
