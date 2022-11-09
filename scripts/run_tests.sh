#!/bin/bash

# Run from esmini root ddirectory: ./scripts/run_unittests.sh

exit_with_msg() {
    echo $1
    exit -1
}

workingDir=$(pwd)

LSAN_OPTIONS="print_suppressions=false:suppressions="${workingDir}"/scripts/LSAN.supp"
ASAN_OPTIONS="detect_invalid_pointer_pairs=1:strict_string_checks=true:detect_stack_use_after_return=true:check_initialization_order=true:fast_unwind_on_malloc=false:suppressions="${workingDir}"/scripts/ASAN.supp"

UNIT_TEST_FOLDER=${workingDir}/build/EnvironmentSimulator/Unittest
SMOKE_TEST_FOLDER=${workingDir}/test

if [[ "$OSTYPE" == "msys" ]]; then
    PATH=${PATH}";../Libraries/esminiLib/Release;../Libraries/esminiRMLib/Release"
    EXE_FOLDER="./Release"
    PYTHON="python"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    LD_LIBRARY_PATH=${workingDir}"/externals/OSI/linux/lib-dyn"
    path="../../../bin"
    EXE_FOLDER="."
    PYTHON="python3"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    LD_LIBRARY_PATH=${workingDir}"/externals/OSI/mac/lib-dyn"
    path="../../../bin"
    EXE_FOLDER="."
    PYTHON="python3"
else
    echo "Unsupported OS: " $OSTYPE
fi

if [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "linux-gnu"* ]]; then

    cd $UNIT_TEST_FOLDER

    echo $'\n'Run unit tests:

    if ! ${EXE_FOLDER}/OperatingSystem_test; then
        exit_with_msg "OperatingSystem_test failed"
    fi

    if ! ${EXE_FOLDER}/CommonMini_test; then
        exit_with_msg "CommonMini_test failed"
    fi

    if ! ${EXE_FOLDER}/RoadManager_test; then
        exit_with_msg "RoadManager_test failed"
    fi

    if ! ${EXE_FOLDER}/ScenarioPlayer_test; then
        exit_with_msg "ScenarioPlayer_test failed"
    fi

    if ! ${EXE_FOLDER}/ScenarioEngineDll_test; then
        exit_with_msg "ScenarioEngineDll_test failed"
    fi
    ls -al *.tga *.ppm

    if ! ${EXE_FOLDER}/RoadManagerDll_test; then
        exit_with_msg "RoadManagerDll_test failed"
    fi

    if ! ${EXE_FOLDER}/FollowRoute_test; then
        exit_with_msg "FollowRoute_test failed"
    fi

    if ! ${EXE_FOLDER}/FollowRouteController_test; then
        exit_with_msg "FollowRouteController_test failed"
    fi
fi

cd $SMOKE_TEST_FOLDER

echo $'\n'Run smoke tests:

if ! ${PYTHON} smoke_test.py; then
    exit_with_msg "smoke test failed"
fi

echo $'\n'Run ALKS test suite:

if ! ${PYTHON} alks_suite.py; then
    exit_with_msg "alks_suite test failed"
fi
