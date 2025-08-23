#!/bin/bash

timeout=600
blacklist=(cut-in_sumo.xosc sumo-test.xosc cut-in_sloppy.xosc cut-in_parameter_set.xosc car_walk.xosc)

# Run from esmini root directory: ./scripts/run_memory_leak_tests.sh

export workingDir=$(pwd)

export MEMORY_TEST_FOLDER=${workingDir}/test

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    export path="../../../bin"
    export EXE_FOLDER="."
    export PYTHON="python3"
else
    echo "Unsupported OS: " $OSTYPE
fi

cd $MEMORY_TEST_FOLDER

echo $'\n'Run memory tests:

if ! ${PYTHON} memory_leak_test.py -t "$timeout" -b "${blacklist[@]}"; then
    exit_with_msg "memory leak test failed"
fi
