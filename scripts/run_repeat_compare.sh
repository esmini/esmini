#!/bin/bash

# This script will execute a scenario multiple times and check expected identical result
#
# Usage: run_repeat_compare.sh <scenariofile> <number of repeats> [additional esmini launch arguments]
#
# On Windows you can run in Git bash
#
# Run from esmini root directory. Examples:
#   ./scrips/run_repeat_compare.sh ./resources/xosc/cut-in.xosc 5
#   ./scrips/run_repeat_compare.sh ./resources/xosc/cut-in.xosc 5 --path ./my_roads

if [ $# -lt 2 ]
  then
    echo "Usage: $0 <scenariofile> <number of repeats> [additional esmini launch arguments]"
    exit -1
fi

xosc=$1
nloops=$2
esmini_args="${@:3}"

echo Running scenario $xosc in esmini $nloops times
echo Additional esmini arguments: $esmini_args

for ((i=1;i<$nloops+1;i++)); do
    ./bin/esmini --headless --osc $xosc --fixed_timestep 0.05 --record sim_$i.dat --disable_stdout $esmini_args
    ./bin/dat2csv sim_$i.dat
    if [ $i -eq 1 ]; then
      # store initial run as reference
      mv sim_$i.dat sim_orig.dat
      mv sim_$i.csv sim_orig.csv
    else
      if diff sim_$i.csv sim_orig.csv; then
        # remove runs that were not different, preserve outliers
        rm sim_$i.dat sim_$i.csv
      fi
    fi
    echo -ne "\rRun $i/$nloops"
done

rm -f sim_orig.dat sim_orig.csv
