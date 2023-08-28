#!/bin/bash

#
# This script will run the build scripts for OSI, OSG and SUMO
# needed for esmini.
# No system installations will be done (no admin rights required)
#
# The ambition is to support Windows, Linux and Mac. However the script is under
# development and has not been tested as is on all three platforms.
# This script should only be used for specific architectures (e.g. aarch64 for linux),
# in all other cases esmini build process should download the right completely built packages
#
# Prerequisites:
# - look into the single build scripts generate_osg_libs.sh, generate_osi_libs.sh and
#   generate_sumo_libs.sh
#
# Usage:
# - go into esmini-root-folder/externals
# - open bash (e.g. Git Bash) in that folder
# - run the script: ../scripts/generate_osi_sumo_osg_libs.sh
# - wait (the build process will take approx. 45 minutes depending on...)
#
# The folders SUMO/, OpenSceneGrapgh/ and OSI/ will be created as needed for esmini
#
#
# -----------------------------------------------------------------------------------
# Review and update settings in this section according to your system and preferences

BASE_DIR=`dirname $(realpath $0)`
echo $BASE_DIR
OSG_BUILD_DIR=OpenSceneGraph
OSI_BUILD_DIR=OSI
SUMO_BUILD_DIR=SUMO
BUILD_SUB_DIR=manualBuild
OSG_BUILD_SCRIPT=$BASE_DIR/generate_osg_libs.sh
OSI_BUILD_SCRIPT=$BASE_DIR/generate_osi_libs.sh
SUMO_BUILD_SCRIPT=$BASE_DIR/generate_sumo_libs.sh

if [ "$OSTYPE" == "msys" ]; then
    target_dir="v10"
    z_exe="/c/Program Files/7-Zip/7z.exe"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    target_dir="linux"
    z_exe=7za
elif [[ "$OSTYPE" == "darwin"* ]]; then
    target_dir="mac"
    z_exe=7z
else
    echo Unknown OSTYPE: $OSTYPE
fi

if [[ ! -d "$OSG_BUILD_DIR/$target_dir" ]]; then
    if [ ! -d $OSG_BUILD_DIR ]; then
    	mkdir "$OSG_BUILD_DIR"
    fi
    if [[ ! -d "$OSG_BUILD_DIR/$BUILD_SUB_DIR" ]]; then
    	mkdir "$OSG_BUILD_DIR/$BUILD_SUB_DIR"
    fi
    cd "$OSG_BUILD_DIR/$BUILD_SUB_DIR"
    $OSG_BUILD_SCRIPT
    cd ..
    "$z_exe" x "$BUILD_SUB_DIR/osg_$target_dir.7z"
    rm -rf "$BUILD_SUB_DIR"
    cd ..
fi
if [[ ! -d "$OSI_BUILD_DIR/$target_dir" ]]; then
    if [[ ! -d "$OSI_BUILD_DIR" ]]; then
    	mkdir "$OSI_BUILD_DIR"
    fi
    if [[ ! -d "$OSI_BUILD_DIR/$BUILD_SUB_DIR" ]]; then
    	mkdir "$OSI_BUILD_DIR/$BUILD_SUB_DIR"
    fi
    cd "$OSI_BUILD_DIR/$BUILD_SUB_DIR"
    $OSI_BUILD_SCRIPT
    cd ..
    "$z_exe" x "$BUILD_SUB_DIR/osi_$target_dir.7z"
    rm -rf "$BUILD_SUB_DIR"
    cd ..
fi
if [[ ! -d "$SUMO_BUILD_DIR/$target_dir" ]]; then
    if [[ ! -d "$SUMO_BUILD_DIR" ]]; then
    	mkdir "$SUMO_BUILD_DIR"
    fi
    if [[ ! -d "$SUMO_BUILD_DIR/$BUILD_SUB_DIR" ]]; then
    	mkdir "$SUMO_BUILD_DIR/$BUILD_SUB_DIR"
    fi
    cd "$SUMO_BUILD_DIR/$BUILD_SUB_DIR"
    $SUMO_BUILD_SCRIPT
    cd ..
    "$z_exe" x "$BUILD_SUB_DIR/sumo_$target_dir.7z"
    rm -rf "$BUILD_SUB_DIR"
    cd ..
fi
