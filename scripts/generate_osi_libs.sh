#!/bin/bash

#
# This script will build Protobuf and Open Simulation Interface (OSI) libraries
# needed for esmini to support OSI and output Ground Truth messages.
# No system installations will be done (no admin rights required)
#
# The ambition is to support Windows, Linux and Mac. However the script is under
# development and has not been tested as is on all three platforms.
#
# Prerequisites:
# - Git (with Bash) (https://git-scm.com/download/win)
# - cmake (https://cmake.org/download/)
# - Visual Studio (with C++ toolkit) (https://visualstudio.microsoft.com/downloads/)
#
# Usage:
# - put this script in an empty folder
# - open bash (e.g. Git Bash) in that folder
# - review and adjust system dependent parameters in section below
# - run the script: ./generate_osi_libs.sh
# - wait (the build process will take approx. 15 minutes depending on...)
#
# The osi_*.7z will contain both headers and needed libraries. (* depends on platform)
#
#

# -----------------------------------------------------------------------------------
# Review and update settings in this section according to your system and preferences

PROTOBUF_VERSION=3.15.2
OSI_VERSION=3.5.0
ZIP_MIN_VERSION=13
PARALLEL_BUILDS=4

if (( "$PARALLEL_BUILDS" < 2 )); then
    PARALLEL_ARG=""
else
    PARALLEL_ARG="$PARALLEL_BUILDS"
fi

if [ "$OSTYPE" == "msys" ]; then
    # Visual Studio 2022 using toolkit from Visual Studio 2017
    GENERATOR=("Visual Studio 17 2022")
    GENERATOR_TOOLSET="v142"
    GENERATOR_ARGUMENTS="-A x64 -T ${GENERATOR_TOOLSET}"

    # Visual Studio 2019 using default toolkit
    # GENERATOR=("Visual Studio 16 2019")
    # GENERATOR_ARGUMENTS="-A x64 -T ${GENERATOR_TOOLSET}"

    # Visual Studio 2017 - default toolkit
    # GENERATOR=("Visual Studio 15 2017 Win64")
    # GENERATOR_ARGUMENTS="-T ${GENERATOR_TOOLSET}"
elif [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Unix Makefiles (for Ubuntu and other Linux systems)
    GENERATOR=("Unix Makefiles")
    GENERATOR_ARGUMENTS=""
else
    echo Unknown OSTYPE: $OSTYPE
fi

if [ "$OSTYPE" == "msys" ]; then
    target_dir="v10"
    zfilename="osi_v10.7z"
    z_exe="/c/Program Files/7-Zip/7z.exe"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    target_dir="linux"
    zfilename="osi_linux.7z"
    z_exe=7za
elif [[ "$OSTYPE" == "darwin"* ]]; then
    target_dir="mac"
    zfilename="osi_mac.7z"
    z_exe=7zz
    macos_arch="arm64;x86_64"
else
    echo Unknown OSTYPE: $OSTYPE
fi

if [ ! -d $target_dir ]; then
    mkdir $target_dir
fi
if [ ! -d $target_dir/include ]; then
    mkdir $target_dir/include
fi

# ---------------------------------------------------------------------------------------
# From this point no adjustments should be necessary, except fixing bugs in the script :)
# However you might want to adjust versions of software packages being checkout and built


osi_root_dir=$(pwd)


echo ------------------------ Installing zlib ------------------------------------
cd $osi_root_dir

if [ ! -d zlib ]; then
    git clone https://github.com/madler/zlib.git --depth 1 --branch v1.2.$ZIP_MIN_VERSION
    cd  zlib
    mkdir install
    mkdir build
    cd build

    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Debug .. -DCMAKE_C_FLAGS="-fPIC"
        cmake --build . $PARALLEL_ARG --target install
        mv ../install/lib/libz.a ../install/lib/libzd.a

        rm CMakeCache.txt
        cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Release .. -DCMAKE_C_FLAGS="-fPIC"
        cmake --build . $PARALLEL_ARG --target install
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Release .. -DCMAKE_C_FLAGS="-fPIC" -DCMAKE_OSX_ARCHITECTURES="$macos_arch"
        cmake --build . $PARALLEL_ARG --target install
    else
        cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install ..
        cmake --build . $PARALLEL_ARG --config Debug --target install
        cmake --build . $PARALLEL_ARG --config Release --target install --clean-first
    fi

else
    echo zlib folder already exists, continue with next step...
fi

echo ------------------------ Installing OSI proto2cpp -----------------------------------

cd $osi_root_dir

if [ ! -d proto2cpp ]; then
    git clone https://github.com/OpenSimulationInterface/proto2cpp.git
else
    echo proto2cpp folder already exists, continue with next step...
fi

function build {

    cd $osi_root_dir

    if [[ $1 == "static" ]]; then
        DYNAMIC_LINKING=0
        folder_postfix="_static"
    elif [[ $1 == "dynamic" ]]; then
        DYNAMIC_LINKING=1
        folder_postfix="_dynamic"
    else
        echo Unexpected build type: $1
        exit
    fi


    echo ------------------------ Installing Protobuf $1 ------------------------------------
    cd $osi_root_dir

    if [ ! -d protobuf$folder_postfix ]; then
        git clone https://github.com/protocolbuffers/protobuf.git --depth 1 --branch v$PROTOBUF_VERSION protobuf$folder_postfix
        cd protobuf$folder_postfix
        mkdir build-code
        cd build-code

        export INSTALL_PROTOBUF_DIR=../protobuf-install

        if [[ "$OSTYPE" == "linux-gnu"* ]] || [[ "$OSTYPE" == "darwin"* ]]; then
            ZLIB_FILE_RELEASE=libz.a
            ZLIB_FILE_DEBUG=libzd.a
        elif [ "$OSTYPE" == "msys" ]; then
            ZLIB_FILE_RELEASE=zlib.lib
            ZLIB_FILE_DEBUG=zlibd.lib
        fi

        if [ $DYNAMIC_LINKING == "1" ]; then
            if [ "$OSTYPE" == "msys" ]; then
                ADDITIONAL_CMAKE_PARAMETERS="-Dprotobuf_BUILD_SHARED_LIBS=ON -DCMAKE_CXX_FLAGS=-DPROTOBUF_USE_DLLS"
            else
                ADDITIONAL_CMAKE_PARAMETERS="-Dprotobuf_BUILD_SHARED_LIBS=ON -DCMAKE_CXX_FLAGS=-DPROTOBUF_USE_DLLS -DCMAKE_CXX_FLAGS=-fPIC"
            fi
        else
            BUILD_SHARED_LIBS="OFF"
            if [ "$OSTYPE" == "msys" ]; then
                ADDITIONAL_CMAKE_PARAMETERS="-Dprotobuf_BUILD_SHARED_LIBS=OFF"
            else
                ADDITIONAL_CMAKE_PARAMETERS="-Dprotobuf_BUILD_SHARED_LIBS=OFF -DCMAKE_CXX_FLAGS=-fPIC"
            fi
        fi

        if [[ "$OSTYPE" != "darwin"* ]]; then
            cmake ../cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DZLIB_LIBRARY=../../zlib/install/lib/$ZLIB_FILE_DEBUG -DZLIB_INCLUDE_DIR=../../zlib/install/include -DCMAKE_INSTALL_PREFIX=$INSTALL_PROTOBUF_DIR -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_WITH_ZLIB=ON -Dprotobuf_MSVC_STATIC_RUNTIME=OFF -DCMAKE_BUILD_TYPE=Debug $ADDITIONAL_CMAKE_PARAMETERS
            cmake --build . $PARALLEL_ARG --config Debug --target install --clean-first
            rm CMakeCache.txt
        else
            ADDITIONAL_CMAKE_PARAMETERS+=" -DCMAKE_OSX_ARCHITECTURES=$macos_arch"
        fi

        cmake ../cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DZLIB_LIBRARY=../../zlib/install/lib/$ZLIB_FILE_RELEASE -DZLIB_INCLUDE_DIR=../../zlib/install/include -DCMAKE_INSTALL_PREFIX=$INSTALL_PROTOBUF_DIR -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_WITH_ZLIB=ON -Dprotobuf_MSVC_STATIC_RUNTIME=OFF -DCMAKE_BUILD_TYPE=Release $ADDITIONAL_CMAKE_PARAMETERS
        cmake --build . $PARALLEL_ARG --config Release --target install --clean-first

    else
        echo protobuf folder already exists, continue with next step...
    fi


    echo --------------------- Installing OSI $1 -----------------------------

    cd $osi_root_dir

    if [ ! -d open-simulation-interface$folder_postfix ]; then
        git clone https://github.com/OpenSimulationInterface/open-simulation-interface.git --depth 1 --branch v$OSI_VERSION open-simulation-interface$folder_postfix
        cd open-simulation-interface$folder_postfix
        sh ./convert-to-proto3.sh
        mkdir build
        cd build

        INSTALL_ROOT_DIR=../install
        INSTALL_INCLUDE_DIR=$INSTALL_ROOT_DIR/include
        export PATH=$PATH:../../graphviz/release/bin:../../protobuf$folder_postfix/protobuf-install/bin
        if [ "$OSTYPE" == "msys" ]; then
            PROTOC_EXE="../../protobuf_$1/protobuf-install/bin/protoc.exe"
        else
            PROTOC_EXE="../../protobuf_$1/protobuf-install/bin/protoc"
        fi

        if [ $DYNAMIC_LINKING == "1" ]; then
            ADDITIONAL_CMAKE_PARAMETERS="-DCMAKE_CXX_FLAGS=-DPROTOBUF_USE_DLLS"
        else
            ADDITIONAL_CMAKE_PARAMETERS=""
        fi

        mkdir $INSTALL_ROOT_DIR
        mkdir $INSTALL_ROOT_DIR/debug
        mkdir $INSTALL_ROOT_DIR/release

        COMMON_ARGS=".. -D CMAKE_INCLUDE_PATH=../protobuf$folder_postfix/protobuf-install/include -D PROTOBUF_SRC_ROOT_FOLDER=../../protobuf_$1/ -D Protobuf_PROTOC_EXECUTABLE=$PROTOC_EXE -D CMAKE_VERBOSE_MAKEFILE=ON -D CMAKE_LIBRARY_PATH=../protobuf$folder_postfix/protobuf-install/lib -D CMAKE_CXX_STANDARD=11"

        if [[ "$OSTYPE" != "darwin"* ]]; then
            # Build debug variant first
            cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} $COMMON_ARGS -D CMAKE_BUILD_TYPE=Debug -D CMAKE_INSTALL_PREFIX=$INSTALL_ROOT_DIR/debug $ADDITIONAL_CMAKE_PARAMETERS ..
            cmake --build . $PARALLEL_ARG --config Debug --target install
            rm CMakeCache.txt
        else
            ADDITIONAL_CMAKE_PARAMETERS+=" -DCMAKE_OSX_ARCHITECTURES=$macos_arch"
        fi

        cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} $COMMON_ARGS -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=$INSTALL_ROOT_DIR/release $ADDITIONAL_CMAKE_PARAMETERS ..
        cmake --build . $PARALLEL_ARG --config Release --target install --clean-first
    else
        echo open-simulation-interface folder already exists, continue with next step...
    fi

    echo ------------------------ Pack $1 ------------------------------------

    cd $osi_root_dir

    cp open-simulation-interface$folder_postfix/VERSION $target_dir
    cp open-simulation-interface$folder_postfix/install/release/include/osi3/* $target_dir/include
    cp -r protobuf$folder_postfix/protobuf-install/include/google $target_dir/include

    if [ $DYNAMIC_LINKING == "1" ]; then
        target_lib_dir_root=$target_dir/lib-dyn
    else
        target_lib_dir_root=$target_dir/lib
    fi
    mkdir $target_lib_dir_root
    variants=("debug" "release")
    for variant in "${variants[@]}"; do

	if [[ "$OSTYPE" == "darwin"* ]] && [[ $variant == "debug" ]]; then
	    continue
	fi
        target_lib_dir=$target_lib_dir_root/$variant
        mkdir $target_lib_dir

        if [ $DYNAMIC_LINKING == "1" ]; then

            if [ "$OSTYPE" == "msys" ]; then
                cp open-simulation-interface$folder_postfix/install/$variant/lib/osi3/open_simulation_interface.dll $target_lib_dir
                cp open-simulation-interface$folder_postfix/install/$variant/lib/osi3/open_simulation_interface_pic.lib $target_lib_dir
                if [ $variant == "debug" ]; then
                    cp protobuf$folder_postfix/protobuf-install/lib/libprotobufd.lib $target_lib_dir
                    cp protobuf$folder_postfix/protobuf-install/bin/libprotobufd.dll $target_lib_dir
                else
                    cp protobuf$folder_postfix/protobuf-install/lib/libprotobuf.lib $target_lib_dir
                    cp protobuf$folder_postfix/protobuf-install/bin/libprotobuf.dll $target_lib_dir
                fi
            elif [[ "$OSTYPE" == "darwin"* ]]; then
                cp -P open-simulation-interface$folder_postfix/install/$variant/lib/osi3/libopen_simulation_interface*.dylib $target_lib_dir
                if [ $variant == "debug" ]; then
                    cp -P protobuf$folder_postfix/protobuf-install/lib/libprotobufd*.dylib $target_lib_dir
                else
                    cp -P protobuf$folder_postfix/protobuf-install/lib/libprotobuf*.dylib $target_lib_dir
                fi
            else
                cp -P open-simulation-interface$folder_postfix/install/$variant/lib/osi3/libopen_simulation_interface.so* $target_lib_dir
                if [ $variant == "debug" ]; then
                    cp -P protobuf$folder_postfix/protobuf-install/lib*/libprotobufd.so* $target_lib_dir
                else
                    cp -P protobuf$folder_postfix/protobuf-install/lib*/libprotobuf.so* $target_lib_dir
                fi
            fi
        else
            if [ "$OSTYPE" == "msys" ]; then
                cp open-simulation-interface$folder_postfix/install/$variant/lib/osi3/*open_simulation_interface_pic.lib $target_lib_dir
                if [ $variant == "debug" ]; then
                    cp protobuf$folder_postfix/protobuf-install/lib/libprotobufd.lib $target_lib_dir
                else
                    cp protobuf$folder_postfix/protobuf-install/lib/libprotobuf.lib $target_lib_dir
                fi
            else
                cp open-simulation-interface$folder_postfix/install/$variant/lib/osi3/*open_simulation_interface_pic.a $target_lib_dir
                if [ $variant == "debug" ]; then
                    cp protobuf$folder_postfix/protobuf-install/lib/libprotobufd.a $target_lib_dir
                else
                    cp protobuf$folder_postfix/protobuf-install/lib/libprotobuf.a $target_lib_dir
                fi
            fi
        fi
        rm -f $target_lib_dir/libprotobuf-lite*
    done
}

build static
build dynamic

"$z_exe" a -r $zfilename -m0=LZMA -bb1 -spf -snl $target_dir/*
# unpack with: 7z x <filename>

echo ------------------------ Done ------------------------------------
cd $osi_root_dir
