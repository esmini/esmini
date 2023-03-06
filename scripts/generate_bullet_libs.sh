#!/bin/bash

#
# This script will build Bullet SDK libraries
# needed for esmini dynamics controller which adds semi-reaslistic vehicle dynamics
# No system installations will be done (no admin rights required)
#
# While the ambition is to support Windows, Linux and Mac the script is under
# development and has not been tested thoroughly on all three platforms. But
# hopefully it can at least provide some guidance and ideas.
#
# Prerequisites:
# - Git (with Bash) (https://git-scm.com/download/win)
# - cmake (https://cmake.org/download/)
# - compiler (Visual Studio with C++ toolkit, gcc, xcode...)
#
# Usage:
# - put this script in an empty folder
# - open bash (e.g. Git Bash) in that folder
# - review and adjust system dependent parameters in section below
# - run the script: ./generate_bullet_libs.sh
# - wait (the build process will take approx. 15 minutes depending on...)
#
# The osi_*.7z will contain both headers and needed libraries. (* depends on platform)
#
#

# -----------------------------------------------------------------------------------
# Review and update settings in this section according to your system and preferences

PARALLEL_BUILDS=4
ZIP_MIN_VERSION=12

if [ "$OSTYPE" == "msys" ]; then
    # Visual Studio 2019 - toolkit from Visual Studio 2017
    GENERATOR=("Visual Studio 16 2019")
    GENERATOR_TOOLSET="v141"
    GENERATOR_ARGUMENTS="-A x64 -T ${GENERATOR_TOOLSET}"

    # Visual Studio 2019 - default toolkit
    # GENERATOR=("Visual Studio 16 2019")
    # GENERATOR_ARGUMENTS="-A x64 -T ${GENERATOR_TOOLSET}"

    # Visual Studio 2017 - default toolkit
    # GENERATOR=("Visual Studio 15 2017 Win64")
    # GENERATOR_ARGUMENTS="-T ${GENERATOR_TOOLSET}"

    # Make sure 7zip is available, else download and install it
    # https://www.7-zip.org/download.html

    LIB_EXT="lib"
    LIB_PREFIX=""

    target_dir="win"
    zfilename="bullet_win.7z"
    z_exe="$PROGRAMFILES/7-Zip/7z"

elif [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Unix Makefiles (for Ubuntu and other Linux systems)
    GENERATOR=("Unix Makefiles")
    GENERATOR_ARGUMENTS=""
    LIB_EXT="a"
    LIB_PREFIX="lib"
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        target_dir="linux"
        zfilename="bullet_linux.7z"
        z_exe=7za
    else
        target_dir="mac"
        zfilename="bullet_mac.7z"
        z_exe=7z
        macos_arch="arm64;x86_64"
    fi
else
    echo Unknown OSTYPE: $OSTYPE
fi

BULLET_VERSION=0e59474e1cf1fd69e3ca512826cfbe15cd6a9cec

# ---------------------------------------------------------------------------------------
# From this point no adjustments should be necessary, except fixing bugs in the script :)
# However you might want to adjust versions of software packages being checkout and built

root_dir=$(pwd)

echo ------------------------ Installing ------------------------------------

if [ ! -d bullet3 ]; then
    git clone https://github.com/bulletphysics/bullet3.git
fi

if [ ! -d bullet3/build ]; then

    cd bullet3

    git checkout $BULLET_VERSION

    mkdir build
    cd build

    COMMON_ARGS="-DINSTALL_LIBS=True"
    if [ "$OSTYPE" == "msys" ]; then
        COMMON_ARGS="$COMMON_ARGS -DUSE_MSVC_RUNTIME_LIBRARY_DLL=True"
    fi
    echo $COMMON_ARGS

    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        COMMON_ARGS2=" -DCMAKE_CXX_FLAGS=-fPIC"
        cmake ../ ${COMMON_ARGS} ${COMMON_ARGS2} -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
        make -j$PARALLEL_BUILDS install

        # build debug variant
        rm CMakeCache.txt
        cmake ../ ${COMMON_ARGS} ${COMMON_ARGS2} -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=../install-debug
        make -j$PARALLEL_BUILDS install

    elif [[ "$OSTYPE" == "darwin"* ]]; then
        cmake ../ ${COMMON_ARGS} -DCMAKE_OSX_ARCHITECTURES=$macos_arch -DCMAKE_CXX_FLAGS="$CMAKE_CXX_FLAGS -fPIC -DGL_SILENCE_DEPRECATION" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
        cmake --build . -j $PARALLEL_BUILDS --config Release --target install

    elif [ "$OSTYPE" == "msys" ]; then
        cmake .. -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} ${COMMON_ARGS} -DCMAKE_INSTALL_PREFIX=../install
        cmake --build . -j $PARALLEL_BUILDS --config Release --target install

        # build debug variant
        rm CMakeCache.txt
        cmake ../ -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} ${COMMON_ARGS} -DCMAKE_INSTALL_PREFIX=../install-debug
        cmake --build . -j $PARALLEL_BUILDS --config Debug --target install
    else
        echo Unknown OSTYPE: $OSTYPE
    fi
fi

echo ------------------------ Pack ------------------------------------

cd $root_dir

if [ ! -d $target_dir ]
then
    mkdir $target_dir
    mkdir $target_dir/include
    mkdir $target_dir/lib
fi

cd bullet3/install/include/bullet
tar -c \
*.h \
BulletCollision/BroadphaseCollision/*.h \
BulletCollision/CollisionDispatch/*.h \
BulletCollision/CollisionShapes/*.h \
BulletCollision/NarrowPhaseCollision/*.h \
BulletDynamics/ConstraintSolver/*.h \
BulletDynamics/Dynamics/*.h \
BulletDynamics/Vehicle/*.h \
LinearMath/*.h \
| tar -C $root_dir/$target_dir/include -x

cd $root_dir/bullet3/install/lib
cp ${LIB_OSG_PREFIX}BulletCollision.${LIB_EXT} $root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}BulletDynamics.${LIB_EXT} $root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}LinearMath.${LIB_EXT} $root_dir/$target_dir/lib

cd $root_dir/bullet3/install-debug/lib
cp ${LIB_OSG_PREFIX}BulletCollision_Debug.${LIB_EXT} $root_dir/$target_dir/lib/${LIB_OSG_PREFIX}BulletCollisiond.${LIB_EXT}
cp ${LIB_OSG_PREFIX}BulletDynamics_Debug.${LIB_EXT} $root_dir/$target_dir/lib/${LIB_OSG_PREFIX}BulletDynamicsd.${LIB_EXT}
cp ${LIB_OSG_PREFIX}LinearMath_Debug.${LIB_EXT} $root_dir/$target_dir/lib/${LIB_OSG_PREFIX}LinearMathd.${LIB_EXT}

cd $root_dir

"$z_exe" a -r $zfilename -m0=LZMA -bb1 -spf $target_dir/*
# unpack with: 7z x <filename>

echo ------------------------ Done ------------------------------------
