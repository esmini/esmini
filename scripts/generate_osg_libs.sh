#!/bin/bash

#
# This script will build OpenSceneGraph (OSG) libraries
# needed for esmini to visualize the road network and scenario.
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
# - run the script: ./generate_osi_libs.sh
# - wait (the build process will take approx. 15 minutes depending on...)
#
# The osi_*.7z will contain both headers and needed libraries. (* depends on platform)
#
# 7-zip uncompress tool:
#   Ubuntu: sudo apt-get install p7zip-full
#   Windows: https://www.7-zip.org/download.html
#

# -----------------------------------------------------------------------------------
# Review and update settings in this section according to your system and preferences

OSG_REPO=https://github.com/esmini/OpenSceneGraph_for_esmini
# Specify version as branch or tag, e.g. OpenSceneGraph_for_esmini or OpenSceneGraph-3.6.5.
OSG_VERSION=OpenSceneGraph-3.6.5_for_esmini_v1
# Parallel compile. Set specific number, e.g. "-j4" or empty "-j" for compiler default. Set to "" for cmake versions < 3.12.
PARALLEL_BUILDS="-j4"
ZIP_MIN_VERSION=12
PNG_MIN_VERSION=50

if ! ( [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux-gnu"* ]] || [[ "$OSTYPE" == "msys" ]] ); then
    echo "Unsupported platform: $OSTYPE. Supported ones are: Win, Linux, Mac. Exiting." >&2
    exit 1
fi

if [ "$OSTYPE" == "msys" ]; then
    # comment out line below to use default generator
    GENERATOR_ARGUMENTS=(-G "Visual Studio 17 2022" -T v142 -A x64)

    # Make sure 7zip is available, else download and install it
    # https://www.7-zip.org/download.html

    LIB_EXT="lib"
    LIB_PREFIX=""
    LIB_OSG_PREFIX="osg*-"
    LIB_OT_PREFIX="ot*-"

    target_dir="v10"
    zfilename="osg_win.7z"
    z_exe="$PROGRAMFILES/7-Zip/7z"

elif [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux-gnu"* ]]; then
    LIB_EXT="a"
    LIB_PREFIX="lib"
    LIB_OSG_PREFIX="lib"
    LIB_OT_PREFIX="lib"
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        target_dir="linux"
        zfilename="osg_linux.7z"
        z_exe=7za
    else
        # Mac build, ensure cmake minimum policy version 3.5 to avoid issues with newer cmake versions
        export CMAKE_POLICY_VERSION_MINIMUM=3.5
        target_dir="mac"
        zfilename="osg_mac.7z"
        z_exe=7z
        macos_arch="arm64;x86_64"
    fi
else
    echo Unknown OSTYPE: $OSTYPE
fi

# ---------------------------------------------------------------------------------------
# From this point no adjustments should be necessary, except fixing bugs in the script :)
# However you might want to adjust versions of software packages being checkout and built

osg_root_dir=$(pwd)
common_cmake_args="-DCMAKE_INSTALL_PREFIX=install -DCMAKE_DEBUG_POSTFIX=d"

echo ------------------------ Build dependencies ------------------------------------

cd $osg_root_dir
if [ ! -d zlib ]; then
    echo ------------------------ Build zlib ------------------------------------
    git clone https://github.com/madler/zlib.git --depth 1 --branch v1.2.$ZIP_MIN_VERSION
    cd  zlib
    mkdir build
    cd build
    mkdir install

    args="$common_cmake_args"

    if [[ "$OSTYPE" == "darwin"* ]]; then
        args+=" -DCMAKE_OSX_ARCHITECTURES=$macos_arch"
    fi

    if [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux-gnu"* ]]; then
        args+=" -DCMAKE_C_FLAGS=-fPIC"
        build_type="-DCMAKE_BUILD_TYPE=Release"
    fi

    cmake .. "${GENERATOR_ARGUMENTS[@]}" $args $build_type
    cmake --build . $PARALLEL_BUILDS --target install --config Release

    if [[ "$OSTYPE" == "linux-gnu"* ]] || [[ "$OSTYPE" == "msys" ]]; then
        if [[ "$OSTYPE" == "linux-gnu"* ]]; then
            rm CMakeCache.txt
            build_type="-DCMAKE_BUILD_TYPE=Debug"
            cmake .. "${GENERATOR_ARGUMENTS[@]}" $args $build_type
        fi
        cmake --build . $PARALLEL_BUILDS --target install --config Debug --clean-first
    fi
else
    echo zlib folder already exists, continue with next step...
fi

cd $osg_root_dir
if [ ! -d libpng ]; then
    echo ------------------------ Build libpng ------------------------------------
    git clone https://github.com/pnggroup/libpng --depth 1 --branch v1.6.$PNG_MIN_VERSION
    cd libpng
    mkdir build
    cd build
    mkdir install

    args="$common_cmake_args -DPNG_STATIC=ON -DPNG_SHARED=OFF -DPNG_TOOLS=OFF -DZLIB_ROOT=$osg_root_dir/zlib/build/install"

    if [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux-gnu"* ]]; then
        C_FLAGS="-fPIC"
        build_type="-DCMAKE_BUILD_TYPE=Release"
    fi

    if [[ "$OSTYPE" == "darwin"* ]]; then
        args+=" -DCMAKE_OSX_ARCHITECTURES=$macos_arch -DPNG_HARDWARE_OPTIMIZATIONS=OFF"
    fi

    cmake .. "${GENERATOR_ARGUMENTS[@]}" $args $build_type
    cmake --build . $PARALLEL_BUILDS --target install --config Release

    if [[ "$OSTYPE" == "linux-gnu"* ]] || [[ "$OSTYPE" == "msys" ]]; then
        if [[ "$OSTYPE" == "linux-gnu"* ]]; then
            rm CMakeCache.txt
            build_type="-DCMAKE_BUILD_TYPE=Debug"
            cmake .. "${GENERATOR_ARGUMENTS[@]}" $args $build_type
        fi
        cmake --build . $PARALLEL_BUILDS --target install --config Debug --clean-first
    fi

else
    echo libpng folder already exists, continue with next step...
fi

cd $osg_root_dir
if [ ! -d libjpeg ]; then

    echo ------------------------ Build libjpeg ------------------------------------
    git clone https://github.com/csparker247/jpeg-cmake.git --depth 1 --branch v1.3.0 libjpeg
    cd libjpeg
    mkdir build
    cd build
    mkdir install

    args="$common_cmake_args -DBUILD_SHARED_LIBS=OFF -DBUILD_STATIC_LIBS=ON -DLINK_STATIC=ON -DBUILD_EXECUTABLES=OFF -DBUILD_ALT_UI=OFF -DBUILD_TESTS=OFF"

    if [[ "$OSTYPE" == "darwin"* ]]; then
        args+=" -DCMAKE_OSX_ARCHITECTURES=$macos_arch"
    fi

    if [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux-gnu"* ]]; then
        args+=" -DCMAKE_C_FLAGS=-fPIC"
        build_type="-DCMAKE_BUILD_TYPE=Release"
    fi

    cmake .. "${GENERATOR_ARGUMENTS[@]}" $args $build_type
    cmake --build . $PARALLEL_BUILDS --target install --config Release

    if [[ "$OSTYPE" == "linux-gnu"* ]] || [[ "$OSTYPE" == "msys" ]]; then
        if [[ "$OSTYPE" == "linux-gnu"* ]]; then
            rm CMakeCache.txt
            build_type="-DCMAKE_BUILD_TYPE=Debug"
            cmake .. "${GENERATOR_ARGUMENTS[@]}" $args $build_type
        fi
        cmake --build . $PARALLEL_BUILDS --target install --config Debug --clean-first
    fi
else
    echo jpeg-cmake folder already exists, continue with next step...
fi

# ------------------------ Build OSG ------------------------------------

cd $osg_root_dir
if [ ! -d OpenSceneGraph ]; then
    echo ------------------------ Checkout OSG ------------------------------------
    # use fork including some fixes
    git clone $OSG_REPO --branch $OSG_VERSION OpenSceneGraph

    cd OpenSceneGraph
    git describe --long --dirty

    # Apply fix for comment format not accepted by all platforms
    git checkout 63bb537132bab1f8b077838f7550e26405e5fa35 CMakeModules/FindFontconfig.cmake

    # Apply fix for Mac window handler
    git checkout 3994378a20948ebc4ed10b7cd33a6cc5393e7157 src/osgViewer/GraphicsWindowCocoa.mm

    # Apply fix '_FPOSOFF' has been deprecated #26231
    git checkout fca3b5b9a9f1c36ddf08ed08cbe02a2668fa4ee9 src/osgPlugins/osga/OSGA_Archive.cpp

    # Enforce pthread sched_yield() in favor of pthread_yield() which was deprecated in glibc 2.34
    if [[ "$OSTYPE" == "darwin"* ]]; then
        sed -i '' 's/CHECK_FUNCTION_EXISTS(pthread_yield/# CHECK_FUNCTION_EXISTS(pthread_yield/g' src/OpenThreads/pthreads/CMakeLists.txt
    else
        sed -i 's/CHECK_FUNCTION_EXISTS(pthread_yield/# CHECK_FUNCTION_EXISTS(pthread_yield/g' src/OpenThreads/pthreads/CMakeLists.txt
    fi

    # unstage and show status of the repo
    git reset
    git describe --long --dirty
else
    echo OSG already checked out, continue with next step...
fi

cd $osg_root_dir
if [ ! -d OpenSceneGraph/build ]; then
    echo ------------------------ Build OSG ------------------------------------

    cd OpenSceneGraph
    mkdir build
    cd build
    mkdir install

    args="-DOPENGL_PROFILE=GL2 -DOSG_AGGRESSIVE_WARNINGS=OFF -DDYNAMIC_OPENSCENEGRAPH=OFF -DDYNAMIC_OPENTHREADS=OFF -DBUILD_OSG_APPLICATIONS=OFF -DBUILD_OSG_EXAMPLES=OFF -DBUILD_OSG_DEPRECATED_SERIALIZERS=OFF -DJPEG_INCLUDE_DIR=$osg_root_dir/libjpeg/build/install/include -DPNG_PNG_INCLUDE_DIR=$osg_root_dir/libpng/build/install/include -DZLIB_INCLUDE_DIR=$osg_root_dir/zlib/build/install/include -DCMAKE_INSTALL_PREFIX=install"

    if [[ "$OSTYPE" == "darwin"* ]]; then
        args+=" -DCMAKE_OSX_ARCHITECTURES=$macos_arch -DOSG_TEXT_USE_FONTCONFIG=OFF "
    fi

    if [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux-gnu"* ]]; then
        args+=" -DCMAKE_CXX_FLAGS=-fPIC"
        build_type="-DCMAKE_BUILD_TYPE=Release"
        libs_to_link="-DZLIB_LIBRARY_RELEASE=$osg_root_dir/zlib/build/install/lib/libz.a -DJPEG_LIBRARY_RELEASE=$osg_root_dir/libjpeg/build/install/lib/libjpeg.a -DPNG_LIBRARY_RELEASE=$osg_root_dir/libpng/build/install/lib/libpng16.a -DZLIB_LIBRARY_DEBUG=$osg_root_dir/zlib/build/install/lib/libzd.a -DJPEG_LIBRARY_DEBUG=$osg_root_dir/libjpeg/build/install/lib/libjpegd.a -DPNG_LIBRARY_DEBUG=$osg_root_dir/libpng/build/install/lib/libpng16d.a"
    elif [[ "$OSTYPE" == "msys" ]]; then
        libs_to_link="-DZLIB_LIBRARY_RELEASE=$osg_root_dir/zlib/build/install/lib/zlib.lib -DJPEG_LIBRARY_RELEASE=$osg_root_dir/libjpeg/build/install/lib/jpeg.lib -DPNG_LIBRARY_RELEASE=$osg_root_dir/libpng/build/install/lib/libpng16_static.lib -DZLIB_LIBRARY_DEBUG=$osg_root_dir/zlib/build/install/lib/zlibd.lib -DJPEG_LIBRARY_DEBUG=$osg_root_dir/libjpeg/build/install/lib/jpegd.lib -DPNG_LIBRARY_DEBUG=$osg_root_dir/libpng/build/install/lib/libpng16_staticd.lib"
    fi

    cmake .. "${GENERATOR_ARGUMENTS[@]}" $args $build_type $build_type $libs_to_link
    cmake --build . $PARALLEL_BUILDS --target install --config Release

    if [[ "$OSTYPE" == "linux-gnu"* ]] || [[ "$OSTYPE" == "msys" ]]; then
        if [[ "$OSTYPE" == "linux-gnu"* ]]; then
            rm CMakeCache.txt
            build_type=" -DCMAKE_BUILD_TYPE=Debug"
            cmake .. "${GENERATOR_ARGUMENTS[@]}" $args $build_type
        fi
        cmake --build . $PARALLEL_BUILDS --target install --config Debug --clean-first
    fi
else
    echo OSG build folder already exists, continue with next step...
fi

echo ------------------------ Pack files ------------------------------------

cd $osg_root_dir
plugins_dir=$(find OpenSceneGraph/build/install/lib -maxdepth 1 -type d -name "osgPlugins-*")
if [ -d "$plugins_dir" ]; then
    plugins_dir_name=$(basename $plugins_dir)
else
    echo "Plugins folder not found. Exiting..."
    exit 1
fi

if [ ! -d $target_dir ]; then
    mkdir $target_dir
    mkdir $target_dir/include
    mkdir $target_dir/lib
    mkdir $target_dir/lib/$plugins_dir_name
fi
cp -r OpenSceneGraph/build/install/include $target_dir/

cp zlib/build/install/include/*.h $target_dir/include
if [[ "$OSTYPE" == "msys" ]]; then
    cp zlib/build/install/lib/zlibstatic*.${LIB_EXT} $target_dir/lib
else
    cp zlib/build/install/lib/libz*.${LIB_EXT} $target_dir/lib
fi
cp libjpeg/build/install/include/*.h $target_dir/include
cp libjpeg/build/install/lib/${LIB_PREFIX}jpeg*.${LIB_EXT} $target_dir/lib
cp libpng/build/install/include/*.h $target_dir/include
cp libpng/build/install/lib/libpng16*.${LIB_EXT} $target_dir/lib

cd $osg_root_dir/OpenSceneGraph/build/install/lib
cp ${LIB_OSG_PREFIX}osg{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgAnimation{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgDB{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgGA{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgShadow{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgSim{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgText{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgUtil{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgViewer{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OT_PREFIX}OpenThreads{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib

cd $osg_root_dir/OpenSceneGraph/build/install/lib/$plugins_dir_name
cp ${LIB_PREFIX}osgdb_jpeg{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib/$plugins_dir_name
cp ${LIB_PREFIX}osgdb_png{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib/$plugins_dir_name
cp ${LIB_PREFIX}osgdb_osg{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib/$plugins_dir_name
cp ${LIB_PREFIX}osgdb_serializers_osg{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib/$plugins_dir_name
cp ${LIB_PREFIX}osgdb_serializers_osgsim{,d}.${LIB_EXT} $osg_root_dir/$target_dir/lib/$plugins_dir_name

cd $osg_root_dir

"$z_exe" a -r $zfilename -m0=LZMA -bb1 -spf $target_dir/*
# unpack with: 7z x <filename>

echo ------------------------ Done ------------------------------------
