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
#

# -----------------------------------------------------------------------------------
# Review and update settings in this section according to your system and preferences

fbx_support=false  # users are encouraged to convert fbx to osgb format whenever possible

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
    LIB_OSG_PREFIX="osg161-"
    LIB_OT_PREFIX="ot21-"

    target_dir="v10"
    zfilename="osg_v10.7z"
    z_exe="$PROGRAMFILES/7-Zip/7z"

elif [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux-gnu"* ]]; then
	# Unix Makefiles (for Ubuntu and other Linux systems)
	GENERATOR=("Unix Makefiles")
	GENERATOR_ARGUMENTS=""
    LIB_EXT="a"
    LIB_PREFIX="lib"
    LIB_OSG_PREFIX="lib"
    LIB_OT_PREFIX="lib"
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        target_dir="linux"
        zfilename="osg_linux.7z"
        z_exe=7za
    else
        target_dir="mac"
        zfilename="osg_mac.7z"
        z_exe=7z    
    fi
else
	echo Unknown OSTYPE: $OSTYPE
fi

OSG_VERSION=OpenSceneGraph-3.6.5

# ---------------------------------------------------------------------------------------
# From this point no adjustments should be necessary, except fixing bugs in the script :)
# However you might want to adjust versions of software packages being checkout and built

osg_root_dir=$(pwd)

echo ------------------------ Installing dependencies ------------------------------------
cd $osg_root_dir


if [ "$OSTYPE" == "msys" ]; then
    if [ ! -d 3rdParty_x64 ]; then
        if [ ! -f 3rdParty_VS2017_v141_x64_V11_full.7z  ]; then
            curl -L https://download.osgvisual.org/3rdParty_VS2017_v141_x64_V11_full.7z -o 3rdParty_VS2017_v141_x64_V11_full.7z 
            "$z_exe" x 3rdParty_VS2017_v141_x64_V11_full.7z
        fi
    fi

    if [ $fbx_support = true ]; then
        if [ ! -f fbx202021_fbxsdk_vs2017_win.exe ]; then
            curl --user-agent  "Mozilla/5.0" -L https://www.autodesk.com/content/dam/autodesk/www/adn/fbx/2020-2-1/fbx202021_fbxsdk_vs2017_win.exe -o fbx202021_fbxsdk_vs2017_win.exe
        fi

        if [ ! -d "$PROGRAMFILES/Autodesk/FBX/FBX SDK/2020.2.1/include" ]; then
            echo Installing FBX SDK...
            powershell -Command "Start-Process fbx202021_fbxsdk_vs2017_win.exe -ArgumentList /S -Wait"
        else
            echo FBX SDK already installed
        fi

        fbx_include="$PROGRAMFILES/Autodesk/FBX/FBX SDK/2020.2.1/include"
        fbx_lib_release="$PROGRAMFILES/Autodesk/FBX/FBX SDK/2020.2.1/lib/vs2017/x64/release/libfbxsdk-md.lib"
        fbx_lib_debug="$PROGRAMFILES/Autodesk/FBX/FBX SDK/2020.2.1/lib/vs2017/x64/debug/libfbxsdk-md.lib"
        fbx_xml_lib="$PROGRAMFILES/Autodesk/FBX/FBX SDK/2020.2.1/lib/vs2017/x64/release/libxml2-md.lib"
        fbx_xml_lib_debug="$PROGRAMFILES/Autodesk/FBX/FBX SDK/2020.2.1/lib/vs2017/x64/debug/libxml2-md.lib"
        fbx_zlib_lib="$PROGRAMFILES/Autodesk/FBX/FBX SDK/2020.2.1/lib/vs2017/x64/release/zlib-md.lib"
        fbx_zlib_lib_debug="$PROGRAMFILES/Autodesk/FBX/FBX SDK/2020.2.1/lib/vs2017/x64/debug/zlib-md.lib"
    fi

elif  [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux-gnu"* ]]; then

    if [ ! -d zlib-1.2.12 ]; then
        if [ ! -f zlib1212.zip ]; then
            curl "https://zlib.net/zlib1212.zip" -o zlib1212.zip
        fi
        unzip zlib1212.zip
        cd zlib-1.2.12
        mkdir install
        mkdir build
        cd build

        if [[ "$OSTYPE" == "linux-gnu"* ]]; then
            cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Debug .. -DCMAKE_C_FLAGS="-fPIC"
            cmake --build . --target install
            mv ../install/lib/libz.a ../install/lib/libzd.a

            rm CMakeCache.txt
            cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Release .. -DCMAKE_C_FLAGS="-fPIC"
            cmake --build . --target install
        elif [[ "$OSTYPE" == "darwin"* ]]; then
            cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Release .. -DCMAKE_C_FLAGS="-fPIC"
            cmake --build . --target install
        fi

    else
        echo zlib folder already exists, continue with next step...
    fi

    if [ $fbx_support = true ]; then
        if [[ "$OSTYPE" == "linux-gnu"* ]]; then
            if [ ! -f fbx202001_fbxsdk_linux.tar.gz ]; then
                curl --user-agent  "Mozilla/5.0" -L "https://www.autodesk.com/content/dam/autodesk/www/adn/fbx/2020-0-1/fbx202001_fbxsdk_linux.tar.gz" -o fbx202001_fbxsdk_linux.tar.gz
                mkdir fbxsdk
            fi
            tar xzvf fbx202001_fbxsdk_linux.tar.gz --directory fbxsdk
            ./fbxsdk/fbx202001_fbxsdk_linux ./fbxsdk
            fbx_include="../../fbxsdk/include"
            fbx_lib_release="../../fbxsdk/lib/gcc/x64/release/libfbxsdk.a"
            fbx_lib_debug="../../fbxsdk/lib/gcc/x64/debug/libfbxsdk.a"
            fbx_xml_lib=libxml2.so
            fbx_zlib_lib=libz.so            
        elif [[ "$OSTYPE" == "darwin"* ]]; then
            curl --user-agent  "Mozilla/5.0" -L "https://www.autodesk.com/content/dam/autodesk/www/adn/fbx/2020-2-1/fbx202021_fbxsdk_clang_mac.pkg.tgz" -o fbx202021_fbxsdk_clang_mac.pkg.tgz
            tar xzvf fbx202021_fbxsdk_clang_mac.pkg.tgz
            sudo installer -pkg fbx202021_fbxsdk_clang_macos.pkg -target /
            fbx_include="/Applications/Autodesk/FBX SDK/2020.2.1/include"
            fbx_lib_release="/Applications/Autodesk/FBX SDK/2020.2.1/lib/clang/release/libfbxsdk.a"
            fbx_lib_debug="/Applications/Autodesk/FBX SDK/2020.2.1/lib/clang/debug/libfbxsdk.a"
            fbx_xml_lib=libxml2.dylib
            fbx_zlib_lib=libz.dylib            
        fi
    fi
fi

cd $osg_root_dir
if [[ "$OSTYPE" == "linux-gnu"* ]] || [[ "$OSTYPE" == "darwin"* ]]; then
    if [ ! -d jpeg-9e ]; then
        if [ ! -f jpegsrc.v9e.tar.gz ]; then
            curl "http://www.ijg.org/files/jpegsrc.v9e.tar.gz" -o jpegsrc.v9e.tar.gz
        fi
        tar xzf jpegsrc.v9e.tar.gz
        cd jpeg-9e
        if [[ "$OSTYPE" == "darwin"* ]]; then
            ./configure
            make
        else
            ./configure CFLAGS='-fPIC -g'; make -j
            mv .libs .libsd
            mv .libsd/libjpeg.a .libsd/libjpegd.a
            make clean
            ./configure CFLAGS='-fPIC'; make -j
        fi
    else
        echo jpeg folder already exists, continue with next step...
    fi
fi

echo ------------------------ Installing OSG ------------------------------------
cd $osg_root_dir

if [ ! -d OpenSceneGraph ]; then
    git clone https://github.com/OpenSceneGraph/OpenSceneGraph
fi

if [ ! -d OpenSceneGraph/build ]; then

    cd OpenSceneGraph
    git checkout $OSG_VERSION

    # Apply fix for comment format not accepted by all platforms
    git checkout 63bb537132bab1f8b077838f7550e26405e5fa35 CMakeModules/FindFontconfig.cmake

    # Apply fix for Mac window handler
    git checkout 3994378a20948ebc4ed10b7cd33a6cc5393e7157 src/osgViewer/GraphicsWindowCocoa.mm

    mkdir build
    cd build

    if [[ "$OSTYPE" == "linux-gnu"* ]]; then

        cmake ../ -DOSG_AGGRESSIVE_WARNINGS=False -DDYNAMIC_OPENSCENEGRAPH=false -DDYNAMIC_OPENTHREADS=false -DBUILD_OSG_APPLICATIONS=False -DOPENGL_PROFILE=GL2 -DBUILD_OSG_DEPRECATED_SERIALIZERS=False -DCMAKE_CXX_FLAGS=-fPIC -DJPEG_LIBRARY=$osg_root_dir/jpeg-9e/.libs/libjpeg.a -DJPEG_INCLUDE_DIR=$osg_root_dir/jpeg-9e -DFBX_INCLUDE_DIR="$fbx_include" -DFBX_LIBRARY="$fbx_lib_release" -DFBX_LIBRARY_DEBUG="$fbx_lib_debug" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install 
 
        make -j16 install

        # build debug variant
        rm CMakeCache.txt

        cmake ../ -DOSG_AGGRESSIVE_WARNINGS=False -DDYNAMIC_OPENSCENEGRAPH=false -DDYNAMIC_OPENTHREADS=false -DBUILD_OSG_APPLICATIONS=False -DOPENGL_PROFILE=GL2 -DBUILD_OSG_DEPRECATED_SERIALIZERS=False -DCMAKE_CXX_FLAGS=-fPIC -DJPEG_LIBRARY=$osg_root_dir/jpeg-9e/.libsd/libjpegd.a -DJPEG_INCLUDE_DIR=$osg_root_dir/jpeg-9e -DFBX_INCLUDE_DIR="$fbx_include" -DFBX_LIBRARY="$fbx_lib_debug" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=../install-debug

        make -j16 install

    elif [[ "$OSTYPE" == "darwin"* ]]; then
        cmake ../ -DDYNAMIC_OPENSCENEGRAPH=false -DDYNAMIC_OPENTHREADS=false -DOPENGL_PROFILE=GL2 -DJPEG_LIBRARY_RELEASE=$osg_root_dir/jpeg-9e/.libs/libjpeg.a -DJPEG_INCLUDE_DIR=$osg_root_dir/jpeg-9e -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-fPIC -DCMAKE_INSTALL_PREFIX=../install

        cmake --build . -j 16 --config Release --target install

    elif [ "$OSTYPE" == "msys" ]; then
        cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} ../ -DDYNAMIC_OPENSCENEGRAPH=false -DOPENGL_PROFILE=GL3 -DDYNAMIC_OPENTHREADS=false -DCMAKE_INSTALL_PREFIX=../install -DACTUAL_3RDPARTY_DIR=../../3rdParty_x64/x64 -DFBX_INCLUDE_DIR="$fbx_include" -DFBX_LIBRARY="$fbx_lib_release" -DFBX_LIBRARY_DEBUG="$fbx_lib_debug" -DFBX_XML2_LIBRARY="$fbx_xml_lib_debug" -DFBX_ZLIB_LIBRARY="$fbx_zlib_lib_debug"

        cmake --build . -j 8 --config Release --target install

        # build debug variant
        rm CMakeCache.txt

        cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} ../ -DDYNAMIC_OPENSCENEGRAPH=false -OPENGL_PROFILE=GL3 -DDYNAMIC_OPENTHREADS=false -DCMAKE_INSTALL_PREFIX=../install-debug -DACTUAL_3RDPARTY_DIR=../../3rdParty_x64/x64

        cmake --build . -j 8 --config Debug --target install
    else
        echo Unknown OSTYPE: $OSTYPE
    fi
fi

echo ------------------------ Pack ------------------------------------

cd $osg_root_dir

if [ ! -d $target_dir ]
then
    mkdir $target_dir
    mkdir $target_dir/include
    mkdir $target_dir/lib
    mkdir $target_dir/lib/osgPlugins-3.6.5
fi
cp -r OpenSceneGraph/install/include $target_dir/

if [ "$OSTYPE" == "msys" ]; then
    cp 3rdParty_x64/x64/include/zlib.h $target_dir/include
    cp 3rdParty_x64/x64/lib/zlibstatic.lib 3rdParty_x64/x64/lib/zlibstaticd.lib $target_dir/lib
    cp 3rdParty_x64/x64/include/jpeglib.h $target_dir/include
    cp 3rdParty_x64/x64/lib/jpeg.lib 3rdParty_x64/x64/lib/jpegd.lib $target_dir/lib
elif [[ "$OSTYPE" == "linux-gnu"* ]] || [[ "$OSTYPE" == "darwin"* ]]; then
    cp zlib-1.2.12/install/include/zlib.h $target_dir/include
    cp zlib-1.2.12/install/lib/libz.${LIB_EXT} zlib-1.2.12/install/lib/libzd.${LIB_EXT} $target_dir/lib
    cp jpeg-9e/jpeglib.h $target_dir/include
    cp jpeg-9e/.libs/libjpeg.${LIB_EXT} jpeg-9e/.libsd/libjpegd.${LIB_EXT} $target_dir/lib
else
    echo Unknown OSTYPE: $OSTYPE
fi

cd $osg_root_dir/OpenSceneGraph/install/lib
cp ${LIB_OSG_PREFIX}osg.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgAnimation.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgDB.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgGA.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgShadow.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgSim.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgText.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgUtil.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OSG_PREFIX}osgViewer.${LIB_EXT} $osg_root_dir/$target_dir/lib
cp ${LIB_OT_PREFIX}OpenThreads.${LIB_EXT} $osg_root_dir/$target_dir/lib

cd $osg_root_dir/OpenSceneGraph/install/lib/osgPlugins-3.6.5
cp ${LIB_PREFIX}osgdb_jpeg.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
cp ${LIB_PREFIX}osgdb_osg.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
cp ${LIB_PREFIX}osgdb_serializers_osg.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
cp ${LIB_PREFIX}osgdb_serializers_osgsim.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5

if [ $fbx_support = true ]; then
    cp ${LIB_PREFIX}osgdb_fbx.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
fi

if [[ ! "$OSTYPE" == "darwin"* ]]; then
    cd $osg_root_dir/OpenSceneGraph/install-debug/lib
    cp ${LIB_OSG_PREFIX}osgd.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgAnimationd.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgDBd.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgGAd.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgShadowd.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgSimd.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgTextd.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgUtild.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgViewerd.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OT_PREFIX}OpenThreadsd.${LIB_EXT} $osg_root_dir/$target_dir/lib

    cd $osg_root_dir/OpenSceneGraph/install-debug/lib/osgPlugins-3.6.5
    cp ${LIB_PREFIX}osgdb_jpegd.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
    cp ${LIB_PREFIX}osgdb_osgd.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
    cp ${LIB_PREFIX}osgdb_serializers_osgd.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
    cp ${LIB_PREFIX}osgdb_serializers_osgsimd.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
    if [ $fbx_support = true ]; then
        cp ${LIB_PREFIX}osgdb_fbxd.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
    fi
fi

cd $osg_root_dir

"$z_exe" a -r $zfilename -m0=LZMA -bb1 -spf $target_dir/*
# unpack with: 7z x <filename>

echo ------------------------ Done ------------------------------------
