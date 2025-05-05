#!/bin/bash

# This script will build OSG applications, including "osgviewer" and "osgconv" on linux
#
# Preparations:
#   On Windows optionally specify Visual Studio compiler, set GENERATOR_ARGUMENTS variable below
#     If GENERATOR_ARGUMENTS is not specified, default compiler will be used
#   Check other settings below, e.g. "build_examples"
#
# Script steps:
#   1. Fetch FBX SDK (headers and pre-compiled libraries)
#   2. Extract FBX files. NOTE: User input is required "yes" to license and "n" to skip readme
#   3. Fetch and extract OSG source code
#   4. Generate build configuration (1st cmake command)
#   5. Compile OSG and copy binaries to install folder "./osg" (2nd cmake command)
#
# After the script is done:
#   If desired the created osg folder can be moved, e.g to the home directory.
#   To run the applications some paths need to be set accordingly (check exact version numbers):
#     Windows powershell:
#       move osg $env:userprofile
#       $env:path="$env:path;$env:userprofile/osg/bin;$env:userprofile/osg/bin/osgPlugins-3.6.5;$env:userprofile/osg/3rdParty_bin;$env:userprofile/osg/lib"
#     Windows cmd:
#       move osg %userprofile%
#       set path=%path%;%userprofile%/osg/bin;%userprofile%/osg/bin/osgPlugins-3.6.5;%userprofile%/osg/3rdParty_bin;%userprofile%/osg/lib
#     Linux:
#       mv osg ~/
#       export LD_LIBRARY_PATH="$HOME/osg/lib:$LD_LIBRARY_PATH"
#       export PATH="$HOME/osg/bin:$PATH"
#     macOS:
#       mv osg ~/
#       export DYLD_LIBRARY_PATH="$HOME/osg/lib:$DYLD_LIBRARY_PATH"
#       export PATH="$HOME/osg/bin:$PATH"
#       (for permanent setting, you might need to add them to .bash_profile or .zshrc)
#   Then try:
#     osgviewer ~/esmini/resources/models/car_white.osgb --window 60 60 800 400
#     (possibly you need to change the path to your esmini root folder)

# settings

OSG_REPO=https://github.com/esmini/OpenSceneGraph_for_esmini
# Branch or tag, e.g. OpenSceneGraph_for_esmini or OpenSceneGraph-3.6.5.
OSG_VERSION=OpenSceneGraph_for_esmini
osg_root_dir=$(pwd)
thirdparty=""
# Option to build examples. Set to true in order to build all examples (takes some time).
build_examples=false
z_exe_win="$PROGRAMFILES/7-Zip/7z"
install_folder="$osg_root_dir/osg"
# Parallel compile. Set specific number, e.g. "-j 4" or just "-j" for compiler default. Set to "" for cmake versions < 3.12.
parallel_make_flag="-j4"

# actions
if [[ "$OSTYPE" == "linux-gnu"* ]]; then

    # [ ! -d fbxsdk ] && mkdir fbxsdk
    if [ ! -d fbxsdk ]; then
        if [ ! -f fbx202001_fbxsdk_linux.tar.gz ];then
            curl -k --user-agent  "Mozilla/5.0" -L "https://www.autodesk.com/content/dam/autodesk/www/adn/fbx/2020-0-1/fbx202001_fbxsdk_linux.tar.gz" -o fbx202001_fbxsdk_linux.tar.gz
        fi
        mkdir fbxsdk
        tar xzvf fbx202001_fbxsdk_linux.tar.gz --directory fbxsdk
        ./fbxsdk/fbx202001_fbxsdk_linux ./fbxsdk
    fi

    sudo apt install libtiff-dev
    sudo apt install libjpeg-dev

    fbx_include="$osg_root_dir/fbxsdk/include"
    fbx_lib_release="$osg_root_dir/fbxsdk/lib/gcc/x64/release/libfbxsdk.a"
    fbx_lib_debug="$osg_root_dir/fbxsdk/lib/gcc/x64/debug/libfbxsdk.a"
    fbx_xml_lib=libxml2.so
    fbx_zlib_lib=libz.so

elif [[ "$OSTYPE" == "darwin"* ]]; then

    curl --user-agent  "Mozilla/5.0" -L "https://www.autodesk.com/content/dam/autodesk/www/adn/fbx/2020-2-1/fbx202021_fbxsdk_clang_mac.pkg.tgz" -O
    tar xzvf fbx202021_fbxsdk_clang_mac.pkg.tgz
    sudo installer -pkg fbx202021_fbxsdk_clang_macos.pkg -target /

    brew install libtiff
    brew install libjpeg

    fbx_include="/Applications/Autodesk/FBX SDK/2020.2.1/include"
    fbx_lib_release="/Applications/Autodesk/FBX SDK/2020.2.1/lib/clang/release/libfbxsdk.a"
    fbx_lib_debug="/Applications/Autodesk/FBX SDK/2020.2.1/lib/clang/debug/libfbxsdk.a"
    fbx_xml_lib=libxml2.dylib
    fbx_zlib_lib=libz.dylib

elif [[ "$OSTYPE" == "msys" ]]; then

    if [ ! -d 3rdParty_x64 ]; then
        if [ ! -f 3rdParty_VS2017_v141_x64_V11_full.7z  ]; then
            curl -L "https://www.dropbox.com/scl/fi/fs54s1g2zbjbi6ou7zjt0/3rdParty_VS2017_v141_x64_V11_full.7z?rlkey=usiax6ry5xvclmye7sdypqwd9&st=fhfauw9e&dl=1" -O
        fi
        "$z_exe_win" x 3rdParty_VS2017_v141_x64_V11_full.7z
    fi
    if [ ! -f fbx202021_fbxsdk_vs2017_win.exe ]; then
        curl --user-agent  "Mozilla/5.0" -L https://www.autodesk.com/content/dam/autodesk/www/adn/fbx/2020-2-1/fbx202021_fbxsdk_vs2017_win.exe -O
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
    fbx_zlib_lib="$PROGRAMFILES/Autodesk/FBX/FBX SDK/2020.2.1/lib/vs2017/x64/release/zlib-md.lib"
    thirdparty="$osg_root_dir/3rdParty_x64/x64"

    # GENERATOR_ARGUMENTS=(-G "Visual Studio 17 2022" -T v143 -A x64)
fi

# OSG source
if [ ! -d OpenSceneGraph ]; then
    # use fork including some fixes
    git clone $OSG_REPO --branch $OSG_VERSION --depth 1 OpenSceneGraph
    cd OpenSceneGraph
else
    # osg seems already checked out, use it
    cd OpenSceneGraph
fi

# Compile OSG with standard settings; dynamic linking and without examples
cmake -B build-dyn "${GENERATOR_ARGUMENTS[@]}" -DFBX_INCLUDE_DIR="$fbx_include" -DFBX_LIBRARY="$fbx_lib_release" -DFBX_LIBRARY_DEBUG="$fbx_lib_debug" -DCMAKE_INSTALL_PREFIX="$install_folder" -DFBX_XML2_LIBRARY="$fbx_xml_lib" -DFBX_ZLIB_LIBRARY="$fbx_zlib_lib" -DACTUAL_3RDPARTY_DIR=$thirdparty -DBUILD_OSG_EXAMPLES="$build_examples" .

cmake --build build-dyn --config Release --target install $parallel_make_flag

if [[ "$OSTYPE" == "msys" ]]; then
    # Copy the 3rd party bin folder to the install folder
    cp -r "$thirdparty/bin" "$install_folder/3rdParty_bin"
fi

cd $osg_root_dir
