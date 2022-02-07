#!/bin/bash

# This script will build OSG applications, including "osgviewer" and "osgconv" on linux
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
#   To run the applications some paths need to be set accordingly:
#     Windows powershell:
#       move osg $env:userprofile
#       $env:path="$env:path;$env:userprofile/osg/bin;$env:userprofile/osg/lib"
#     Windows cmd:
#       move osg %userprofile%
#       set path=%path%;%userprofile%/osg/bin;%userprofile%/osg/lib
#     Linux:
#       mv osg ~/
#       export LD_LIBRARY_PATH="$HOME/osg/lib:$LD_LIBRARY_PATH"
#       export PATH="$HOME/osg/bin:$PATH"
#     macOS:
#       mv osg ~/
#       export DYLD_LIBRARY_PATH="$HOME/osg/lib:$DYLD_LIBRARY_PATH"
#       export PATH="$HOME/osg/bin:$PATH"
#   Then try:
#     osgviewer ~/esmini/resources/models/car_white.osgb --window 60 60 800 400
#     (possibly you need to change the path to your esmini root folder)

OSG_VERSION=OpenSceneGraph-3.6.5
osg_root_dir=$(pwd)
thirdparty=""

if [[ "$OSTYPE" == "linux-gnu"* ]]; then

    curl --user-agent  "Mozilla/5.0" -L "https://www.autodesk.com/content/dam/autodesk/www/adn/fbx/2020-0-1/fbx202001_fbxsdk_linux.tar.gz" -o fbx202001_fbxsdk_linux.tar.gz
    [ ! -d fbxsdk ] && mkdir fbxsdk
    tar xzvf fbx202001_fbxsdk_linux.tar.gz --directory fbxsdk
    ./fbxsdk/fbx202001_fbxsdk_linux ./fbxsdk

    sudo apt install libtiff-dev
    sudo apt install libjpeg-dev

    fbx_include="../../fbxsdk/include"
    fbx_lib_release="../../fbxsdk/lib/gcc/x64/release/libfbxsdk.a"
    fbx_lib_debug="../../fbxsdk/lib/gcc/x64/debug/libfbxsdk.a"
    fbx_xml_lib=libxml2.so
    fbx_zlib_lib=libz.so

elif [[ "$OSTYPE" == "darwin"* ]]; then

    curl --user-agent  "Mozilla/5.0" -L "https://www.autodesk.com/content/dam/autodesk/www/adn/fbx/2020-2-1/fbx202021_fbxsdk_clang_mac.pkg.tgz" -o fbx202021_fbxsdk_clang_mac.pkg.tgz
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

    curl --user-agent  "Mozilla/5.0" -L https://www.autodesk.com/content/dam/autodesk/www/adn/fbx/2020-2-1/fbx202021_fbxsdk_vs2017_win.exe -o fbx202021_fbxsdk_vs2017_win.exe
    powershell -Command "Start-Process fbx202021_fbxsdk_vs2017_win.exe -ArgumentList /S -Wait"

    curl -L https://download.osgvisual.org/3rdParty_VS2017_v141_x64_V11_full.7z -o 3rdParty_VS2017_v141_x64_V11_full.7z
    "$PROGRAMFILES/7-Zip/7z" x 3rdParty_VS2017_v141_x64_V11_full.7z

    fbx_include="$PROGRAMFILES/Autodesk/FBX/FBX SDK/2020.2.1/include"
    fbx_lib_release="$PROGRAMFILES/Autodesk/FBX/FBX SDK/2020.2.1/lib/vs2017/x64/release/libfbxsdk-md.lib"
    fbx_lib_debug="$PROGRAMFILES/Autodesk/FBX/FBX SDK/2020.2.1/lib/vs2017/x64/debug/libfbxsdk-md.lib"
    fbx_xml_lib="$PROGRAMFILES/Autodesk/FBX/FBX SDK/2020.2.1/lib/vs2017/x64/release/libxml2-md.lib"
    fbx_zlib_lib="$PROGRAMFILES/Autodesk/FBX/FBX SDK/2020.2.1/lib/vs2017/x64/release/zlib-md.lib"
    thirdparty="../../3rdParty_x64/x64"    
fi

# OSG source
git clone https://github.com/OpenSceneGraph/OpenSceneGraph
cd OpenSceneGraph
git checkout $OSG_VERSION

# Apply fix for comment format not accepted by all platforms
git checkout 63bb537132bab1f8b077838f7550e26405e5fa35 CMakeModules/FindFontconfig.cmake

# Apply fix for Mac window handler
git checkout 3994378a20948ebc4ed10b7cd33a6cc5393e7157 src/osgViewer/GraphicsWindowCocoa.mm

mkdir build-dyn
cd build-dyn

# Compile OSG with standard settings; dynamic linking and without examples

cmake -DFBX_INCLUDE_DIR="$fbx_include" -DFBX_LIBRARY="$fbx_lib_release" -DFBX_LIBRARY_DEBUG="$fbx_lib_debug" -DCMAKE_INSTALL_PREFIX=../../osg -DFBX_XML2_LIBRARY="$fbx_xml_lib" -DFBX_ZLIB_LIBRARY="$fbx_zlib_lib" -DACTUAL_3RDPARTY_DIR=$thirdparty ..

cmake --build . -j 4 --config Release --target install

cd $osg_root_dir
