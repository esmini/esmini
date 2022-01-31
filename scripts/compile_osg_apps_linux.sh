#!/bin/bash

# This script will build OSG applications, including "osgviewer" and "osgconv" on linux
# Script steps:
#   1. Fetch FBX SDK (headers and pre-compiled libraries)
#   2. Extract FBX files. NOTE: User input is required "yes" to license and "n" to skip readme
#   3. Fetch and extract OSG source code
#   4. Generate build configuration (1st cmake command)
#   5. Compile OSG and copy binaries to install folder "./osg" (2nd cmake command)
#
# After the script is done:
#   If desired the created osg folder can be moved, e.g to the $HOME directory:
#     mv osg ~/
#   To run the applications some paths need to be set accordingly:
#     export LD_LIBRARY_PATH="$HOME/osg/lib:$LD_LIBRARY_PATH"
#     export PATH="$HOME/osg/bin:$PATH"
#   Then try:
#     osgviewer ~/esmini/resources/models/car_white.osgb --window 60 60 800 400
#     (possibly you need to change the path to your esmini root folder)

OSG_VERSION=OpenSceneGraph-3.6.5
osg_root_dir=$(pwd)

# FBX
curl --user-agent  "Mozilla/5.0" -L "https://www.autodesk.com/content/dam/autodesk/www/adn/fbx/2020-0-1/fbx202001_fbxsdk_linux.tar.gz" -o fbx202001_fbxsdk_linux.tar.gz
mkdir fbxsdk
tar xzvf fbx202001_fbxsdk_linux.tar.gz --directory fbxsdk
./fbxsdk/fbx202001_fbxsdk_linux ./fbxsdk

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
cmake -DFBX_INCLUDE_DIR=../../fbxsdk/include -DFBX_LIBRARY=../../fbxsdk/lib/gcc/x64/release/libfbxsdk.so -DFBX_LIBRARY_DEBUG=../../fbxsdk/lib/gcc/x64/debug/libfbxsdk.so -DCMAKE_INSTALL_PREFIX=../../osg -DFBX_XML2_LIBRARY=libxml2.so -DFBX_ZLIB_LIBRARY=libz.so ..

cmake --build . --config Release --target install -j 4
cp -r ../../fbxsdk/include/* ../../osg/include
cp -r ../../fbxsdk/lib/gcc/x64/release/* ../../osg/lib

cd $osg_root_dir
