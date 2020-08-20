#!/bin/bash

# 
# This script will build osi-visualizer for Windows from scratch 
# as well as dependent software like protobuf and OSI classes
# No system installations will be done (no admin rights required)
# The result will be portable (dependent files copied into the install/release folder)
#
# Prerequisites:
# - Qt (https://www.qt.io/download)
# - Git (with Bash) (https://git-scm.com/download/win)
# - cmake (https://cmake.org/download/)
# - Visual Studio (with C++ toolkit) (https://visualstudio.microsoft.com/downloads/)
#
# Usage: 
# - put this script in an empty folder
# - open Git Bash in that folder 
# - review and adjust system dependent parameters in section below
# - run the script: ./setup-osi-visualizer.sh
# - wait (the build process will take approx. 15 minutes depending on...)
# 
# The folder osi-visualizer/build/Release (or Debug) will contain everything needed
# 
#

# -----------------------------------------------------------------------------------
# Review and update settings in this section according to your system and preferences

# Visual Studio 2019 - toolkit from Visual Studio 2017
GENERATOR=("Visual Studio 16 2019")
GENERATOR_TOOLSET="v142"
GENERATOR_ARGUMENTS="-A x64 -T ${GENERATOR_TOOLSET}"

# Visual Studio 2019 - default toolkit
# GENERATOR=("Visual Studio 16 2019")
# GENERATOR_ARGUMENTS="-A x64 -T ${GENERATOR_TOOLSET}"

# Visual Studio 2017 - default toolkit
# GENERATOR=("Visual Studio 15 2017 Win64")
# GENERATOR_ARGUMENTS="-T ${GENERATOR_TOOLSET}"

# Path to Qt installation - should point to the parent folder of bin, lib etc.
QT_DIR="C:\eknabe1\Programs\Qt\msvc2017_64"

FMI_VERSION=2.2
PROTOBUF_VERSION=v3.5.0

USE_DOXYGEN=false

BUILD_MODE=Release  # Relase / Debug 

# ---------------------------------------------------------------------------------------
# From this point no adjustments should be necessary, except fixing bugs in the script :)
# However you might want to adjust versions of software packages being checkout and built


osi_root_dir=$(pwd)

if [ $USE_DOXYGEN = true ]
then
	if [ ! -d doxygen ] 
	then
	    echo -------------------------- Downloading Doxygen ----------------------------------
	    cd $osi_root_dir
	    mkdir doxygen
	    cd doxygen
	    curl "https://www.doxygen.nl/files/doxygen-1.8.18.windows.x64.bin.zip" -o doxygen-1.8.18.windows.x64.bin.zip
	    unzip doxygen-1.8.18.windows.x64.bin.zip
	fi
	
	if [ ! -d graphviz ] 
	then
	    echo -------------------- Downloading Graphviz \(dot for doxygen\) ---------------------------
	    cd $osi_root_dir
	    mkdir graphviz
	    cd graphviz
	    curl https://graphviz.gitlab.io/_pages/Download/windows/graphviz-2.38.zip -o graphviz-2.38.zip
	    unzip graphviz-2.38.zip
	fi
fi

echo ------------------------ Installing FMI-library ------------------------------------
if [ ! -d fmi-library ] 
then
	cd $osi_root_dir
	git clone https://github.com/modelon-community/fmi-library.git
	cd fmi-library
	git checkout 2.2
	mkdir install
	mkdir build-fmil; cd build-fmil

	cmake .. -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DFMILIB_INSTALL_PREFIX=../install -DCMAKE_CXX_FLAGS=$C_RUNTIME_MODE_FLAGS

	cmake --build . --config MinSizeRel --target install
else
    echo fmi-library folder already exists, continue with next step...
fi

echo ------------------------ Installing Protobuf ------------------------------------
if [ ! -d protobuf ] 
then
	cd $osi_root_dir
	mkdir zlib_from_fmi-lib
	git clone https://github.com/protocolbuffers/protobuf.git
	cd protobuf
	git checkout v3.11.4
	cp ../fmi-library/ThirdParty/Zlib/zlib-1.2.6/zlib.h ../fmi-library/build-fmil/zlib/zconf.h ../fmi-library/build-fmil/zlib/MinSizeRel/zlib.lib ../zlib_from_fmi-lib
	mkdir build-code; cd build-code

	cmake ../cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DZLIB_LIBRARY=../../zlib_from_fmi-lib/zlib.lib -DZLIB_INCLUDE_DIR=../../zlib_from_fmi-lib -DCMAKE_INSTALL_PREFIX=../protobuf-install -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_WITH_ZLIB=ON -Dprotobuf_MSVC_STATIC_RUNTIME=OFF

	cmake --build . --config $BUILD_MODE --target install	
else
    echo protobuf folder already exists, continue with next step...
fi




echo ------------------------ Installing zmq ------------------------------------
if [ ! -d libzmq ] 
then
	cd $osi_root_dir
	git clone https://github.com/zeromq/libzmq.git
	cd libzmq
	git checkout v4.3.2
	mkdir zmq_install
	mkdir build
	cd build

	cmake .. -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D WITH_PERF_TOOL=OFF -D ZMQ_BUILD_TESTS=OFF -D ENABLE_CPACK=OFF -D CMAKE_BUILD_TYPE=$BUILD_MODE -D CMAKE_INSTALL_PREFIX=../zmq_install

	cmake --build . --config $BUILD_MODE --target install
else
    echo libzmq folder already exists, continue with next step...
fi



echo ------------------------ Installing cppzmq ------------------------------------
if [ ! -d cppzmq ] 
then
	cd $osi_root_dir
	git clone https://github.com/zeromq/cppzmq.git
	cd cppzmq
	git checkout v4.6.0
	mkdir build
	cd build

	cmake .. -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DCPPZMQ_BUILD_TESTS=OFF -D ZeroMQ_DIR=../libzmq/zmq_install/CMake -D CMAKE_INSTALL_PREFIX=../cppzmq_install

	cmake --build . --config $BUILD_MODE --target install
else
    echo cppzmq folder already exists, continue with next step...
fi


echo ------------------------ Installing OSI proto2cpp -----------------------------------
if [ ! -d proto2cpp ] 
then
	cd $osi_root_dir
	git clone https://github.com/OpenSimulationInterface/proto2cpp.git
else
    echo proto2cpp folder already exists, continue with next step...
fi

echo --------------------- Installing OSI and OSI-visualizer -----------------------------

cd $osi_root_dir
git clone https://github.com/OpenSimulationInterface/osi-visualizer.git
cd osi-visualizer
git submodule update --init

cd open-simulation-interface
./convert-to-proto3.sh
cd ..

mkdir build
cd build

export INSTALL_ROOT_DIR=../../install
export INSTALL_OSI_LIB_DIR=$INSTALL_ROOT_DIR/osi-lib
export INSTALL_OSI_VIZ_DIR=$INSTALL_ROOT_DIR/osi-visualizer
mkdir $INSTALL_ROOT_DIR
mkdir $INSTALL_OSI_LIB_DIR
mkdir $INSTALL_OSI_LIB_DIR/lib
mkdir $INSTALL_OSI_LIB_DIR/include
mkdir $INSTALL_OSI_VIZ_DIR
mkdir $INSTALL_OSI_VIZ_DIR/platforms
mkdir $INSTALL_OSI_VIZ_DIR/resources
mkdir $INSTALL_OSI_VIZ_DIR/resources/Images
mkdir $INSTALL_OSI_VIZ_DIR/resources/Shaders

export PATH=$PATH:../../graphviz/release/bin:../../protobuf/protobuf-install/bin

if [ $USE_DOXYGEN = true ]
then
	DOXYGEN_ARG="-DDOXYGEN_EXECUTABLE=../../doxygen/doxygen.exe"
else
	DOXYGEN_ARG=""
fi

cmake .. -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DCMAKE_INCLUDE_PATH=../protobuf/protobuf-install/include $DOXYGEN_ARG -DFILTER_PROTO2CPP_PY_PATH=../../proto2cpp -DQt5_DIR=$QT_DIR/lib/cmake/Qt5 -DFMILIB_LIBRARY="../../fmi-library/install/lib/fmilib_shared.lib" -DFMILIB_INCLUDE_DIRS="../../fmi-library/src/CAPI/include/FMI2;../../fmi-library/src/Util/include;../../fmi-library/build-fmil;../../fmi-library/src/Import/include;../../fmi-library/ThirdParty/FMI/default" -DCMAKE_CXX_FLAGS="-I../../cppzmq -I../../libzmq/include -DZMQ_STATIC /EHsc /MT" -DCMAKE_CXX_STANDARD_LIBRARIES="ws2_32.lib Iphlpapi.lib Advapi32.lib" -DINSTALL_LIB_DIR=$INSTALL_OSI_LIB_DIR/lib -DINSTALL_INCLUDE_DIR=$INSTALL_OSI_LIB_DIR/include -DCMAKE_INSTALL_PREFIX=$INSTALL_OSI_LIB_DIR -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON  -DCMAKE_CFLAGS="-MT"

# First bild OSI submodule separately since we need to rename the library before linking with the application
cmake --build open-simulation-interface --config $BUILD_MODE --target install 

# rename and move libraries so they are found by the linker
echo cp open-simulation-interface/$BUILD_MODE/open_simulation_interface_static.lib open-simulation-interface/$BUILD_MODE/open_simulation_interface.lib 
cp open-simulation-interface/$BUILD_MODE/open_simulation_interface_static.lib open-simulation-interface/$BUILD_MODE/open_simulation_interface.lib 

if [ $BUILD_MODE = "Release" ]; then
  cp ../../protobuf/protobuf-install/lib/libprotobuf.lib protobuf.lib
  cp ../../libzmq/zmq_install/lib/libzmq-${GENERATOR_TOOLSET}-mt-s-4_3_2.lib zmq.lib
else
  cp ../../protobuf/protobuf-install/lib/libprotobufd.lib protobuf.lib
  cp ../../libzmq/zmq_install/lib/libzmq-${GENERATOR_TOOLSET}-mt-sgd-4_3_2.lib zmq.lib
fi

# build and link the osi-visualizer application
cmake --build . --config $BUILD_MODE 
if [ $BUILD_MODE = "Release" ]; then
  cp $BUILD_MODE/osi-visualizer.exe $INSTALL_OSI_VIZ_DIR
else
  cp $BUILD_MODE/osi-visualizer.exe $INSTALL_OSI_VIZ_DIR/osi-visualizerd.exe
fi

# Make available shared libraries needed for execution
cp ../../fmi-library/install/lib/fmilib_shared.dll $INSTALL_OSI_VIZ_DIR
cp $QT_DIR/bin/{Qt5Core.dll,Qt5Gui.dll,Qt5Network.dll,Qt5OpenGL.dll,Qt5Widgets.dll,Qt5Xml.dll} $INSTALL_OSI_VIZ_DIR
cp $QT_DIR/bin/{Qt5Cored.dll,Qt5Guid.dll,Qt5Networkd.dll,Qt5OpenGLd.dll,Qt5Widgetsd.dll,Qt5Xmld.dll} $INSTALL_OSI_VIZ_DIR
cp $QT_DIR/plugins/platforms/qwindows.dll $INSTALL_OSI_VIZ_DIR/platforms
cp $QT_DIR/plugins/platforms/qwindowsd.dll $INSTALL_OSI_VIZ_DIR/platforms
cp ../resources/Images/* $INSTALL_OSI_VIZ_DIR/resources/Images
cp ../resources/Shaders/* $INSTALL_OSI_VIZ_DIR/resources/Shaders

echo ------------------------ Setup OSI for Python ------------------------------------
cd $osi_root_dir
cd osi-visualizer/open-simulation-interface
python setup.py install


echo ------------------------ Done ------------------------------------
cd $osi_root_dir

