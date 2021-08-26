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
OSI_VERSION=3.3.1

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
    z_exe=7z
elif [[ "$OSTYPE" == "darwin"* ]]; then
    target_dir="mac"
    zfilename="osi_mac.7z"
    z_exe=7z
else
    echo Unknown OSTYPE: $OSTYPE
fi

mkdir $target_dir
mkdir $target_dir/include


# ---------------------------------------------------------------------------------------
# From this point no adjustments should be necessary, except fixing bugs in the script :)
# However you might want to adjust versions of software packages being checkout and built


osi_root_dir=$(pwd)


echo ------------------------ Installing zlib ------------------------------------
cd $osi_root_dir

if [ ! -d zlib-1.2.11 ] 
then
 	if [ ! -f zlib1211.zip ]; then
  	    curl "https://zlib.net/zlib1211.zip" -o zlib1211.zip
    fi
    unzip zlib1211.zip
    cd zlib-1.2.11
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
	else
		cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install ..
		cmake --build . --config Debug --target install
		cmake --build . --config Release --target install --clean-first
	fi

else
    echo zlib folder already exists, continue with next step...
fi


echo ------------------------ Installing OSI proto2cpp -----------------------------------
cd $osi_root_dir

if [ ! -d proto2cpp ] 
then
    git clone https://github.com/OpenSimulationInterface/proto2cpp.git
else
    echo proto2cpp folder already exists, continue with next step...
fi

function build {

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

    if [ ! -d protobuf$folder_postfix ] 
    then
        git clone https://github.com/protocolbuffers/protobuf.git
        mv protobuf protobuf$folder_postfix
        cd protobuf$folder_postfix
        git checkout v$PROTOBUF_VERSION
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
            cmake ../cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DZLIB_LIBRARY=../../zlib-1.2.11/install/lib/$ZLIB_FILE_DEBUG -DZLIB_INCLUDE_DIR=../../zlib-1.2.11/install/include -DCMAKE_INSTALL_PREFIX=$INSTALL_PROTOBUF_DIR -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_WITH_ZLIB=ON -Dprotobuf_MSVC_STATIC_RUNTIME=OFF -DCMAKE_BUILD_TYPE=Debug $ADDITIONAL_CMAKE_PARAMETERS
            cmake --build . --config Debug --target install --clean-first  
        fi
        
        rm CMakeCache.txt

        cmake ../cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DZLIB_LIBRARY=../../zlib-1.2.11/install/lib/$ZLIB_FILE_RELEASE -DZLIB_INCLUDE_DIR=../../zlib-1.2.11/install/include -DCMAKE_INSTALL_PREFIX=$INSTALL_PROTOBUF_DIR -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_WITH_ZLIB=ON -Dprotobuf_MSVC_STATIC_RUNTIME=OFF -DCMAKE_BUILD_TYPE=Release $ADDITIONAL_CMAKE_PARAMETERS
        cmake --build . --config Release --target install --clean-first  
        
    else
        echo protobuf folder already exists, continue with next step...
    fi


    echo --------------------- Installing OSI $1 -----------------------------
    cd $osi_root_dir

    if [ ! -d open-simulation-interface$folder_postfix ]
    then
        git clone https://github.com/OpenSimulationInterface/open-simulation-interface.git
        mv open-simulation-interface open-simulation-interface$folder_postfix
        cd open-simulation-interface$folder_postfix
        git checkout v$OSI_VERSION
        sh ./convert-to-proto3.sh
        mkdir build
        cd build
        
        export INSTALL_ROOT_DIR=../install
        export INSTALL_OSI_LIB_DIR=$INSTALL_ROOT_DIR/osi-lib
        mkdir $INSTALL_ROOT_DIR
        mkdir $INSTALL_OSI_LIB_DIR
        mkdir $INSTALL_OSI_LIB_DIR/lib
        mkdir $INSTALL_OSI_LIB_DIR/include

        export PATH=$PATH:../../graphviz/release/bin:../../protobuf$folder_postfix/protobuf-install/bin

        if [ $DYNAMIC_LINKING == "1" ]; then
            ADDITIONAL_CMAKE_PARAMETERS="-DCMAKE_CXX_FLAGS=-DPROTOBUF_USE_DLLS"
        else
            ADDITIONAL_CMAKE_PARAMETERS=""
        fi
        
        if [[ "$OSTYPE" != "darwin"* ]]; then
            cmake .. -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DCMAKE_INCLUDE_PATH=../protobuf$folder_postfix/protobuf-install/include -DFILTER_PROTO2CPP_PY_PATH=../../proto2cpp -DINSTALL_LIB_DIR=$INSTALL_OSI_LIB_DIR/lib -DINSTALL_INCLUDE_DIR=$INSTALL_OSI_LIB_DIR/include -DCMAKE_INSTALL_PREFIX=$INSTALL_OSI_LIB_DIR -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_BUILD_TYPE=Debug -DCMAKE_LIBRARY_PATH=../protobuf$folder_postfix/protobuf-install/lib -DCMAKE_CXX_STANDARD=11 $ADDITIONAL_CMAKE_PARAMETERS ..

            # First bild OSI submodule separately since we need to rename the library before linking with the application
            cmake --build . --config Debug --target install

            if [[ "$OSTYPE" == "linux-gnu"* ]]; then
                if [ $DYNAMIC_LINKING == "1" ]; then
                    mv $INSTALL_OSI_LIB_DIR/lib/osi3/libopen_simulation_interface.so $INSTALL_OSI_LIB_DIR/lib/osi3/libopen_simulation_interfaced.so
                    mv $INSTALL_OSI_LIB_DIR/lib/osi3/libopen_simulation_interface.so.$OSI_VERSION $INSTALL_OSI_LIB_DIR/lib/osi3/libopen_simulation_interfaced.so.$OSI_VERSION
                else
                    mv $INSTALL_OSI_LIB_DIR/lib/osi3/libopen_simulation_interface_pic.a $INSTALL_OSI_LIB_DIR/lib/osi3/libopen_simulation_interface_picd.a
                    mv $INSTALL_OSI_LIB_DIR/lib/osi3/libopen_simulation_interface_static.a $INSTALL_OSI_LIB_DIR/lib/osi3/libopen_simulation_interface_staticd.a
                fi
                touch $INSTALL_OSI_LIB_DIR/lib/osi3/kalle.txt
            elif [ "$OSTYPE" == "msys" ]; then 
                mv $INSTALL_OSI_LIB_DIR/lib/osi3/open_simulation_interface.dll $INSTALL_OSI_LIB_DIR/lib/osi3/open_simulation_interfaced.dll 
                mv $INSTALL_OSI_LIB_DIR/lib/osi3/open_simulation_interface_pic.lib $INSTALL_OSI_LIB_DIR/lib/osi3/open_simulation_interface_picd.lib 
                mv $INSTALL_OSI_LIB_DIR/lib/osi3/open_simulation_interface_static.lib $INSTALL_OSI_LIB_DIR/lib/osi3/open_simulation_interface_staticd.lib
            fi
        fi

        rm CMakeCache.txt

        cmake .. -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DCMAKE_INCLUDE_PATH=../protobuf$folder_postfix/protobuf-install/include -DFILTER_PROTO2CPP_PY_PATH=../../proto2cpp -DINSTALL_LIB_DIR=$INSTALL_OSI_LIB_DIR/lib -DINSTALL_INCLUDE_DIR=$INSTALL_OSI_LIB_DIR/include -DCMAKE_INSTALL_PREFIX=$INSTALL_OSI_LIB_DIR -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_LIBRARY_PATH=../protobuf$folder_postfix/protobuf-install/lib -DCMAKE_CXX_STANDARD=11 $ADDITIONAL_CMAKE_PARAMETERS .. 

        cmake --build . --config Release --target install --clean-first

        cd $osi_root_dir

    else
        echo open-simulation-interface folder already exists, continue with next step...
    fi

    echo ------------------------ Pack $1 ------------------------------------

    cp open-simulation-interface$folder_postfix/install/osi-lib/include/osi3/* $target_dir/include
    cp -r protobuf$folder_postfix/protobuf-install/include/google $target_dir/include

    if [ $DYNAMIC_LINKING == "1" ]; then
        target_lib_dir="lib-dyn"
        mkdir $target_dir/$target_lib_dir
        if [ "$OSTYPE" == "msys" ]; then
            cp open-simulation-interface$folder_postfix/install/osi-lib/lib/osi3/open_simulation_interface*.dll $target_dir/$target_lib_dir
            cp open-simulation-interface$folder_postfix/install/osi-lib/lib/osi3/open_simulation_interface_pic*.lib $target_dir/$target_lib_dir
            cp protobuf$folder_postfix/protobuf-install/lib/libprotobuf*.lib $target_dir/$target_lib_dir
        elif [[ "$OSTYPE" == "darwin"* ]]; then
            cp -P open-simulation-interface$folder_postfix/install/osi-lib/lib/osi3/libopen_simulation_interface.dylib $target_dir/$target_lib_dir
            cp -P open-simulation-interface$folder_postfix/install/osi-lib/lib/osi3/libopen_simulation_interfaced.dylib $target_dir/$target_lib_dir
            cp -P protobuf$folder_postfix/protobuf-install/lib/libprotobuf.dylib $target_dir/$target_lib_dir
            cp -P protobuf$folder_postfix/protobuf-install/lib/libprotobufd.dylib $target_dir/$target_lib_dir
        else
            cp -P open-simulation-interface$folder_postfix/install/osi-lib/lib/osi3/libopen_simulation_interface.so* $target_dir/$target_lib_dir
            cp -P open-simulation-interface$folder_postfix/install/osi-lib/lib/osi3/libopen_simulation_interfaced.so* $target_dir/$target_lib_dir
            cp -P protobuf$folder_postfix/protobuf-install/lib/libprotobuf.so* $target_dir/$target_lib_dir
            cp -P protobuf$folder_postfix/protobuf-install/lib/libprotobufd.so* $target_dir/$target_lib_dir
        fi
    else
        target_lib_dir="lib"
        mkdir $target_dir/$target_lib_dir
        if [ "$OSTYPE" == "msys" ]; then
            cp open-simulation-interface$folder_postfix/install/osi-lib/lib/osi3/*open_simulation_interface_pic*.lib $target_dir/$target_lib_dir
            cp protobuf$folder_postfix/protobuf-install/lib/libprotobuf*.lib $target_dir/$target_lib_dir
        else
            cp open-simulation-interface$folder_postfix/install/osi-lib/lib/osi3/*open_simulation_interface_pic*.a $target_dir/$target_lib_dir
            cp protobuf$folder_postfix/protobuf-install/lib/libprotobuf*.a $target_dir/$target_lib_dir
        fi
    fi
    
    rm $target_dir/$target_lib_dir/libprotobuf-lite*

}

build static
build dynamic

mv 

"$z_exe" a -r $zfilename -m0=LZMA -bb1 -spf $target_dir/*
# unpack with: 7z x <filename>

echo ------------------------ Done ------------------------------------
cd $osi_root_dir
