#!/bin/bash

#
# This script will build Sumo libraries for headless (no GUI) embedded integration
# in esmini. Only needed libs will be created, e.g. GUI and tests will be omitted.
#
# No system installations will be done (no admin rights required)
#
# The ambition is to support Windows, Linux and Mac. However the script is under
# development and has not been tested as is on all three platforms.
#
# Prerequisites:
# - Git (with Bash) (https://git-scm.com/download/win)
# - cmake (https://cmake.org/download/)
# - Visual Studio (with C++ toolkit) (https://visualstudio.microsoft.com/downloads/)
# - zip or 7zip
#   Linux: 7za. Install: "sudo apt install p7zip-full"
#   Mac: 7z
#
# Usage:
# - put this script in an empty folder
# - open Git Bash in that folder
# - review and adjust system dependent parameters in section below
# - run the script: ./generate_sumo_libs.sh
# - wait (the build process will take approx. 15 minutes depending on...)
#
# The folder esmini/externals/sumo will contain everything needed
#
# More info at: https://sumo.dlr.de/docs/Installing/Windows_Build.html
#
#

# -----------------------------------------------------------------------------------
# Review and update settings in this section according to your system and preferences

PARALLEL_BUILDS=8
ZLIB_VERSION=v1.2.12
XERCES_VERSION=v3.2.5
SUMO_VERSION=v1_19_0

if [ "$OSTYPE" == "msys" ]; then
    # Visual Studio 2022 using toolkit from Visual Studio 2017
    GENERATOR=("Visual Studio 17 2022")
    GENERATOR_TOOLSET="v141"
    GENERATOR_ARGUMENTS="-A x64 -T ${GENERATOR_TOOLSET}"

    # Visual Studio 2019 using default toolkit
    # GENERATOR=("Visual Studio 16 2019")
    # GENERATOR_ARGUMENTS="-A x64 -T ${GENERATOR_TOOLSET}"

    # Visual Studio 2017 using default toolkit
    # GENERATOR=("Visual Studio 15 2017 Win64")
    # GENERATOR_ARGUMENTS="-T ${GENERATOR_TOOLSET}"

    LIB_EXT="lib"
    CXXFLAGS=""
    CFLAGS=""
    LIB_PREFIX=""

elif [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux"* ]]; then
    # Unix Makefiles (for Ubuntu and other Linux systems)
    GENERATOR=("Unix Makefiles")
    GENERATOR_ARGUMENTS=""
    LIB_EXT="a"
    CXXFLAGS="-fPIC"
    CFLAGS="-fPIC"
    LIB_PREFIX="lib"
else
    echo Unknown OSTYPE: $OSTYPE
fi

if [ "$OSTYPE" == "msys" ]; then
    target_dir="v10"
    zfilename="sumo_v10_${SUMO_VERSION}.7z"
    z_exe="/c/Program Files/7-Zip/7z.exe"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    target_dir="linux"
    zfilename="sumo_linux_${SUMO_VERSION}.7z"
    z_exe=7za
elif [[ "$OSTYPE" == "darwin"* ]]; then
    target_dir="mac"
    zfilename="sumo_mac_${SUMO_VERSION}.7z"
    z_exe=7z
    macos_arch="arm64;x86_64"
else
    echo Unknown OSTYPE: $OSTYPE
fi


# ---------------------------------------------------------------------------------------
# From this point no adjustments should be necessary, except fixing bugs in the script :)
# However you might want to adjust versions of software packages being checkout and built


sumo_root_dir=$(pwd)

echo ------------------------ Installing zlib ------------------------------------
cd $sumo_root_dir

if [ ! -d zlib ]
then
    git clone https://github.com/madler/zlib.git --depth 1 --branch $ZLIB_VERSION
    cd zlib
    mkdir install
    mkdir build
    cd build

    if [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux"* ]]; then
        ADDITIONAL_CMAKE_PARAMETERS="-DCMAKE_C_FLAGS=-fPIC"

        if [[ "$OSTYPE" == "linux"* ]]; then
            # Also build debug version on Linux
            cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install -D CMAKE_BUILD_TYPE=Debug $ADDITIONAL_CMAKE_PARAMETERS ..
            cmake --build . -j $PARALLEL_BUILDS --target install
            mv ../install/lib/libz.${LIB_EXT} ../install/lib/libzlibstaticd.${LIB_EXT}
            rm CMakeCache.txt
        else
            ADDITIONAL_CMAKE_PARAMETERS+=" -DCMAKE_OSX_ARCHITECTURES=$macos_arch"
        fi

        cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install $ADDITIONAL_CMAKE_PARAMETERS ..
        cmake --build . -j $PARALLEL_BUILDS --target install
        mv ../install/lib/libz.${LIB_EXT} ../install/lib/libzlibstatic.${LIB_EXT}

    elif [ "$OSTYPE" == "msys" ]; then
        cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install ..
        cmake --build . -j $PARALLEL_BUILDS --config Debug --target install
        cmake --build . -j $PARALLEL_BUILDS --config Release --target install --clean-first
    else
        echo Unknown OSTYPE: $OSTYPE
    fi

else
    echo zlib folder already exists, continue with next step...
fi

echo ------------------------ Installing Xerces ------------------------------------
cd $sumo_root_dir

if [ ! -d xerces-c ]; then
    git clone https://github.com/apache/xerces-c.git --depth 1 --branch $XERCES_VERSION
    cd xerces-c
    mkdir xerces-install

    # Patch config to exlude ICU
    sed -i 's/include(XercesICU)/#include(XercesICU)/g' CMakeLists.txt

    if [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux"* ]]; then
        ADDITIONAL_CMAKE_PARAMETERS="-Dnetwork=OFF -DCMAKE_CXX_FLAGS=-fPIC"
        if [[ "$OSTYPE" == "linux"* ]]; then
            # Build debug version only on Linux (and Win)
            cmake . -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=xerces-install -DCMAKE_BUILD_TYPE=Debug $ADDITIONAL_CMAKE_PARAMETERS
            cmake --build . -j $PARALLEL_BUILDS --target install
            cp xerces-install/lib/libxerces-c.${LIB_EXT} xerces-install/lib/libxerces-c_3D.${LIB_EXT}
            rm CMakeCache.txt
        else
            ADDITIONAL_CMAKE_PARAMETERS+=" -DCMAKE_OSX_ARCHITECTURES=$macos_arch"
        fi

        cmake . -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=xerces-install -DCMAKE_BUILD_TYPE=Release $ADDITIONAL_CMAKE_PARAMETERS
        cmake --build . -j $PARALLEL_BUILDS --target install
        cp xerces-install/lib/libxerces-c.${LIB_EXT} xerces-install/lib/libxerces-c_3.${LIB_EXT}

    elif [ "$OSTYPE" == "msys" ]; then
        cmake . -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=xerces-install -Dnetwork=OFF
        cmake --build . -j $PARALLEL_BUILDS --config Debug --target install --clean-first
        cmake --build . -j $PARALLEL_BUILDS --config Release --target install
    else
        echo Unknown OSTYPE: $OSTYPE
    fi
else
    echo Xerces folder already exists, continue with next step...
fi

echo ------------------------ Installing Sumo ------------------------------------
cd $sumo_root_dir

modules="utils_common utils_options utils_distribution utils_geom utils_vehicle utils_xml microsim microsim_transportables microsim_devices netload utils_emissions microsim_output foreign_tcpip microsim_traffic_lights microsim_trigger utils_shapes utils_iodevices mesosim traciserver microsim_lcmodels microsim_cfmodels utils_traction_wire microsim_actions foreign_phemlight_V5 foreign_phemlight microsim_engine"

if [ ! -d sumo ]; then
    git clone https://github.com/eclipse/sumo.git --depth 1 --branch $SUMO_VERSION
    cd sumo

    mkdir build-code
    cd build-code

    ZLIB_LIBRARY_RELEASE=$sumo_root_dir/zlib/install/lib/${LIB_PREFIX}zlibstatic.${LIB_EXT}
    ZLIB_LIBRARY_DEBUG=$sumo_root_dir/zlib/install/lib/${LIB_PREFIX}zlibstaticd.${LIB_EXT}

    XercesC_LIBRARY_RELEASE=$sumo_root_dir/xerces-c/xerces-install/lib/${LIB_PREFIX}xerces-c_3.${LIB_EXT}
    XercesC_LIBRARY_DEBUG=$sumo_root_dir/xerces-c/xerces-install/lib/${LIB_PREFIX}xerces-c_3D.${LIB_EXT}
    XercesC_INCLUDE_DIR=$sumo_root_dir/xerces-c/xerces-install/include
    XercesC_VERSION=${XERCES_VERSION:1}  # skip first "v" character in version tag

    ADDITIONAL_CMAKE_PARAMETERS="-DXercesC_INCLUDE_DIR=${XercesC_INCLUDE_DIR} -DENABLE_PYTHON_BINDINGS=OFF -DENABLE_JAVA_BINDINGS=OFF -DCHECK_OPTIONAL_LIBS=OFF -DXercesC_VERSION=${XercesC_VERSION} -DPROJ_LIBRARY= -DFOX_CONFIG="

    if [[ "$OSTYPE" != "darwin"* ]]; then
        cmake .. -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DZLIB_INCLUDE_DIR=${sumo_root_dir}/zlib/install/include -DZLIB_LIBRARY=${ZLIB_LIBRARY_DEBUG} -DCMAKE_BUILD_TYPE=Debug  -DXercesC_LIBRARY=${XercesC_LIBRARY_DEBUG} $ADDITIONAL_CMAKE_PARAMETERS
        cmake --build . -j $PARALLEL_BUILDS --config Debug

        for f in $modules
        do
            if [[ "$OSTYPE" = "msys" ]]; then
                file_path=`find . -path "*Debug/${LIB_PREFIX}$f.${LIB_EXT}"`
            else
                file_path=`find . -name "${LIB_PREFIX}$f.${LIB_EXT}"`
            fi
            new_file=$(echo $file_path | sed s/\\.${LIB_EXT}/d\\.${LIB_EXT}/)
            cp $file_path $new_file
        done

        rm CMakeCache.txt
    else
        ADDITIONAL_CMAKE_PARAMETERS+=" -DCMAKE_OSX_ARCHITECTURES=$macos_arch"
        export LDFLAGS="-framework CoreServices"
    fi

    cmake .. -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DZLIB_INCLUDE_DIR=${sumo_root_dir}/zlib/install/include -DZLIB_LIBRARY=${ZLIB_LIBRARY_RELEASE} -DCMAKE_BUILD_TYPE=Release -DXercesC_LIBRARY=${XercesC_LIBRARY_RELEASE} $ADDITIONAL_CMAKE_PARAMETERS
    cmake --build . -j $PARALLEL_BUILDS --config Release --clean-first

else
    echo sumo folder already exists, continue with next step...
fi

echo --------------------- Copying files needed for esmini integration -------------------------------
cd $sumo_root_dir

if [ ! -d $target_dir ]
then
    mkdir $target_dir
    mkdir $target_dir/lib
    mkdir $target_dir/include
    mkdir $target_dir/include/libsumo
    mkdir $target_dir/include/utils
    mkdir $target_dir/include/utils/common
    mkdir $target_dir/include/utils/geom
    mkdir $target_dir/include/utils/vehicle
    mkdir $target_dir/include/utils/xml

    echo Copying header files

    cp $sumo_root_dir/sumo/build-code/src/config.h $sumo_root_dir/$target_dir/include

    cd $sumo_root_dir/sumo/src/utils/common
    for f in Parameterised.h RGBColor.h StdDefs.h StringBijection.h SUMOTime.h SUMOVehicleClass.h UtilExceptions.h
    do
        cp $f $sumo_root_dir/$target_dir/include/utils/common
    done

    cd $sumo_root_dir/sumo/src/utils/geom
    for f in AbstractPoly.h Position.h PositionVector.h
    do
        cp $f $sumo_root_dir/$target_dir/include/utils/geom
    done

    cp $sumo_root_dir/sumo/src/utils/vehicle/SUMOVehicleParameter.h $sumo_root_dir/$target_dir/include/utils/vehicle
    cp $sumo_root_dir/sumo/src/utils/xml/SUMOXMLDefinitions.h $sumo_root_dir/$target_dir/include/utils/xml

    cd $sumo_root_dir/sumo/src/libsumo
    for f in Simulation.h TraCIConstants.h TraCIDefs.h Vehicle.h VehicleType.h
    do
        cp $f $sumo_root_dir/$target_dir/include/libsumo
    done

    echo Copying libraries

    cp $sumo_root_dir/zlib/install/lib/${LIB_PREFIX}zlibstatic*.${LIB_EXT} $sumo_root_dir/$target_dir/lib
    cp $sumo_root_dir/xerces-c/xerces-install/lib/${LIB_PREFIX}xerces-c_3*.${LIB_EXT} $sumo_root_dir/$target_dir/lib

    cd $sumo_root_dir
    cp "./sumo/bin/libsumostaticD.${LIB_EXT}" "$target_dir/lib/libsumostaticd.${LIB_EXT}"
    cp "./sumo/bin/libsumostatic.${LIB_EXT}" "$target_dir/lib/libsumostatic.${LIB_EXT}"

    cd $sumo_root_dir/sumo/build-code/src

    for f in sumostatic $modules
    do
        find . -type f -regex .*${LIB_PREFIX}"$f"d?.${LIB_EXT} -exec cp {} $sumo_root_dir/$target_dir/lib/ \;
    done

else
    echo sumo files probably already copied, continue with next step...
fi

cd $sumo_root_dir
if [ ! -f $zfilename ]; then
    "$z_exe" a -r $zfilename -m0=LZMA -bb1 -spf $target_dir/*
fi
# unpack with: 7z x <filename>

echo ------------------------ Done ------------------------------------
