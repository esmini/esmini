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

if [ "$OSTYPE" == "msys" ]; then
	# Visual Studio 2019 - toolkit from Visual Studio 2017
	GENERATOR=("Visual Studio 16 2019")
	GENERATOR_TOOLSET="v141"
	GENERATOR_ARGUMENTS="-A x64 -T ${GENERATOR_TOOLSET}"
    LIB_EXT="lib"
    CXXFLAGS=""
    CFLAGS=""
    LIB_PREFIX=""
    
	# Visual Studio 2019 - default toolkit
	# GENERATOR=("Visual Studio 16 2019")
	# GENERATOR_ARGUMENTS="-A x64 -T ${GENERATOR_TOOLSET}"

	# Visual Studio 2017 - default toolkit
	# GENERATOR=("Visual Studio 15 2017 Win64")
	# GENERATOR_ARGUMENTS="-T ${GENERATOR_TOOLSET}"
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


# ---------------------------------------------------------------------------------------
# From this point no adjustments should be necessary, except fixing bugs in the script :)
# However you might want to adjust versions of software packages being checkout and built


sumo_root_dir=$(pwd)

echo ------------------------ Installing zlib ------------------------------------
cd $sumo_root_dir

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

    if [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux"* ]]; then
        
        if [[ "$OSTYPE" == "linux"* ]]; then
            # Also build debug version on Linux
            cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_FLAGS="-fPIC" ..
            cmake --build . --target install
            mv ../install/lib/libz.${LIB_EXT} ../install/lib/libzlibstaticd.${LIB_EXT}
            rm CMakeCache.txt
        fi

        cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS="-fPIC" ..
		cmake --build . --target install 
        mv ../install/lib/libz.${LIB_EXT} ../install/lib/libzlibstatic.${LIB_EXT}

	elif [ "$OSTYPE" == "msys" ]; then
		cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install ..
		cmake --build . --config Debug --target install
		cmake --build . --config Release --target install --clean-first
	else
        echo Unknown OSTYPE: $OSTYPE
    fi

else
    echo zlib folder already exists, continue with next step...
fi

echo ------------------------ Installing Xerces ------------------------------------
cd $sumo_root_dir

# in module cmake/XercesICU.cmake, comment out FIND statement: #find_package(ICU COMPONENTS uc data)

if [ ! -d xerces-c-3.2.2 ]; then
 	if [ ! -f xerces-c-3.2.2.zip ]; then
  	    curl "https://archive.apache.org/dist/xerces/c/3/sources/xerces-c-3.2.2.zip" -o xerces-c-3.2.2.zip
    fi
    unzip xerces-c-3.2.2.zip
    cd xerces-c-3.2.2
    mkdir xerces-install

    # Patch config to exlude ICU
    sed -ie 's/include(XercesICU)/#include(XercesICU)/g' CMakeLists.txt
    
    if [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux"* ]]; then
        if [[ "$OSTYPE" == "linux"* ]]; then        
            # Build debug version only on Linux (and Win)
            cmake . -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=xerces-install -DCMAKE_BUILD_TYPE=Debug -Dnetwork=OFF -DCMAKE_CXX_FLAGS="-fPIC"
            cmake --build . --target install
            mv xerces-install/lib/libxerces-c-3.2.${LIB_EXT} xerces-install/lib/libxerces-c_3D.${LIB_EXT}
            rm CMakeCache.txt
        fi
        
        cmake . -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=xerces-install -DCMAKE_BUILD_TYPE=Release -Dnetwork=OFF -DCMAKE_CXX_FLAGS="-fPIC"
        cmake --build . --target install
        mv xerces-install/lib/libxerces-c-3.2.${LIB_EXT} xerces-install/lib/libxerces-c_3.${LIB_EXT}

    elif [ "$OSTYPE" == "msys" ]; then
        cmake . -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=xerces-install -Dnetwork=OFF
        cmake --build . --config Debug --target install --clean-first  
        cmake --build . --config Release --target install 
	else
        echo Unknown OSTYPE: $OSTYPE
    fi
else
    echo Xerces folder already exists, continue with next step...
fi

echo ------------------------ Installing Sumo ------------------------------------
cd $sumo_root_dir

if [ ! -d sumo ]; then
    git clone https://github.com/eclipse/sumo.git --depth 1 --branch v1_6_0
    cd sumo

    mkdir build-code; cd build-code
    
    ZLIB_LIBRARY_RELEASE=$sumo_root_dir/zlib-1.2.11/install/lib/${LIB_PREFIX}zlibstatic.${LIB_EXT}
    ZLIB_LIBRARY_DEBUG=$sumo_root_dir/zlib-1.2.11/install/lib/${LIB_PREFIX}zlibstaticd.${LIB_EXT}
    
    XercesC_LIBRARY_RELEASE=$sumo_root_dir/xerces-c-3.2.2/xerces-install/lib/${LIB_PREFIX}xerces-c_3.${LIB_EXT}
    XercesC_LIBRARY_DEBUG=$sumo_root_dir/xerces-c-3.2.2/xerces-install/lib/${LIB_PREFIX}xerces-c_3D.${LIB_EXT}
    XercesC_INCLUDE_DIR=$sumo_root_dir/xerces-c-3.2.2/xerces-install/include
    XercesC_VERSION=3.2.2
    
    if [[ "$OSTYPE" != "darwin"* ]]; then
        cmake .. -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DZLIB_INCLUDE_DIR=${sumo_root_dir}/zlib-1.2.11/install/include -DZLIB_LIBRARY=${ZLIB_LIBRARY_DEBUG} -DENABLE_PYTHON_BINDINGS=OFF -DENABLE_JAVA_BINDINGS=OFF -DCHECK_OPTIONAL_LIBS=OFF -DCMAKE_BUILD_TYPE=Debug -DXercesC_INCLUDE_DIR=${XercesC_INCLUDE_DIR} -DXercesC_LIBRARY=${XercesC_LIBRARY_DEBUG} -DXercesC_VERSION=${XercesC_VERSION} -DPROJ_LIBRARY= -DFOX_CONFIG=
        cmake --build . --config Debug
        
        for f in ${LIB_PREFIX}libsumostatic.${LIB_EXT} ${LIB_PREFIX}microsim_engine.${LIB_EXT} ${LIB_PREFIX}foreign_tcpip.${LIB_EXT} ${LIB_PREFIX}utils_traction_wire.${LIB_EXT} ${LIB_PREFIX}microsim_trigger.${LIB_EXT} ${LIB_PREFIX}microsim_actions.${LIB_EXT} ${LIB_PREFIX}traciserver.${LIB_EXT} ${LIB_PREFIX}mesosim.${LIB_EXT} ${LIB_PREFIX}foreign_phemlight.${LIB_EXT} ${LIB_PREFIX}microsim_cfmodels.${LIB_EXT} ${LIB_PREFIX}utils_iodevices.${LIB_EXT} ${LIB_PREFIX}microsim_lcmodels.${LIB_EXT} ${LIB_PREFIX}microsim_traffic_lights.${LIB_EXT} ${LIB_PREFIX}utils_shapes.${LIB_EXT} ${LIB_PREFIX}utils_emissions.${LIB_EXT} ${LIB_PREFIX}microsim_output.${LIB_EXT} ${LIB_PREFIX}netload.${LIB_EXT} ${LIB_PREFIX}microsim_devices.${LIB_EXT} ${LIB_PREFIX}microsim_transportables.${LIB_EXT} ${LIB_PREFIX}microsim.${LIB_EXT} ${LIB_PREFIX}utils_xml.${LIB_EXT} ${LIB_PREFIX}utils_vehicle.${LIB_EXT} ${LIB_PREFIX}utils_geom.${LIB_EXT} ${LIB_PREFIX}utils_common.${LIB_EXT} ${LIB_PREFIX}utils_distribution.${LIB_EXT} ${LIB_PREFIX}utils_options.${LIB_EXT}
        do
            if [[ "$OSTYPE" = "msys" ]]; then
                file_path=`find . -path "*Debug/$f"`
            else
                file_path=`find . -name "$f"`
            fi
            new_file=$(echo $file_path | sed s/\\.${LIB_EXT}/d\\.${LIB_EXT}/)
            echo "Renaming $file_path -> $new_file"
            mv $file_path $new_file
        done
    
        rm CMakeCache.txt
    fi

    if [[ "$OSTYPE" != "darwin"* ]]; then
        export LDFLAGS="-framework CoreServices"
    fi

    cmake .. -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DZLIB_INCLUDE_DIR=${sumo_root_dir}/zlib-1.2.11/install/include -DZLIB_LIBRARY=${ZLIB_LIBRARY_RELEASE} -DENABLE_PYTHON_BINDINGS=OFF -DENABLE_JAVA_BINDINGS=OFF -DCHECK_OPTIONAL_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DXercesC_INCLUDE_DIR=${XercesC_INCLUDE_DIR} -DXercesC_LIBRARY=${XercesC_LIBRARY_RELEASE} -DXercesC_VERSION=${XercesC_VERSION}

    cmake --build . --config Release --clean-first  

else
    echo sumo folder already exists, continue with next step...
fi

echo --------------------- Copying files needed for esmini integration -------------------------------
cd $sumo_root_dir

if [ ! -d esmini/externals/sumo ] 
then
    mkdir esmini
    mkdir esmini/externals
    mkdir esmini/externals/sumo
    mkdir esmini/externals/sumo/lib
    mkdir esmini/externals/sumo/include
    mkdir esmini/externals/sumo/include/libsumo
    mkdir esmini/externals/sumo/include/utils
    mkdir esmini/externals/sumo/include/utils/common
    mkdir esmini/externals/sumo/include/utils/geom
    mkdir esmini/externals/sumo/include/utils/vehicle
    mkdir esmini/externals/sumo/include/utils/xml
    
    echo Copying header files
    
    cp $sumo_root_dir/sumo/build-code/src/config.h $sumo_root_dir/esmini/externals/sumo/include

    cd $sumo_root_dir/sumo/src/utils/common
    for f in Parameterised.h RGBColor.h StdDefs.h StringBijection.h SUMOTime.h SUMOVehicleClass.h UtilExceptions.h
    do
        cp $f $sumo_root_dir/esmini/externals/sumo/include/utils/common
    done
    
    cd $sumo_root_dir/sumo/src/utils/geom
    for f in AbstractPoly.h Position.h PositionVector.h
    do
        cp $f $sumo_root_dir/esmini/externals/sumo/include/utils/geom
    done
    
    cp $sumo_root_dir/sumo/src/utils/vehicle/SUMOVehicleParameter.h $sumo_root_dir/esmini/externals/sumo/include/utils/vehicle
    cp $sumo_root_dir/sumo/src/utils/xml/SUMOXMLDefinitions.h $sumo_root_dir/esmini/externals/sumo/include/utils/xml

    cd $sumo_root_dir/sumo/src/libsumo
    for f in Simulation.h TraCIConstants.h TraCIDefs.h Vehicle.h VehicleType.h 
    do
        cp $f $sumo_root_dir/esmini/externals/sumo/include/libsumo
    done

    echo Copying libraries

    cp $sumo_root_dir/zlib-1.2.11/install/lib/zlibstatic*.${LIB_EXT} $sumo_root_dir/esmini/externals/sumo/lib
    cp $sumo_root_dir/xerces-c-3.2.2/xerces-install/lib/xerces-c_3*.${LIB_EXT} $sumo_root_dir/esmini/externals/sumo/lib
    
    cd $sumo_root_dir/sumo/build-code/src

    for f in ${LIB_PREFIX}libsumostaticd?.${LIB_EXT} ${LIB_PREFIX}microsim_engined?.${LIB_EXT} ${LIB_PREFIX}foreign_tcpipd?.${LIB_EXT} ${LIB_PREFIX}utils_traction_wired?.${LIB_EXT} ${LIB_PREFIX}microsim_triggerd?.${LIB_EXT} ${LIB_PREFIX}microsim_actionsd?.${LIB_EXT} ${LIB_PREFIX}traciserverd?.${LIB_EXT} ${LIB_PREFIX}mesosimd?.${LIB_EXT} ${LIB_PREFIX}foreign_phemlightd?.${LIB_EXT} ${LIB_PREFIX}microsim_cfmodelsd?.${LIB_EXT} ${LIB_PREFIX}utils_iodevicesd?.${LIB_EXT} ${LIB_PREFIX}microsim_lcmodelsd?.${LIB_EXT} ${LIB_PREFIX}microsim_traffic_lightsd?.${LIB_EXT} ${LIB_PREFIX}utils_shapesd?.${LIB_EXT} ${LIB_PREFIX}utils_emissionsd?.${LIB_EXT} ${LIB_PREFIX}microsim_outputd?.${LIB_EXT} ${LIB_PREFIX}netloadd?.${LIB_EXT} ${LIB_PREFIX}microsim_devicesd?.${LIB_EXT} ${LIB_PREFIX}microsim_transportablesd?.${LIB_EXT} ${LIB_PREFIX}microsimd?.${LIB_EXT} ${LIB_PREFIX}utils_xmld?.${LIB_EXT} ${LIB_PREFIX}utils_vehicled?.${LIB_EXT} ${LIB_PREFIX}utils_geomd?.${LIB_EXT} ${LIB_PREFIX}utils_commond?.${LIB_EXT} ${LIB_PREFIX}utils_distributiond?.${LIB_EXT} ${LIB_PREFIX}utils_optionsd?.${LIB_EXT}
    do
        echo $f
        find -E . -type f -regex ".*$f" -exec cp {} $sumo_root_dir/esmini/externals/sumo/lib/ \;
    done

    # To create the 7z package, use: 7z a filename.7z -m0=LZMA .
    
else
    echo sumo files probably already copied, continue with next step...
fi


echo ------------------------ Done ------------------------------------
cd $sumo_root_dir


