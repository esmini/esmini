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
# The folder osi-visualizer/build/Release (or Debug) will contain everything needed
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
    
	# Visual Studio 2019 - default toolkit
	# GENERATOR=("Visual Studio 16 2019")
	# GENERATOR_ARGUMENTS="-A x64 -T ${GENERATOR_TOOLSET}"

	# Visual Studio 2017 - default toolkit
	# GENERATOR=("Visual Studio 15 2017 Win64")
	# GENERATOR_ARGUMENTS="-T ${GENERATOR_TOOLSET}"
elif [ "$OSTYPE" == "darwin"* ] || [ "$OSTYPE" == "linux-gnu"* ]; then
	# Unix Makefiles (for Ubuntu and other Linux systems)
	GENERATOR=("Unix Makefiles")
	GENERATOR_ARGUMENTS=""
    LIB_EXT="a"
    CXXFLAGS="-fPIC" 
    CFLAGS="-fPIC"
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

    if [ "$OSTYPE" == "darwin"* ] || [ "$OSTYPE" == "linux-gnu"* ]; then
		cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Release ..
		cmake --build . --target install

        if [ "$OSTYPE" == "linux-gnu"* ]; then
            # Also build debug version on Linux
            rm CMakeCache.txt
            cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Debug ..
            cmake --build . --target install
            mv ../install/lib/zlib.${LIB_EXT} ../install/lib/zlibd.${LIB_EXT}
        fi
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

if [ ! -d xerces-c-3.2.2 ]; then
 	if [ ! -f xerces-c-3.2.2.zip ]; then
  	    curl "https://archive.apache.org/dist/xerces/c/3/sources/xerces-c-3.2.2.zip" -o xerces-c-3.2.2.zip
    fi
    unzip xerces-c-3.2.2.zip
    cd xerces-c-3.2.2
    mkdir xerces-install
    
    
    if [ "$OSTYPE" == "darwin"* ] || [ "$OSTYPE" == "linux-gnu"* ]; then
		cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=xerces-install -DCMAKE_BUILD_TYPE=Release ..
		cmake --build . --target install

        if [ "$OSTYPE" == "linux-gnu"* ]; then
            # Also build debug version on Linux
            rm CMakeCache.txt
            cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=xerces-install -DCMAKE_BUILD_TYPE=Debug ..
            cmake --build . --target install
#            mv xerces-install/lib/zlib.${LIB_EXT} ../install/lib/zlibd.${LIB_EXT}
        fi
	elif [ "$OSTYPE" == "msys" ]; then
        cmake . -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=xerces-install
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
#    git clone https://github.com/eclipse/sumo.git --depth 1 --branch v1_6_0
    cd sumo
    mkdir build-code; cd build-code

    ZLIB_LIBRARY_RELEASE=$sumo_root_dir/zlib-1.2.11/install/lib/zlib.${LIB_EXT}
    ZLIB_LIBRARY_DEBUG=$sumo_root_dir/zlib-1.2.11/install/lib/zlibd.${LIB_EXT}
    XercesC_LIBRARY_RELEASE=$sumo_root_dir/xerces-c-3.2.2/xerces-install/lib/xerces-c_3.${LIB_EXT}
    XercesC_LIBRARY_DEBUG=$sumo_root_dir/xerces-c-3.2.2/xerces-install/lib/xerces-c_3D.${LIB_EXT}
    XercesC_INCLUDE_DIR=$sumo_root_dir/xerces-c-3.2.2/xerces-install/include
    XercesC_VERSION=3.2.2

    if [ "$OSTYPE" != "darwin"* ]; then
        cmake .. -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DZLIB_INCLUDE_DIR=${sumo_root_dir}/zlib-1.2.11/install/include -DZLIB_LIBRARY=${ZLIB_LIBRARY_DEBUG} -DENABLE_PYTHON_BINDINGS=OFF -DENABLE_JAVA_BINDINGS=OFF -DCHECK_OPTIONAL_LIBS=OFF -DCMAKE_BUILD_TYPE=Debug -DXercesC_INCLUDE_DIR=${XercesC_INCLUDE_DIR} -DXercesC_LIBRARY=${XercesC_LIBRARY_DEBUG} -DXercesC_VERSION=${XercesC_VERSION}
        cmake --build . --config Debug
    fi

    rm CMakeCache.txt
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

#    cd $sumo_root_dir/sumo/src
#    cp **/*.h --parents $sumo_root_dir/esmini/externals/sumo/include

    echo Copying libraries

    cp $sumo_root_dir/zlib-1.2.11/install/lib/zlibstatic*.${LIB_EXT} $sumo_root_dir/esmini/externals/sumo/lib
    cp $sumo_root_dir/xerces-c-3.2.2/xerces-install/lib/xerces-c_3*.${LIB_EXT} $sumo_root_dir/esmini/externals/sumo/lib
    
    cd $sumo_root_dir/sumo/build-code/src
    for f in libsumostatic.${LIB_EXT} microsim_engine.${LIB_EXT} foreign_tcpip.${LIB_EXT} utils_traction_wire.${LIB_EXT} microsim_trigger.${LIB_EXT} microsim_actions.${LIB_EXT} traciserver.${LIB_EXT} mesosim.${LIB_EXT} foreign_phemlight.${LIB_EXT} microsim_cfmodels.${LIB_EXT} utils_iodevices.${LIB_EXT} microsim_lcmodels.${LIB_EXT} microsim_traffic_lights.${LIB_EXT} utils_shapes.${LIB_EXT} utils_emissions.${LIB_EXT} microsim_output.${LIB_EXT} netload.${LIB_EXT} microsim_devices.${LIB_EXT} microsim_transportables.${LIB_EXT} microsim.${LIB_EXT} utils_xml.${LIB_EXT} utils_vehicle.${LIB_EXT} utils_geom.${LIB_EXT} utils_common.${LIB_EXT} utils_distribution.${LIB_EXT} utils_options.${LIB_EXT}
    do
        newfile=$(echo $f |sed -e "s/\.${LIB_EXT}/d\.${LIB_EXT}/")
        echo "Renaming $f -> $newfile"
        find . -path "*Debug/$f" -exec cp {} $sumo_root_dir/esmini/externals/sumo/lib/$newfile \;
        find . -path "*Release/$f" -exec cp {} $sumo_root_dir/esmini/externals/sumo/lib/$f \;
    done
    
else
    echo sumo files probably already copied, continue with next step...
fi


echo ------------------------ Done ------------------------------------
cd $sumo_root_dir


