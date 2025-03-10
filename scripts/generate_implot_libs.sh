#!/bin/bash

#
# This script will build implot and dependencies glfw and imgui
# needed for esmini real-time plotting feature (credit Simon Lundell).
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
# - open bash (e.g. Git Bash) in any temporary and empty folder
# - review and adjust system dependent parameters in section below
# - run the script, e.g. ~/esmini/scrips/generate_implot.sh
# - wait (the build process will take some minutes)
#
# The implot_*.7z will contain both headers and needed libraries. (* depends on platform)
#
#

# -----------------------------------------------------------------------------------
# Review and update settings in this section according to your system and preferences

PARALLEL_BUILDS=4
ZIP_MIN_VERSION=12

if (( "$PARALLEL_BUILDS" < 2 )); then
    PARALLEL_ARG=""
else
    PARALLEL_ARG="-j $PARALLEL_BUILDS"
fi

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

    target_dir="v10"
    zfilename="implot_v10.7z"
    z_exe="$PROGRAMFILES/7-Zip/7z"

elif [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Unix Makefiles (for Ubuntu and other Linux systems)
    GENERATOR=("Unix Makefiles")
    GENERATOR_ARGUMENTS=""
    LIB_EXT="a"
    LIB_PREFIX="lib"
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        target_dir="linux"
        zfilename="implot_linux.7z"
        z_exe=7za
    else
        target_dir="mac"
        zfilename="implot_mac.7z"
        z_exe=7z
        macos_arch="arm64;x86_64"
    fi
else
    echo Unknown OSTYPE: $OSTYPE
fi

GLFW_VERSION=3.3.8
IMGUI_VERSION=v1.90.4
IMPLOT_VERSION=v0.16

# ---------------------------------------------------------------------------------------
# From this point no adjustments should be necessary, except fixing bugs in the script :)

implot_root_dir=$(pwd)

echo ------------------------ checkout ------------------------------------

git clone https://github.com/glfw/glfw --depth 1 --branch "$GLFW_VERSION"
git clone https://github.com/ocornut/imgui --depth 1 --branch "$IMGUI_VERSION"
git clone https://github.com/epezent/implot --depth 1 --branch "$IMPLOT_VERSION"

echo ------------------------ create cmake file ---------------------------

file="CMakeLists.txt"
echo "cmake_minimum_required(VERSION 3.7.1)" > $file
echo "project(implot)" >> $file
echo "include_directories("implot" "imgui" "imgui/backends" "glfw/include")" >> $file
echo "add_library(implot "implot/implot.cpp" "implot/implot_items.cpp" \
"imgui/imgui.cpp" "imgui/imgui_draw.cpp" "imgui/imgui_tables.cpp" "imgui/imgui_widgets.cpp" \
"imgui/backends/imgui_impl_glfw.cpp" "imgui/backends/imgui_impl_opengl3.cpp")" >> $file

echo ------------------------ compile glfw --------------------------------

cd $implot_root_dir
mkdir glfw/build;cd glfw/build

if [ "$OSTYPE" == "msys" ]; then

    cmake .. -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS}
    cmake --build . $PARALLEL_ARG --config Release
    cmake --build . $PARALLEL_ARG --config Debug

elif  [[ "$OSTYPE" == "darwin"* ]] ; then

    cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DCMAKE_BUILD_TYPE=Debug .. -DCMAKE_C_FLAGS="-fPIC" -DCMAKE_OSX_ARCHITECTURES="$macos_arch"
    cmake --build . $PARALLEL_ARG

elif [[ "$OSTYPE" == "linux-gnu"* ]]; then

    cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DCMAKE_BUILD_TYPE=Debug .. -DCMAKE_C_FLAGS="-fPIC"
    cmake --build . $PARALLEL_ARG
    mv src/libglfw3.a src/libglfw3d.a

    rm CMakeCache.txt
    cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DCMAKE_BUILD_TYPE=Release .. -DCMAKE_C_FLAGS="-fPIC"
    cmake --build . $PARALLEL_ARG

fi

echo ------------------------ compile implot ------------------------------

cd $implot_root_dir
mkdir build;cd build

if [ "$OSTYPE" == "msys" ]; then

	cmake .. -G "Visual Studio 16 2019" -T v142 -A x64
	cmake --build . -j $PARALLEL_ARG --config Release
	cmake --build . -j $PARALLEL_ARG --config Debug

elif  [[ "$OSTYPE" == "darwin"* ]] ; then

    cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DCMAKE_BUILD_TYPE=Release .. -DCMAKE_C_FLAGS="-fPIC" -DCMAKE_CXX_FLAGS="-std=c++11" -DCMAKE_OSX_ARCHITECTURES="$macos_arch"
    cmake --build . $PARALLEL_ARG

elif [[ "$OSTYPE" == "linux-gnu"* ]]; then

    cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DCMAKE_BUILD_TYPE=Debug .. -DCMAKE_C_FLAGS="-fPIC"
    cmake --build . $PARALLEL_ARG
    mv libimplot.a libimplotd.a

    rm CMakeCache.txt
    cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -DCMAKE_BUILD_TYPE=Release .. -DCMAKE_C_FLAGS="-fPIC"
    cmake --build . $PARALLEL_ARG

fi

echo --------------------- Copying files needed for esmini integration -------------------------------
cd $implot_root_dir

if [ ! -d $target_dir ]
then
    mkdir $target_dir
    mkdir $target_dir/lib
    mkdir $target_dir/include
    mkdir $target_dir/include/imgui
    mkdir $target_dir/include/imgui/backends
    mkdir $target_dir/include/implot
    mkdir $target_dir/include/glfw
	mkdir $target_dir/include/glfw/GLFW

    echo Copying header files

    cp imgui/imconfig.h $target_dir/include/imgui
    cp imgui/imgui.h $target_dir/include/imgui
    cp imgui/imgui_internal.h $target_dir/include/imgui
    cp imgui/imstb_rectpack.h $target_dir/include/imgui
    cp imgui/imstb_textedit.h $target_dir/include/imgui
    cp imgui/imstb_truetype.h $target_dir/include/imgui
    cp imgui/backends/imgui_impl_glfw.h $target_dir/include/imgui/backends
    cp imgui/backends/imgui_impl_opengl3.h $target_dir/include/imgui/backends
    cp imgui/backends/imgui_impl_opengl3_loader.h $target_dir/include/imgui/backends
	cp implot/implot.h $target_dir/include/implot
    cp implot/implot_internal.h $target_dir/include/implot
    cp glfw/include/GLFW/glfw3.h $target_dir/include/glfw/GLFW
	cp glfw/include/GLFW/glfw3native.h $target_dir/include/glfw/GLFW

    echo Copying libs

	if [ "$OSTYPE" == "msys" ]; then

		cp glfw/build/src/Release/glfw3.lib $target_dir/lib
		cp glfw/build/src/Debug/glfw3.lib $target_dir/lib/glfw3d.lib
		cp build/Release/implot.lib $target_dir/lib
		cp build/Debug/implot.lib $target_dir/lib/implotd.lib

	elif  [[ "$OSTYPE" == "darwin"* ]]; then

		cp glfw/build/src/libglfw3.a $target_dir/lib
		cp build/libimplot.a $target_dir/lib

    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then

		cp glfw/build/src/libglfw3.a $target_dir/lib
		cp glfw/build/src/libglfw3d.a $target_dir/lib
		cp build/libimplot.a $target_dir/lib
		cp build/libimplotd.a $target_dir/lib

	fi

fi

echo ------------------------ Pack ------------------------------------

cd $implot_root_dir

"$z_exe" a -r $zfilename -m0=LZMA -bb1 -spf $target_dir/*
# unpack with: 7z x <filename>

echo ------------------------ Done ------------------------------------
