cmake_minimum_required(VERSION 3.25)

project(
    rm-basic-cs
    CSharp)

set(TARGET
    rm-basic-cs)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/rm-basic.cs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Libraries/esminiRMLib/esminiRMWrapper.cs)

add_executable(
    ${TARGET}
    ${SOURCES})

# cmake-format: off
#
# build as:
# mkdir build
# cd build
# cmake ..
# cmake --build . --config Release
#
# Run from esmini/bin:
# ../EnvironmentSimulator/code-examples/rm-basic-cs/build/Release/osi-groundtruth.exe
#
# cmake-format: on
