# Set to the current lowest tested version of CMake
cmake_minimum_required (VERSION 3.7.0 FATAL_ERROR)

project (EnvironmentSimulator)

if(NOT MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()
set(CMAKE_CXX_STANDARD 14)

# Allow for folder structure
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# The static libaries needs to be compiled with position independent code
# otherwise we cant link with them when building dynamic libaries
# An alternative to this global option is to set POSITION_INDEPENDENT_CODE on each target
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


if ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "Win32")
  set(INSTALL_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin/win32")
elseif (WIN32)
  set(INSTALL_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin/win64")
elseif(UBUNTU32)
  set(INSTALL_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin/ubuntu32")
elseif(APPLE)
  set(INSTALL_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin/apple")
elseif((UNIX) AND (NOT APPLE))
  set(INSTALL_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin/linux")
else()
  message(FATAL_ERROR "Unrecognized platform therefore there isn't an installation directory. Stopping the cmake process.")
endif()

set(PUGIXML_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/externals/pugixml")

include_directories( ${PUGIXML_INCLUDE_DIR} )

set(USE_OSG True CACHE BOOL "If projects that depend on OpenSceneGraph should be compiled.")

add_subdirectory(EnvironmentSimulator)
