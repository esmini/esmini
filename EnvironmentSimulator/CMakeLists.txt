
set ( FILE_STORAGE "github" ) # "github", "dropbox", "google"  (Dropbox limited GB/Day)

set ( VIEWER_BASE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Modules/ViewerBase" )
set ( PLAYER_BASE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Modules/PlayerBase" )
set ( ROADMANAGER_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Modules/RoadManager" )
set ( ROADMANAGER_DLL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Libraries/esminiRMLib" )
set ( SCENARIOENGINE_DLL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Libraries/esminiLib" )
set ( REPLAY_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Applications/replayer" )
set ( SCENARIOENGINE_INCLUDE_DIRS
 "${CMAKE_CURRENT_SOURCE_DIR}/Modules/ScenarioEngine/SourceFiles"
 "${CMAKE_CURRENT_SOURCE_DIR}/Modules/ScenarioEngine/OSCTypeDefs"
 )
set ( COMMON_MINI_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Modules/CommonMini")
set ( CONTROLLERS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Modules/Controllers")

# OpenSceneGraph package adapted for this project
set ( OSG_VERSION "osg161" )
set ( OSG_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../externals/OpenSceneGraph" )
set ( OSI_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../externals/OSI" )
set ( SUMO_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../externals/SUMO" )

# GoogleTest package
set ( GTEST_VERSION "1.10.1" )
set ( GTEST_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../externals/googletest" )

set( CMAKE_VERBOSE_MAKEFILE true )

if ( ${FILE_STORAGE} STREQUAL "github" )
    set ( MODELS_PACKAGE_URL https://github.com/esmini/resources/raw/main/models/models.7z )
    if (APPLE)
        set ( OSG_PACKAGE_URL https://github.com/esmini/resources/raw/main/libs/OpenSceneGraph_mac.7z )
        set ( OSI_PACKAGE_URL https://github.com/esmini/resources/raw/main/libs/osi_mac.7z )
        set ( SUMO_PACKAGE_URL https://github.com/esmini/resources/raw/main/libs/sumo_mac.7z )
    elseif (LINUX)
        set ( OSG_PACKAGE_URL https://github.com/esmini/resources/raw/main/libs/OpenSceneGraph_linux.7z )
        set ( OSI_PACKAGE_URL https://github.com/esmini/resources/raw/main/libs/osi_linux.7z )
        set ( SUMO_PACKAGE_URL https://github.com/esmini/resources/raw/main/libs/sumo_linux.7z )
        set ( GTEST_PACKAGE_URL https://github.com/esmini/resources/raw/main/libs/googletest_linux.7z )
    elseif (MSVC)
        set ( OSG_PACKAGE_URL https://github.com/esmini/resources/raw/main/libs/OpenSceneGraph_v10.7z )
        set ( OSI_PACKAGE_URL https://github.com/esmini/resources/raw/main/libs/osi_v10.7z )
        set ( SUMO_PACKAGE_URL https://github.com/esmini/resources/raw/main/libs/sumo_v10.7z )
        set ( GTEST_PACKAGE_URL https://github.com/esmini/resources/raw/main/libs/googletest_v10.7z )
    elseif (MINGW)
        message("MinGW, enforcing slimmed esmini")
    else ()
        message ("Unsupported configuration")
    endif ()
elseif ( ${FILE_STORAGE} STREQUAL "dropbox" )
    set ( MODELS_PACKAGE_URL https://dl.dropboxusercontent.com/s/5gk8bvgzqiaaoco/models.7z?dl=0 )
    if (APPLE)
        set ( OSG_PACKAGE_URL https://dl.dropboxusercontent.com/s/d0czj6b89p9jyvv/OpenSceneGraph_mac.7z?dl=0 )
        set ( OSI_PACKAGE_URL https://dl.dropboxusercontent.com/s/m62v19gp0m73dte/osi_mac.7z?dl=0 )
        set ( SUMO_PACKAGE_URL https://dl.dropboxusercontent.com/s/0x8kwztk7nmacs1/sumo_mac.7z?dl=0 )
    elseif (LINUX)
        set ( OSG_PACKAGE_URL https://dl.dropboxusercontent.com/s/3dlev34kj94lir5/OpenSceneGraph_linux.7z?dl=0 )
        set ( OSI_PACKAGE_URL https://dl.dropboxusercontent.com/s/kwtdg0c1c8pawa1/osi_linux.7z?dl=0 )
        set ( SUMO_PACKAGE_URL https://dl.dropboxusercontent.com/s/gfwtqd3gf76f86a/sumo_linux.7z?dl=0 )
        set ( GTEST_PACKAGE_URL https://dl.dropboxusercontent.com/s/si7jsjjsy5bpoym/googletest_linux.7z?dl=0 )
    elseif (MSVC)
        set ( OSG_PACKAGE_URL https://dl.dropboxusercontent.com/s/e95hnoo782p40uc/OpenSceneGraph_v10.7z?dl=0 )
        set ( OSI_PACKAGE_URL https://dl.dropboxusercontent.com/s/an58ckp2qfx5069/osi_v10.7z?dl=0 )
        set ( SUMO_PACKAGE_URL https://dl.dropboxusercontent.com/s/5jtpnnd61wonxuh/sumo_v10.7z?dl=0 )
        set ( GTEST_PACKAGE_URL https://dl.dropboxusercontent.com/s/aaiehwzc6woqbc6/googletest_v10.7z?dl=0 )
    elseif (MINGW)
        message("MinGW, enforcing slimmed esmini")
    else ()
        message ("Unsupported configuration")
    endif ()
elseif ( ${FILE_STORAGE} STREQUAL "google" )
    set ( MODELS_PACKAGE_URL https://drive.google.com/u/1/uc?id=1c3cqRzwY41gWXbg0rmugQkL5I_5L6DH_&export=download )
    if (APPLE)
        set ( OSG_PACKAGE_URL https://drive.google.com/u/1/uc?id=1-1ySbmgX_Z1wXGOp-s6uouDpbmubw9zg&export=download )
        set ( OSI_PACKAGE_URL https://drive.google.com/u/1/uc?id=1UVzO8cPQaDU9KVn9v2v8Suj0uUw1dzYI&export=download )
        set ( SUMO_PACKAGE_URL https://drive.google.com/u/1/uc?id=1FAve0-MlJPv6lUZy0HvriZI7xstLAzvX&export=download )
    elseif (LINUX)
        set ( OSG_PACKAGE_URL https://drive.google.com/u/1/uc?id=135nYmQlwrG0j8mhsXmcwsd94NYPGLOPD&export=download )
        set ( OSI_PACKAGE_URL https://drive.google.com/u/1/uc?id=1Q8O9YciIC0BPEszIKtQ2UW9KcVRZS4iB&export=download )
        set ( SUMO_PACKAGE_URL https://drive.google.com/u/1/uc?id=1m4znxNIXapP0D-l21oIm2l7L5ti-JbZH&export=download )
        set ( GTEST_PACKAGE_URL https://drive.google.com/u/1/uc?id=1Hyr9eJX2GmgpYwZhx14xOoXlZ2j-FY_p&export=download )
    elseif (MSVC)
        set ( OSG_PACKAGE_URL https://drive.google.com/u/1/uc?id=1RTag0aUn_pJPK697j0-E72ABW10wZvOm&export=download )
        set ( OSI_PACKAGE_URL https://drive.google.com/u/1/uc?id=1pcQcVHUESOk2Wmi-zUA7uzdxxE6iwRJx&export=download )
        set ( SUMO_PACKAGE_URL https://drive.google.com/u/1/uc?id=18PhbSLyvs0IGWTAY3YBoYzpVnMFPbOuR&export=download )
        set ( GTEST_PACKAGE_URL https://drive.google.com/u/1/uc?id=1So-3gtrmEdW9RhEvVQisj1QFksHM_otU&export=download )
    elseif (MINGW)
        message("MinGW, enforcing slimmed esmini")
    else ()
        message ("Unsupported configuration")
    endif ()
else ()
    Message("Unknown storage type: " ${FILE_STORAGE})
endif()

if (APPLE)
  set ( EXT_DIR_NAME "mac" )
  set ( TIME_LIB "" )
elseif (LINUX)
  set ( EXT_DIR_NAME "linux" )
  set ( TIME_LIB "" )
elseif (MINGW)
    set ( SOCK_LIB Ws2_32.lib )
    set ( TIME_LIB winmm )
elseif (MSVC)
  if ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "x64")
    set ( EXT_DIR_NAME "v10" )
    set ( SOCK_LIB Ws2_32.lib )
    set ( TIME_LIB "" )
  else ()
    message ("Unsupported Windows configuration")
  endif ()
endif (APPLE)

set ( MODELS_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../resources" )
set ( MODELS_DIR "${MODELS_BASE_DIR}/models" )
set ( MODELS_PACKAGE_FILENAME models.7z)

set ( OSG_DIR "${OSG_BASE_DIR}/${EXT_DIR_NAME}" )
set ( OSI_DIR "${OSI_BASE_DIR}/${EXT_DIR_NAME}" )
set ( SUMO_DIR "${SUMO_BASE_DIR}/${EXT_DIR_NAME}" )
set ( GTEST_DIR "${GTEST_BASE_DIR}/${EXT_DIR_NAME}" )

set ( OSG_INCLUDE_DIR
    "${OSG_DIR}/build/include"
    "${OSG_DIR}/include"
)
set ( OSG_LIBRARIES_PATH
    "${OSG_DIR}/lib"
    "${OSG_DIR}/lib/osgPlugins-3.6.5"
)

set ( OSI_INCLUDE_DIR "${OSI_DIR}/include" )
if (DYN_PROTOBUF)
  set ( OSI_LIBRARIES_PATH "${OSI_DIR}/lib-dyn" )
else ()
  set ( OSI_LIBRARIES_PATH "${OSI_DIR}/lib" )
endif (DYN_PROTOBUF)

set ( SUMO_INCLUDE_DIR "${SUMO_DIR}/include" )
set ( SUMO_LIBRARIES_PATH "${SUMO_DIR}/lib" )

set ( GTEST_INCLUDE_DIR "${GTEST_DIR}/include" )
set ( GTEST_LIBRARIES_PATH "${GTEST_DIR}/lib" )

link_directories(${OSG_LIBRARIES_PATH} ${OSI_LIBRARIES_PATH} ${SUMO_LIBRARIES_PATH} ${GTEST_LIBRARIES_PATH} )

if(APPLE)
  SET(CMAKE_CXX_FLAGS "-std=c++14 -pthread -flto -Wl,-dead_strip")

  set ( OSG_LIBRARIES
    osg
    osgViewer
    osgDB
    osgdb_serializers_osg
    osgGA
    osgText
    osgdb_osg
    osgdb_jpeg
    osgUtil
    osgAnimation
    osg
    OpenThreads
    jpeg

    "-framework OpenGL"
    "-framework Cocoa"
    dl
    z
  )

  if (DYN_PROTOBUF)
    set ( OSI_LIBRARIES
      open_simulation_interface
      protobuf
    )
  else ()
    set ( OSI_LIBRARIES
      open_simulation_interface_pic
      protobuf
    )
  endif (DYN_PROTOBUF)


  set ( SUMO_LIBRARIES
    optimized libsumostatic debug libsumostaticd
    optimized netload debug netloadd
    optimized traciserver debug traciserverd
    optimized libsumostatic debug libsumostaticd
    optimized utils_vehicle debug utils_vehicled
    optimized utils_distribution debug utils_distributiond
    optimized utils_shapes debug utils_shapesd
    optimized utils_options debug utils_optionsd
    optimized utils_xml debug utils_xmld
    optimized utils_geom debug utils_geomd
    optimized utils_common debug utils_commond
    optimized utils_iodevices debug utils_iodevicesd
    optimized utils_traction_wire debug utils_traction_wired
    optimized utils_emissions debug utils_emissionsd
    optimized microsim_engine debug microsim_engined
    optimized microsim_lcmodels debug microsim_lcmodelsd
    optimized microsim_devices debug microsim_devicesd
    optimized microsim_trigger debug microsim_triggerd
    optimized microsim_output debug microsim_outputd
    optimized microsim_transportables debug microsim_transportablesd
    optimized microsim_actions debug microsim_actionsd
    optimized microsim_traffic_lights debug microsim_traffic_lightsd
    optimized microsim debug microsimd
    optimized mesosim debug mesosimd
    optimized foreign_phemlight debug foreign_phemlightd
    optimized foreign_tcpip debug foreign_tcpipd
    optimized microsim_cfmodels debug microsim_cfmodelsd
    optimized zlibstatic debug zlibstaticd
    optimized xerces-c_3 debug xerces-c_3D
    "-framework CoreServices"
  )

elseif(LINUX)

  include(CheckCXXCompilerFlag)

  # Check for standard to use
  check_cxx_compiler_flag(-std=c++14 HAVE_FLAG_STD_CXX14)
  if(HAVE_FLAG_STD_CXX14)
      SET(CXX_STD_FLAG "-std=c++14")
  else()
     check_cxx_compiler_flag(-std=c++1y HAVE_FLAG_STD_CXX1Y)
     if(HAVE_FLAG_STD_CXX1Y)
       SET(CXX_STD_FLAG "-std=c++1y")
     else()
       message ("Need compiler support for c++14 \(or 1y as the beta was called\)")
     endif()
  endif()

  set (CMAKE_C_FLAGS "-std=c11 ${CMAKE_C_FLAGS}")

  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    SET(CMAKE_CXX_FLAGS "${CXX_STD_FLAG} -pthread -fPIC -Wl,-strip-all")
  elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    SET(CMAKE_CXX_FLAGS "${CXX_STD_FLAG} -march=native -O0 -pthread -fPIC")
  else()
    SET(CMAKE_CXX_FLAGS "${CXX_STD_FLAG} -pthread -fPIC -Wl,-strip-all")
  endif()

  set ( OSG_LIBRARIES
    optimized osg debug osgd
    optimized osgViewer debug osgViewerd
    optimized osgDB debug osgDBd
    optimized osgdb_serializers_osg debug osgdb_serializers_osgd
    optimized osgGA debug osgGAd
    optimized osgText debug osgTextd
    optimized osgdb_osg debug osgdb_osgd
    optimized osgdb_jpeg debug osgdb_jpegd
    optimized osgUtil debug osgUtild
    optimized osgAnimation debug osgAnimationd
    optimized osg debug osgd
    optimized OpenThreads debug OpenThreadsd
    optimized jpeg debug jpegd

    GL
    X11
    Xrandr
    dl
    z
    Xinerama
    fontconfig
  )

  if (DYN_PROTOBUF)
    set ( OSI_LIBRARIES
      optimized open_simulation_interface debug open_simulation_interfaced
      optimized protobuf debug protobufd
    )
  else ()
    set ( OSI_LIBRARIES
      optimized open_simulation_interface_pic debug open_simulation_interface_picd
      optimized protobuf debug protobufd
    )
  endif (DYN_PROTOBUF)

  set ( SUMO_LIBRARIES
    optimized libsumostatic debug libsumostaticd
    optimized netload debug netloadd
    optimized traciserver debug traciserverd
    optimized libsumostatic debug libsumostaticd
    optimized utils_vehicle debug utils_vehicled
    optimized utils_distribution debug utils_distributiond
    optimized utils_shapes debug utils_shapesd
    optimized utils_options debug utils_optionsd
    optimized utils_xml debug utils_xmld
    optimized utils_geom debug utils_geomd
    optimized utils_common debug utils_commond
    optimized utils_iodevices debug utils_iodevicesd
    optimized utils_traction_wire debug utils_traction_wired
    optimized utils_emissions debug utils_emissionsd
    optimized microsim_engine debug microsim_engined
    optimized microsim_lcmodels debug microsim_lcmodelsd
    optimized microsim_devices debug microsim_devicesd
    optimized microsim_trigger debug microsim_triggerd
    optimized microsim_output debug microsim_outputd
    optimized microsim_transportables debug microsim_transportablesd
    optimized microsim_actions debug microsim_actionsd
    optimized microsim_traffic_lights debug microsim_traffic_lightsd
    optimized microsim debug microsimd
    optimized mesosim debug mesosimd
    optimized foreign_phemlight debug foreign_phemlightd
    optimized foreign_tcpip debug foreign_tcpipd
    optimized microsim_cfmodels debug microsim_cfmodelsd
    optimized zlibstatic debug zlibstaticd
    optimized xerces-c_3 debug xerces-c_3D
  )

  set (GTEST_LIBRARIES
    optimized gmock debug gmockd
    optimized gmock_main debug gmock_maind
    optimized gtest debug gtestd
    optimized gtest_main debug gtest_maind
  )

elseif(MSVC)

  # Get rid of historical macros preventing SUMO integration
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D NOMINMAX ")

  if ((MSVC) AND (MSVC_VERSION GREATER_EQUAL 1910))
    # suppress warning 4127 (conditional expression is constant) to avoid harmless warning from Google protobuf header
    # suppress warning 4100 (unreferenced formal parameter) to allow function headers with unreferenced parameters for increased readability
    add_compile_options(/permissive- /W4 /wd4127 /wd4100)
  endif ()

  set ( OSG_LIBRARIES
      opengl32.lib
      optimized zlibstatic.lib debug zlibstaticd.lib
      optimized jpeg.lib debug jpegd.lib
      optimized ${OSG_VERSION}-osg.lib debug ${OSG_VERSION}-osgd.lib
      optimized ${OSG_VERSION}-osgAnimation.lib debug ${OSG_VERSION}-osgAnimationd.lib
      optimized ${OSG_VERSION}-osgDB.lib debug ${OSG_VERSION}-osgDBd.lib
      optimized ${OSG_VERSION}-osgGA.lib debug ${OSG_VERSION}-osgGAd.lib
      optimized ${OSG_VERSION}-osgViewer.lib debug ${OSG_VERSION}-osgViewerd.lib
      optimized ${OSG_VERSION}-osgText.lib debug ${OSG_VERSION}-osgTextd.lib
      optimized ${OSG_VERSION}-osgUtil.lib debug ${OSG_VERSION}-osgUtild.lib
      optimized ot21-OpenThreads.lib debug ot21-OpenThreadsd.lib
      optimized osgdb_serializers_osg.lib debug osgdb_serializers_osgd.lib
      optimized osgdb_osg.lib debug osgdb_osgd.lib
      optimized osgdb_jpeg.lib debug osgdb_jpegd.lib
  )

  set ( OSI_LIBRARIES
    optimized libprotobuf debug libprotobufd
    optimized open_simulation_interface_pic debug open_simulation_interface_picd
  )

  set ( SUMO_LIBRARIES
    optimized foreign_phemlight.lib debug foreign_phemlightd.lib
    optimized foreign_tcpip.lib debug foreign_tcpipd.lib
    optimized libsumostatic.lib debug libsumostaticd.lib
    optimized mesosim.lib debug mesosimd.lib
    optimized microsim.lib debug microsimd.lib
    optimized microsim_actions.lib debug microsim_actionsd.lib
    optimized microsim_cfmodels.lib debug microsim_cfmodelsd.lib
    optimized microsim_devices.lib debug microsim_devicesd.lib
    optimized microsim_engine.lib debug microsim_engined.lib
    optimized microsim_lcmodels.lib debug microsim_lcmodelsd.lib
    optimized microsim_output.lib debug microsim_outputd.lib
    optimized microsim_traffic_lights.lib debug microsim_traffic_lightsd.lib
    optimized microsim_transportables.lib debug microsim_transportablesd.lib
    optimized microsim_trigger.lib debug microsim_triggerd.lib
    optimized netload.lib debug netloadd.lib
    optimized traciserver.lib debug traciserverd.lib
    optimized utils_common.lib debug utils_commond.lib
    optimized utils_distribution.lib debug utils_distributiond.lib
    optimized utils_emissions.lib debug utils_emissionsd.lib
    optimized utils_geom.lib debug utils_geomd.lib
    optimized utils_iodevices.lib debug utils_iodevicesd.lib
    optimized utils_options.lib debug utils_optionsd.lib
    optimized utils_shapes.lib debug utils_shapesd.lib
    optimized utils_traction_wire.lib debug utils_traction_wired.lib
    optimized utils_vehicle.lib debug utils_vehicled.lib
    optimized utils_xml.lib debug utils_xmld.lib
    optimized xerces-c_3.lib debug xerces-c_3D.lib
    optimized zlibstatic.lib debug zlibstaticd.lib
  )

  set (GTEST_LIBRARIES
    optimized gmock.lib debug gmockd.lib
    optimized gmock_main.lib debug gmock_maind.lib
    optimized gtest.lib debug gtestd.lib
    optimized gtest_main.lib debug gtest_maind.lib
  )

elseif(MINGW)

  include(CheckCXXCompilerFlag)

  # Check for standard to use
  check_cxx_compiler_flag(-std=c++14 HAVE_FLAG_STD_CXX14)
  if(HAVE_FLAG_STD_CXX14)
      SET(CXX_STD_FLAG "-std=c++14")
  else()
     check_cxx_compiler_flag(-std=c++1y HAVE_FLAG_STD_CXX1Y)
     if(HAVE_FLAG_STD_CXX1Y)
       SET(CXX_STD_FLAG "-std=c++1y")
     else()
       message ("Need compiler support for c++14 \(or 1y as the beta was called\)")
     endif()
  endif()

    set (CMAKE_C_FLAGS "-std=c11 ${CMAKE_C_FLAGS}")

  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    SET(CMAKE_CXX_FLAGS "${CXX_STD_FLAG} -fPIC -Wl,-strip-all")
  elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    SET(CMAKE_CXX_FLAGS "${CXX_STD_FLAG} -march=native -O0 -fPIC")
  else()
    SET(CMAKE_CXX_FLAGS "${CXX_STD_FLAG} -fPIC -Wl,-strip-all")
  endif()

  set ( OSG_LIBRARIES
      opengl32.lib
      optimized zlibstatic.lib debug zlibstaticd.lib
      optimized jpeg.lib debug jpegd.lib
      optimized ${OSG_VERSION}-osg.lib debug ${OSG_VERSION}-osgd.lib
      optimized ${OSG_VERSION}-osgAnimation.lib debug ${OSG_VERSION}-osgAnimationd.lib
      optimized ${OSG_VERSION}-osgDB.lib debug ${OSG_VERSION}-osgDBd.lib
      optimized ${OSG_VERSION}-osgGA.lib debug ${OSG_VERSION}-osgGAd.lib
      optimized ${OSG_VERSION}-osgViewer.lib debug ${OSG_VERSION}-osgViewerd.lib
      optimized ${OSG_VERSION}-osgText.lib debug ${OSG_VERSION}-osgTextd.lib
      optimized ${OSG_VERSION}-osgUtil.lib debug ${OSG_VERSION}-osgUtild.lib
      optimized ${OSG_VERSION}-osg.lib debug ${OSG_VERSION}-osgd.lib
      optimized ot21-OpenThreads.lib debug ot21-OpenThreadsd.lib
      optimized osgdb_serializers_osg.lib debug osgdb_serializers_osgd.lib
      optimized osgdb_osg.lib debug osgdb_osgd.lib
      optimized osgdb_jpeg.lib debug osgdb_jpegd.lib
  )

endif()

add_subdirectory(Modules/CommonMini)
add_subdirectory(Modules/Controllers)
add_subdirectory(Modules/PlayerBase)
add_subdirectory(Modules/RoadManager)
add_subdirectory(Modules/ScenarioEngine)
add_subdirectory(Libraries/esminiLib)
add_subdirectory(Libraries/esminiRMLib)
add_subdirectory(Applications/esmini)
add_subdirectory(Applications/esmini-dyn)

# Add unittest folder
if (APPLE OR MINGW)
  message ("Googletest not yet supported for macOS and MinGW")
elseif (USE_GTEST)
  add_subdirectory(Unittest)
endif()

set ( ModulesFolder Modules )
set ( ApplicationsFolder Applications )
set ( LibrariesFolder Libraries )
set ( CodeExamplesFolder code-examples )

set_target_properties (RoadManager PROPERTIES FOLDER ${ModulesFolder} )
set_target_properties (CommonMini PROPERTIES FOLDER ${ModulesFolder} )
set_target_properties (ScenarioEngine PROPERTIES FOLDER ${ModulesFolder} )
set_target_properties (RoadManager PROPERTIES FOLDER ${ModulesFolder} )
set_target_properties (ScenarioEngine PROPERTIES FOLDER ${ModulesFolder} )


# Add all projects under code-examples folder using a macro
MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()

  if (NOT USE_OSI)
    LIST(REMOVE_ITEM dirlist "osi-groundtruth")
    LIST(REMOVE_ITEM dirlist "osi-groundtruth-dyn")
  endif ()

  SET(${result} ${dirlist})
ENDMACRO()

SUBDIRLIST(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR}/code-examples)

FOREACH(subdir ${SUBDIRS})
    add_subdirectory(code-examples/${subdir})
    if (NOT ${subdir} STREQUAL "hello_world")
        set_target_properties (${subdir} PROPERTIES FOLDER ${CodeExamplesFolder} )
    endif ()
ENDFOREACH()


#
# Download library and content binary packets
#

function (download_and_extract url target_folder target_filename)
    message (STATUS "downloading ${target_filename} ...")
    file (DOWNLOAD ${url} ${target_folder}/${target_filename} STATUS DOWNLOAD_STATUS)

    if(DOWNLOAD_STATUS AND NOT DOWNLOAD_STATUS EQUAL 0)
        message(FATAL_ERROR "FAILED to download ${target_filename} (Status: ${DOWNLOAD_STATUS})")
    endif()

    execute_process (COMMAND sleep 1)  # allow for file to be completely flushed

    message (STATUS "extracting ${target_filename} ... ")
    execute_process (COMMAND ${CMAKE_COMMAND} -E tar xfz ${target_filename} WORKING_DIRECTORY ${target_folder} RESULT_VARIABLE STATUS)

    if(STATUS AND NOT STATUS EQUAL 0)
        message(FATAL_ERROR "FAILED to unpack ${target_filename}")
    endif()

    file (REMOVE ${target_folder}/${target_filename})
endfunction (download_and_extract)

# download OpenSceneGraph
set ( OSG_PACKAGE_FILENAME "osg.7z" )
if (DEFINED OSG_DIR AND (FORCE_DOWNLOAD_BINARIES OR NOT EXISTS ${OSG_DIR} ))
    download_and_extract( ${OSG_PACKAGE_URL} ${OSG_BASE_DIR} ${OSG_PACKAGE_FILENAME} )
endif()

# download OSI
set ( OSI_PACKAGE_FILENAME "osi.7z" )
if (DEFINED OSI_DIR AND (FORCE_DOWNLOAD_BINARIES OR NOT EXISTS ${OSI_DIR} ))
    download_and_extract( ${OSI_PACKAGE_URL} ${OSI_BASE_DIR} ${OSI_PACKAGE_FILENAME} )
endif()

# download SUMO
set ( SUMO_PACKAGE_FILENAME "sumo.7z" )
if (DEFINED SUMO_DIR AND (FORCE_DOWNLOAD_BINARIES OR NOT EXISTS ${SUMO_DIR} ))
  download_and_extract( ${SUMO_PACKAGE_URL} ${SUMO_BASE_DIR} ${SUMO_PACKAGE_FILENAME} )
endif()

# download googletest
if(NOT (APPLE OR MINGW))  # not available for Mac yet
  set ( GTEST_PACKAGE_FILENAME "googletest.7z" )
  if (DEFINED GTEST_DIR AND (FORCE_DOWNLOAD_BINARIES OR NOT EXISTS ${GTEST_DIR} ))
      download_and_extract( ${GTEST_PACKAGE_URL} ${GTEST_BASE_DIR} ${GTEST_PACKAGE_FILENAME} )
  endif()
endif()

if (DEFINED MODELS_DIR AND (FORCE_DOWNLOAD_BINARIES OR NOT EXISTS ${MODELS_DIR} ))
    download_and_extract(${MODELS_PACKAGE_URL} ${MODELS_BASE_DIR} ${MODELS_PACKAGE_FILENAME})
endif()

add_subdirectory(Applications/odrplot)
add_subdirectory(Applications/replayer)

if (USE_OSG)
  add_subdirectory(Applications/odrviewer)
  add_subdirectory(Modules/ViewerBase)
endif (USE_OSG)

if (USE_OSI)
  set_target_properties (osireceiver PROPERTIES FOLDER ${ApplicationsFolder} )
endif ()
  set_target_properties (PlayerBase PROPERTIES FOLDER ${ModulesFolder} )
  set_target_properties (Controllers PROPERTIES FOLDER ${ModulesFolder} )
  set_target_properties (esminiLib PROPERTIES FOLDER ${LibrariesFolder} )
  set_target_properties (esminiRMLib PROPERTIES FOLDER ${LibrariesFolder} )
  set_target_properties (esmini PROPERTIES FOLDER ${ApplicationsFolder} )
  set_target_properties (esmini-dyn PROPERTIES FOLDER ${ApplicationsFolder} )
  set_target_properties (dat2csv PROPERTIES FOLDER ${ApplicationsFolder} )
  set_target_properties (odrplot PROPERTIES FOLDER ${ApplicationsFolder} )
if (USE_OSG)
  set_target_properties (replayer PROPERTIES FOLDER ${ApplicationsFolder} )
  set_target_properties (odrviewer PROPERTIES FOLDER ${ApplicationsFolder} )
  set_target_properties (ViewerBase PROPERTIES FOLDER ${ModulesFolder} )
endif (USE_OSG)



# Add variables to global scope, e.g. when esmini is used as submodule
set(SCENARIOENGINE_INCLUDE_DIRS ${SCENARIOENGINE_INCLUDE_DIRS} CACHE INTERNAL "")
set(ROADMANAGER_INCLUDE_DIR ${ROADMANAGER_INCLUDE_DIR} CACHE INTERNAL "")
set(VIEWER_BASE_INCLUDE_DIR ${VIEWER_BASE_INCLUDE_DIR} CACHE INTERNAL "")
set(PLAYER_BASE_INCLUDE_DIR ${PLAYER_BASE_INCLUDE_DIR} CACHE INTERNAL "")
set(CONTROLLERS_INCLUDE_DIR ${CONTROLLERS_INCLUDE_DIR} CACHE INTERNAL "")
set(OSG_INCLUDE_DIR ${OSG_INCLUDE_DIR} CACHE INTERNAL "")
set(SUMO_INCLUDE_DIR ${SUMO_INCLUDE_DIR} CACHE INTERNAL "")
set(COMMON_MINI_INCLUDE_DIR ${COMMON_MINI_INCLUDE_DIR} CACHE INTERNAL "")
set(OSI_INCLUDE_DIR ${OSI_INCLUDE_DIR} CACHE INTERNAL "")


set(TIME_LIB ${TIME_LIB} CACHE INTERNAL "")
set(OSG_DIR ${OSG_DIR} CACHE INTERNAL "")
set(OSI_PACKAGE_URL ${OSI_PACKAGE_URL} CACHE INTERNAL "")
set(OSI_DIR ${OSI_DIR} CACHE INTERNAL "")
set(SUMO_PACKAGE_URL ${SUMO_PACKAGE_URL} CACHE INTERNAL "")
set(SUMO_DIR ${SUMO_DIR} CACHE INTERNAL "")
set(GTEST_PACKAGE_URL ${GTEST_PACKAGE_URL} CACHE INTERNAL "")
set(GTEST_DIR ${GTEST_DIR} CACHE INTERNAL "")
set(SOCK_LIB ${SOCK_LIB} CACHE INTERNAL "")
set(sumo_libs ${sumo_libs} CACHE INTERNAL "")
set(esminiRMLib ${esminiRMLib} CACHE INTERNAL "")
set(esminiLib ${esminiLib} CACHE INTERNAL "")
set(osi_libs ${osi_libs} CACHE INTERNAL "")
set(viewer_libs ${viewer_libs} CACHE INTERNAL "")


set(OSG_LIBRARIES_PATH ${OSG_LIBRARIES_PATH} CACHE INTERNAL "")
set(OSI_LIBRARIES_PATH ${OSI_LIBRARIES_PATH} CACHE INTERNAL "")
set(SUMO_LIBRARIES_PATH ${SUMO_LIBRARIES_PATH} CACHE INTERNAL "")
set(GTEST_LIBRARIES_PATH ${GTEST_LIBRARIES_PATH} CACHE INTERNAL "")

set(OSG_LIBRARIES ${OSG_LIBRARIES} CACHE INTERNAL "")
set(OSI_LIBRARIES ${OSI_LIBRARIES} CACHE INTERNAL "")
set(SUMO_LIBRARIES ${SUMO_LIBRARIES} CACHE INTERNAL "")
set(GTEST_LIBRARIES ${GTEST_LIBRARIES} CACHE INTERNAL "")

set(SCENARIOENGINE_DLL_INCLUDE_DIR ${SCENARIOENGINE_DLL_INCLUDE_DIR} CACHE INTERNAL "")
set(ROADMANAGER_DLL_INCLUDE_DIR ${ROADMANAGER_DLL_INCLUDE_DIR} CACHE INTERNAL "")
