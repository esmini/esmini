
set ( VIEWER_BASE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Modules/ViewerBase" )
set ( PLAYER_BASE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Modules/PlayerBase" )
set ( ROADMANAGER_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Modules/RoadManager" )
set ( SCENARIOENGINE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Modules/ScenarioEngine/SourceFiles")
set ( ROADMANAGER_DLL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Libraries/esminiRMLib" )
set ( SCENARIOENGINE_DLL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Libraries/esminiLib" )
set ( REPLAY_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Applications/replayer" )
set ( SCENARIOENGINE_INCLUDE_DIRS
 "${SCENARIOENGINE_INCLUDE_DIR}"
 "${CMAKE_CURRENT_SOURCE_DIR}/Modules/ScenarioEngine/Actions"
 "${CMAKE_CURRENT_SOURCE_DIR}/Modules/ScenarioEngine/Conditions"
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

if (APPLE)
  set ( TIME_LIB "" )
  set ( OSG_PACKAGE_URL https://dl.dropboxusercontent.com/s/d0czj6b89p9jyvv/OpenSceneGraph_mac.7z?dl=0 )
  set ( OSG_DIR "${OSG_BASE_DIR}/mac" )
  set ( OSI_PACKAGE_URL https://dl.dropboxusercontent.com/s/m62v19gp0m73dte/osi_mac.7z?dl=0 )
  set ( OSI_DIR "${OSI_BASE_DIR}/mac" )
  set ( SUMO_PACKAGE_URL https://dl.dropboxusercontent.com/s/0x8kwztk7nmacs1/sumo_mac.7z?dl=0 )
  set ( SUMO_DIR "${SUMO_BASE_DIR}/mac" )
elseif (UNIX)
  set ( TIME_LIB "" )
  set ( OSG_PACKAGE_URL https://dl.dropboxusercontent.com/s/3dlev34kj94lir5/OpenSceneGraph_linux.7z?dl=0 )
  set ( OSG_DIR "${OSG_BASE_DIR}/linux" )
  set ( OSI_PACKAGE_URL https://dl.dropboxusercontent.com/s/kwtdg0c1c8pawa1/osi_linux.7z?dl=0 )
  set ( OSI_DIR "${OSI_BASE_DIR}/linux" )
  set ( SUMO_PACKAGE_URL https://dl.dropboxusercontent.com/s/gfwtqd3gf76f86a/sumo_linux.7z?dl=0 )
  set ( SUMO_DIR "${SUMO_BASE_DIR}/linux" )
  set ( GTEST_PACKAGE_URL https://dl.dropboxusercontent.com/s/si7jsjjsy5bpoym/googletest_linux.7z?dl=0 )
  set ( GTEST_DIR "${GTEST_BASE_DIR}/linux" )
elseif (WIN32)
  if (WINSDK7_1)
    message ("Configuring for Windows SDK v7.1 x64")
    set ( SOCK_LIB Ws2_32.lib )
    set ( TIME_LIB winmm.lib )
    set ( OSG_PACKAGE_URL https://drive.google.com/uc?export=download&id=1NBEvGZiTWmqxk-MEOq7uK1uh_vaUMaEL )
    set ( OSG_DIR "${OSG_BASE_DIR}/winsdk71" )
  elseif ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "Win32")
    message ("Configuring for Windows SDK v10 win32")
    set ( SOCK_LIB Ws2_32.lib )
    set ( TIME_LIB "" )
    set ( OSG_PACKAGE_URL https://drive.google.com/uc?export=download&id=10dV9P0qOeJUgTtsSDld4AlbClE--SivX )
    set ( OSG_DIR "${OSG_BASE_DIR}/win32" )
  elseif ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "x64")
    message ("Configuring for Windows SDK v10 x64")
    set ( SOCK_LIB Ws2_32.lib )
    set ( TIME_LIB "" )
    set ( OSG_PACKAGE_URL https://dl.dropboxusercontent.com/s/e95hnoo782p40uc/OpenSceneGraph_v10.7z?dl=0 )
    set ( OSI_PACKAGE_URL https://dl.dropboxusercontent.com/s/an58ckp2qfx5069/osi_v10.7z?dl=0 )
    set ( SUMO_PACKAGE_URL https://dl.dropboxusercontent.com/s/5jtpnnd61wonxuh/sumo_v10.7z?dl=0 )
    set ( GTEST_PACKAGE_URL https://dl.dropboxusercontent.com/s/aaiehwzc6woqbc6/googletest_v10.7z?dl=0 )
    set ( OSG_DIR "${OSG_BASE_DIR}/v10" )
    set ( OSI_DIR "${OSI_BASE_DIR}/v10" )
    set ( SUMO_DIR "${SUMO_BASE_DIR}/v10" )
    set ( GTEST_DIR "${GTEST_BASE_DIR}/v10" )
  else ()
    message ("Unknown configuration")
  endif ()
endif (APPLE)

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
    osgdb_serializers_osgsim
    osgdb_serializers_osg
    osgdb_osg
    osgdb_jpeg
    osgAnimation
    osgText
    osgSim
    osgViewer
    osgUtil
    osgDB
    osgGA
    osg
    OpenThreads
    jpeg

    "-framework OpenGL"
    "-framework Cocoa"
    dl
    z
    #db_fbx
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

elseif(UNIX)

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
    optimized osgdb_serializers_osgsim debug osgdb_serializers_osgsimd
    optimized osgdb_serializers_osg debug osgdb_serializers_osgd
    optimized osgdb_osg debug osgdb_osgd
    optimized osgdb_jpeg debug osgdb_jpegd
    optimized osgAnimation debug osgAnimationd
    optimized osgText debug osgTextd
    optimized osgSim debug osgSimd
    optimized osgViewer debug osgViewerd
    optimized osgUtil debug osgUtild
    optimized osgDB debug osgDBd
    optimized osgGA debug osgGAd
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
    #db_fbx
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

else() #not UNIX

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
      optimized ${OSG_VERSION}-osgSim.lib debug ${OSG_VERSION}-osgSimd.lib
      optimized ot21-OpenThreads.lib debug ot21-OpenThreadsd.lib
      optimized osgdb_serializers_osgsim.lib debug osgdb_serializers_osgsimd.lib
      optimized osgdb_serializers_osg.lib debug osgdb_serializers_osgd.lib
      optimized osgdb_osg.lib debug osgdb_osgd.lib
      optimized osgdb_fbx.lib debug osgdb_fbxd.lib
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

endif() #Endif UNIX

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
if (APPLE)
  message ("Not available yet")
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
    set_target_properties (${subdir} PROPERTIES FOLDER ${CodeExamplesFolder} )
ENDFOREACH()


#
# Download library and content binary packets
#
function (download_and_extract url target_folder target_filename)
    message (STATUS "downloading ${target_filename} ...")
    file (DOWNLOAD ${url} ${target_folder}/${target_filename})

    message (STATUS "extracting ${target_filename} ... ")
    execute_process (COMMAND ${CMAKE_COMMAND} -E tar xfz ${target_filename} WORKING_DIRECTORY ${target_folder})
    file (REMOVE ${target_folder}/${target_filename})
endfunction (download_and_extract)

# download OpenSceneGraph
set ( OSG_PACKAGE_FILENAME "osg.7z" )
if (FORCE_DOWNLOAD_BINARIES OR NOT EXISTS ${OSG_DIR} )
    download_and_extract( ${OSG_PACKAGE_URL} ${OSG_BASE_DIR} ${OSG_PACKAGE_FILENAME} )
endif()

# download OSI
set ( OSI_PACKAGE_FILENAME "osi.7z" )
if (FORCE_DOWNLOAD_BINARIES OR NOT EXISTS ${OSI_DIR} )
    download_and_extract( ${OSI_PACKAGE_URL} ${OSI_BASE_DIR} ${OSI_PACKAGE_FILENAME} )
endif()

# download SUMO
set ( SUMO_PACKAGE_FILENAME "sumo.7z" )
if (FORCE_DOWNLOAD_BINARIES OR NOT EXISTS ${SUMO_DIR} )
  download_and_extract( ${SUMO_PACKAGE_URL} ${SUMO_BASE_DIR} ${SUMO_PACKAGE_FILENAME} )
endif()

# download googletest
if(APPLE)
  message ("Not available yet")

else()
  set ( GTEST_PACKAGE_FILENAME "googletest.7z" )
  if (FORCE_DOWNLOAD_BINARIES OR NOT EXISTS ${GTEST_DIR} )
      download_and_extract( ${GTEST_PACKAGE_URL} ${GTEST_BASE_DIR} ${GTEST_PACKAGE_FILENAME} )
  endif()
endif()

set ( MODELS_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../resources" )
set ( MODELS_DIR "${MODELS_BASE_DIR}/models" )
set ( MODELS_PACKAGE_FILENAME models.7z)
set ( MODELS_PACKAGE_URL https://dl.dropboxusercontent.com/s/5gk8bvgzqiaaoco/models.7z?dl=0 )

if (FORCE_DOWNLOAD_BINARIES OR NOT EXISTS ${MODELS_DIR} )
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


