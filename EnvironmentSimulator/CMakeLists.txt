
set ( VIEWER_BASE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ViewerBase" )
set ( PLAYER_BASE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/PlayerBase" )
set ( WORLDMANAGER_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/WorldManager" )
set ( REPLAY_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Replayer" )
set ( ROADMANAGER_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/RoadManager" )
set ( ROADMANAGER_DLL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/RoadManagerDLL" )
set ( SCENARIOENGINE_DLL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ScenarioEngineDLL" )
set ( SCENARIOENGINE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ScenarioEngine/SourceFiles")
set ( SCENARIOENGINE_INCLUDE_DIRS
 "${SCENARIOENGINE_INCLUDE_DIR}"
 "${CMAKE_CURRENT_SOURCE_DIR}/ScenarioEngine/Actions"
 "${CMAKE_CURRENT_SOURCE_DIR}/ScenarioEngine/Conditions"
 "${CMAKE_CURRENT_SOURCE_DIR}/ScenarioEngine/OSCTypeDefs"
 )
set ( COMMON_MINI_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CommonMini")

# OpenSceneGraph package adapted for this project
set ( OSG_VERSION "osg161" )
set ( OSG_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../externals/OpenSceneGraph" )
#set ( PB_VERSION "3.5.0" )
#set ( OSI_VERSION "1?" )
set ( OSI_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../externals/OSI" )
set ( SUMO_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../externals/SUMO" )

# GoogleTest package
set ( GTEST_VERSION "1.10.1" )
set ( GTEST_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../externals/googletest" )

set( CMAKE_VERBOSE_MAKEFILE true )

if (APPLE)
  set ( TIME_LIB "" ) 
  set ( OSG_PACKAGE_URL https://drive.google.com/uc?export=download&id=1zy9neztZP7mY88zbm-oVP24Vzr-LTUeY )
  set ( OSG_DIR "${OSG_BASE_DIR}/mac" )
  set ( OSI_PACKAGE_URL https://drive.google.com/uc?export=download&id=1xcwteK9t4O--9cnnY3kuEdYuXz8_S2nQ )
  set ( OSI_DIR "${OSI_BASE_DIR}/mac" )
  set ( SUMO_PACKAGE_URL https://drive.google.com/uc?export=download&id=1VSXqtNwP5xLuyqu76zyxyL5NGv1IcBRA )
  set ( SUMO_DIR "${SUMO_BASE_DIR}/mac" )
elseif (UNIX)
  set ( TIME_LIB "" )
  set ( OSG_PACKAGE_URL https://drive.google.com/uc?export=download&id=1OufA3TUQjBTkaRvAuo0rSUwryvoqde8G )
  set ( OSG_DIR "${OSG_BASE_DIR}/linux" )
  set ( OSI_PACKAGE_URL https://drive.google.com/uc?export=download&id=1ZIQ4Hq6VUrYO-cs4hUe7iTB6_CtYVPUH )
  set ( OSI_DIR "${OSI_BASE_DIR}/linux" )
  set ( SUMO_PACKAGE_URL https://drive.google.com/uc?export=download&id=1ThXnLQjULcMr_wxVprns5JMgXzm1NGcu )
  set ( SUMO_DIR "${SUMO_BASE_DIR}/linux" )
  set ( GTEST_PACKAGE_URL https://drive.google.com/uc?export=download&id=19bFhtkoCD6Nk84EgxFrgVTa9Z4Gt7_3g )
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
    set ( OSG_PACKAGE_URL https://drive.google.com/uc?export=download&id=1YxLVdQLhKBMGW4HB_ArJglRIpzuDiwhJ )
    set ( OSI_PACKAGE_URL https://drive.google.com/uc?export=download&id=168Qf08TktrVc-WNFm4xZ1SNUG3v7ENeG )
    set ( SUMO_PACKAGE_URL https://drive.google.com/uc?export=download&id=1C7FzC7fwjdm8jWP43gaDijj4gRCfI_4L )
    set ( GTEST_PACKAGE_URL https://drive.google.com/uc?export=download&id=1ZKKBWhnIZMDJ-Dt8isuFnvWavflutkQJ )
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
set ( OSI_LIBRARIES_PATH "${OSI_DIR}/lib" )

set ( SUMO_INCLUDE_DIR "${SUMO_DIR}/include" )
set ( SUMO_LIBRARIES_PATH "${SUMO_DIR}/lib" )

link_directories(${OSG_LIBRARIES_PATH} ${OSI_LIBRARIES_PATH} ${SUMO_LIBRARIES_PATH})

set ( GTEST_INCLUDE_DIR "${GTEST_DIR}/include" )
set ( GTEST_LIBRARIES_PATH "${GTEST_DIR}/lib" )

link_directories(${OSG_LIBRARIES_PATH} ${OSI_LIBRARIES_PATH} ${GTEST_LIBRARIES_PATH})

if(APPLE)
  SET(CMAKE_CXX_FLAGS "-std=c++14 -pthread -flto -Wl,-dead_strip")

  SET( "-framework OpenGL -framework Cocoa")
  
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

  set ( OSI_LIBRARIES 
	open_simulation_interface_pic
	protobuf
  )

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

elseif(UNIX)
  SET(CMAKE_CXX_FLAGS "-std=c++14 -pthread -fPIC -Wl,-strip-all")
  
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
  
  set ( OSI_LIBRARIES 
	optimized open_simulation_interface_pic debug open_simulation_interface_picd
	optimized protobuf debug protobufd
  )
  
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
	optimized libprotobuf.lib debug libprotobufd.lib
	optimized open_simulation_interface_pic.lib debug open_simulation_interface_picd.lib
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
#
#if (NOT WINSDK7_1)  # FBX not supporting winsdk 7.1
#  list ( APPEND OSG_LIBRARIES optimized libfbxsdk-md.lib debug libfbxsdk-mdd.lib )
#endif()

endif() #Endif UNIX

add_subdirectory(RoadManager)
add_subdirectory(CommonMini)
add_subdirectory(ScenarioEngine)
add_subdirectory(RoadManagerDLL)
add_subdirectory(ScenarioEngineDLL)
add_subdirectory(PlayerBase)
add_subdirectory(ScenarioViewer)
add_subdirectory(EnvironmentSimulator)
add_subdirectory(EgoSimulator)

# Add unittest folder
if(APPLE)
  message ("Not available yet")
else()
  add_subdirectory(Unittest)
endif()
    
set ( ModulesFolder Modules )
set ( ApplicationsFolder Applications )  

set_target_properties (RoadManager PROPERTIES FOLDER ${ModulesFolder} )
set_target_properties (CommonMini PROPERTIES FOLDER ${ModulesFolder} )
set_target_properties (ScenarioEngine PROPERTIES FOLDER ${ModulesFolder} )
set_target_properties (RoadManagerDLL PROPERTIES FOLDER ${ModulesFolder} )
set_target_properties (ScenarioEngineDLL PROPERTIES FOLDER ${ModulesFolder} )

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
set ( MODELS_PACKAGE_URL https://drive.google.com/uc?export=download&id=11a8njhkRIjLYMqCsSL9pU-d5_A8pMVhc )

if (FORCE_DOWNLOAD_BINARIES OR NOT EXISTS ${MODELS_DIR} )
	download_and_extract(${MODELS_PACKAGE_URL} ${MODELS_BASE_DIR} ${MODELS_PACKAGE_FILENAME})
endif()

if (USE_OSG)
  if (NOT USE_ENVSIM_ADAPT)
    add_subdirectory(Replayer)
    add_subdirectory(OpenDriveViewer)
    add_subdirectory(OdrPlot)
  endif (NOT USE_ENVSIM_ADAPT)
  add_subdirectory(ViewerBase)

  if (NOT USE_ENVSIM_ADAPT)
    set_target_properties (EnvironmentSimulator PROPERTIES FOLDER ${ApplicationsFolder} )
    set_target_properties (ScenarioViewer PROPERTIES FOLDER ${ApplicationsFolder} )
    set_target_properties (EgoSimulator PROPERTIES FOLDER ${ApplicationsFolder} )
    set_target_properties (Replayer PROPERTIES FOLDER ${ApplicationsFolder} )
    set_target_properties (dat2ascii PROPERTIES FOLDER ${ApplicationsFolder} )    
    set_target_properties (osi_receiver PROPERTIES FOLDER ${ApplicationsFolder} )    
    set_target_properties (OpenDriveViewer PROPERTIES FOLDER ${ApplicationsFolder} )
    set_target_properties (OdrPlot PROPERTIES FOLDER ${ApplicationsFolder} )
  endif (NOT USE_ENVSIM_ADAPT)
  set_target_properties (ViewerBase PROPERTIES FOLDER ${ModulesFolder} )
  set_target_properties (PlayerBase PROPERTIES FOLDER ${ModulesFolder} )

endif (USE_OSG)
