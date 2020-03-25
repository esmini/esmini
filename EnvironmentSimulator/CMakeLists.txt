
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

set( CMAKE_VERBOSE_MAKEFILE false )

if (APPLE)
  set ( TIME_LIB "" )
  set ( OSG_PACKAGE_URL https://drive.google.com/uc?export=download&id=1yEFOB8HmNP5lPxYD4VJC-ry7XaBG9dsJ )
  set ( OSG_DIR "${OSG_BASE_DIR}/mac" )
elseif (UNIX)
  set ( TIME_LIB "" )
  set ( OSG_PACKAGE_URL https://drive.google.com/uc?export=download&id=1OufA3TUQjBTkaRvAuo0rSUwryvoqde8G )
  set ( OSG_DIR "${OSG_BASE_DIR}/linux" )
elseif (WINSDK7_1)
  set ( SOCK_LIB Ws2_32.lib )
  set ( TIME_LIB winmm.lib )
  set ( OSG_PACKAGE_URL https://drive.google.com/uc?export=download&id=1NBEvGZiTWmqxk-MEOq7uK1uh_vaUMaEL )
  set ( OSG_DIR "${OSG_BASE_DIR}/winsdk71" )
elseif ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "Win32")
  set ( SOCK_LIB Ws2_32.lib )
  set ( TIME_LIB "" )  
  set ( OSG_PACKAGE_URL https://drive.google.com/uc?export=download&id=10dV9P0qOeJUgTtsSDld4AlbClE--SivX )
  set ( OSG_DIR "${OSG_BASE_DIR}/win32" )
else()  # v10
  set ( SOCK_LIB Ws2_32.lib )
  set ( TIME_LIB "" )
  set ( OSG_PACKAGE_URL https://drive.google.com/uc?export=download&id=1YxLVdQLhKBMGW4HB_ArJglRIpzuDiwhJ )
  set ( OSG_DIR "${OSG_BASE_DIR}/v10" )
endif (APPLE)

set ( OSG_INCLUDE_DIR 
	"${OSG_DIR}/build/include"
	"${OSG_DIR}/include"
)
set ( OSG_LIBRARIES_PATH 
	"${OSG_DIR}/lib" 
	"${OSG_DIR}/lib/osgPlugins-3.6.5"
)

if(APPLE)
  SET(CMAKE_CXX_FLAGS "-std=c++14 -pthread -flto -Wl,-dead_strip")

  SET( "-framework OpenGL -framework Cocoa")
  
  link_directories(${OSG_LIBRARIES_PATH})

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
elseif(UNIX)
   SET(CMAKE_CXX_FLAGS "-std=c++14 -pthread -flto -Wl,-strip-all")
  
  link_directories(${OSG_LIBRARIES_PATH})
  
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

	GL
	X11
	Xrandr
	dl
	z
    Xinerama
	#db_fbx
  )
else() #not UNIX

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
if (NOT WINSDK7_1)  # FBX not supporting winsdk 7.1
  list ( APPEND OSG_LIBRARIES optimized libfbxsdk-md.lib debug libfbxsdk-mdd.lib )
endif()

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
    set_target_properties (OpenDriveViewer PROPERTIES FOLDER ${ApplicationsFolder} )
    set_target_properties (OdrPlot PROPERTIES FOLDER ${ApplicationsFolder} )
  endif (NOT USE_ENVSIM_ADAPT)
  set_target_properties (ViewerBase PROPERTIES FOLDER ${ModulesFolder} )
  set_target_properties (PlayerBase PROPERTIES FOLDER ${ModulesFolder} )

endif (USE_OSG)
