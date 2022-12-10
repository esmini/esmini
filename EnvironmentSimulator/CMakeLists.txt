link_directories(${EXTERNALS_OSG_LIBRARY_PATH} ${EXTERNALS_OSI_LIBRARY_PATH} ${EXTERNALS_SUMO_LIBRARY_PATH} ${EXTERNALS_GTEST_LIBRARY_PATH} )

if(APPLE)
  SET(CMAKE_CXX_FLAGS "${CXX_STD_FLAG} -std=c++14 -pthread -fPIC -flto -DGL_SILENCE_DEPRECATION -Wl,-dead_strip")

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
  elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    SET(CMAKE_CXX_FLAGS "${CXX_STD_FLAG} -march=native -pthread -fPIC")
  elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    SET(CMAKE_CXX_FLAGS "${CXX_STD_FLAG} -march=native -O0 -pthread -fPIC")
  else()
    SET(CMAKE_CXX_FLAGS "${CXX_STD_FLAG} -pthread -fPIC -Wl,-strip-all")
  endif()

elseif(MSVC)

  # Get rid of historical macros preventing SUMO integration
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D NOMINMAX ")

  if ((MSVC) AND (MSVC_VERSION GREATER_EQUAL 1910))
    # suppress warning 4127 (conditional expression is constant) to avoid harmless warning from Google protobuf header
    # suppress warning 4100 (unreferenced formal parameter) to allow function headers with unreferenced parameters for increased readability
    add_compile_options(/permissive- /W4 /wd4127 /wd4100)
  endif ()

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
  elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    SET(CMAKE_CXX_FLAGS "${CXX_STD_FLAG} -march=native -pthread -fPIC")
  elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
    SET(CMAKE_CXX_FLAGS "${CXX_STD_FLAG} -march=native -O0 -fPIC")
  else()
    SET(CMAKE_CXX_FLAGS "${CXX_STD_FLAG} -fPIC -Wl,-strip-all")
  endif()

endif()

add_subdirectory(Modules/CommonMini)
add_subdirectory(Modules/Controllers)
add_subdirectory(Modules/PlayerBase)
add_subdirectory(Modules/RoadManager)
add_subdirectory(Modules/ScenarioEngine)

if (USE_OSG)
  add_subdirectory(Modules/ViewerBase)
endif (USE_OSG)

add_subdirectory(Libraries/esminiLib)
add_subdirectory(Libraries/esminiRMLib)
add_subdirectory(Applications/esmini)
add_subdirectory(Applications/esmini-dyn)
if (USE_OSG)
  add_subdirectory(Applications/odrviewer)
endif (USE_OSG)

# Add unittest folder
if (NOT (APPLE OR MINGW))
  if (USE_GTEST)
    add_subdirectory(Unittest)
  endif()
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
    if (NOT ${subdir} STREQUAL "trig_lane_change")
        add_subdirectory(code-examples/${subdir})
        if (NOT ${subdir} STREQUAL "hello_world")
            set_target_properties (${subdir} PROPERTIES FOLDER ${CodeExamplesFolder} )
        endif ()
    endif ()
ENDFOREACH()

add_subdirectory(Applications/odrplot)
add_subdirectory(Applications/replayer)



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
set(SCENARIO_ENGINE_PATH ${SCENARIO_ENGINE_PATH} CACHE INTERNAL "")
set(ROAD_MANAGER_PATH ${ROAD_MANAGER_PATH} CACHE INTERNAL "")
set(VIEWER_BASE_PATH ${VIEWER_BASE_PATH} CACHE INTERNAL "")
set(PLAYER_BASE_PATH ${PLAYER_BASE_PATH} CACHE INTERNAL "")
set(CONTROLLERS_PATH ${CONTROLLERS_PATH} CACHE INTERNAL "")
set(EXTERNALS_OSG_INCLUDES ${EXTERNALS_OSG_INCLUDES} CACHE INTERNAL "")
set(EXTERNALS_SUMO_INCLUDES ${EXTERNALS_SUMO_INCLUDES} CACHE INTERNAL "")
set(COMMON_MINI_PATH ${COMMON_MINI_PATH} CACHE INTERNAL "")
set(EXTERNALS_OSI_INCLUDES ${EXTERNALS_OSI_INCLUDES} CACHE INTERNAL "")


set(TIME_LIB ${TIME_LIB} CACHE INTERNAL "")
set(EXTERNALS_OSG_OS_SPECIFIC_PATH ${EXTERNALS_OSG_OS_SPECIFIC_PATH} CACHE INTERNAL "")
set(OSI_PACKAGE_URL ${OSI_PACKAGE_URL} CACHE INTERNAL "")
set(EXTERNALS_OSI_OS_SPECIFIC_PATH ${EXTERNALS_OSI_OS_SPECIFIC_PATH} CACHE INTERNAL "")
set(SUMO_PACKAGE_URL ${SUMO_PACKAGE_URL} CACHE INTERNAL "")
set(EXTERNALS_SUMO_OS_SPECIFIC_PATH ${EXTERNALS_SUMO_OS_SPECIFIC_PATH} CACHE INTERNAL "")
set(GTEST_PACKAGE_URL ${GTEST_PACKAGE_URL} CACHE INTERNAL "")
set(EXTERNALS_GOOGLETEST_OS_SPECIFIC_PATH ${EXTERNALS_GOOGLETEST_OS_SPECIFIC_PATH} CACHE INTERNAL "")
set(SOCK_LIB ${SOCK_LIB} CACHE INTERNAL "")
set(sumo_libs ${sumo_libs} CACHE INTERNAL "")
set(esminiRMLib ${esminiRMLib} CACHE INTERNAL "")
set(esminiLib ${esminiLib} CACHE INTERNAL "")
set(osi_libs ${osi_libs} CACHE INTERNAL "")
set(viewer_libs ${viewer_libs} CACHE INTERNAL "")


set(EXTERNALS_OSG_LIBRARY_PATH ${EXTERNALS_OSG_LIBRARY_PATH} CACHE INTERNAL "")
set(EXTERNALS_OSI_LIBRARY_PATH ${EXTERNALS_OSI_LIBRARY_PATH} CACHE INTERNAL "")
set(EXTERNALS_SUMO_LIBRARY_PATH ${EXTERNALS_SUMO_LIBRARY_PATH} CACHE INTERNAL "")
set(EXTERNALS_GTEST_LIBRARY_PATH ${EXTERNALS_GTEST_LIBRARY_PATH} CACHE INTERNAL "")

set(OSG_LIBRARIES ${OSG_LIBRARIES} CACHE INTERNAL "")
set(OSI_LIBRARIES ${OSI_LIBRARIES} CACHE INTERNAL "")
set(SUMO_LIBRARIES ${SUMO_LIBRARIES} CACHE INTERNAL "")
set(GTEST_LIBRARIES ${GTEST_LIBRARIES} CACHE INTERNAL "")

set(ESMINI_LIB_PATH ${ESMINI_LIB_PATH} CACHE INTERNAL "")
set(ESMINI_RM_LIB_PATH ${ESMINI_RM_LIB_PATH} CACHE INTERNAL "")
