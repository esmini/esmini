link_directories(${EXTERNALS_OSG_LIBRARY_PATH} ${EXTERNALS_OSI_LIBRARY_PATH} ${EXTERNALS_SUMO_LIBRARY_PATH} ${EXTERNALS_GTEST_LIBRARY_PATH} )

# ############################### Building Modules #####################################################################

add_subdirectory(Modules/CommonMini)
add_subdirectory(Modules/Controllers)
add_subdirectory(Modules/PlayerBase)
add_subdirectory(Modules/RoadManager)
add_subdirectory(Modules/ScenarioEngine)

if (USE_OSG)
  add_subdirectory(Modules/ViewerBase)
endif (USE_OSG)

# ############################### Building Libraries ###################################################################

add_subdirectory(Libraries/esminiLib)
add_subdirectory(Libraries/esminiRMLib)

# ############################### Building Applications ################################################################

add_subdirectory(Applications/esmini)
add_subdirectory(Applications/esmini-dyn)

if (USE_OSG)
  add_subdirectory(Applications/odrviewer)
endif (USE_OSG)

add_subdirectory(Applications/odrplot)
add_subdirectory(Applications/replayer)

# ############################### Building Unittest ####################################################################

if ((NOT (APPLE OR MINGW)) AND USE_GTEST)
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

# ############################### Getting code-examples ################################################################

get_subdirectories(${CODE_EXAMPLES_PATH} subdirectories)

if (NOT USE_OSI)
  list(REMOVE_ITEM subdirectories "osi-groundtruth")
  list(REMOVE_ITEM subdirectories "osi-groundtruth-dyn")
endif ()

# ############################### Building code-examples ###############################################################

FOREACH(subdir ${subdirectories})
    if (NOT ${subdir} STREQUAL "trig_lane_change")
        add_subdirectory(code-examples/${subdir})
        if (NOT ${subdir} STREQUAL "hello_world")
            set_target_properties (${subdir} PROPERTIES FOLDER ${CodeExamplesFolder} )
        endif ()
    endif ()
ENDFOREACH()

# ############################### Others ###############################################################################

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
