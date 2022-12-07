include_directories (
  ${SCENARIO_ENGINE_PATH}/SourceFiles
  ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
  ${ESMINI_LIB_PATH}
  ${ESMINI_RM_LIB_PATH}
  ${COMMON_MINI_PATH}
  ${ROAD_MANAGER_PATH}
  ${VIEWER_BASE_PATH}
  ${PLAYER_BASE_PATH}
  ${CONTROLLERS_PATH}
  ${REPLAYER_PATH}
  ${EXTERNALS_GOOGLETEST_INCLUDES}
  ${EXTERNALS_OSI_INCLUDES}
  ${EXTERNALS_OSG_INCLUDES}
  ${DIRENT_INCLUDE_DIR}
  ${EXTERNALS_PUGIXML_PATH}
)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

macro(package_add_test_with_libraries TESTNAME FILES LIBRARIES)
    add_executable(${TESTNAME} ${FILES})

    target_link_libraries(${TESTNAME} ${GTEST_LIBRARIES} ${LIBRARIES} ${ARGN} project_options)

    set_target_properties(${TESTNAME} PROPERTIES FOLDER Unittest)

    add_test(
      NAME ${TESTNAME}
      COMMAND ${TESTNAME}
    )
endmacro()

if (USE_OSG)
  add_definitions(-DOSG_LIBRARY_STATIC)
  set (viewer_libs ViewerBase ${OSG_LIBRARIES})
endif (USE_OSG)

if (USE_SUMO)
  set (sumo_libs ${SUMO_LIBRARIES})
endif (USE_SUMO)

if (USE_OSI)
  set (osi_libs ${OSI_LIBRARIES})
endif (USE_OSI)

package_add_test_with_libraries(OperatingSystem_test OperatingSystem_test.cpp PlayerBase)
package_add_test_with_libraries(RoadManager_test RoadManager_test.cpp RoadManager)
package_add_test_with_libraries(ScenarioEngine_test ScenarioEngine_test.cpp
    ScenarioEngine
    Controllers
	RoadManager
	CommonMini
    ${viewer_libs}
	${osi_libs}
	${sumo_libs}
	${SOCK_LIB}
)
package_add_test_with_libraries(ScenarioPlayer_test ScenarioPlayer_test.cpp
    PlayerBase
    ScenarioEngine
    Controllers
	RoadManager
	CommonMini
    ${viewer_libs}
	${osi_libs}
	${sumo_libs}
	${SOCK_LIB}
)
set (sources ScenarioEngineDll_test.cpp  "../Applications/replayer/Replay.cpp")
package_add_test_with_libraries(ScenarioEngineDll_test "${sources}" esminiLib CommonMini ${OSI_LIBRARIES})
package_add_test_with_libraries(RoadManagerDll_test RoadManagerDll_test.cpp esminiRMLib CommonMini)
package_add_test_with_libraries(CommonMini_test CommonMini_test.cpp CommonMini)
package_add_test_with_libraries(FollowRoute_test FollowRoute_test.cpp
    ScenarioEngine
    Controllers
  RoadManager
  CommonMini
    ${viewer_libs}
	${SUMO_LIBRARIES}
	${SOCK_LIB}
)
package_add_test_with_libraries(FollowRouteController_test FollowRouteController_test.cpp
    ScenarioEngine
    Controllers
  RoadManager
  CommonMini
    ${viewer_libs}
	${SUMO_LIBRARIES}
	${SOCK_LIB}
)
