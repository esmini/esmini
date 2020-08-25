include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${SCENARIOENGINE_DLL_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}  
  ${ROADMANAGER_INCLUDE_DIR}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${PLAYER_BASE_INCLUDE_DIR}
  ${GTEST_INCLUDE_DIR}
)

macro(package_add_test_with_libraries TESTNAME FILES LIBRARIES)
    add_executable(${TESTNAME} ${FILES})

    target_link_libraries(${TESTNAME} ${GTEST_LIBRARIES} ${LIBRARIES})

    set_target_properties(${TESTNAME} PROPERTIES FOLDER Unittest)

    add_test(
    NAME ${TESTNAME}
    COMMAND ${TESTNAME}
    )
endmacro()

package_add_test_with_libraries(OperatingSystem_test OperatingSystem_test.cpp PlayerBase)
package_add_test_with_libraries(RoadManager_test RoadManager_test.cpp RoadManager)
package_add_test_with_libraries(ScenarioEngineDll_test ScenarioEngineDll_test.cpp ScenarioEngineDLL)
