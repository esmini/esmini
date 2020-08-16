include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
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

package_add_test_with_libraries(tmp_test tmp_test.cpp PlayerBase)
package_add_test_with_libraries(os_test os_test.cpp PlayerBase)
