# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Loading unittest macro #############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/common/unittest.cmake)

# ############################### Creating executable (OperatingSystem_test) #########################################

if(USE_OSG)
    set(VIEWER_LIBS_FOR_TEST
        ViewerBase)
endif()

set(UNITTEST_COMMON_SRC
    TestHelper.hpp
    TestHelper.cpp)

set(OperatingSystem_test_sources
    OperatingSystem_test.cpp)

unittest(
    OperatingSystem_test
    "${OperatingSystem_test_sources}"
    PlayerBase)

# ############################### Creating executable (RoadManager_test) #############################################

set(RoadManager_test_sources
    RoadManager_test.cpp
    ${UNITTEST_COMMON_SRC})

unittest(
    RoadManager_test
    "${RoadManager_test_sources}"
    RoadManager)

# ############################### Creating executable (ScenarioEngine_test) ##########################################

set(ScenarioEngine_test_sources
    ScenarioEngine_test.cpp
    ${UNITTEST_COMMON_SRC})

unittest(
    ScenarioEngine_test
    "${ScenarioEngine_test_sources}"
    ScenarioEngine
    Controllers
    RoadManager
    CommonMini
    PlayerBase
    ScenarioEngine
    ${VIEWER_LIBS_FOR_TEST}
    ${OSG_LIBRARIES}
    ${OSI_LIBRARIES}
    ${SUMO_LIBRARIES}
    ${SPDLOG_LIBRARIES}
    ${SOCK_LIB})

# ############################### Creating executable (ScenarioPlayer_test) ##########################################
set(ScenarioPlayer_test_sources
    ScenarioPlayer_test.cpp
    ${UNITTEST_COMMON_SRC})

unittest(
    ScenarioPlayer_test
    "${ScenarioPlayer_test_sources}"
    PlayerBase
    ScenarioEngine
    Controllers
    RoadManager
    CommonMini
    ${VIEWER_LIBS_FOR_TEST}
    ${OSG_LIBRARIES}
    ${OSI_LIBRARIES}
    ${SUMO_LIBRARIES}
    ${SPDLOG_LIBRARIES}
    ${SOCK_LIB})

# ############################### Creating executable (ScenarioEngineDll_test) #######################################

set(ScenarioEngineDll_sources
    ScenarioEngineDll_test.cpp
    ${UNITTEST_COMMON_SRC}
    "${REPLAYER_PATH}/Replay.cpp")

unittest(
    ScenarioEngineDll_test
    "${ScenarioEngineDll_sources}"
    esminiLib
    CommonMini
    ${OSI_LIBRARIES})

# ############################### Creating executable (RoadManagerDll_test) ##########################################

set(RoadManagerDll_test_sources
    RoadManagerDll_test.cpp
    ${UNITTEST_COMMON_SRC})

unittest(
    RoadManagerDll_test
    "${RoadManagerDll_test_sources}"
    esminiRMLib
    CommonMini)

# ############################### Creating executable (CommonMini_test) ##############################################
set(CommonMini_test_sources
    CommonMini_test.cpp
    ${UNITTEST_COMMON_SRC})

unittest(
    CommonMini_test
    "${CommonMini_test_sources}"
    esminiLib
    CommonMini)

# ############################### Creating executable (FollowRoute_test) ##############################################
set(FollowRoute_test_sources
    FollowRoute_test.cpp
    ${UNITTEST_COMMON_SRC})

unittest(
    FollowRoute_test
    "${FollowRoute_test_sources}"
    RoadManager)

# ############################### Creating executable (FollowRouteController_test) ##############################################
set(FollowRouteController_test_sources
    FollowRouteController_test.cpp
    ${UNITTEST_COMMON_SRC})

unittest(
    FollowRouteController_test
    "${FollowRouteController_test_sources}"
    ScenarioEngine
    Controllers
    PlayerBase
    ScenarioEngine
    RoadManager
    ${VIEWER_LIBS_FOR_TEST}
    ${OSG_LIBRARIES}
    ${OSI_LIBRARIES}
    ${SUMO_LIBRARIES}
    ${SPDLOG_LIBRARIES}
    ${SOCK_LIB})
