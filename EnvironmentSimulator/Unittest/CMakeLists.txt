# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/system_link.cmake)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

# ############################### Loading unittest macro #############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/common/unittest.cmake)

# ############################### Adding includes ####################################################################

include_directories(
    ${SCENARIO_ENGINE_PATH}/SourceFiles
    ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
    ${ESMINI_LIB_PATH}
    ${ESMINI_RM_LIB_PATH}
    ${COMMON_MINI_PATH}
    ${VIEWER_BASE_PATH}
    ${PLAYER_BASE_PATH}
    ${CONTROLLERS_PATH}
    ${REPLAYER_PATH})

# ############################### Creating executable (OperatingSystem_test) #########################################

unittest(
    OperatingSystem_test
    OperatingSystem_test.cpp
    PlayerBase)

# ############################### Creating executable (RoadManager_test) #############################################

unittest(
    RoadManager_test
    RoadManager_test.cpp
    RoadManager)

# ############################### Creating executable (ScenarioEngine_test) ##########################################

unittest(
    ScenarioEngine_test
    ScenarioEngine_test.cpp
    ScenarioEngine
    Controllers
    RoadManager
    CommonMini
    ViewerBase
    OSG
    OSI
    SUMO
    ${SOCK_LIB})

# ############################### Creating executable (ScenarioPlayer_test) ##########################################

unittest(
    ScenarioPlayer_test
    ScenarioPlayer_test.cpp
    PlayerBase
    ScenarioEngine
    Controllers
    RoadManager
    CommonMini
    ViewerBase
    OSG
    OSI
    SUMO
    ${SOCK_LIB})

# ############################### Creating executable (ScenarioEngineDll_test) #######################################

set(ScenarioEngineDll_sources
    ScenarioEngineDll_test.cpp
    "${REPLAYER_PATH}/Replay.cpp")

unittest(
    ScenarioEngineDll_test
    "${ScenarioEngineDll_sources}"
    esminiLib
    CommonMini
    OSI)

# ############################### Creating executable (RoadManagerDll_test) ##########################################

unittest(
    RoadManagerDll_test
    RoadManagerDll_test.cpp
    esminiRMLib
    CommonMini)

# ############################### Creating executable (CommonMini_test) ##############################################

unittest(
    CommonMini_test
    CommonMini_test.cpp
    CommonMini)

# ############################### Creating executable (FollowRoute_test) ##############################################

unittest(
    FollowRoute_test
    FollowRoute_test.cpp
    ScenarioEngine
    Controllers
    RoadManager
    CommonMini
    ViewerBase
    OSG
    SUMO
    ${SOCK_LIB})

# ############################### Creating executable (FollowRouteController_test) ##############################################

unittest(
    FollowRouteController_test
    FollowRouteController_test.cpp
    ScenarioEngine
    Controllers
    RoadManager
    CommonMini
    ViewerBase
    OSG
    SUMO
    ${SOCK_LIB})
