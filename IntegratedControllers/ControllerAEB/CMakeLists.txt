cmake_minimum_required(VERSION 3.4)

# point to esmini root folder, used for locating needed header files
set(ESMINI_PATH ../../)

project(AEBController)

include_directories(
    "${ESMINI_PATH}/EnvironmentSimulator/Modules/CommonMini",
    "${ESMINI_PATH}/EnvironmentSimulator/Modules/Controllers",
    "${ESMINI_PATH}/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles",
    "${ESMINI_PATH}/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs",
    "${ESMINI_PATH}/EnvironmentSimulator/Modules/RoadManager",
    "${ESMINI_PATH}/externals/pugixml"
)

add_library(${PROJECT_NAME} SHARED ControllerAEB.cpp)

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION "${INSTALL_PATH}/IntegratedControllers")