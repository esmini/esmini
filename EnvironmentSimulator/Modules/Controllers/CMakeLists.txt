
include_directories (
  ${CONTROLLERS_PATH}
  ${SCENARIO_ENGINE_PATH}/SourceFiles
  ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
  ${ROAD_MANAGER_PATH}
  ${COMMON_MINI_PATH}
  ${EXTERNALS_SUMO_INCLUDES}
)

file(GLOB CONTROLLERS_SOURCES
    "*.cpp"
)

file(GLOB CONTROLLERS_INCLUDES
    "*.hpp"
)

if (NOT USE_SUMO)
    list(REMOVE_ITEM CONTROLLERS_SOURCES "${CMAKE_CURRENT_LIST_DIR}/ControllerSumo.cpp")
    list(REMOVE_ITEM CONTROLLERS_INCLUDES "${CMAKE_CURRENT_LIST_DIR}/ControllerSumo.hpp")
endif (NOT USE_SUMO)

add_library ( Controllers STATIC ${CONTROLLERS_SOURCES} ${CONTROLLERS_INCLUDES} )
target_link_libraries(Controllers PRIVATE project_options)