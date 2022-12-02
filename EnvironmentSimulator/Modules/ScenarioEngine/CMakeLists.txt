
include_directories (
  ${PUGIXML_INCLUDE_DIR}
  ${SCENARIO_ENGINE_PATH}/SourceFiles
  ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
  ${ROAD_MANAGER_PATH}
  ${COMMON_MINI_PATH}
  ${CONTROLLERS_PATH}
  ${REPLAYER_PATH}
  ${RDB_INCLUDE_DIR}
  ${EXTERNALS_OSI_INCLUDES}
  ${EXTERNALS_SUMO_INCLUDES}
)


file(GLOB SRC_OSCTYPEDEFS "${CMAKE_CURRENT_LIST_DIR}/OSCTypeDefs/*.[hc]pp")
file(GLOB SRC_SOURCEFILES "${CMAKE_CURRENT_LIST_DIR}/SourceFiles/*.[hc]pp" "${CMAKE_CURRENT_LIST_DIR}/SourceFiles/*.[hc]")

if (NOT USE_OSI)
    list(REMOVE_ITEM SRC_SOURCEFILES "${CMAKE_CURRENT_LIST_DIR}/SourceFiles/OSIReporter.hpp")
    list(REMOVE_ITEM SRC_SOURCEFILES "${CMAKE_CURRENT_LIST_DIR}/SourceFiles/OSIReporter.cpp")
endif (NOT USE_OSI)

set ( SRC_ADDITIONAL ../../../externals/pugixml/pugixml.cpp )

SOURCE_GROUP(OSCTypeDefs FILES ${SRC_OSCTYPEDEFS})
SOURCE_GROUP("Source Files" FILES ${SRC_SOURCEFILES})
SOURCE_GROUP("External Libraries" FILES ${SRC_ADDITIONAL})

add_library ( ScenarioEngine STATIC
  ${SRC_OSCTYPEDEFS}
  ${SRC_SOURCEFILES}
  ${SRC_ADDITIONAL}
)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

target_link_libraries(ScenarioEngine PRIVATE project_options)


