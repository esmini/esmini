
include_directories (
  ${SCENARIO_ENGINE_PATH}/SourceFiles
  ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
  ${COMMON_MINI_PATH}
  ${ROAD_MANAGER_PATH}
  ${EXTERNALS_OSG_INCLUDES}
  ${VIEWER_BASE_PATH}
  ${CONTROLLERS_PATH}
  ${EXTERNALS_OSI_INCLUDES}
)

set ( SOURCES
  playerbase.cpp
)

set ( INCLUDES
  playerbase.hpp
  helpText.hpp
)

if (USE_OSG)
  add_definitions(-DOSG_LIBRARY_STATIC)
endif (USE_OSG)

add_library ( PlayerBase STATIC ${SOURCES} ${INCLUDES} )
target_link_libraries(PlayerBase PRIVATE project_options)
