
include_directories (
  ${SCENARIO_ENGINE_PATH}/SourceFiles
  ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
  ${COMMON_MINI_PATH}
  ${ROAD_MANAGER_PATH}
  ${CONTROLLERS_PATH}
  ${EXTERNALS_OSG_INCLUDES}
  ${EXTERNALS_OSI_INCLUDES}
)

set ( SOURCES
  viewer.cpp
  roadgeom.cpp
  RubberbandManipulator.cpp
)

set ( INCLUDES
  viewer.hpp
  roadgeom.hpp
  RubberbandManipulator.hpp
)

add_definitions(-DOSG_LIBRARY_STATIC)

add_library ( ViewerBase STATIC ${SOURCES} ${INCLUDES} )

target_link_libraries(ViewerBase PRIVATE ScenarioEngine project_options)