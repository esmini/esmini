
include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${COMMON_MINI_INCLUDE_DIR}  
  ${ROADMANAGER_INCLUDE_DIR}
  ${CONTROLLERS_INCLUDE_DIR}  
  ${OSG_INCLUDE_DIR}
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
