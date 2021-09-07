
include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${COMMON_MINI_INCLUDE_DIR}
  ${ROADMANAGER_INCLUDE_DIR}
  ${OSG_INCLUDE_DIR}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${CONTROLLERS_INCLUDE_DIR}
  ${OSI_INCLUDE_DIR}
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
