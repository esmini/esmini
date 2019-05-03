
include_directories (
  ${ROADMANAGER_INCLUDE_DIR}
  ${RDB_INCLUDE_DIR}
  ${RAPIDXML_INCLUDE_DIR}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${OSG_INCLUDE_DIR}
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${COMMON_MINI_INCLUDE_DIR}
)

set ( SOURCES
  main.cpp
)

set ( INCLUDES
)

link_directories( ${OSG_LIBRARIES_PATH} )

add_executable ( EnvironmentSimulator ${SOURCES} ${INCLUDES} )

add_definitions(-DOSG_LIBRARY_STATIC)

target_link_libraries ( 
	EnvironmentSimulator
	ScenarioEngine
	RoadManager
	ViewerBase
	CommonMini
	${OSG_LIBRARIES}
	${TIME_LIB}
)

install ( TARGETS EnvironmentSimulator CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}/Release")
install ( TARGETS EnvironmentSimulator CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}/Debug")
