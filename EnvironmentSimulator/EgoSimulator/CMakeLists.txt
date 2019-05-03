
include_directories (
  ${ROADMANAGER_INCLUDE_DIR}
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${OSG_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}
)

set ( SOURCES
	main.cpp
	vehicle.cpp
)

set ( INCLUDES
	vehicle.hpp
)

link_directories( ${OSG_LIBRARIES_PATH} )

add_executable ( EgoSimulator ${SOURCES} ${INCLUDES} )

add_definitions(-DOSG_LIBRARY_STATIC)

target_link_libraries ( 
	EgoSimulator
	RoadManager
	ScenarioEngine
	ViewerBase
	CommonMini
	${OSG_LIBRARIES}
	${TIME_LIB}	
)

install ( TARGETS EgoSimulator CONFIGURATIONS release DESTINATION "${INSTALL_DIRECTORY}/Release")
install ( TARGETS EgoSimulator CONFIGURATIONS debug DESTINATION "${INSTALL_DIRECTORY}/Debug")
