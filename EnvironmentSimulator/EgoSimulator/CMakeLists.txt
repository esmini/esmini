
include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${ROADMANAGER_INCLUDE_DIR}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${PLAYER_BASE_INCLUDE_DIR}
  ${OSG_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}
)

set(TARGET EgoSimulator)

set ( SOURCES
	main.cpp
	vehicle.cpp
)

set ( INCLUDES
	vehicle.hpp
)

link_directories( ${OSG_LIBRARIES_PATH} )

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

if (USE_OSG)
  add_definitions(-DOSG_LIBRARY_STATIC -D_SCENARIO_VIEWER)
  set (viewer_libs ViewerBase ${OSG_LIBRARIES})
endif (USE_OSG)

target_link_libraries ( 
	${TARGET}
	PlayerBase    
	ScenarioEngine
	RoadManager
	CommonMini
    	${viewer_libs}
	${TIME_LIB}	
    	${SOCK_LIB}
)

if (UNIX)
  install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
else()
  install ( TARGETS ${TARGET} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}")
endif (UNIX)
