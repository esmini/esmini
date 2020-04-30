
include_directories (
  ${ROADMANAGER_INCLUDE_DIR}
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${OSG_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}
)

set ( INCLUDES
	Replay.hpp
)

# Replayer target
set (TARGET1 Replayer)
add_definitions(-DOSG_LIBRARY_STATIC)
set ( SOURCES
	main.cpp
	Replay.cpp
)
link_directories( ${OSG_LIBRARIES_PATH} )
add_executable ( ${TARGET1} ${SOURCES} ${INCLUDES} )
target_link_libraries ( 
	${TARGET1}
	RoadManager
	ScenarioEngine
	ViewerBase
	CommonMini
	${OSG_LIBRARIES}
	${TIME_LIB}	
)

# dat2ascii target
set (TARGET2 dat2ascii)
add_executable ( ${TARGET2} dat2ascii.cpp Replay.cpp )
target_link_libraries ( ${TARGET2} RoadManager CommonMini ${TIME_LIB} )


if (UNIX)
  install ( TARGETS ${TARGET1} ${TARGET2} DESTINATION "${INSTALL_DIRECTORY}")
else()
  install ( TARGETS ${TARGET} ${TARGET2} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} ${TARGET2} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}")
endif (UNIX)
