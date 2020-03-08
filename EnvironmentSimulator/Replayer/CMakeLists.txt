
include_directories (
  ${ROADMANAGER_INCLUDE_DIR}
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${OSG_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}
)

set (TARGET Replayer)

set ( SOURCES
	main.cpp
	Replay.cpp
)

set ( INCLUDES
	Replay.hpp
)

link_directories( ${OSG_LIBRARIES_PATH} )

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

add_definitions(-DOSG_LIBRARY_STATIC)

target_link_libraries ( 
	${TARGET}
	RoadManager
	ScenarioEngine
	ViewerBase
	CommonMini
	${OSG_LIBRARIES}
	${TIME_LIB}	
)

if (UNIX)
  install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
else()
  install ( TARGETS ${TARGET} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}")
endif (UNIX)

