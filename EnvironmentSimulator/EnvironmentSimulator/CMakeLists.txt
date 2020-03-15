
include_directories (
  ${ROADMANAGER_INCLUDE_DIR}
  ${RDB_INCLUDE_DIR}
  ${RAPIDXML_INCLUDE_DIR}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${PLAYER_BASE_INCLUDE_DIR}  
  ${OSG_INCLUDE_DIR}
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${COMMON_MINI_INCLUDE_DIR}
)

set(TARGET EnvironmentSimulator)

set ( SOURCES
  main.cpp
)

set ( INCLUDES
)

link_directories( ${OSG_LIBRARIES_PATH} )

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

add_definitions(-DOSG_LIBRARY_STATIC)

target_link_libraries ( 
	${TARGET}
	PlayerBase
	ScenarioEngine
	RoadManager
	ViewerBase
	CommonMini
	${OSG_LIBRARIES}
	${TIME_LIB}
    ${SOCK_LIB}
)

if (UNIX)
  install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
else()
  install ( TARGETS ${TARGET} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}")
endif (UNIX)