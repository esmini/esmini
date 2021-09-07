
include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${ROADMANAGER_INCLUDE_DIR}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${PLAYER_BASE_INCLUDE_DIR}
  ${CONTROLLERS_INCLUDE_DIR}
  ${OSG_INCLUDE_DIR}
  ${SUMO_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}
  ${OSI_INCLUDE_DIR}
)

set(TARGET esmini)

set ( SOURCES
	main.cpp
)

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

if (USE_OSG)
  add_definitions(-DOSG_LIBRARY_STATIC)
  set (viewer_libs ViewerBase ${OSG_LIBRARIES})
endif (USE_OSG)

if (USE_SUMO)
  set (sumo_libs ${SUMO_LIBRARIES})
endif (USE_SUMO)

if (USE_OSI)
  set (osi_libs ${OSI_LIBRARIES})
endif (USE_OSI)

target_link_libraries (
	${TARGET}
	PlayerBase
	ScenarioEngine
    Controllers
	RoadManager
	CommonMini
    ${viewer_libs}
	${osi_libs}
	${sumo_libs}
	${TIME_LIB}
    ${SOCK_LIB}
)

if (UNIX)
  install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
else()
  install ( TARGETS ${TARGET} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}")
endif (UNIX)
