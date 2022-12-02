
include_directories (
  ${SCENARIO_ENGINE_PATH}/SourceFiles
  ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
  ${ROAD_MANAGER_PATH}
  ${VIEWER_BASE_PATH}
  ${PLAYER_BASE_PATH}
  ${CONTROLLERS_PATH}
  ${EXTERNALS_OSG_INCLUDES}
  ${EXTERNALS_SUMO_INCLUDES}
  ${COMMON_MINI_PATH}
  ${EXTERNALS_OSI_INCLUDES}
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
    project_options
)

install ( TARGETS ${TARGET} DESTINATION "${INSTALL_PATH}")
