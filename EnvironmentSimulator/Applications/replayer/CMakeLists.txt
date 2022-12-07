
include_directories (
  ${ROAD_MANAGER_PATH}
  ${SCENARIO_ENGINE_PATH}/SourceFiles
  ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
  ${VIEWER_BASE_PATH}
if (USE_OSG)
  ${EXTERNALS_OSG_INCLUDES}
endif (USE_OSG)
  ${COMMON_MINI_PATH}
if (USE_OSI)
  ${EXTERNALS_OSI_INCLUDES}
endif (USE_OSI)
  ${CONTROLLERS_PATH}
  ${DIRENT_INCLUDE_DIR}
  ${EXTERNALS_PUGIXML_PATH}
)

if (USE_OSI)
  set (osi_libs ${OSI_LIBRARIES})
endif (USE_OSI)

set ( INCLUDES
	Replay.hpp
)

if (USE_OSG)
# Replayer target
set (TARGET1 replayer)
add_definitions(-DOSG_LIBRARY_STATIC)
set ( SOURCES
	main.cpp
	Replay.cpp
    helpText.hpp
    collision.hpp
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
  ${osi_libs}
  project_options
)
endif (USE_OSG)

# dat2csv target
set (TARGET2 dat2csv)
add_executable ( ${TARGET2} dat2csv.cpp Replay.cpp )
target_link_libraries ( ${TARGET2} RoadManager CommonMini ${TIME_LIB} project_options)


# osi_receiver target
if (USE_OSI)
  link_directories( ${OSI_LIBRARIES_PATH} )
  set (TARGET3 osireceiver)
  add_executable ( ${TARGET3} osi_receiver.cpp )
  target_link_libraries ( ${TARGET3} ${OSI_LIBRARIES} ${TIME_LIB} ${SOCK_LIB} project_options)
endif ()
# Install directives

install ( TARGETS ${TARGET1} ${TARGET2} ${TARGET3} DESTINATION "${INSTALL_PATH}")

