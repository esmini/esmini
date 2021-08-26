
include_directories (
  ${ROADMANAGER_INCLUDE_DIR}
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${VIEWER_BASE_INCLUDE_DIR}
if (USE_OSG)
  ${OSG_INCLUDE_DIR}
endif (USE_OSG)
  ${COMMON_MINI_INCLUDE_DIR}
if (USE_OSI)
  ${OSI_INCLUDE_DIR}
endif (USE_OSI)
  ${CONTROLLERS_INCLUDE_DIR}
)


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
endif (USE_OSG)

# dat2csv target
set (TARGET2 dat2csv)
add_executable ( ${TARGET2} dat2csv.cpp Replay.cpp )
target_link_libraries ( ${TARGET2} RoadManager CommonMini ${TIME_LIB} )


# osi_receiver target
if (USE_OSI)
  link_directories( ${OSI_LIBRARIES_PATH} )
  set (TARGET3 osireceiver)
  add_executable ( ${TARGET3} osi_receiver.cpp )
  target_link_libraries ( ${TARGET3} ${OSI_LIBRARIES} ${TIME_LIB} ${SOCK_LIB})
endif ()
# Install directives

if (UNIX)
  install ( TARGETS ${TARGET1} ${TARGET2} ${TARGET3} DESTINATION "${INSTALL_DIRECTORY}")
else()
  install ( TARGETS ${TARGET1} ${TARGET2} ${TARGET3} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET1} ${TARGET2} ${TARGET3} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}")
endif (UNIX)
