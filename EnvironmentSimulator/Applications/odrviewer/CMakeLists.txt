
include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${ROADMANAGER_INCLUDE_DIR}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${OSG_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}
  ${CONTROLLERS_INCLUDE_DIR}
  ${OSI_INCLUDE_DIR}
)

set(TARGET odrviewer)

if (USE_OSI)
  set (osi_libs ${OSI_LIBRARIES})
endif (USE_OSI)

set ( SOURCES
	main.cpp
    helpText.hpp
)

link_directories( ${OSG_LIBRARIES_PATH} )

add_executable ( ${TARGET} ${SOURCES} )

add_definitions(-DOSG_LIBRARY_STATIC)

target_link_libraries (
	${TARGET}
	RoadManager
	ViewerBase
	CommonMini
	${OSG_LIBRARIES}
	${TIME_LIB}
  ${osi_libs}
)

if (UNIX)
  install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
else()
  install ( TARGETS ${TARGET} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}")
endif (UNIX)
