
include_directories (
  ${SCENARIO_ENGINE_PATH}/SourceFiles
  ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
  ${ROAD_MANAGER_PATH}
  ${VIEWER_BASE_PATH}
  ${EXTERNALS_OSG_INCLUDES}
  ${COMMON_MINI_PATH}
  ${CONTROLLERS_PATH}
  ${EXTERNALS_OSI_INCLUDES}
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
  project_options
)

install ( TARGETS ${TARGET} DESTINATION "${INSTALL_PATH}")
