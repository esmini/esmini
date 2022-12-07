
include_directories (
  ${ROAD_MANAGER_PATH}
  ${COMMON_MINI_PATH}
  ${EXTERNALS_PUGIXML_PATH}
)

set(TARGET odrplot)

set ( SOURCES
  main.cpp
)

set ( INCLUDES
)

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

target_link_libraries (
	${TARGET}
	RoadManager
	CommonMini
	${TIME_LIB}
	project_options
)

install ( TARGETS ${TARGET} DESTINATION "${INSTALL_PATH}")
