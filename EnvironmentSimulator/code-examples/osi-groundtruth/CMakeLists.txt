
include_directories (
  ${ESMINI_LIB_PATH}
  ${COMMON_MINI_PATH}
  ${EXTERNALS_OSI_INCLUDES}
  ${EXTERNALS_PUGIXML_PATH}
)

set (TARGET osi-groundtruth)

set ( SOURCES
  osi-groundtruth.cpp
)

set ( INCLUDES
)

link_directories( ${EXTERNALS_OSI_OS_SPECIFIC_PATH}/lib )

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

target_link_libraries (
	${TARGET}
	esminiLib
	CommonMini
    RoadManager
	${TIME_LIB}
    ${OSI_LIBRARIES}
    project_options
)

install ( TARGETS ${TARGET} DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
