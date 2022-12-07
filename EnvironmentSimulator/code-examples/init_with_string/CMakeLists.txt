
include_directories (
  ${ESMINI_LIB_PATH}
  ${COMMON_MINI_PATH}
  ${EXTERNALS_PUGIXML_PATH}
)

set (TARGET init_with_string)

set ( SOURCES
  init_with_string.cpp
)

set ( INCLUDES
)

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

target_link_libraries (
	${TARGET}
	esminiLib
	CommonMini
    RoadManager
	${TIME_LIB}
  project_options
)

install ( TARGETS ${TARGET} DESTINATION "${CODE_EXAMPLES_BIN_PATH}")

