
include_directories (
  ${ESMINI_RM_LIB_PATH}
  ${COMMON_MINI_PATH}
  ${EXTERNALS_PUGIXML_PATH}
)

set (TARGET rm-basic)

set ( SOURCES
  rm-basic.cpp
)

set ( INCLUDES
)

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

target_link_libraries (
	${TARGET}
	esminiRMLib
	CommonMini
	${TIME_LIB}
	project_options
)

install ( TARGETS ${TARGET} DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
