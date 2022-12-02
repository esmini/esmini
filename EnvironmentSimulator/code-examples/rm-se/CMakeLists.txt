
include_directories (
  ${ESMINI_LIB_PATH}
  ${ESMINI_RM_LIB_PATH}
  ${COMMON_MINI_PATH}
)

set (TARGET rm-se)

set ( SOURCES
  rm-se.cpp
)

set ( INCLUDES
)

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

target_link_libraries (
	${TARGET}
	esminiLib
    esminiRMLib
	CommonMini
	${TIME_LIB}
  project_options
)

install ( TARGETS ${TARGET} DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
