
include_directories (
  ${ESMINI_LIB_PATH}
)

set (TARGET ad_hoc_traffic)

set ( SOURCES
  ad_hoc_traffic.cpp
)

set ( INCLUDES
)

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

target_link_libraries (
	${TARGET}
	esminiLib
  project_options
)

install ( TARGETS ${TARGET} DESTINATION "${CODE_EXAMPLES_BIN_PATH}")

