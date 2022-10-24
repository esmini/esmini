
include_directories (
  ${SCENARIOENGINE_DLL_INCLUDE_DIR}
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

install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY_CODE_EXAMPLES}")

