
include_directories (
  ${SCENARIOENGINE_DLL_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}
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

install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY_CODE_EXAMPLES}")

