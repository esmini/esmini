
include_directories (
  ${ESMINI_LIB_PATH}
  ${COMMON_MINI_PATH}
)

set (TARGET test-driver)

set ( SOURCES
  test-driver.cpp
)

set ( INCLUDES
)

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

target_link_libraries (
	${TARGET}
	esminiLib
    RoadManager
	CommonMini
	${TIME_LIB}
  project_options
)

install ( TARGETS ${TARGET} DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
