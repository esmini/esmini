
include_directories (
  ${ESMINI_LIB_PATH}
  ${COMMON_MINI_PATH}
  ${EXTERNALS_OSI_INCLUDES}
)

set (TARGET esmini-dyn)

set ( SOURCES
  main.cpp
)

set ( INCLUDES
)

link_directories( ${EXTERNALS_OSI_OS_SPECIFIC_PATH}/lib )

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )


target_link_libraries (
	${TARGET}
	esminiLib
	CommonMini
	${TIME_LIB}
	${OSI_LIBRARIES}
	project_options
)

install ( TARGETS ${TARGET} DESTINATION "${INSTALL_PATH}")
