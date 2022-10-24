
include_directories (
  ${SCENARIOENGINE_DLL_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}  
  ${OSI_INCLUDE_DIR}
)

set (TARGET esmini-dyn)

set ( SOURCES
  main.cpp
)

set ( INCLUDES
)

link_directories( ${OSI_DIR}/lib )

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )


target_link_libraries ( 
	${TARGET}
	esminiLib
	CommonMini
	${TIME_LIB}
  project_options
)

install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
