
include_directories (
  ${ROADMANAGER_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}  
)

set(TARGET odrplot)

set ( SOURCES
  main.cpp
)

set ( INCLUDES
)

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

target_link_libraries ( 
	${TARGET}
	RoadManager
	CommonMini	
	${TIME_LIB}
	project_options
)

install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
