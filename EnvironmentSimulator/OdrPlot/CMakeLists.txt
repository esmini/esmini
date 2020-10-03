
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
	CommonMini	
	RoadManager
	${TIME_LIB}
)

if (UNIX)
  install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
else()
  install ( TARGETS ${TARGET} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}")
endif (UNIX)
