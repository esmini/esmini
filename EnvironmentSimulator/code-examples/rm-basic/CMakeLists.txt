
include_directories (
  ${ROADMANAGER_DLL_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}  
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
)

if (UNIX)
  install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY_CODE_EXAMPLES}")
else()
  install ( TARGETS ${TARGET} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY_CODE_EXAMPLES}")
  install ( TARGETS ${TARGET} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY_CODE_EXAMPLES}")
endif (UNIX)
