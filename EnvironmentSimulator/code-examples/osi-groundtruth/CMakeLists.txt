
include_directories (
  ${SCENARIOENGINE_DLL_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}  
  ${OSI_INCLUDE_DIR}
)

set (TARGET osi-groundtruth)

set ( SOURCES
  osi-groundtruth.cpp
)

set ( INCLUDES
)

link_directories( ${OSI_DIR}/lib )

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

target_link_libraries ( 
	${TARGET}
	esminiLib
	CommonMini
    RoadManager
	${TIME_LIB}
    ${OSI_LIBRARIES}
)

if (UNIX)
  install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY_CODE_EXAMPLES}")
else()
  install ( TARGETS ${TARGET} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY_CODE_EXAMPLES}")
  install ( TARGETS ${TARGET} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY_CODE_EXAMPLES}")
endif (UNIX)

