
include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${ROADMANAGER_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}  
)

set(TARGET RoadManagerDLL)

set ( SOURCES roadmanagerdll.cpp )
set ( INCLUDES roadmanagerdll.hpp )

add_library ( ${TARGET} SHARED ${SOURCES} ${INCLUDES} )

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

target_link_libraries ( 
	${TARGET}
	PRIVATE RoadManager
	PRIVATE CommonMini
	PRIVATE ${TIME_LIB}	
)

if (UNIX)
  install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
else()
  install ( TARGETS ${TARGET} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}")
endif (UNIX)
