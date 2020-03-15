
include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${ROADMANAGER_INCLUDE_DIR}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${PLAYER_BASE_INCLUDE_DIR}
  ${OSG_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}  
)

set(TARGET ScenarioEngineDLL)

set ( SOURCES scenarioenginedll.cpp )
set ( INCLUDES scenarioenginedll.hpp )

link_directories( ${OSG_LIBRARIES_PATH} )

add_library ( ${TARGET} SHARED ${SOURCES} ${INCLUDES} )


add_definitions(-D_CRT_SECURE_NO_WARNINGS)

if (USE_OSG)
  add_definitions(-DOSG_LIBRARY_STATIC -D_SCENARIO_VIEWER)
  set (viewer_libs 	PRIVATE ViewerBase PRIVATE ${OSG_LIBRARIES})
endif (USE_OSG)


target_link_libraries ( 
	${TARGET}
	PRIVATE ScenarioEngine
	PRIVATE RoadManager
	PRIVATE CommonMini
    PRIVATE PlayerBase
	PRIVATE ${TIME_LIB}	
    PRIVATE ${SOCK_LIB}
	${viewer_libs}
)

if (UNIX)
  install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
else()
  install ( TARGETS ${TARGET} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}")
endif (UNIX)

