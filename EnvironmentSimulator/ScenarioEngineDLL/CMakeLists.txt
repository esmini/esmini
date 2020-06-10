
include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${ROADMANAGER_INCLUDE_DIR}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${PLAYER_BASE_INCLUDE_DIR}
  ${OSG_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR} 
  ${OSI_DIR}/include 
)

set(TARGET ScenarioEngineDLL)

set ( SOURCES scenarioenginedll.cpp )
set ( INCLUDES scenarioenginedll.hpp )

link_directories( ${OSG_LIBRARIES_PATH})

add_library ( ${TARGET} SHARED ${SOURCES} ${INCLUDES} )


add_definitions(-D_CRT_SECURE_NO_WARNINGS)

if (USE_OSG)
  add_definitions(-DOSG_LIBRARY_STATIC -D_SCENARIO_VIEWER)
  set (viewer_libs 	PRIVATE ViewerBase PRIVATE ${OSG_LIBRARIES})
endif (USE_OSG)

add_library(osi_lib SHARED IMPORTED GLOBAL)
set_property(TARGET osi_lib PROPERTY IMPORTED_LOCATION "${OSI_DIR}/lib/osi_proto_combined.a")

target_link_libraries ( 
	${TARGET}
  PRIVATE PlayerBase
	PRIVATE ScenarioEngine
	PRIVATE RoadManager
	PRIVATE CommonMini
	PRIVATE ${TIME_LIB}	
  PRIVATE ${SOCK_LIB}
  ${viewer_libs}
  PRIVATE osi_lib
)

if (UNIX)
  install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
else()
  install ( TARGETS ${TARGET} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}")
endif (UNIX)

