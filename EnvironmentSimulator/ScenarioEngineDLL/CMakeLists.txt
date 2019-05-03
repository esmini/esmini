
include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${ROADMANAGER_INCLUDE_DIR}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${OSG_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}  
)

set ( SOURCES scenarioenginedll.cpp )
set ( INCLUDES scenarioenginedll.hpp )

link_directories( ${OSG_LIBRARIES_PATH} )

add_library ( ScenarioEngineDLL SHARED ${SOURCES} ${INCLUDES} )


add_definitions(-D_CRT_SECURE_NO_WARNINGS)

if (USE_OSG)
  add_definitions(-DOSG_LIBRARY_STATIC -D_SCENARIO_VIEWER)
  set (viewer_libs 	PRIVATE ViewerBase PRIVATE ${OSG_LIBRARIES})
endif (USE_OSG)


target_link_libraries ( 
	ScenarioEngineDLL
	PRIVATE ScenarioEngine
	PRIVATE RoadManager
	PRIVATE CommonMini
	PRIVATE ${TIME_LIB}	
	${viewer_libs}
)


install ( TARGETS ScenarioEngineDLL CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}/Release")
install ( TARGETS ScenarioEngineDLL CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}/Debug")
