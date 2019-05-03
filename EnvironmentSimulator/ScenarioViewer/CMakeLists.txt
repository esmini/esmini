
include_directories (
  ${SCENARIOENGINE_DLL_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}  
)

set ( SOURCES
  main.cpp
)

set ( INCLUDES
)

add_executable ( ScenarioViewer ${SOURCES} ${INCLUDES} )

add_definitions(-D_SCENARIO_VIEWER)

target_link_libraries ( 
	ScenarioViewer
	ScenarioEngineDLL
	CommonMini
	${TIME_LIB}
)


install ( TARGETS ScenarioViewer CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}/Release")
install ( TARGETS ScenarioViewer CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}/Debug")
