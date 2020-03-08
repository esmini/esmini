
include_directories (
  ${SCENARIOENGINE_DLL_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}  
)

set (TARGET ScenarioViewer)

set ( SOURCES
  main.cpp
)

set ( INCLUDES
)

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

add_definitions(-D_SCENARIO_VIEWER)

target_link_libraries ( 
	${TARGET}
	ScenarioEngineDLL
	CommonMini
	${TIME_LIB}
)

if (UNIX)
  install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
else()
  install ( TARGETS ${TARGET} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}")
endif (UNIX)
