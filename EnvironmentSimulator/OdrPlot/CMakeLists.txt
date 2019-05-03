
include_directories (
  ${ROADMANAGER_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}  
)

set ( SOURCES
  main.cpp
)

set ( INCLUDES
)

add_executable ( OdrPlot ${SOURCES} ${INCLUDES} )

target_link_libraries ( 
	OdrPlot
	CommonMini	
	RoadManager
	${TIME_LIB}
)

install ( TARGETS OdrPlot CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}/Release")
install ( TARGETS OdrPlot CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}/Debug")
