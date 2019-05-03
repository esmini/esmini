
include_directories (
  ${ROADMANAGER_INCLUDE_DIR}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${OSG_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}
)

set ( SOURCES
	main.cpp
)

link_directories( ${OSG_LIBRARIES_PATH} )

add_executable ( OpenDriveViewer ${SOURCES} )

add_definitions(-DOSG_LIBRARY_STATIC)

target_link_libraries ( 
	OpenDriveViewer
	RoadManager
	ViewerBase
	CommonMini
	${OSG_LIBRARIES}
	${TIME_LIB}	
)

install ( TARGETS OpenDriveViewer CONFIGURATIONS release DESTINATION "${INSTALL_DIRECTORY}/Release")
install ( TARGETS OpenDriveViewer CONFIGURATIONS debug DESTINATION "${INSTALL_DIRECTORY}/Debug")
