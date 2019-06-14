
include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${ROADMANAGER_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}  
)

set ( SOURCES roadmanagerdll.cpp )
set ( INCLUDES roadmanagerdll.hpp )

add_library ( RoadManagerDLL SHARED ${SOURCES} ${INCLUDES} )

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

target_link_libraries ( 
	RoadManagerDLL
	PRIVATE RoadManager
	PRIVATE CommonMini
)

install ( TARGETS RoadManagerDLL CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}/Release")
install ( TARGETS RoadManagerDLL CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}/Debug")
