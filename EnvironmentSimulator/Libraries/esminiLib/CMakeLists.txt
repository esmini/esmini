
include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${ROADMANAGER_INCLUDE_DIR}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${PLAYER_BASE_INCLUDE_DIR}
  ${OSG_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR} 
  ${OSI_INCLUDE_DIR}
  ${CONTROLLERS_INCLUDE_DIR}  
)

set(TARGET esminiLib)

set ( SOURCES esminiLib.cpp ../../Modules/Controllers/vehicle.cpp )
set ( INCLUDES esminiLib.hpp ../../Modules/Controllers/vehicle.hpp )


add_library ( ${TARGET} SHARED ${SOURCES} ${INCLUDES} )

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

if (USE_OSG)
  add_definitions(-DOSG_LIBRARY_STATIC)
  set (viewer_libs 	PRIVATE ViewerBase PRIVATE ${OSG_LIBRARIES})
endif (USE_OSG)

if (USE_SUMO)
  set (sumo_libs ${SUMO_LIBRARIES})
endif (USE_SUMO)

if (USE_OSI)
  set (osi_libs ${OSI_LIBRARIES})
endif (USE_OSI)

target_link_libraries ( 
	${TARGET}
    PRIVATE PlayerBase
	PRIVATE ScenarioEngine
	PRIVATE RoadManager
	PRIVATE CommonMini
    PRIVATE Controllers
    PRIVATE ${osi_libs}
    PRIVATE ${sumo_libs}
	PRIVATE ${TIME_LIB}	
    PRIVATE ${SOCK_LIB}
    ${viewer_libs}
)

if (UNIX)
  install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} LIBRARY DESTINATION "${INSTALL_DIRECTORY_CODE_EXAMPLES}")
else()
  install ( TARGETS ${TARGET} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} RUNTIME CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY_CODE_EXAMPLES}")
  install ( TARGETS ${TARGET} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} RUNTIME CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY_CODE_EXAMPLES}")
endif (UNIX)

