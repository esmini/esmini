
include_directories (
  ${SCENARIO_ENGINE_PATH}/SourceFiles
  ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
  ${ROAD_MANAGER_PATH}
  ${VIEWER_BASE_PATH}
  ${PLAYER_BASE_PATH}
  ${EXTERNALS_OSG_INCLUDES}
  ${COMMON_MINI_PATH}
  ${EXTERNALS_OSI_INCLUDES}
  ${EXTERNALS_PUGIXML_PATH}
  ${CONTROLLERS_PATH}
)

set(TARGET esminiLib)

set ( SOURCES esminiLib.cpp ../../Modules/Controllers/vehicle.cpp )
set ( INCLUDES esminiLib.hpp ../../Modules/Controllers/vehicle.hpp )


add_library ( ${TARGET} SHARED ${SOURCES} ${INCLUDES} )

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

target_link_libraries(${TARGET} PRIVATE project_options)

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
  PRIVATE CommonMini
  PRIVATE Controllers
  PRIVATE RoadManager
  PRIVATE ${osi_libs}
  PRIVATE ${sumo_libs}
  PRIVATE ${TIME_LIB}
  PRIVATE ${SOCK_LIB}
  ${viewer_libs}
)

install ( TARGETS ${TARGET} DESTINATION "${INSTALL_PATH}")
if (WIN32)
  install ( TARGETS ${TARGET} RUNTIME DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
else ()
  install ( TARGETS ${TARGET} LIBRARY DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
endif ()
