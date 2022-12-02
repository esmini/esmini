
include_directories (
  ${SCENARIO_ENGINE_PATH}/SourceFiles
  ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
  ${ROAD_MANAGER_PATH}
  ${COMMON_MINI_PATH}
)

set(TARGET esminiRMLib)

set ( SOURCES esminiRMLib.cpp )
set ( INCLUDES esminiRMLib.hpp )

add_library ( ${TARGET} SHARED ${SOURCES} ${INCLUDES} )

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

target_link_libraries(${TARGET} PRIVATE project_options)

target_link_libraries (
	${TARGET}
	PRIVATE RoadManager
	PRIVATE CommonMini
	PRIVATE ${TIME_LIB}
)

install ( TARGETS ${TARGET} DESTINATION "${INSTALL_PATH}")
if (WIN32)
  install ( TARGETS ${TARGET} RUNTIME DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
else ()
  install ( TARGETS ${TARGET} LIBRARY DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
endif ()

