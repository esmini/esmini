
include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${ROADMANAGER_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}
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

install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
if (WIN32)
  install ( TARGETS ${TARGET} RUNTIME DESTINATION "${INSTALL_DIRECTORY_CODE_EXAMPLES}")
else ()
  install ( TARGETS ${TARGET} LIBRARY DESTINATION "${INSTALL_DIRECTORY_CODE_EXAMPLES}")
endif ()

