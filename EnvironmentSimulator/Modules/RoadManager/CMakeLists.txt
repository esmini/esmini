
include_directories (
  ${PUGIXML_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}
  ${ROADMANAGER_INCLUDE_DIR}  
)

set ( SOURCES
  RoadManager.cpp
  odrSpiral.cpp
  OpenDrive.cpp
  Geometry.cpp
  OSI.cpp
  Polynomial.cpp
  Position.cpp
  Road.cpp
)

set ( SRC_ADDITIONAL ../../../externals/pugixml/pugixml.cpp)

SOURCE_GROUP("External Libraries" FILES ${SRC_ADDITIONAL})

set ( INCLUDES
  RoadManager.hpp
  odrSpiral.h
  OpenDrive.hpp
  Geometry.hpp
  OSI.hpp
  Polynomial.hpp
  Position.hpp
  Road.hpp
)

if(MSVC)
	add_definitions("/wd4482")
endif()

add_library ( RoadManager STATIC ${SOURCES} ${SRC_ADDITIONAL} ${INCLUDES} )

target_link_libraries ( RoadManager CommonMini )