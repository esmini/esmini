
include_directories (
  ${PUGIXML_INCLUDE_DIR}/pugixml
  ${COMMON_MINI_PATH}
  ${ROAD_MANAGER_PATH}
)

set ( SOURCES
  RoadManager.cpp
  odrSpiral.cpp
  LaneIndependentRouter.cpp
)

set ( SRC_ADDITIONAL ../../../externals/pugixml/pugixml.cpp)

SOURCE_GROUP("External Libraries" FILES ${SRC_ADDITIONAL})

set ( INCLUDES
  RoadManager.hpp
  odrSpiral.h
  LaneIndependentRouter.hpp
)

if(MSVC)
	add_definitions("/wd4482")
endif()

add_library ( RoadManager STATIC ${SOURCES} ${SRC_ADDITIONAL} ${INCLUDES} )

target_link_libraries ( RoadManager CommonMini project_options)