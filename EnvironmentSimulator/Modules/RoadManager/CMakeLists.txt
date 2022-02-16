
include_directories (
  ${PUGIXML_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}
  ${ROADMANAGER_INCLUDE_DIR}  
  "${CMAKE_CURRENT_SOURCE_DIR}/inc"
)

set ( SOURCES
  src/Arc.cpp
  src/Bridge.cpp
  src/Connection.cpp
  src/Controller.cpp
  src/Elevation.cpp
  src/Geometry.cpp
  src/GeoReference.cpp
  src/Junction.cpp
  src/Lane.cpp
  src/LaneLink.cpp
  src/LaneMaterial.cpp
  src/LaneOffset.cpp
  src/LaneRoadLaneConnection.cpp
  src/LaneRoadMark.cpp
  src/LaneRoadMarkType.cpp
  src/LaneRoadMarkTypeLine.cpp
  src/LaneSection.cpp
  src/LaneSpeed.cpp
  src/LaneWidth.cpp
  src/Line.cpp
  src/ObjectReference.cpp
  src/odrSpiral.cpp
  src/OpenDrive.cpp
  src/OSI.cpp
  src/Outline.cpp 
  src/ParamPoly3.cpp
  src/Poly3.cpp
  src/Polynomial.cpp
  #src/Position.cpp
  src/Repeat.cpp
  src/RMObject.cpp
  src/Road.cpp
  src/RoadLink.cpp
  # src/RoadManager.cpp
  src/RoadObject.cpp  
  src/Signal.cpp
  src/Spiral.cpp
  src/Userdata.cpp
)

set ( SRC_ADDITIONAL ../../../externals/pugixml/pugixml.cpp)

SOURCE_GROUP("External Libraries" FILES ${SRC_ADDITIONAL})

set ( INCLUDES
  inc/Arc.hpp
  inc/Bridge.hpp
  inc/Connection.hpp
  inc/Controller.hpp
  inc/Elevation.hpp
  inc/Geometry.hpp
  inc/GeoReference.hpp
  inc/Junction.hpp
  inc/Lane.hpp
  inc/LaneLink.hpp
  inc/LaneMaterial.hpp
  inc/LaneOffset.hpp
  inc/LaneRoadLaneConnection.hpp
  inc/LaneRoadMark.hpp
  inc/LaneRoadMarkType.hpp
  inc/LaneRoadMarkTypeLine.hpp
  inc/LaneSection.hpp
  inc/LaneSpeed.hpp
  inc/LaneWidth.hpp
  inc/Line.hpp
  inc/ObjectReference.hpp
  inc/odrSpiral.h
  inc/OpenDrive.hpp
  inc/OSI.hpp
  inc/Outline.hpp 
  inc/ParamPoly3.hpp
  inc/Poly3.hpp
  inc/Polynomial.hpp
  #inc/Position.hpp
  inc/Repeat.hpp
  inc/RMObject.hpp
  inc/Road.hpp
  inc/RoadLink.hpp
  # inc/RoadManager.hpp
  inc/RoadObject.hpp  
  inc/Signal.hpp
  inc/Spiral.hpp
  inc/StructsandDefines.hpp
  inc/Userdata.hpp
)

if(MSVC)
	add_definitions("/wd4482")
endif()

add_library ( RoadManager STATIC ${SOURCES} ${SRC_ADDITIONAL} ${INCLUDES} )

target_link_libraries ( RoadManager CommonMini )