
include_directories (
  ${SCENARIOENGINE_INCLUDE_DIRS}
  ${ROADMANAGER_INCLUDE_DIR}
  ${VIEWER_BASE_INCLUDE_DIR}
  ${PLAYER_BASE_INCLUDE_DIR}
  ${OSG_INCLUDE_DIR}
  ${COMMON_MINI_INCLUDE_DIR}
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/home/e9387011/sumo/build/cmake_modules/")
find_package(XercesC)
find_package(ZLIB)
find_package(Proj)
find_package(FOX)

include_directories("/home/e9387011/sumo/src/libsumo")
include_directories("/home/e9387011/sumo/src")
include_directories("/home/e9387011/sumo/build/cmake-build/src/")
include_directories("/home/e9387011/sumo/src/utils/geom")
include_directories(${XercesC_INCLUDE_DIRS})
include_directories(${Fox_INCLUDE_DIRS})
set(TARGET EgoSimulator)

set ( SOURCES
	main.cpp
	vehicle.cpp
)

set ( INCLUDES
	vehicle.hpp
)

add_executable ( ${TARGET} ${SOURCES} ${INCLUDES} )

if (USE_OSG)
  add_definitions(-DOSG_LIBRARY_STATIC -D_SCENARIO_VIEWER)
  set (viewer_libs ViewerBase ${OSG_LIBRARIES})
endif (USE_OSG)

target_link_libraries ( 
	${TARGET}
	PlayerBase    
	ScenarioEngine
	RoadManager
	CommonMini
    ${viewer_libs}
	${OSI_LIBRARIES}
	${TIME_LIB}	
    ${SOCK_LIB}
        /home/e9387011/sumo/build/cmake-build/src/libsumo/liblibsumostatic.a
/home/e9387011/sumo/build/cmake-build/src/traci-server/libtraciserver.a
/home/e9387011/sumo/build/cmake-build/src/netload/libnetload.a
/home/e9387011/sumo/build/cmake-build/src/microsim/cfmodels/libmicrosim_cfmodels.a
/home/e9387011/sumo/build/cmake-build/src/microsim/engine/libmicrosim_engine.a
/home/e9387011/sumo/build/cmake-build/src/microsim/lcmodels/libmicrosim_lcmodels.a
/home/e9387011/sumo/build/cmake-build/src/microsim/devices/libmicrosim_devices.a
/home/e9387011/sumo/build/cmake-build/src/microsim/trigger/libmicrosim_trigger.a
/home/e9387011/sumo/build/cmake-build/src/microsim/output/libmicrosim_output.a
/home/e9387011/sumo/build/cmake-build/src/microsim/transportables/libmicrosim_transportables.a
/home/e9387011/sumo/build/cmake-build/src/microsim/actions/libmicrosim_actions.a
/home/e9387011/sumo/build/cmake-build/src/microsim/traffic_lights/libmicrosim_traffic_lights.a
/home/e9387011/sumo/build/cmake-build/src/microsim/libmicrosim.a
/home/e9387011/sumo/build/cmake-build/src/mesosim/libmesosim.a
/home/e9387011/sumo/build/cmake-build/src/utils/emissions/libutils_emissions.a
/home/e9387011/sumo/build/cmake-build/src/foreign/PHEMlight/cpp/libforeign_phemlight.a
/home/e9387011/sumo/build/cmake-build/src/utils/vehicle/libutils_vehicle.a
/home/e9387011/sumo/build/cmake-build/src/utils/distribution/libutils_distribution.a
/home/e9387011/sumo/build/cmake-build/src/utils/shapes/libutils_shapes.a
/home/e9387011/sumo/build/cmake-build/src/utils/options/libutils_options.a
/home/e9387011/sumo/build/cmake-build/src/utils/xml/libutils_xml.a
/home/e9387011/sumo/build/cmake-build/src/utils/geom/libutils_geom.a
/home/e9387011/sumo/build/cmake-build/src/utils/common/libutils_common.a
/home/e9387011/sumo/build/cmake-build/src/utils/importio/libutils_importio.a
/home/e9387011/sumo/build/cmake-build/src/utils/iodevices/libutils_iodevices.a
/home/e9387011/sumo/build/cmake-build/src/utils/traction_wire/libutils_traction_wire.a
/home/e9387011/sumo/build/cmake-build/src/foreign/tcpip/libforeign_tcpip.a
${XercesC_LIBRARIES}
${ZLIB_LIBRARIES}
${PROJ_LIBRARY}
${FOX_LIBRARY}
/home/e9387011/sumo/build/cmake-build/src/traci-server/libtraciserver.a
/home/e9387011/sumo/build/cmake-build/src/libsumo/liblibsumostatic.a
/home/e9387011/sumo/build/cmake-build/src/microsim/cfmodels/libmicrosim_cfmodels.a
/home/e9387011/sumo/build/cmake-build/src/microsim/devices/libmicrosim_devices.a
/home/e9387011/sumo/build/cmake-build/src/microsim/engine/libmicrosim_engine.a
/home/e9387011/sumo/build/cmake-build/src/microsim/output/libmicrosim_output.a
/home/e9387011/sumo/build/cmake-build/src/microsim/transportables/libmicrosim_transportables.a
)

if (UNIX)
  install ( TARGETS ${TARGET} DESTINATION "${INSTALL_DIRECTORY}")
else()
  install ( TARGETS ${TARGET} CONFIGURATIONS Release DESTINATION "${INSTALL_DIRECTORY}")
  install ( TARGETS ${TARGET} CONFIGURATIONS Debug DESTINATION "${INSTALL_DIRECTORY}")
endif (UNIX)
