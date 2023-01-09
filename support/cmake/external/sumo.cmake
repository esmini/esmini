include_guard()

# ############################### Setting sumo libraries ##############################################################

macro(set_sumo_libs)

    if(APPLE)
        set(SUMO_LIBRARIES
            ${EXTERNALS_SUMO_LIBRARY_PATH}/liblibsumostatic.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libnetload.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libtraciserver.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/liblibsumostatic.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_vehicle.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_distribution.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_shapes.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_options.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_xml.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_geom.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_common.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_iodevices.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_traction_wire.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_emissions.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_engine.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_lcmodels.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_devices.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_trigger.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_output.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_transportables.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_actions.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_traffic_lights.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libmesosim.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libforeign_phemlight.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libforeign_tcpip.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_cfmodels.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libzlibstatic.a
            ${EXTERNALS_SUMO_LIBRARY_PATH}/libxerces-c_3.a
            "-framework CoreServices")
    elseif(LINUX)
        set(SUMO_LIBRARIES
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/liblibsumostatic.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libnetload.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libtraciserver.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/liblibsumostatic.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_vehicle.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_distribution.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_shapes.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_options.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_xml.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_geom.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_common.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_iodevices.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_traction_wire.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_emissions.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_engine.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_lcmodels.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_devices.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_trigger.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_output.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_transportables.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_actions.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_traffic_lights.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libmesosim.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libforeign_phemlight.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libforeign_tcpip.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_cfmodels.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libzlibstatic.a
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libxerces-c_3.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/liblibsumostaticd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libnetloadd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libtraciserverd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/liblibsumostaticd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_vehicled.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_distributiond.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_shapesd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_optionsd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_xmld.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_geomd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_commond.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_iodevicesd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_traction_wired.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libutils_emissionsd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_engined.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_lcmodelsd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_devicesd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_triggerd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_outputd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_transportablesd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_actionsd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_traffic_lightsd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosimd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libmesosimd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libforeign_phemlightd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libforeign_tcpipd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libmicrosim_cfmodelsd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libzlibstaticd.a
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libxerces-c_3D.a)
    elseif(MSVC)
        set(SUMO_LIBRARIES
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libsumostatic.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/netload.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/traciserver.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/libsumostatic.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_vehicle.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_distribution.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_shapes.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_options.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_xml.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_geom.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_common.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_iodevices.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_traction_wire.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_emissions.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_engine.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_lcmodels.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_devices.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_trigger.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_output.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_transportables.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_actions.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_traffic_lights.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/mesosim.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/foreign_phemlight.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/foreign_tcpip.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_cfmodels.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/zlibstatic.lib
            optimized ${EXTERNALS_SUMO_LIBRARY_PATH}/xerces-c_3.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libsumostaticd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/netloadd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/traciserverd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/libsumostaticd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_vehicled.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_distributiond.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_shapesd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_optionsd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_xmld.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_geomd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_commond.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_iodevicesd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_traction_wired.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/utils_emissionsd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_engined.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_lcmodelsd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_devicesd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_triggerd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_outputd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_transportablesd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_actionsd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_traffic_lightsd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/microsimd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/mesosimd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/foreign_phemlightd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/foreign_tcpipd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/microsim_cfmodelsd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/zlibstaticd.lib
            debug ${EXTERNALS_SUMO_LIBRARY_PATH}/xerces-c_3D.lib)
    endif()
endmacro()
