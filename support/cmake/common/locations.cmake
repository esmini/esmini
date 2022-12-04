include_guard()

# ############################### Setting internal paths ##############################################################

macro(set_project_internal_paths)

    set(CODE_EXAMPLES_BIN_PATH
        ${CMAKE_SOURCE_DIR}/code-examples-bin)
    set(ENVIRONMENT_SIMULATOR_PATH
        ${CMAKE_SOURCE_DIR}/EnvironmentSimulator)
    set(RESOURCES_PATH
        ${CMAKE_SOURCE_DIR}/resources)
    set(RUN_PATH
        ${CMAKE_SOURCE_DIR}/run)
    set(SUPPORT_PATH
        ${CMAKE_SOURCE_DIR}/support)
    set(INSTALL_PATH
        ${CMAKE_SOURCE_DIR}/bin)

    set(APPLICATIONS_PATH
        ${ENVIRONMENT_SIMULATOR_PATH}/Applications)
    set(CODE_EXAMPLES_PATH
        ${ENVIRONMENT_SIMULATOR_PATH}/code-examples)
    set(LIBRARIES_PATH
        ${ENVIRONMENT_SIMULATOR_PATH}/Libraries)
    set(MODULES_PATH
        ${ENVIRONMENT_SIMULATOR_PATH}/Modules)

    set(COMMON_MINI_PATH
        ${MODULES_PATH}/CommonMini)
    set(CONTROLLERS_PATH
        ${MODULES_PATH}/Controllers)
    set(PLAYER_BASE_PATH
        ${MODULES_PATH}/PlayerBase)
    set(ROAD_MANAGER_PATH
        ${MODULES_PATH}/RoadManager)
    set(SCENARIO_ENGINE_PATH
        ${MODULES_PATH}/ScenarioEngine)
    set(VIEWER_BASE_PATH
        ${MODULES_PATH}/ViewerBase)

    set(ESMINI_RM_LIB_PATH
        ${LIBRARIES_PATH}/esminiRMLib)
    set(ESMINI_LIB_PATH
        ${LIBRARIES_PATH}/esminiLib)

    set(REPLAYER_PATH
        ${APPLICATIONS_PATH}/replayer)

endmacro()

# ############################### Setting project external paths ###################################################

macro(set_project_external_paths)

    set(EXTERNALS_PATH
        ${CMAKE_SOURCE_DIR}/externals)
    set(EXTERNALS_DIRENT_PATH
        ${EXTERNALS_PATH}/dirent)
    set(EXTERNALS_EXPR_PATH
        ${EXTERNALS_PATH}/expr)
    set(EXTERNALS_GOOGLETEST_PATH
        ${EXTERNALS_PATH}/googletest)
    set(EXTERNALS_OSG_PATH
        ${EXTERNALS_PATH}/osg)
    set(EXTERNALS_OSI_PATH
        ${EXTERNALS_PATH}/osi)
    set(EXTERNALS_PUGIXML_PATH
        ${EXTERNALS_PATH}/pugixml)
    set(EXTERNALS_SUMO_PATH
        ${EXTERNALS_PATH}/sumo)
    set(MODELS_PATH
        ${RESOURCES_PATH}/models)

endmacro()

# ############################### Setting OS specific paths ########################################################

macro(set_project_os_specific_paths)

    if(APPLE)
        set(EXTERNALS_OSG_OS_SPECIFIC_PATH
            ${EXTERNALS_OSG_PATH}/mac)
        set(EXTERNALS_OSI_OS_SPECIFIC_PATH
            ${EXTERNALS_OSI_PATH}/mac)
        set(EXTERNALS_SUMO_OS_SPECIFIC_PATH
            ${EXTERNALS_SUMO_PATH}/mac)
        set(EXTERNALS_GOOGLETEST_OS_SPECIFIC_PATH
            ${EXTERNALS_GOOGLETEST_PATH}/mac)
        set(TIME_LIB
            "")
    elseif(LINUX)
        set(EXTERNALS_OSG_OS_SPECIFIC_PATH
            ${EXTERNALS_OSG_PATH}/linux)
        set(EXTERNALS_OSI_OS_SPECIFIC_PATH
            ${EXTERNALS_OSI_PATH}/linux)
        set(EXTERNALS_SUMO_OS_SPECIFIC_PATH
            ${EXTERNALS_SUMO_PATH}/linux)
        set(EXTERNALS_GOOGLETEST_OS_SPECIFIC_PATH
            ${EXTERNALS_GOOGLETEST_PATH}/linux)
        set(TIME_LIB
            "")
    elseif(MINGW)
        set(SOCK_LIB
            Ws2_32.lib)
        set(TIME_LIB
            winmm)
    elseif(MSVC)
        if("${CMAKE_VS_PLATFORM_NAME}"
           STREQUAL
           "x64")
            set(EXTERNALS_OSG_OS_SPECIFIC_PATH
                ${EXTERNALS_OSG_PATH}/v10)
            set(EXTERNALS_OSI_OS_SPECIFIC_PATH
                ${EXTERNALS_OSI_PATH}/v10)
            set(EXTERNALS_SUMO_OS_SPECIFIC_PATH
                ${EXTERNALS_SUMO_PATH}/v10)
            set(EXTERNALS_GOOGLETEST_OS_SPECIFIC_PATH
                ${EXTERNALS_GOOGLETEST_PATH}/v10)
            set(SOCK_LIB
                Ws2_32.lib)
            set(TIME_LIB
                "")
        else()
            message("Unsupported Windows configuration")
        endif()
    endif()

    if(MSVC)
        set(DIRENT_INCLUDE_DIR
            "${EXTERNALS_DIRENT_PATH}/win")
    else()
        set(DIRENT_INCLUDE_DIR
            "")
    endif()

endmacro()

# ############################### Setting project includes #########################################################

macro(set_project_includes)

    set(EXTERNALS_OSG_INCLUDES
        ${EXTERNALS_OSG_OS_SPECIFIC_PATH}/build/include
        ${EXTERNALS_OSG_OS_SPECIFIC_PATH}/include)
    set(EXTERNALS_OSI_INCLUDES
        ${EXTERNALS_OSI_OS_SPECIFIC_PATH}/include)
    set(EXTERNALS_SUMO_INCLUDES
        ${EXTERNALS_SUMO_OS_SPECIFIC_PATH}/include)
    set(EXTERNALS_GOOGLETEST_INCLUDES
        ${EXTERNALS_GOOGLETEST_OS_SPECIFIC_PATH}/include)

endmacro()
