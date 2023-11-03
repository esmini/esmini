include_guard()

# ############################### Building given unittest target ###################################################

macro(
    unittest
    TARGET
    FILES)

    add_executable(
        ${TARGET}
        ${FILES})

    set(LIBRARIES
        ${ARGN}) # any number of optional libraries

    target_link_libraries(
        ${TARGET}
        PRIVATE project_options)

    target_include_directories(
        ${TARGET}
        PRIVATE ${SCENARIO_ENGINE_PATH}/SourceFiles
                ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
                ${ESMINI_LIB_PATH}
                ${ESMINI_RM_LIB_PATH}
                ${COMMON_MINI_PATH}
                ${VIEWER_BASE_PATH}
                ${PLAYER_BASE_PATH}
                ${CONTROLLERS_PATH}
                ${REPLAYER_PATH}
                ${DAT2CSV_PATH})

    target_include_directories(
        ${TARGET}
        SYSTEM
        PUBLIC ${ROAD_MANAGER_PATH}
               ${EXTERNALS_GOOGLETEST_INCLUDES}
               ${EXTERNALS_OSI_INCLUDES}
               ${EXTERNALS_OSG_INCLUDES}
               ${EXTERNALS_DIRENT_INCLUDES}
               ${EXTERNALS_PUGIXML_PATH})

    target_link_libraries(
        ${TARGET}
        PRIVATE ${LIBRARIES}
                ${GTEST_LIBRARIES})

    disable_static_analysis(${TARGET})
    disable_iwyu(${TARGET})

    set_folder(
        ${TARGET}
        Unittest)

    add_test(
        NAME ${TARGET}
        COMMAND ${TARGET})

endmacro()
