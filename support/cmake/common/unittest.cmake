include_guard()

# ############################### Building given unittest target ###################################################

macro(
    unittest
    TARGET
    FILES
    LIBRARIES)

    add_executable(
        ${TARGET}
        ${FILES})

    target_link_libraries(
        ${TARGET}
        PRIVATE project_options)

    foreach(
        library
        ${LIBRARIES}
        ${ARGN})

        target_include_system_directories(
            ${TARGET}
            PRIVATE
            ${ROAD_MANAGER_PATH}
            ${EXTERNALS_GOOGLETEST_INCLUDES}
            ${EXTERNALS_OSI_INCLUDES}
            ${EXTERNALS_OSG_INCLUDES}
            ${EXTERNALS_DIRENT_INCLUDES}
            ${EXTERNALS_PUGIXML_PATH})

        if(${library}
           STREQUAL
           "OSG")

            if(USE_OSG)

                target_link_system_libraries(
                    ${TARGET}
                    PRIVATE
                    ${library})

            endif()

        elseif(
            ${library}
            STREQUAL
            "OSI")

            if(${TARGET}
               STREQUAL
               "ScenarioEngineDll_test")

                target_link_system_libraries(
                    ${TARGET}
                    PRIVATE
                    ${library})

            else()

                if(USE_OSI)

                    target_link_system_libraries(
                        ${TARGET}
                        PRIVATE
                        ${library})

                endif()

            endif()

        elseif(
            ${library}
            MATCHES
            ".*externals/sumo")

            if(USE_SUMO)

                target_link_system_libraries(
                    ${TARGET}
                    PRIVATE
                    ${library})

            endif()

        else()

            if(${library}
               STREQUAL
               "ViewerBase")

                if(USE_OSG)
                    target_link_libraries(
                        ${TARGET}
                        PRIVATE ${library})
                endif()

            else()

                target_link_libraries(
                    ${TARGET}
                    PRIVATE ${library})

            endif()

        endif()

    endforeach()

    target_link_system_libraries(
        ${TARGET}
        PRIVATE
        GTEST)

    disable_static_analysis(${TARGET})
    disable_iwyu(${TARGET})

    set_folder(${TARGET} Unittest)

    add_test(
        NAME ${TARGET}
        COMMAND ${TARGET})

endmacro()
