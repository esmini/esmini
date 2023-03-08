# ############################### Setting targets ####################################################################

set(TARGET
    esmini-dyn)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

# ############################### Creating executable ################################################################

add_executable(
    ${TARGET}
    ${SOURCES})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options)

target_include_directories(
    ${TARGET}
    PRIVATE ${ESMINI_LIB_PATH}
            ${COMMON_MINI_PATH})

target_include_directories(
    ${TARGET}
    SYSTEM
    PUBLIC ${EXTERNALS_OSI_INCLUDES})

target_link_libraries(
    ${TARGET}
    PRIVATE esminiLib
    PRIVATE CommonMini
    PRIVATE ${TIME_LIB})

if(USE_OSI)
    target_link_libraries(
        ${TARGET}
        PRIVATE ${OSI_LIBRARIES})
endif()

# ############################### Install ############################################################################

install(
    TARGETS ${TARGET}
    DESTINATION "${INSTALL_PATH}")
