# ############################### Setting targets ####################################################################

set(TARGET
    CommonMini)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/enable_fpic.cmake)

# ############################### Setting target files ###############################################################

set(SOURCES
    CommonMini.cpp
    UDP.cpp
    version.cpp
    logger.cpp)

set(INCLUDES
    CommonMini.hpp
    UDP.hpp
    logger.hpp)

# ############################### Creating library ###################################################################

add_library(
    ${TARGET}
    STATIC
    ${SOURCES}
    ${INCLUDES})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options)

disable_static_analysis(${TARGET})
disable_iwyu(${TARGET})
enable_fpic(${TARGET})

# ############################### OTHERS ###################################################################

include(${CMAKE_CURRENT_SOURCE_DIR}/version.cmake)

# Add a custom command that produces version.cpp, plus a dummy output that's not actually produced, in order to force version.cmake to always be
# re-run before the build
add_custom_command(
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/_version.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/version.cpp
    COMMAND ${CMAKE_COMMAND} -DESMINI_BUILD_VERSION=${ESMINI_BUILD_VERSION} -P ${CMAKE_CURRENT_SOURCE_DIR}/version.cmake)
