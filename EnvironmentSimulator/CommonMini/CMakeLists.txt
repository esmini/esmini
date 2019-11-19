
set ( SOURCES
  CommonMini.cpp
  version.cpp
)

set ( INCLUDES
  CommonMini.hpp
)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

add_library ( CommonMini STATIC ${SOURCES} ${INCLUDES} )


# Add a custom command that produces version.hpp, plus
# a dummy output that's not actually produced, in order
# to force version.cmake to always be re-run before the build
ADD_CUSTOM_COMMAND(
    OUTPUT  ${CMAKE_CURRENT_BINARY_DIR}/_version.cpp
            ${CMAKE_CURRENT_BINARY_DIR}/version.cpp
    COMMAND ${CMAKE_COMMAND} -P
            ${CMAKE_CURRENT_SOURCE_DIR}/version.cmake)
