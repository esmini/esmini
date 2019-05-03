
include_directories (
)

set ( SOURCES
  CommonMini.cpp
)

set ( INCLUDES
  CommonMini.hpp
)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

add_library ( CommonMini STATIC ${SOURCES} ${INCLUDES} )
