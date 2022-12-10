include_guard()

# ############################### Setting gtest libraries ##############################################################

macro(set_gtest_libs)

    if(APPLE)
        message("Googletest not yet supported for macOS")
    elseif(LINUX)
        if(CMAKE_BUILD_TYPE
           STREQUAL
           "Release")
            set(GTEST_LIBRARIES
                ${EXTERNALS_GTEST_LIBRARY_PATH}/libgmock.a
                ${EXTERNALS_GTEST_LIBRARY_PATH}/libgmock_main.a
                ${EXTERNALS_GTEST_LIBRARY_PATH}/libgtest.a
                ${EXTERNALS_GTEST_LIBRARY_PATH}/libgtest_main.a)
        elseif(
            CMAKE_BUILD_TYPE
            STREQUAL
            "Debug")
            set(GTEST_LIBRARIES
                ${EXTERNALS_GTEST_LIBRARY_PATH}/libgmockd.a
                ${EXTERNALS_GTEST_LIBRARY_PATH}/libgmock_maind.a
                ${EXTERNALS_GTEST_LIBRARY_PATH}/libgtestd.a
                ${EXTERNALS_GTEST_LIBRARY_PATH}/libgtest_maind.a)
        else()
            set(GTEST_LIBRARIES
                ${EXTERNALS_GTEST_LIBRARY_PATH}/libgmock.a
                ${EXTERNALS_GTEST_LIBRARY_PATH}/libgmock_main.a
                ${EXTERNALS_GTEST_LIBRARY_PATH}/libgtest.a
                ${EXTERNALS_GTEST_LIBRARY_PATH}/libgtest_main.a)
        endif()

    elseif(MSVC)
        if(CMAKE_BUILD_TYPE
           STREQUAL
           "Release")
            set(GTEST_LIBRARIES
                ${EXTERNALS_GTEST_LIBRARY_PATH}/gmock.lib
                ${EXTERNALS_GTEST_LIBRARY_PATH}/gmock_main.lib
                ${EXTERNALS_GTEST_LIBRARY_PATH}/gtest.lib
                ${EXTERNALS_GTEST_LIBRARY_PATH}/gtest_main.lib)
        elseif(
            CMAKE_BUILD_TYPE
            STREQUAL
            "Debug")
            set(GTEST_LIBRARIES
                ${EXTERNALS_GTEST_LIBRARY_PATH}/gmockd.lib
                ${EXTERNALS_GTEST_LIBRARY_PATH}/gmock_maind.lib
                ${EXTERNALS_GTEST_LIBRARY_PATH}/gtestd.lib
                ${EXTERNALS_GTEST_LIBRARY_PATH}/gtest_maind.lib)
        else()
            set(GTEST_LIBRARIES
                ${EXTERNALS_GTEST_LIBRARY_PATH}/gmock.lib
                ${EXTERNALS_GTEST_LIBRARY_PATH}/gmock_main.lib
                ${EXTERNALS_GTEST_LIBRARY_PATH}/gtest.lib
                ${EXTERNALS_GTEST_LIBRARY_PATH}/gtest_main.lib)
        endif()
    elseif(MINGW)
        message("Googletest not yet supported for MinGW")
    endif()

endmacro()
