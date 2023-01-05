include_guard()

# ############################### Setting gtest libraries ##############################################################

macro(set_gtest_libs)

    if(APPLE)
        message("Googletest not yet supported for macOS")
    elseif(LINUX)
        set(GTEST_LIBRARIES
            optimized ${EXTERNALS_GTEST_LIBRARY_PATH}/libgmock.a
            optimized ${EXTERNALS_GTEST_LIBRARY_PATH}/libgmock_main.a
            optimized ${EXTERNALS_GTEST_LIBRARY_PATH}/libgtest.a
            optimized ${EXTERNALS_GTEST_LIBRARY_PATH}/libgtest_main.a
            debug ${EXTERNALS_GTEST_LIBRARY_PATH}/libgmockd.a
            debug ${EXTERNALS_GTEST_LIBRARY_PATH}/libgmock_maind.a
            debug ${EXTERNALS_GTEST_LIBRARY_PATH}/libgtestd.a
            debug ${EXTERNALS_GTEST_LIBRARY_PATH}/libgtest_maind.a)
    elseif(MSVC)
        set(GTEST_LIBRARIES
            optimized ${EXTERNALS_GTEST_LIBRARY_PATH}/gmock.lib
            optimized ${EXTERNALS_GTEST_LIBRARY_PATH}/gmock_main.lib
            optimized ${EXTERNALS_GTEST_LIBRARY_PATH}/gtest.lib
            optimized ${EXTERNALS_GTEST_LIBRARY_PATH}/gtest_main.lib
            debug ${EXTERNALS_GTEST_LIBRARY_PATH}/gmockd.lib
            debug ${EXTERNALS_GTEST_LIBRARY_PATH}/gmock_maind.lib
            debug ${EXTERNALS_GTEST_LIBRARY_PATH}/gtestd.lib
            debug ${EXTERNALS_GTEST_LIBRARY_PATH}/gtest_maind.lib)
    elseif(MINGW)
        message("Googletest not yet supported for MinGW")
    endif()

endmacro()

# ############################### Creating gtest interface library #####################################################

macro(add_gtest_library)

    set_gtest_libs()

    add_library(
        GTEST
        INTERFACE)

    target_include_directories(
        GTEST
        INTERFACE ${EXTERNALS_GOOGLETEST_INCLUDES})

    target_link_libraries(
        GTEST
        INTERFACE ${GTEST_LIBRARIES})

endmacro()
