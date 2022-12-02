include_guard()

# ############################### Setting gtest libraries ##############################################################

macro(set_gtest_libs)

    if(APPLE)
        message("Googletest not yet supported for macOS")
    elseif(LINUX)
        set(GTEST_LIBRARIES
            optimized
            gmock
            debug
            gmockd
            optimized
            gmock_main
            debug
            gmock_maind
            optimized
            gtest
            debug
            gtestd
            optimized
            gtest_main
            debug
            gtest_maind)
    elseif(MSVC)
        set(GTEST_LIBRARIES
            optimized
            gmock.lib
            debug
            gmockd.lib
            optimized
            gmock_main.lib
            debug
            gmock_maind.lib
            optimized
            gtest.lib
            debug
            gtestd.lib
            optimized
            gtest_main.lib
            debug
            gtest_maind.lib)
    elseif(MINGW)
        message("Googletest not yet supported for MinGW")
    endif()

endmacro()
