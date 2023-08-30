include_guard()

# ############################### Setting implot libraries ######################################################

macro(set_implot_libs)

    if(APPLE)
        set(IMPLOT_LIBRARIES
            ${EXTERNALS_IMPLOT_LIBRARY_PATH}/libglfw3.a
            ${EXTERNALS_IMPLOT_LIBRARY_PATH}/libimplot.a
            "-framework OpenGL"
            "-framework Cocoa"
            "-framework IOKit")
    elseif(LINUX)
        set(IMPLOT_LIBRARIES
            optimized
            ${EXTERNALS_IMPLOT_LIBRARY_PATH}/libglfw3.a
            debug
            ${EXTERNALS_IMPLOT_LIBRARY_PATH}/libglfw3d.a
            optimized
            ${EXTERNALS_IMPLOT_LIBRARY_PATH}/libimplot.a
            debug
            ${EXTERNALS_IMPLOT_LIBRARY_PATH}/libimplot.a)
    elseif(MSVC)
        set(IMPLOT_LIBRARIES
            optimized
            ${EXTERNALS_IMPLOT_LIBRARY_PATH}/glfw3.lib
            debug
            ${EXTERNALS_IMPLOT_LIBRARY_PATH}/glfw3d.lib
            optimized
            ${EXTERNALS_IMPLOT_LIBRARY_PATH}/implot.lib
            debug
            ${EXTERNALS_IMPLOT_LIBRARY_PATH}/implot.lib)
    elseif(MINGW)
        message("implot not yet supported for MinGW")
    endif()

endmacro()
