include_guard()

# ############################### Setting bullet libraries ###########################################################

macro(set_bullet_libs)

    if(APPLE)
        set(BULLET_LIBRARIES "")
    elseif(LINUX)
        set(BULLET_LIBRARIES "")
    elseif(MSVC)
        set(BULLET_LIBRARIES
            optimized ${EXTERNALS_BULLET_LIBRARY_PATH}/BulletCollision.lib
            optimized ${EXTERNALS_BULLET_LIBRARY_PATH}/BulletDynamics.lib
            optimized ${EXTERNALS_BULLET_LIBRARY_PATH}/LinearMath.lib
            debug ${EXTERNALS_BULLET_LIBRARY_PATH}/BulletCollisiond.lib
            debug ${EXTERNALS_BULLET_LIBRARY_PATH}/BulletDynamicsd.lib
            debug ${EXTERNALS_BULLET_LIBRARY_PATH}/LinearMathd.lib)
    endif()
endmacro()

