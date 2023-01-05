include_guard()

# ############################### Setting osg libraries ##############################################################

macro(set_osg_libs)

    set(OSG_VERSION
        "osg161")

    if(APPLE)
        set(OSG_LIBRARIES
            ${EXTERNALS_OSG_LIBRARY_PATH}/libosg.a
            ${EXTERNALS_OSG_LIBRARY_PATH}/libosgViewer.a
            ${EXTERNALS_OSG_LIBRARY_PATH}/libosgDB.a
            ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/libosgdb_serializers_osgsim.a
            ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/libosgdb_serializers_osg.a
            ${EXTERNALS_OSG_LIBRARY_PATH}/libosgGA.a
            ${EXTERNALS_OSG_LIBRARY_PATH}/libosgText.a
            ${EXTERNALS_OSG_LIBRARY_PATH}/libosgSim.a
            ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/libosgdb_osg.a
            ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/libosgdb_jpeg.a
            ${EXTERNALS_OSG_LIBRARY_PATH}/libosgUtil.a
            ${EXTERNALS_OSG_LIBRARY_PATH}/libosgAnimation.a
            ${EXTERNALS_OSG_LIBRARY_PATH}/libosg.a
            ${EXTERNALS_OSG_LIBRARY_PATH}/libOpenThreads.a
            ${EXTERNALS_OSG_LIBRARY_PATH}/libjpeg.a
            "-framework OpenGL"
            "-framework Cocoa"
            dl
            z)
    elseif(LINUX)
        set(OSG_LIBRARIES
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/libosg.a
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/libosgViewer.a
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/libosgDB.a
            optimized ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/libosgdb_serializers_osgsim.a
            optimized ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/libosgdb_serializers_osg.a
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/libosgGA.a
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/libosgText.a
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/libosgSim.a
            optimized ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/libosgdb_osg.a
            optimized ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/libosgdb_jpeg.a
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/libosgUtil.a
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/libosgAnimation.a
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/libosg.a
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/libOpenThreads.a
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/libjpeg.a
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/libosgd.a
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/libosgViewerd.a
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/libosgDBd.a
            debug ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/libosgdb_serializers_osgsimd.a
            debug ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/libosgdb_serializers_osgd.a
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/libosgGAd.a
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/libosgTextd.a
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/libosgSimd.a
            debug ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/libosgdb_osgd.a
            debug ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/libosgdb_jpegd.a
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/libosgUtild.a
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/libosgAnimationd.a
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/libosgd.a
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/libOpenThreadsd.a
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/libjpegd.a
            GL
            X11
            Xrandr
            dl
            z
            Xinerama
            fontconfig)
    elseif(MSVC)
        set(OSG_LIBRARIES
            opengl32.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/zlibstatic.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/jpeg.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osg.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgAnimation.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgDB.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgGA.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgViewer.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgText.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgUtil.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgSim.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/ot21-OpenThreads.lib
            optimized ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_serializers_osgsim.lib
            optimized ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_serializers_osg.lib
            optimized ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_osg.lib
            optimized ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_jpeg.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/zlibstaticd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/jpegd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgAnimationd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgDBd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgGAd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgViewerd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgTextd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgUtild.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgSimd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/ot21-OpenThreadsd.lib
            debug ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_serializers_osgsimd.lib
            debug ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_serializers_osgd.lib
            debug ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_osgd.lib
            debug ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_jpegd.lib)
    elseif(MINGW)
        set(OSG_LIBRARIES
            opengl32.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/zlibstatic.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/jpeg.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osg.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgAnimation.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgDB.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgGA.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgViewer.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgText.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgUtil.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgSim.lib
            optimized ${EXTERNALS_OSG_LIBRARY_PATH}/ot21-OpenThreads.lib
            optimized ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_serializers_osgsim.lib
            optimized ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_serializers_osg.lib
            optimized ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_osg.lib
            optimized ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_jpeg.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/zlibstaticd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/jpegd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgAnimationd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgDBd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgGAd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgViewerd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgTextd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgUtild.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/${OSG_VERSION}-osgSimd.lib
            debug ${EXTERNALS_OSG_LIBRARY_PATH}/ot21-OpenThreadsd.lib
            debug ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_serializers_osgsimd.lib
            debug ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_serializers_osgd.lib
            debug ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_osgd.lib
            debug ${EXTERNALS_OSG_PLUGINS_LIBRARY_PATH}/osgdb_jpegd.lib)
    endif()
endmacro()

# ############################### Creating osg interface library #####################################################

macro(add_osg_library)

    set_osg_libs()

    add_library(
        OSG
        INTERFACE)

    target_include_directories(
        OSG
        INTERFACE ${EXTERNALS_OSG_INCLUDES})

    target_link_libraries(
        OSG
        INTERFACE ${OSG_LIBRARIES})

endmacro()
