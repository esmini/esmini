include_guard()

macro(set_esmini_cloud_links)

    set(MODELS_PACKAGE_URL
        https://esmini.asuscomm.com/AICLOUD779364751/models/models.7z)

    if(APPLE)
        set(OSG_PACKAGE_URL
            https://esmini.asuscomm.com/AICLOUD766065121/libs/OpenSceneGraph_mac.7z)
        set(OSI_PACKAGE_URL
            https://esmini.asuscomm.com/AICLOUD766065121/libs/osi_mac.7z)
        set(SUMO_PACKAGE_URL
            https://esmini.asuscomm.com/AICLOUD766065121/libs/sumo_mac.7z)
    elseif(LINUX)
        set(OSG_PACKAGE_URL
            https://esmini.asuscomm.com/AICLOUD766065121/libs/osg_linux_glibc_2_31_gcc_7_5_0.7z)
        set(OSI_PACKAGE_URL
            https://esmini.asuscomm.com/AICLOUD766065121/libs/osi_linux.7z)
        set(SUMO_PACKAGE_URL
            https://esmini.asuscomm.com/AICLOUD766065121/libs/sumo_linux.7z)
        set(GTEST_PACKAGE_URL
            https://esmini.asuscomm.com/AICLOUD766065121/libs/googletest_linux.7z)
    elseif(MSVC)
        set(OSG_PACKAGE_URL
            https://esmini.asuscomm.com/AICLOUD766065121/libs/OpenSceneGraph_v10.7z)
        set(OSI_PACKAGE_URL
            https://esmini.asuscomm.com/AICLOUD766065121/libs/osi_v10.7z)
        set(SUMO_PACKAGE_URL
            https://esmini.asuscomm.com/AICLOUD766065121/libs/sumo_v10.7z)
        set(GTEST_PACKAGE_URL
            https://esmini.asuscomm.com/AICLOUD766065121/libs/googletest_v10.7z)
    elseif(MINGW)
        message("MinGW, enforcing slimmed esmini")
    else()
        message("Unsupported configuration")
    endif()

endmacro()
