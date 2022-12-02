include_guard()

macro(set_dropbox_cloud_links)

    set(MODELS_PACKAGE_URL
        https://dl.dropboxusercontent.com/s/5gk8bvgzqiaaoco/models.7z?dl=0)

    if(APPLE)
        set(OSG_PACKAGE_URL
            https://www.dropbox.com/s/d0czj6b89p9jyvv/OpenSceneGraph_mac.7z?dl=1)
        set(OSI_PACKAGE_URL
            https://www.dropbox.com/s/m62v19gp0m73dte/osi_mac.7z?dl=1)
        set(SUMO_PACKAGE_URL
            https://www.dropbox.com/s/0x8kwztk7nmacs1/sumo_mac.7z?dl=1)
    elseif(LINUX)
        set(OSG_PACKAGE_URL
            https://www.dropbox.com/s/4ug0gmkgdavzyb4/osg_linux_glibc_2_31_gcc_7_5_0.7z?dl=1)
        set(OSI_PACKAGE_URL
            https://dl.dropboxusercontent.com/s/kwtdg0c1c8pawa1/osi_linux.7z?dl=1)
        set(SUMO_PACKAGE_URL
            https://dl.dropboxusercontent.com/s/gfwtqd3gf76f86a/sumo_linux.7z?dl=1)
        set(GTEST_PACKAGE_URL
            https://dl.dropboxusercontent.com/s/si7jsjjsy5bpoym/googletest_linux.7z?dl=1)
    elseif(MSVC)
        set(OSG_PACKAGE_URL
            https://dl.dropboxusercontent.com/s/e95hnoo782p40uc/OpenSceneGraph_v10.7z?dl=1)
        set(OSI_PACKAGE_URL
            https://dl.dropboxusercontent.com/s/an58ckp2qfx5069/osi_v10.7z?dl=1)
        set(SUMO_PACKAGE_URL
            https://dl.dropboxusercontent.com/s/5jtpnnd61wonxuh/sumo_v10.7z?dl=1)
        set(GTEST_PACKAGE_URL
            https://dl.dropboxusercontent.com/s/aaiehwzc6woqbc6/googletest_v10.7z?dl=1)
    elseif(MINGW)
        message("MinGW, enforcing slimmed esmini")
    else()
        message("Unsupported configuration")
    endif()

endmacro()
