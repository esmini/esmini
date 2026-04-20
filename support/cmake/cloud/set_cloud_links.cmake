include_guard()

macro(set_cloud_links)

    set(MODELS_PACKAGE_URL
        https://drive.usercontent.google.com/download?id=1z_NdozxK7P7q2CiI6hIJdgOzSNEjF5Nb&export=download&authuser=0&confirm=t;
        https://www.dropbox.com/scl/fi/3jwz3ie6xkgpga33cbyju/models_with_lights.7z?rlkey=9obptnofidldoagn4154iyntl&st=zxoupatb&dl=1;
        https://esmini.asuscomm.com/AICLOUD984833539/models/models_with_lights.7z)

    if(APPLE)
        set(OSG_PACKAGE_URL
            https://drive.usercontent.google.com/download?id=16fHAwCctVtaheLB5oqipuxSZvsIW_eUu&export=download&authuser=0&confirm=t;
            https://esmini.asuscomm.com/AICLOUD972108097/libs/osg_mac_macos_13_png.7z;
            https://www.dropbox.com/scl/fi/pp8ml2pdhi26eyrzy91ki/osg_mac_macos_13_png.7z?rlkey=ejzcxlbrvl8gnqwih4ywsgfdo&st=i7799iip&dl=1)
        if(DEFINED OSI_RELEASE_TAG AND NOT OSI_RELEASE_TAG STREQUAL "")
            set(OSI_PACKAGE_URL
                https://github.com/slundel6/esmini-sandbox/releases/download/${OSI_RELEASE_TAG}/osi-macos.tar.xz)
        endif()
        set(SUMO_PACKAGE_URL
            https://drive.usercontent.google.com/download?id=1FAve0-MlJPv6lUZy0HvriZI7xstLAzvX&export=download&authuser=0&confirm=t;
            https://esmini.asuscomm.com/AICLOUD972108097/libs/sumo_mac.7z;
            https://www.dropbox.com/s/0x8kwztk7nmacs1/sumo_mac.7z?dl=1)
        set(IMPLOT_PACKAGE_URL
            https://drive.usercontent.google.com/download?id=12RycamdsewKIhwg63-AKr33Xr00xSyZ8&export=download&authuser=0&confirm=t;
            https://esmini.asuscomm.com/AICLOUD972108097/libs/implot_mac_14.7z;
            https://www.dropbox.com/scl/fi/s1c4ytqqaxk77l5wf3h3l/implot_mac_macos_14.7z?rlkey=xligo4mju7a8f0dtm4mjeliuw&st=ca2lm7jz&dl=1)
    elseif(LINUX)
        set(OSG_PACKAGE_URL
            https://drive.usercontent.google.com/download?id=1NQlZBTFBWHpu-ImEjJN6CIxvwuinelzL&export=download&authuser=0&confirm=t;
            https://esmini.asuscomm.com/AICLOUD972108097/libs/osg_linux_glibc_2_31_gcc_7_5_0_png.7z;
            https://www.dropbox.com/scl/fi/0rq5uefqmvfva0idsgsjn/osg_linux_glibc_2_31_gcc_7_5_0_png.7z?rlkey=x549jvuwljvgvivhbi0ysj5xj&st=qz6fll2i&dl=1)
        if(DEFINED OSI_RELEASE_TAG AND NOT OSI_RELEASE_TAG STREQUAL "")
            set(OSI_PACKAGE_URL
                https://github.com/slundel6/esmini-sandbox/releases/download/${OSI_RELEASE_TAG}/osi-linux.tar.xz)
        endif()
        set(SUMO_PACKAGE_URL
            https://drive.usercontent.google.com/download?id=1m4znxNIXapP0D-l21oIm2l7L5ti-JbZH&export=download&authuser=0&confirm=t;
            https://esmini.asuscomm.com/AICLOUD972108097/libs/sumo_linux.7z;
            https://dl.dropboxusercontent.com/s/gfwtqd3gf76f86a/sumo_linux.7z?dl=1)
        set(GTEST_PACKAGE_URL
            https://drive.usercontent.google.com/download?id=1Hyr9eJX2GmgpYwZhx14xOoXlZ2j-FY_p&export=download&authuser=0&confirm=t;
            https://esmini.asuscomm.com/AICLOUD972108097/libs/googletest_linux.7z;
            https://dl.dropboxusercontent.com/s/si7jsjjsy5bpoym/googletest_linux.7z?dl=1)
        set(IMPLOT_PACKAGE_URL
            https://drive.usercontent.google.com/download?id=1MC6xvGbEG-GiEsNpwuj6hK_NDx-FPcp8&export=download&authuser=0&confirm=t;
            https://esmini.asuscomm.com/AICLOUD972108097/libs/implot_linux_glibc_2_31_gcc_9_4_0.7z;
            https://www.dropbox.com/scl/fi/s4lu9y6z8krlymuq4nxwe/implot_linux_glibc_2_31_gcc_9_4_0.7z?rlkey=1ui1h0azyuiap5ehogumcjrk1&st=82yzvb4f&dl=1)
    elseif(MSVC)
        set(OSG_PACKAGE_URL
            https://drive.usercontent.google.com/download?id=11QIsQkYbY0I8_YsivPH9r7_qqWBqbsfH&export=download&authuser=0&confirm=t;
            https://esmini.asuscomm.com/AICLOUD972108097/libs/osg_win_vs17_v142_png.7z;
            https://www.dropbox.com/scl/fi/074keimjeec2e6jul3rn0/osg_win_vs17_v142_png.7z?rlkey=1631xcd9muiaayasf5p8jh597&st=ufyogs3h&dl=1)
        if(DEFINED OSI_RELEASE_TAG AND NOT OSI_RELEASE_TAG STREQUAL "")
            set(OSI_PACKAGE_URL
                https://github.com/slundel6/esmini-sandbox/releases/download/${OSI_RELEASE_TAG}/osi-windows.tar.xz)
        endif()
        set(SUMO_PACKAGE_URL
            https://drive.usercontent.google.com/download?id=18PhbSLyvs0IGWTAY3YBoYzpVnMFPbOuR&export=download&authuser=0&confirm=t;
            https://esmini.asuscomm.com/AICLOUD972108097/libs/sumo_v10.7z;
            https://dl.dropboxusercontent.com/s/5jtpnnd61wonxuh/sumo_v10.7z?dl=1)
        set(GTEST_PACKAGE_URL
            https://drive.usercontent.google.com/download?id=1So-3gtrmEdW9RhEvVQisj1QFksHM_otU&export=download&authuser=0&confirm=t;
            https://esmini.asuscomm.com/AICLOUD972108097/libs/googletest_v10.7z;
            https://dl.dropboxusercontent.com/s/aaiehwzc6woqbc6/googletest_v10.7z?dl=1)
        set(IMPLOT_PACKAGE_URL
            https://drive.usercontent.google.com/download?id=1zIw35UCHhyRAEVbbUmOuUgqMOfWdj7zO&export=download&authuser=0&confirm=t;
            https://esmini.asuscomm.com/AICLOUD972108097/libs/implot_vs17_v142.7z;
            https://www.dropbox.com/scl/fi/cgf93rq4gwtbsv635f90y/implot_vs17_v142.7z?rlkey=0pwhdlyty69qrdcx7agzjdg9x&st=7omkcafx&dl=1)
    elseif(MINGW)
        message("MinGW, enforcing slimmed esmini")
    else()
        message("Unsupported configuration")
    endif()

endmacro()
