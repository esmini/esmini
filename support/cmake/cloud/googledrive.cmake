include_guard()

macro(set_googledrive_cloud_links)

    set(MODELS_PACKAGE_URL
        https://drive.google.com/u/1/uc?id=1c3cqRzwY41gWXbg0rmugQkL5I_5L6DH_&export=download)

    if(APPLE)
        set(OSG_PACKAGE_URL
            https://drive.google.com/u/1/uc?id=1mfn_vrcXBoFBekR_t8RXTWB4sD59JD7p&export=download)
        if(OSI_VERSION_3_3_1)
            set(OSI_PACKAGE_URL
                https://drive.google.com/u/1/uc?id=1UVzO8cPQaDU9KVn9v2v8Suj0uUw1dzYI&export=download)
        else()
            set(OSI_PACKAGE_URL
                https://drive.google.com/u/1/uc?id=1Owvoy9N_PGXe-s9GUgDxg-4h2AhlQ_Pi&export=download)
        endif()
        set(SUMO_PACKAGE_URL
            https://drive.google.com/u/1/uc?id=1FAve0-MlJPv6lUZy0HvriZI7xstLAzvX&export=download)
    elseif(LINUX)
        set(OSG_PACKAGE_URL
            https://drive.google.com/u/1/uc?id=1Ya1bLp_0-qqlhs67WAwbGW7l37wqP3o2&export=download)
        if(OSI_VERSION_3_3_1)
            set(OSI_PACKAGE_URL
                https://drive.google.com/u/1/uc?id=1Q8O9YciIC0BPEszIKtQ2UW9KcVRZS4iB&export=download)
        else()
            set(OSI_PACKAGE_URL
                https://drive.google.com/u/1/uc?id=1Xq_zQ5xaszYuN6G_dgSFHchXdnDxN6hS&export=download)
        endif()
        set(SUMO_PACKAGE_URL
            https://drive.google.com/u/1/uc?id=1m4znxNIXapP0D-l21oIm2l7L5ti-JbZH&export=download)
        set(GTEST_PACKAGE_URL
            https://drive.google.com/u/1/uc?id=1Hyr9eJX2GmgpYwZhx14xOoXlZ2j-FY_p&export=download)
        set(IMPLOT_PACKAGE_URL
			https://drive.google.com/u/1/uc?id=1YVetAhT7IBLTvkbDeyCDgZ3mjbExcxFY&export=download)
    elseif(MSVC)
        set(OSG_PACKAGE_URL
            https://drive.google.com/u/1/uc?id=1RTag0aUn_pJPK697j0-E72ABW10wZvOm&export=download)
        if(OSI_VERSION_3_3_1)
            set(OSI_PACKAGE_URL
                https://drive.google.com/u/1/uc?id=1pcQcVHUESOk2Wmi-zUA7uzdxxE6iwRJx&export=download)
        else()
            set(OSI_PACKAGE_URL
                https://drive.google.com/u/1/uc?id=1v490rg7s4kpuT28O5DI-fXc9BxyHqhVA&export=download)
        endif()
        set(SUMO_PACKAGE_URL
            https://drive.google.com/u/1/uc?id=18PhbSLyvs0IGWTAY3YBoYzpVnMFPbOuR&export=download)
        set(GTEST_PACKAGE_URL
            https://drive.google.com/u/1/uc?id=1So-3gtrmEdW9RhEvVQisj1QFksHM_otU&export=download)
        set(IMPLOT_PACKAGE_URL
			https://drive.google.com/u/1/uc?id=1k7dEITeu3vy6RTqKUw2o4FiTYljVuHkk&export=download)
    elseif(MINGW)
        message("MinGW, enforcing slimmed esmini")
    else()
        message("Unsupported configuration")
    endif()

endmacro()
