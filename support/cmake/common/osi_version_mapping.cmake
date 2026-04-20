function(set_osi_release_tag_from_version)
    set(_supported_osi_versions
        "3.5.0"
        "3.8.0")

    if(NOT OSI_VERSION IN_LIST _supported_osi_versions)
        string(JOIN ", " _versions_print_str ${_supported_osi_versions})
        message(FATAL_ERROR "Invalid OSI version '${OSI_VERSION}', Supported versions: ${_versions_print_str}")
    endif()

    if("${OSI_VERSION}" STREQUAL "3.5.0")
        set(OSI_RELEASE_TAG "v3.5.0_2" PARENT_SCOPE)
    elseif("${OSI_VERSION}" STREQUAL "3.8.0")
        set(OSI_RELEASE_TAG "v3.8.0_3" PARENT_SCOPE)
    endif()
endfunction()