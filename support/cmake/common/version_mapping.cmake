macro(set_default_dependency_libs_versions)
    set(OSI_VERSION
        "3.5.0"
        CACHE STRING
            "osi version")

    set(IMPLOT_VERSION
        "0.16"
        CACHE STRING
            "implot version")
endmacro()

function(_map_version_to_release_tag _lib_name _requested_version _out_release_tag _out_tag_url)
    set(_mapping_entries ${ARGN})
    list(LENGTH _mapping_entries _mapping_len)

    if(_mapping_len EQUAL 0)
        message(FATAL_ERROR "No version mapping entries provided for ${_lib_name}")
    endif()

    math(EXPR _mapping_mod "${_mapping_len} % 2")
    if(NOT _mapping_mod EQUAL 0)
        message(FATAL_ERROR "Version mapping for ${_lib_name} must be version/tag pairs")
    endif()

    # Use first entry as implicit default when version is not specified
    list(GET _mapping_entries 0 _default_version)
    if("${_requested_version}" STREQUAL "")
        set(_requested_version "${_default_version}")
        message(STATUS "${_lib_name} version not set, defaulting to ${_requested_version}")
    endif()

    set(_supported_versions)
    unset(_resolved_release_tag)

    math(EXPR _last_idx "${_mapping_len} - 1")
    foreach(_idx RANGE 0 ${_last_idx} 2)
        list(GET _mapping_entries ${_idx} _candidate_version)
        math(EXPR _tag_idx "${_idx} + 1")
        list(GET _mapping_entries ${_tag_idx} _candidate_release_tag)

        list(APPEND _supported_versions ${_candidate_version})

        if("${_requested_version}" STREQUAL "${_candidate_version}")
            set(_resolved_release_tag "${_candidate_release_tag}")
        endif()
    endforeach()

    if(NOT _requested_version IN_LIST _supported_versions)
        string(JOIN ", " _versions_print_str ${_supported_versions})
        message(FATAL_ERROR "Invalid ${_lib_name} version '${_requested_version}', supported versions: ${_versions_print_str}")
    endif()

    set(${_out_release_tag}
        "${_resolved_release_tag}"
        PARENT_SCOPE)
    set(${_out_tag_url}
        "${_lib_name}-${_resolved_release_tag}"
        PARENT_SCOPE)
endfunction()

function(set_tags_from_version)
    # If a library version hasn't been set (i.e. this function called from elsewhere than esmini root CMakeLists.txt, the first version in the list below will be used as default)
    _map_version_to_release_tag(
        "osi"
        "${OSI_VERSION}"
        OSI_RELEASE_TAG
        OSI_TAG_URL
        "3.5.0"
        "v3.5.0_2")

    _map_version_to_release_tag(
        "implot"
        "${IMPLOT_VERSION}"
        IMPLOT_RELEASE_TAG
        IMPLOT_TAG_URL
        "0.16"
        "v0.16_1")

    set(OSI_RELEASE_TAG
        "${OSI_RELEASE_TAG}"
        PARENT_SCOPE)
    set(OSI_TAG_URL
        "${OSI_TAG_URL}"
        PARENT_SCOPE)
    set(IMPLOT_RELEASE_TAG
        "${IMPLOT_RELEASE_TAG}"
        PARENT_SCOPE)
    set(IMPLOT_TAG_URL
        "${IMPLOT_TAG_URL}"
        PARENT_SCOPE)
endfunction()