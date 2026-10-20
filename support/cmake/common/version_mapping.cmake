macro(set_default_dependency_libs_versions)
    set(DEFAULT_OSI_VERSION "3.5.0_2")
    set(DEFAULT_IMPLOT_VERSION "0.16_1")
    set(DEFAULT_PROJ_VERSION "9.8.1_1")

    set(OSI_VERSION
        "${DEFAULT_OSI_VERSION}"
        CACHE STRING
            "osi version")

    set(IMPLOT_VERSION
        "${DEFAULT_IMPLOT_VERSION}"
        CACHE STRING
            "implot version")

    set(PROJ_VERSION
        "${DEFAULT_PROJ_VERSION}"
        CACHE STRING
            "proj version")
endmacro()

function(set_tags_from_version)
    string(REGEX REPLACE "_.*$" "" _default_osi_base_version "${DEFAULT_OSI_VERSION}")
    string(REGEX REPLACE "_.*$" "" _default_implot_base_version "${DEFAULT_IMPLOT_VERSION}")
    string(REGEX REPLACE "_.*$" "" _default_proj_base_version "${DEFAULT_PROJ_VERSION}")

    _map_version_to_release_tag(
        "osi"
        "${OSI_VERSION}"
        OSI_RELEASE_TAG
        OSI_TAG_URL
        "${_default_osi_base_version}"
        "v${DEFAULT_OSI_VERSION}"
        "3.8.0"
        "v3.8.0_1")

    _map_version_to_release_tag(
        "implot"
        "${IMPLOT_VERSION}"
        IMPLOT_RELEASE_TAG
        IMPLOT_TAG_URL
        "${_default_implot_base_version}"
        "v${DEFAULT_IMPLOT_VERSION}")

    _map_version_to_release_tag(
        "proj"
        "${PROJ_VERSION}"
        PROJ_RELEASE_TAG
        PROJ_TAG_URL
        "${_default_proj_base_version}"
        "v${DEFAULT_PROJ_VERSION}")

    # Propigate normal variables up to the parent scope
    set(OSI_RELEASE_TAG "${OSI_RELEASE_TAG}" PARENT_SCOPE)
    set(OSI_TAG_URL "${OSI_TAG_URL}" PARENT_SCOPE)
    set(IMPLOT_RELEASE_TAG "${IMPLOT_RELEASE_TAG}" PARENT_SCOPE)
    set(IMPLOT_TAG_URL "${IMPLOT_TAG_URL}" PARENT_SCOPE)
    set(PROJ_TAG_URL "${PROJ_TAG_URL}" PARENT_SCOPE)

    # Set if tests should run or not based on version and external lib usage flags
    _calculate_test_flag_default(_run_osi_tests USE_OSI "v${DEFAULT_OSI_VERSION}" "${OSI_RELEASE_TAG}")
    _calculate_test_flag_default(_run_implot_tests USE_IMPLOT "v${DEFAULT_IMPLOT_VERSION}" "${IMPLOT_RELEASE_TAG}")

    # Force set these to make sure cache is aligned
    set(RUN_OSI_TESTS ${_run_osi_tests} CACHE BOOL "Run OSI tests." FORCE)
    set(RUN_IMPLOT_TESTS ${_run_implot_tests} CACHE BOOL "Run ImPlot tests." FORCE)
endfunction()

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
    set(_requested_version_supported FALSE)

    math(EXPR _last_idx "${_mapping_len} - 1")
    foreach(_idx RANGE 0 ${_last_idx} 2)
        list(GET _mapping_entries ${_idx} _candidate_version)
        math(EXPR _tag_idx "${_idx} + 1")
        list(GET _mapping_entries ${_tag_idx} _candidate_release_tag)

        list(APPEND _supported_versions ${_candidate_version})

        if("${_requested_version}" STREQUAL "${_candidate_version}")
            set(_resolved_release_tag "${_candidate_release_tag}")
            set(_requested_version_supported TRUE)
        else()
            string(FIND "${_requested_version}" "${_candidate_version}_" _version_prefix_idx)
            if(_version_prefix_idx EQUAL 0)
                string(LENGTH "${_candidate_version}_" _version_prefix_len)
                string(SUBSTRING "${_requested_version}" ${_version_prefix_len} -1 _tag_suffix)
                if(_tag_suffix MATCHES "^[A-Za-z0-9.-]+$")
                    set(_resolved_release_tag "v${_requested_version}")
                    set(_requested_version_supported TRUE)
                endif()
            endif()
        endif()
    endforeach()

    if(NOT _requested_version_supported)
        string(JOIN ", " _versions_print_str ${_supported_versions})
        message(FATAL_ERROR "Invalid ${_lib_name} version '${_requested_version}', supported versions: ${_versions_print_str} (or a tag suffix, e.g. ${_default_version}_2)")
    endif()

    set(${_out_release_tag}
        "${_resolved_release_tag}"
        PARENT_SCOPE)
    set(${_out_tag_url}
        "${_lib_name}-${_resolved_release_tag}"
        PARENT_SCOPE)
endfunction()

function(_calculate_test_flag_default _out_var _use_flag _default_version _selected_version)
    # Start assuming TRUE
    set(_result TRUE)

    # Safely extract the value of the USE flag (e.g., USE_OSI)
    set(_use_flag_value ${${_use_flag}})

    # If USE flag is false, or versions don't match, default should be FALSE
    if(NOT DEFINED ${_use_flag} OR NOT _use_flag_value OR NOT "${_default_version}" STREQUAL "${_selected_version}")
        set(_result FALSE)
    endif()

    # Pass the calculated local value back up
    set(${_out_var} ${_result} PARENT_SCOPE)
endfunction()
