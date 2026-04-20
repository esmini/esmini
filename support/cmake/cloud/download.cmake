include_guard()

function(
    download_and_extract
    url
    target_folder
    target_filename
    clean_sub_folder)

    set(download_result
        1
        PARENT_SCOPE) # assume failure

    file(
        DOWNLOAD
        ${url}
        ${target_folder}/${target_filename}
        STATUS DOWNLOAD_STATUS
        LOG DOWNLOAD_LOG
        INACTIVITY_TIMEOUT 60)

    set(log_file
        ${target_filename}.log)

    if(NOT
       DOWNLOAD_STATUS
       OR NOT
          DOWNLOAD_STATUS
          EQUAL
          0)
        message(STATUS "FAILED to download ${target_filename} Status: ${DOWNLOAD_STATUS} (see ${log_file} for details)")
        file(
            WRITE
            ${log_file}
            ${DOWNLOAD_LOG})
        return()
    endif()

    set(download_result
        0
        PARENT_SCOPE) # update result status

    execute_process(COMMAND sleep 1) # allow for file to be completely flushed

    if(FORCE_DOWNLOAD_BINARIES)
        message(STATUS "Download OK. Removing any old ${target_folder}/${clean_sub_folder}")
        file(
            REMOVE_RECURSE
            ${target_folder}/${clean_sub_folder})
        message(STATUS "Extracting new package... ")
    else()
        message(STATUS "Download OK. Extracting... ")
    endif()

    execute_process(
        COMMAND ${CMAKE_COMMAND} -E tar xfz ${target_filename}
        WORKING_DIRECTORY ${target_folder}
        RESULT_VARIABLE STATUS)

    if(STATUS
       AND NOT
           STATUS
           EQUAL
           0)
        message(STATUS "FAILED to unpack ${target_filename}")
        set(download_result
            1
            PARENT_SCOPE) # update status at failed extraction
    endif()

    file(
        REMOVE
        ${target_folder}/${target_filename})
endfunction()

function(
    download
    entity_name
    path
    os_specific_path
    clean_sub_folder
    urls)

    if(NOT
       FORCE_DOWNLOAD_BINARIES
       AND EXISTS
           ${os_specific_path})
        message(STATUS "${entity_name} already exists, skipping")
        return()
    endif()

    foreach(
        url
        IN
        LISTS urls)
        set(package_name
            "${entity_name}.7z")

        message(STATUS "Downloading ${package_name} from ${url}")

        download_and_extract(
            ${url}
            ${path}
            ${package_name}
            ${clean_sub_folder})

        if(${download_result}
           EQUAL
           0)
            return()
        endif()

    endforeach()

endfunction()
