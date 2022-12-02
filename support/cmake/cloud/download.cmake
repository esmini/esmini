include_guard()

function(
    download_and_extract
    url
    target_folder
    target_filename)
    message(STATUS "downloading ${target_filename} ...")
    file(
        DOWNLOAD
        ${url}
        ${target_folder}/${target_filename}
        STATUS DOWNLOAD_STATUS)

    if(DOWNLOAD_STATUS
       AND NOT
           DOWNLOAD_STATUS
           EQUAL
           0)
        message(FATAL_ERROR "FAILED to download ${target_filename} (Status: ${DOWNLOAD_STATUS})")
    endif()

    execute_process(COMMAND sleep 1) # allow for file to be completely flushed

    message(STATUS "extracting ${target_filename} ... ")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E tar xfz ${target_filename}
        WORKING_DIRECTORY ${target_folder}
        RESULT_VARIABLE STATUS)

    if(STATUS
       AND NOT
           STATUS
           EQUAL
           0)
        message(FATAL_ERROR "FAILED to unpack ${target_filename}")
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
    url)

    set(PACKAGE_NAME
        "${entity_name}.7z")
    if(DEFINED
       os_specific_path
       AND (FORCE_DOWNLOAD_BINARIES
            OR NOT
               EXISTS
               ${os_specific_path}
           ))
        download_and_extract(
            ${url}
            ${path}
            ${PACKAGE_NAME})
    endif()

endfunction()
