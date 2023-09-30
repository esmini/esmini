# Credit: https://www.mattkeeter.com/blog/2018-01-06-versioning/
#
execute_process(
    COMMAND git describe --long --dirty --broken
    OUTPUT_VARIABLE GIT_REV
    ERROR_QUIET)

# Check whether we got any revision (which isn't always the case, e.g. when someone downloaded a zip file from Github instead of a checkout)
if("${GIT_REV}"
   STREQUAL
   "")
    set(GIT_REV
        "N/A")
    set(GIT_TAG
        "N/A")
    set(GIT_BRANCH
        "N/A")
else()
    execute_process(
        COMMAND git describe --exact-match --tags
        OUTPUT_VARIABLE GIT_TAG
        ERROR_QUIET)
    execute_process(
        COMMAND git name-rev --name-only HEAD
        OUTPUT_VARIABLE GIT_BRANCH)

    string(
        STRIP "${GIT_REV}"
              GIT_REV)
    string(
        STRIP "${GIT_TAG}"
              GIT_TAG)
    string(
        STRIP "${GIT_BRANCH}"
              GIT_BRANCH)
endif()

set(VERSION_TO_TXT_FILE
    "ESMINI_GIT_REV=\"${GIT_REV}\"\nESMINI_GIT_TAG=\"${GIT_TAG}\"\nESMINI_GIT_BRANCH=\"${GIT_BRANCH}\"\nESMINI_BUILD_VERSION=\"${ESMINI_BUILD_VERSION}\""
)

configure_file(
    "version.cpp.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/version.cpp"
    ESCAPE_QUOTES)

file(
    WRITE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../version.txt
    "${VERSION_TO_TXT_FILE}")
