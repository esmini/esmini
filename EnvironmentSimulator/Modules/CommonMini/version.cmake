# Credit: https://www.mattkeeter.com/blog/2018-01-06-versioning/
#
execute_process(COMMAND git log --pretty=format:'%h' -n 1
                OUTPUT_VARIABLE GIT_REV
                ERROR_QUIET)

# Check whether we got any revision (which isn't
# always the case, e.g. when someone downloaded a zip
# file from Github instead of a checkout)
if ("${GIT_REV}" STREQUAL "")
    set(GIT_REV "N/A")
    set(GIT_DIFF "")
    set(GIT_TAG "N/A")
    set(GIT_BRANCH "N/A")
else()
    execute_process(
        COMMAND bash -c "git diff --quiet --exit-code || echo +"
        OUTPUT_VARIABLE GIT_DIFF)
    execute_process(
        COMMAND git describe --exact-match --tags
        OUTPUT_VARIABLE GIT_TAG ERROR_QUIET)
    execute_process(
        COMMAND git name-rev --name-only HEAD
        OUTPUT_VARIABLE GIT_BRANCH)

    string(STRIP "${GIT_REV}" GIT_REV)
    string(SUBSTRING "${GIT_REV}" 1 7 GIT_REV)
    string(STRIP "${GIT_DIFF}" GIT_DIFF)
    string(STRIP "${GIT_TAG}" GIT_TAG)
    string(STRIP "${GIT_BRANCH}" GIT_BRANCH)
endif()

set(VERSION "const char* ESMINI_GIT_REV=\"${GIT_REV}${GIT_DIFF}\";
const char* ESMINI_GIT_TAG=\"${GIT_TAG}\";
const char* ESMINI_GIT_BRANCH=\"${GIT_BRANCH}\";\n")

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/version.cpp)
    file(READ ${CMAKE_CURRENT_SOURCE_DIR}/version.cpp VERSION_FROM_FILE)
else()
    set(VERSION_FROM_FILE "")
endif()

if (NOT "${VERSION_FROM_FILE}" STREQUAL "${VERSION}")
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/version.cpp "${VERSION}" )
    set (VERSION_FROM_FILE ${VERSION} )
endif()

if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/buildnr.cpp)
    file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/buildnr.cpp "const char* ESMINI_BUILD_VERSION=\"N/A - client build\";\n")
endif()

STRING(REGEX REPLACE "const char\\* " "" VERSION_TO_TXT_FILE ${VERSION_FROM_FILE} )

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/buildnr.cpp BUILD_NR_)

STRING(REGEX REPLACE "const char\\* " "" BUILD_NR_TO_TXT_FILE ${BUILD_NR_})

file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/../../../version.txt "${VERSION_TO_TXT_FILE}${BUILD_NR_TO_TXT_FILE}")
