link_directories(
    ${EXTERNALS_OSG_LIBRARY_PATH}
    ${EXTERNALS_OSI_LIBRARY_PATH}
    ${EXTERNALS_SUMO_LIBRARY_PATH}
    ${EXTERNALS_GTEST_LIBRARY_PATH})

# ############################### Building Modules #####################################################################

add_subdirectory(Modules/CommonMini)
add_subdirectory(Modules/Controllers)
add_subdirectory(Modules/PlayerBase)
add_subdirectory(Modules/RoadManager)
add_subdirectory(Modules/ScenarioEngine)

if(USE_OSG)
    add_subdirectory(Modules/ViewerBase)
endif(USE_OSG)

# ############################### Building Libraries ###################################################################

add_subdirectory(Libraries/esminiLib)
add_subdirectory(Libraries/esminiRMLib)

# ############################### Building Applications ################################################################

add_subdirectory(Applications/esmini)
add_subdirectory(Applications/esmini-dyn)

if(USE_OSG)
    add_subdirectory(Applications/odrviewer)
endif(USE_OSG)

add_subdirectory(Applications/odrplot)
add_subdirectory(Applications/replayer)

# ############################### Getting code-examples ################################################################

get_subdirectories(
    ${CODE_EXAMPLES_PATH}
    subdirectories)

if(NOT
   USE_OSI)
    list(
        REMOVE_ITEM
        subdirectories
        "osi-groundtruth")
    list(
        REMOVE_ITEM
        subdirectories
        "osi-groundtruth-dyn")
endif()

# ############################### Building code-examples ###############################################################

foreach(
    subdir
    ${subdirectories})
    if(NOT
       ${subdir}
       STREQUAL
       "trig_lane_change")
        add_subdirectory(code-examples/${subdir})
    endif()
endforeach()

# ############################### Building Unittest ####################################################################

if((NOT
    (APPLE
     OR MINGW))
   AND USE_GTEST)
    add_subdirectory(Unittest)
endif()
