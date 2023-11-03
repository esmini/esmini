set(ModulesFolder
    Modules)
set(ApplicationsFolder
    Applications)
set(LibrariesFolder
    Libraries)
set(CodeExamplesFolder
    code-examples)

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
add_subdirectory(Applications/dat2csv)
add_subdirectory(code-examples)

# ############################### Building Unittest ####################################################################

if((NOT
    (APPLE
     OR MINGW))
   AND USE_GTEST)
    add_subdirectory(Unittest)
endif()

# ############################### Establish folder structure ###########################################################

set_folder(
    RoadManager
    ${ModulesFolder})
set_folder(
    CommonMini
    ${ModulesFolder})
set_folder(
    ScenarioEngine
    ${ModulesFolder})
set_folder(
    Externals
    ${ModulesFolder})
set_folder(
    RoadManager
    ${ModulesFolder})
set_folder(
    ScenarioEngine
    ${ModulesFolder})

if(USE_OSI)
    set_folder(
        osireceiver
        ${ApplicationsFolder})
endif()
set_folder(
    PlayerBase
    ${ModulesFolder})
set_folder(
    Controllers
    ${ModulesFolder})
set_folder(
    esminiLib
    ${LibrariesFolder})
set_folder(
    esminiLib_static
    ${LibrariesFolder})
set_folder(
    esminiRMLib
    ${LibrariesFolder})
set_folder(
    esmini
    ${ApplicationsFolder})
set_folder(
    esmini-dyn
    ${ApplicationsFolder})
set_folder(
    dat2csv
    ${ApplicationsFolder})
set_folder(
    odrplot
    ${ApplicationsFolder})
if(USE_OSG)
    set_folder(
        replayer
        ${ApplicationsFolder})
    set_folder(
        odrviewer
        ${ApplicationsFolder})
    set_folder(
        ViewerBase
        ${ModulesFolder})
endif(USE_OSG)
