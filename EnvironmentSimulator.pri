# This file can be included in Qt projects to use the components of esmini.

HEADERS += \ 
    $$PWD/EnvironmentSimulator/Modules/CommonMini/CommonMini.hpp \
    $$PWD/EnvironmentSimulator/Modules/RoadManager/odrSpiral.h \
    $$PWD/EnvironmentSimulator/Modules/RoadManager/RoadManager.hpp \
    $$PWD/externals/pugixml/pugiconfig.hpp \
    $$PWD/externals/pugixml/pugixml.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/Catalogs.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/Entities.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/Init.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/RoadNetwork.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/ScenarioEngine.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/ScenarioGateway.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/ScenarioReader.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/Story.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/Parameters.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/Trail.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCAction.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCCommon.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCCondition.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCDirectory.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCFile.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCGlobalAction.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCManeuver.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCParameterDeclarations.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCPrivateAction.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCProperties.hpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCPosition.hpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/ControllerSumo.hpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/Controller.hpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/ControllerSumo.hpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/ControllerExternal.hpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/ControllerFollowGhost.hpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/ControllerInteractive.hpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/ControllerSloppyDriver.hpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/vehicle.hpp \
    $$PWD/EnvironmentSimulator/Applications/replayer/Replay.hpp

SOURCES += \ 
    $$PWD/EnvironmentSimulator/Modules/CommonMini/CommonMini.cpp \
    $$PWD/EnvironmentSimulator/Modules/CommonMini/version.cpp \
    $$PWD/EnvironmentSimulator/Modules/RoadManager/odrSpiral.cpp \
    $$PWD/EnvironmentSimulator/Modules/RoadManager/RoadManager.cpp \
    $$PWD/externals/pugixml/pugixml.cpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/RoadNetwork.cpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/ScenarioEngine.cpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/ScenarioGateway.cpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/ScenarioReader.cpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/Story.cpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/Entities.cpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/Parameters.cpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/Catalogs.cpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/Trail.cpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCAction.cpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCCondition.cpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCManeuver.cpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCPrivateAction.cpp \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs/OSCPosition.cpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/Controller.cpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/ControllerSumo.cpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/ControllerExternal.cpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/ControllerFollowGhost.cpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/ControllerInteractive.cpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/ControllerSloppyDriver.cpp \
    $$PWD/EnvironmentSimulator/Modules/Controllers/vehicle.cpp \
    $$PWD/EnvironmentSimulator/Applications/replayer/Replay.cpp

RESOURCES += 

INCLUDEPATH += \
    $$PWD/externals/pugixml \
    $$PWD/EnvironmentSimulator/Modules/CommonMini \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles \
    $$PWD/EnvironmentSimulator/Modules/ScenarioEngine/OSCTypeDefs \
    $$PWD/EnvironmentSimulator/Modules/RoadManager \
    $$PWD/EnvironmentSimulator/Modules/Controllers \
    $$PWD/EnvironmentSimulator/Applications/replayer
