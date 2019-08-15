# This file can be included in Qt projects to use the components of esmini.

HEADERS += \ 
    $$PWD/EnvironmentSimulator/CommonMini/CommonMini.hpp \
    $$PWD/EnvironmentSimulator/RoadManager/odrSpiral.h \
    $$PWD/EnvironmentSimulator/RoadManager/RoadManager.hpp \
    $$PWD/externals/pugixml/pugiconfig.hpp \
    $$PWD/externals/pugixml/pugixml.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/SourceFiles/Catalogs.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/SourceFiles/Entities.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/SourceFiles/Init.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/SourceFiles/RoadNetwork.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/SourceFiles/ScenarioEngine.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/SourceFiles/ScenarioGateway.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/SourceFiles/ScenarioReader.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/SourceFiles/Story.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCAction.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCCommon.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCCondition.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCConditionGroup.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCDirectory.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCFile.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCGlobalAction.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCManeuver.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCParameterDeclaration.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCPrivateAction.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCProperties.hpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCPosition.hpp \
    $$PWD/EnvironmentSimulator/Replayer/Replay.hpp

SOURCES += \ 
    $$PWD/EnvironmentSimulator/CommonMini/CommonMini.cpp \
    $$PWD/EnvironmentSimulator/RoadManager/odrSpiral.cpp \
    $$PWD/EnvironmentSimulator/RoadManager/RoadManager.cpp \
    $$PWD/externals/pugixml/pugixml.cpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/SourceFiles/RoadNetwork.cpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/SourceFiles/ScenarioEngine.cpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/SourceFiles/ScenarioGateway.cpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/SourceFiles/ScenarioReader.cpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/SourceFiles/Story.cpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCAction.cpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCCondition.cpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCManeuver.cpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCPrivateAction.cpp \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs/OSCPosition.cpp \
    $$PWD/EnvironmentSimulator/Replayer/Replay.cpp

RESOURCES += 

INCLUDEPATH += \
    $$PWD/externals/pugixml \
    $$PWD/EnvironmentSimulator/CommonMini \
    $$PWD/EnvironmentSimulator/ScenarioEngine/SourceFiles \
    $$PWD/EnvironmentSimulator/ScenarioEngine/OSCTypeDefs \
    $$PWD/EnvironmentSimulator/RoadManager \
    $$PWD/EnvironmentSimulator/Replayer
