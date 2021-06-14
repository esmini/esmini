@rem Run unit tests on Windows

cd ..\buildVS2019_64_v141\EnvironmentSimulator\Unittest

set PATH=%PATH%;..\Libraries\esminiLib\Release

Release\OperatingSystem_test.exe
pause

Release\RoadManager_test.exe
pause

Release\ScenarioEngine_test.exe
pause

Release\ScenarioEngineDll_test.exe
pause

cd ..\..\..\run