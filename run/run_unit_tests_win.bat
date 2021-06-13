@rem Run unit tests on Windows

cd ..\build\EnvironmentSimulator\Unittest

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