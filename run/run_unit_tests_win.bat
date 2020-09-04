@rem Run unit tests on Windows

cd ..\buildVS2019_64_v141\EnvironmentSimulator\Unittest

set PATH=%PATH%;..\ScenarioEngineDLL\Release

Release\OperatingSystem_test.exe
pause

Release\RoadManager_test.exe
pause

Release\ScenarioEngineDll_test.exe
pause
