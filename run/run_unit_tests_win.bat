@rem Run unit tests on Windows

cd ..\buildVS2019_64_v141\EnvironmentSimulator\Unittest

set PATH=%PATH%;..\Libraries\esminiLib\Release;..\Libraries\esminiRMLib\Release

Release\OperatingSystem_test.exe
@if %ERRORLEVEL% NEQ 0 goto failure

Release\CommonMini_test.exe
@if %ERRORLEVEL% NEQ 0 goto failure

Release\RoadManager_test.exe
@if %ERRORLEVEL% NEQ 0 goto failure

Release\ScenarioEngine_test.exe
@if %ERRORLEVEL% NEQ 0 goto failure

Release\ScenarioPlayer_test.exe
@if %ERRORLEVEL% NEQ 0 goto failure

Release\ScenarioEngineDll_test.exe
@if %ERRORLEVEL% NEQ 0 goto failure

Release\RoadManagerDll_test.exe
@if %ERRORLEVEL% NEQ 0 goto failure

Release\FollowRoute_test.exe
@if %ERRORLEVEL% NEQ 0 goto failure

Release\FollowRouteController_test.exe
@if %ERRORLEVEL% NEQ 0 goto failure

@echo SUCCESS
@set exitcode=0
goto quit

:failure
@set exitcode=1
@echo FAILURE
goto quit

:quit
cd ..\..\..\run
@exit %exitcode%
