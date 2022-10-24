$workingDir = (Get-Item .).FullName

$env:LD_LIBRARY_PATH = $workingDir + "/externals/OSI/linux/lib-dyn"
$env:LSAN_OPTIONS = "print_suppressions=false:suppressions=" + $workingDir + "/LSAN.supp"
$env:ASAN_OPTIONS = "detect_invalid_pointer_pairs=1:strict_string_checks=true:detect_stack_use_after_return=true:check_initialization_order=true:fast_unwind_on_malloc=false:suppressions=" + $workingDir + "/ASAN.supp"

cd build/EnvironmentSimulator/Unittest
$env:path += "../../../bin"

./OperatingSystem_test
if ($lastExitCode -ne 0) {
    throw "OperatingSystem_test failed"
}

./CommonMini_test
if ($lastExitCode -ne 0) {
    throw "CommonMini_test failed"
}

./RoadManager_test
if ($lastExitCode -ne 0) {
    throw "RoadManager_test failed"
}

./ScenarioEngine_test
if ($lastExitCode -ne 0) {
    throw "ScenarioEngine_test failed"
}

./ScenarioPlayer_test
if ($lastExitCode -ne 0) {
    throw "ScenarioPlayer_test failed"
}

./ScenarioEngineDll_test
if ($lastExitCode -ne 0) {
    throw "ScenarioEngineDll_test failed"
}
dir *.tga, *.ppm

./RoadManagerDll_test
if ($lastExitCode -ne 0) {
    throw "RoadManagerDll_test failed"
}

./FollowRoute_test
if ($lastExitCode -ne 0) {
    throw "FollowRoute_test failed"
}

./FollowRouteController_test
if ($lastExitCode -ne 0) {
    throw "FollowRouteController_test failed"
}

cd ../../../test

python3 smoke_test.py
$r0 = $LASTEXITCODE

python3 alks_suite.py
$r1 = $LASTEXITCODE

if (($r0 -ne 0) -or ($r1 -ne 0))
{
    throw "Test NOK"
}