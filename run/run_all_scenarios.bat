@rem Run all prepared batch scripts in selected subfolders 

cd EnvironmentSimulator
forfiles /m *.bat /c "cmd /c @path"

cd ..\EgoSimulator
forfiles /m *.bat /c "cmd /c @path"

cd ..\ScenarioViewer
forfiles /m *.bat /c "cmd /c @path"

cd ..\OpenDriveViewer
forfiles /m *.bat /c "cmd /c @path"

cd ..\Replayer
play_ltap-od_win64_hi-speed.bat

cd ..
