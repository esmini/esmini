@rem Run all prepared batch scripts in selected subfolders 

cd EgoSimulator
forfiles /m *.bat /c "cmd /c @path"

cd ..\EnvironmentSimulator
forfiles /m *.bat /c "cmd /c @path"

cd ..\ScenarioViewer
forfiles /m *.bat /c "cmd /c @path"

cd ..\OpenDriveViewer
forfiles /m *.bat /c "cmd /c @path"

cd ..\Replayer
forfiles /m *.bat /c "cmd /c @path"

cd ..
