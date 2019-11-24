@rem Run all prepared batch scripts in selected subfolders 

call run_all_examples.bat

cd EgoSimulator\test
forfiles /m *.bat /c "cmd /c @path"

cd ..\..\OpenDriveViewer\test
forfiles /m *.bat /c "cmd /c @path"

cd ..\..\