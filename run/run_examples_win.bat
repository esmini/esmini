@rem Run all prepared batch scripts in selected subfolders 

cd esmini
forfiles /m *.bat /c "cmd /c @path"

cd ..\esmini-dyn
forfiles /m *.bat /c "cmd /c @path"

cd ..\odrviewer
forfiles /m *.bat /c "cmd /c @path"

cd ..\replayer
forfiles /m *.bat /c "cmd /c @path"

cd ..
