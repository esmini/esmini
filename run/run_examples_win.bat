@rem Run all prepared batch scripts in selected subfolders 

cd esmini
del *.dat *.osi log.txt
forfiles /m *.bat /c "cmd /c @path"

cd ..\esmini-dyn
del *.dat *.osi log.txt *.log
forfiles /m *.bat /c "cmd /c @path"

cd ..\odrviewer
del log.txt
forfiles /m *.bat /c "cmd /c @path"

cd ..\replayer
del *.csv *.dat log.txt
forfiles /m *.bat /c "cmd /c @path"

cd ..
