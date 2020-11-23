@rem Run with ODR file as argument, or drag-n-drop ODR file on this .bat in Windows Explorer
@rem Requires Python and a couple of libs - see import statements pretty standard ones, easily installed with pip

cd /d "%~dp0"

"../../bin/OdrPlot" "%~1"
python "../../EnvironmentSimulator/Applications/OdrPlot/xodr.py" track.csv

del track.csv
