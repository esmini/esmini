@rem Run with ODR file as argument, or drag-n-drop ODR file on this .bat in Windows Explorer
@rem Requires Python and a couple of libs - see import statements pretty standard ones, easily installed with pip

cd /d "%~dp0"

..\..\bin\win64\Release\OdrPlot.exe "%~1"
..\..\EnvironmentSimulator\OdrPlot\xodr.py track.csv

del track.csv
