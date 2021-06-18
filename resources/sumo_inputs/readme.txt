
multi_intersections.rou.xml file was generated as follows (Windows 10 PowerShell):

netconvert --opendrive multi_intersections.xodr -o multi_intersections.net.xml  --junctions.scurve-stretch 1.0 --no-turnarounds.except-deadend

&'C:\Program Files (x86)\Eclipse\Sumo\tools\randomTrips.py' -n multi_intersections.net.xml -e 100 -o multi_intersections.trips.xml

duarouter.exe -n multi_intersections.net.xml --route-files multi_intersections.trips.xml -o multi_intersections.rou.xml --ignore-errors

Then edit multi_intersections.rou.xml and replace all depart=".*" with depart="0.00"
