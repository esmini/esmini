odrviewer is a simple application for viewing OpenDRIVE files.

Optionally it can populate the road lanes with randomized dummy vehicles, which will go until end of road then starting over from initial position. If reaching a junction with multiple options, it will randomly chose its way out.

Application is launched from command line (or batch file). 

Usage: C:\eknabe1\GIT\esmini\bin\odrviewer.exe --odr filename [options]

Options [and default value]:
  --density <number>       [1]
          density (cars / 100 m)
  --model <filename>
          3D model filename
  --odr <filename>
          OpenDRIVE filename
  --osi_features <string>  [off]
          Show OSI road features ("on"/"off") (toggle during simulation with key 'u')
  --speed_factor <number>  [1]
          speed factor
  -h or --help
          Display command line parameters

Additional options forwarded to OpenSceneGraph:
  --window <x y w h>
          Set the position (x,y) and size (w,h) of the viewer window.		
  --screen <num>
          Set the screen to use when multiple screens are present.
  --clear-color <color>
          Set the background color of the viewer in the form "r,g,b[,a]".		  

Example 1 - View the ODR file and some random traffic on a 3D model, window mode 1000 x 500:

   OpenDriveViewer.exe --odr xodr\e6mini.xodr --model models\e6mini.osgb --window 50 50 1000 500

Example 2 - just ODR, fullscreen

   OpenDriveViewer.exe --odr xodr\e6mini.xodr

Example 3 - remove traffic

   OpenDriveViewer.exe --odr xodr\e6mini.xodr --model models\e6mini.osgb --density 0 --window 50 50 1000 500

Example 4 - sparse traffic (about 0.5 vehicle per 100 meter = 1 per 200 m)

   OpenDriveViewer.exe --odr xodr\e6mini.xodr --model models\e6mini.osgb --density 0.5 --window 50 50 1000 500


Key commands

	TAB: Move camera to next vehicle
	Shift-TAB: Move camera to previoius vehicle

	1-9: Camera models acording to:
		1. Custom camera model
		2. Flight
		3. Drive
		4. Terrain
		5. Orbit
		6. FirstPerson
		7. Spherical
		8. NodeTracker
		9. Trackball

	When custom camera model (1) is activated:
		c: Switch between the following sub models:
			- RubberBand (imagine the camera attached to vehicle via elastic band 
			- Orbit (camera facing vehicle, rotating around it)
			- Fixed (fix rotation, always straight behind vehicle) 

	o: Toggle show/hide OpenDRIVE road feature lines
	m: Toggle show/hide environmnet 3D model
	s: Graphics performance statistics
	f: Toggle full screen mode
	ESC: quit


Mouse control

	Left: Rotate
	Right: Zoom
	Middle: Pan

	This is typical. Exact behaviour depends on active camera model.

