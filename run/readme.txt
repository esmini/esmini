
Environment Simulator Minimalistic (esmini) is a basic OpenSCENARIO player.
Home: https://github.com/esmini/esmini

Run esmini demo:
	1. Navigate to run/EgoSimulator
	2. Run any of the provided batch-script examples (double click on or run from command line)

Key commands:

	Arrow keys is used to drive externally controlled Ego vehicle:
	  Up:    Accelerate
	  Down:  Brake
	  Left:  Steer left
	  Right: Steer right

	  (w, x, a and d alternatives) 

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
		k: Switch between the following sub models:
			- Flex (imagine the camera attached to vehicle via an elastic string)
			- Orbit (camera facing vehicle, rotating around it)
			- Fixed (fix rotation, always straight behind vehicle) 

	o: Toggle show/hide OpenDRIVE road feature lines
	p: Toggle show/hide environment 3D model
	r: Toggle show/hide sensor view frustums
	i: Toggle info text showing time and speed
	j: Toggle show trails after vehicles
	ESC: quit

	Viewer options:
      f: Toggle full screen mode
      t: Toggle textures
      s: Rendering statistics
      l: Toggle light
      w: Toggle geometry mode (shading, wireframe, dots)


Mouse control:
	Left: Rotate
	Right: Zoom
	Middle: Pan

	This is typical. Exact behaviour depends on active camera model.
	