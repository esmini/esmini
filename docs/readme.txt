
Environment Simulator Minimalistic (esmini) is a basic OpenSCENARIO player.
Home: https://github.com/esmini/esmini

Run esmini demo:
    1. Navigate to run/esmini
    2. Run any of the provided batch-script examples (double click on or run from command line)

Key commands:

    Arrow keys is used to drive externally controlled Ego vehicle:
      Up:    Accelerate
      Down:  Brake
      Left:  Steer left
      Right: Steer right


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
            - Orbit      (camera facing vehicle, rotating around it)
            - Fixed      (fix rotation, always straight behind vehicle) 
            - Flex       (imagine the camera attached to vehicle via an elastic string)
            - Flex-orbit (Like flex but allows for roatation around vehicle)
            - Top        (top view, fixed rotation, always straight above vehicle) 

    o: Toggle show/hide OpenDRIVE road feature lines
    u: Toggle show/hide OSI road lines
    y: Toggle show/hide OSI road points
    p: Toggle show/hide environment 3D model
    r: Toggle show/hide sensor view frustums
    i: Toggle info text showing time and speed
    j: Toggle show trails after vehicles
    , (comma): Switch entity view: Model only / Bounding box / Model + Bounding box / None 
    ESC: quit

    Viewer options:
      f: Toggle full screen mode
      t: Toggle textures
      s: Rendering statistics
      l: Toggle light
      w: Toggle geometry mode (shading, wireframe, dots)
      c: Save screenshot in JPEG format - in the folder where the application was started from
      h: Help 


Mouse control:
    Left: Rotate
    Right: Zoom
    Middle: Pan

    This is typical. Exact behaviour depends on active camera model.
    