
Environment Simulator Minimalistic (esmini) is a basic OpenSCENARIO player.
Home: https://github.com/esmini/esmini

Run esmini demo:
    1. Navigate to run/esmini
    2. Run any of the provided batch-script examples (double click on or run from command line)

Key shortcuts

    H (shift + h): Print this help text to console
    Space:         Toggle pause/play simulation
    Return:        Step simulation (one timestep) then pause
    TAB:           Move camera to next entity (0, 1, 2..., ALL, ROAD)
    Shift + TAB:   Move camera to previous entity
    Delete:        Same as above (Shift + TAB)
    o:             Toggle show / hide OpenDRIVE road feature lines
    O:             Toggle show / hide odr signal bounding boxes
    u:             Toggle show / hide OSI road lines
    y:             Toggle show / hide OSI road points
    p:             Toggle show / hide environment 3D model
    r:             Toggle show / hide sensor view frustums
    R:             Toggle route waypoint visualization
    i:             Toggle on-screen info text modes
    j:             Toggle show trails after vehicles(4 modes: none / dots / lines / both)
    n:             Toggle show active trajectories
    , (comma):     Switch entity view : Model only / Bounding box / Model + Bounding box / None
    K:             Print current camera position and orientation to console
    x:             Cycle axis indicator view mode (off, on, xray)
    ESC:           quit

    Arrow keys is used to drive externally controlled Ego vehicle:
        Up:    Accelerate
        Down:  Brake
        Left:  Steer left
        Right: Steer right

    1 - 9: Camera models according to :
        1: Custom camera model
        2: Flight
        3: Drive
        4: Terrain
        5: Orbit
        6: FirstPerson
        7: Spherical
        8: NodeTracker
        9: Trackball

    When custom camera model(1) is activated
        k: Switch between the following sub models:
           - Orbit        (camera facing vehicle, rotating around it)
           - Fixed        (fix rotation, always straight behind vehicle)
           - Flex         (imagine the camera attached to vehicle via an elastic string)
           - Flex - orbit (Like flex but allows for rotation around vehicle)
           - Top          (top view, fixed rotation, always straight above vehicle)
           - Driver       ("driver" view, fixed at center of vehicle)

    Viewer options
        f: Toggle full screen mode
        t: Toggle textures
        s: Rendering statistics
        l: Toggle light
        w: Toggle geometry mode(shading, wireframe, dots)
        c: Save screenshot in JPEG format - in the folder where the application was started from
        C: Toggle continuous screen capture (e.g for video creation)
        h: Help

Mouse control

    Left:   Rotate
    Right:  Zoom
    Middle: Pan

    This is typical, exact behaviour depends on active camera model.
