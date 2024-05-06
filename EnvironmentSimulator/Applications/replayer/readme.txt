replayer is a simple application for re-playing .dat files recorded by esmini.

Application is launched from command line (or batch file).

Usage: replayer [options]
Options:
  --file <filename>
      Simulation recording data file (.dat)
  --aa_mode <mode>
      Anti-alias mode=number of multisamples (subsamples, 0=off, 4=default)
  --camera_mode <mode>
      Initial camera mode ("orbit" (default), "fixed", "flex", "flex-orbit", "top", "driver") (toggle during simulation by press 'k')
  --capture_screen
      Continuous screen capture. Warning: Many jpeg files will be created
  --collision [mode]  (default = pause)
      Detect collisions and optionally pauses the replay <pause/continue> (pause is default)
  --custom_camera <position>
      Additional custom camera position <x,y,z>[,h,p] (multiple occurrences supported)
  --custom_fixed_camera <position and optional orientation>
      Additional custom fixed camera position <x,y,z>[,h,p] (multiple occurrences supported)
  --custom_fixed_top_camera <position and rotation>
      Additional custom top camera <x,y,z,rot> (multiple occurrences supported)
  --dir <path>
      Directory containing replays to overlay, pair with "file" argument, where "file" is .dat filename match substring
  --ground_plane
      Add a large flat ground surface
  --headless
      Run without viewer window
  --hide_trajectories
      Hide trajectories from start (toggle with key 'n')
  --info_text <mode>
      Show on-screen info text (toggle key 'i') mode 0=None 1=current (default) 2=per_object 3=both
  --no_ghost
      Remove ghost entities
  --no_ghost_model
      Remove only ghost model, show trajectory (toggle with key 'g')
  --osg_screenshot_event_handler
      Revert to OSG default jpg images ('c'/'C' keys handler)
  --path <path>
      Search path prefix for assets, e.g. model_ids.txt file (multiple occurrences supported)
  --quit_at_end
      Quit application when reaching end of scenario
  --remove_object <id>
      Remove object(s). Multiple ids separated by comma, e.g. 2,3,4.
  --repeat
      loop scenario
  --res_path <path>
      Path to resources root folder - relative or absolut
  --road_features
      Show OpenDRIVE road features
  --save_merged <filename>
      Save merged data into one dat file, instead of viewing
  --start_time <ms>
      Start playing at timestamp
  --stop_time <ms>
      Stop playing at timestamp (set equal to time_start for single frame)
  --text_scale [factor]  (default = 1.0)
      Scale screen overlay text
  --time_scale <factor>
      Playback speed scale factor (1.0 == normal)
  --view_mode <view_mode>
      Entity visualization: "model"(default)/"boundingbox"/"both"
  --use_signs_in_external_model
      When external scenegraph 3D model is loaded, skip creating signs from OpenDRIVE

Additional OSG graphics options:
  --clear-color <color>                      Set the background color of the viewer in the form "r,g,b[,a]"
  --screen <num>                             Set the screen to use when multiple screens are present
  --window <x y w h>                         Set the position x, y and size w, h of the viewer window. -1 -1 -1 -1 for fullscreen.
  --borderless-window <x y w h>              Set the position x, y and size w, h of a borderless viewer window. -1 -1 -1 -1 for fullscreen.
  --SingleThreaded                           Run application and all graphics tasks in one single thread.
  --lodScale <LOD scalefactor>               Adjust Level Of Detail 1=default >1 decrease fidelity <1 increase fidelity

Key shortcuts

    H (shift + h): Print this help text to console
    TAB:           Move camera to next vehicle
    Shift + TAB:   Move camera to previoius vehicle
    Delete:        Same as above (Shift + TAB)
    Space:         Toggle pause / play
    g:             Toggle show / hide ghost models
    o:             Toggle show / hide OpenDRIVE road feature lines
    u:             Toggle show / hide OSI road lines
    y:             Toggle show / hide OSI road points
    p:             Toggle show / hide environment 3D model
    i:             Toggle on-screen info text modes
    n:             Toggle show active trajectories
    , (comma):     Switch entity view : Model only / Bounding box / Model + Bounding box / None
    K:             Print current camera position and orientation to console
    ESC:           quit

    Arrow keys
        Left:               Pause and move to previous frame(+Shift to skip 10 frames)
        Right:              Pause and move to next frame(+Shift to skip 10 frames)
        Shift + Left:       Pause and jump 0.1s back
        Shift + Right:      Pause and jump 0.1s forward
        Shift + Ctrl Left:  Pause and jump 1.0s back
        Shift + Ctrl Right: Pause and jump 1.0s forward
        Ctrl + Left:        Pause and jump to beginning
        Ctrl + Right:       Pause and jump to end
        Up:                 Increase timeScale(play faster)
        Down:               Decrease timeScale(play slower)

    1 - 9: Camera models acording to :
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
           - Flex - orbit (Like flex but allows for roatation around vehicle)
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

Recommended usage:
    Run esmini headless (fast without viewer) and produce a .dat file. Then launch replayer to view it. Example in Windows PowerShell, starting from esmini/bin folder:

    .\esmini --osc ..\resources\xosc\cut-in.xosc --record sim.dat --headless --fixed_timestep 0.01 ; .\replayer --file sim.dat --window 60 60 800 400 --res_path ..\resources --repeat