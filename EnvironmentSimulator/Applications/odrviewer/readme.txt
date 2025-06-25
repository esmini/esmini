odrviewer is a simple application for viewing OpenDRIVE files.

Optionally it can populate the road lanes with randomized dummy vehicles, which will go until end of road then starting over from initial position. If reaching a junction with multiple options, it will randomly chose its way out.

Application is launched from command line (or batch file).

Usage: odrviewer [options]
Options:
  --help
      Show this help message (-h works as well)
  --odr <odr_filename>
      OpenDRIVE filename (required)
  --aa_mode [mode]  (default if value omitted: 4)
      Anti-alias mode=number of multisamples (subsamples, 0=off)
  --capture_screen
      Continuous screen capture. Warning: Many .tga files will be created
  --config_file_path [path]...  (default if option or value omitted: config.yml)
      Configuration file path/filename, e.g. "../my_config.txt"
  --custom_fixed_camera <position and optional orientation>...
      Additional custom camera position <x,y,z>[,h,p]
  --custom_fixed_top_camera <position and rotation>...
      Additional custom top camera <x,y,z,rot>
  --density [density]  (default if value omitted: 1.000000)
      density (cars / 100 m)
  --enforce_generate_model
      Generate road 3D model even if --model is specified
  --disable_log
      Prevent logfile from being created
  --disable_off_screen
      Disable esmini off-screen rendering, revert to OSG viewer default handling
  --disable_stdout
      Prevent messages to stdout
  --duration <duration>
      Quit automatically after specified time (seconds, floating point)
  --fixed_timestep <timestep>
      Run simulation decoupled from realtime, with specified timesteps
  --generate_no_road_objects
      Do not generate any OpenDRIVE road objects (e.g. when part of referred 3D model)
  --generate_without_textures
      Do not apply textures on any generated road model (set colors instead as for missing textures)
  --ground_plane
      Add a large flat ground surface
  --headless
      Run without viewer window
  --log_append
      Log all scenarios in the same txt file
  --logfile_path [path]  (default if value omitted: odrviewer_log.txt)
      Logfile path/filename, e.g. "../my_log.txt"
  --log_meta_data
      Log file name, function name and line number
  --log_level [mode]  (default if option or value omitted: info)
      Log level debug, info, warn, error
  --log_only_modules <modulename(s)>
      Log from only these modules. Overrides log_skip_modules. See User guide for more info
  --log_skip_modules <modulename(s)>
      Skip log from these modules, all remaining modules will be logged. See User guide for more info
  --model <model_filename>
      3D Model filename
  --osg_screenshot_event_handler
      Revert to OSG default jpg images ('c'/'C' keys handler)
  --osi_lines
      Show OSI road lines. Toggle key 'u'
  --osi_points
      Show OSI road points. Toggle key 'y'
  --path <path>...
      Search path prefix for assets, e.g. OpenDRIVE files.
  --pause
      Pause simulation after initialization. Press 'space' to start.
  --road_features [mode]  (default if value omitted: on)
      Show OpenDRIVE road features. Modes: on, off. Toggle key 'o'
  --save_generated_model
      Save generated 3D model (n/a when a scenegraph is loaded)
  --seed <number>
      Specify seed number for random generator
  --speed_factor [speed_factor]  (default if value omitted: 1.000000)
      speed_factor <number>
  --stop_at_end_of_road
      Instead of respawning elsewhere, stop when no connection exists
  --text_scale [size factor]  (default if option or value omitted: 1.0)
      Scale screen overlay text
  --traffic_rule <rule (right/left)>
      Enforce left or right hand traffic, regardless OpenDRIVE rule attribute (default: right)
  --tunnel_transparency [transparency]  (default if value omitted: 0.0)
      Set level of transparency for generated tunnels [0:1]
  --use_signs_in_external_model
      When external scenegraph 3D model is loaded, skip creating signs from OpenDRIVE
  --version
      Show version and quit

Additional OSG graphics options:
  --clear-color <color>                      Set the background color of the viewer in the form "r,g,b[,a]"
  --screen <num>                             Set the screen to use when multiple screens are present
  --window <x y w h>                         Set the position x, y and size w, h of the viewer window. -1 -1 -1 -1 for fullscreen.
  --borderless-window <x y w h>              Set the position x, y and size w, h of a borderless viewer window. -1 -1 -1 -1 for fullscreen.
  --SingleThreaded                           Run application and all graphics tasks in one single thread.
  --lodScale <LOD scalefactor>               Adjust Level Of Detail 1=default >1 decrease fidelity <1 increase fidelity

Examples:

1. View the ODR file and some random traffic on a 3D model, window mode 1000 x 500:
   odrviewer --odr xodr\e6mini.xodr --model models\e6mini.osgb --window 60 60 1000 500

2. Just ODR, fullscreen
   odrviewer --odr xodr\e6mini.xodr

3. Remove traffic
   odrviewer --odr xodr\e6mini.xodr --model models\e6mini.osgb --density 0 --window 60 60 1000 500

4. Sparse traffic (about 0.5 vehicle per 100 meter = 1 per 200 m)
   odrviewer --odr xodr\e6mini.xodr --model models\e6mini.osgb --density 0.5 --window 60 60 1000 500


Key shortcuts

    H (shift + h): Print this help text to console
    Space:         Toggle pause/play simulation
    Return:        Step simulation(one timestep) then pause
    TAB:           Move camera to next vehicle (0, 1, 2..., ALL, ROAD)
    Shift + TAB:   Move camera to previous vehicle
    Delete:        Same as above (Shift + TAB)
    o:             Toggle show / hide OpenDRIVE road feature lines
    u:             Toggle show / hide OSI road lines
    y:             Toggle show / hide OSI road points
    p:             Toggle show / hide environment 3D model
    i:             Toggle info text showing time and speed
    , (comma):     Switch entity view : Model only / Bounding box / Model + Bounding box / None
    K:             Print current camera position and orientation to console
    ESC:           quit

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
        C: Toggle continuous screen capture
        h: Help

Mouse control

    Left:   Rotate
    Right:  Zoom
    Middle: Pan

    This is typical, exact behaviour depends on active camera model.
