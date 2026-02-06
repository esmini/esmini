## esmini release notes

### 2026-02-05 Version 2.59.0

Breaking changes:
- Implement true road superelevation (banking)
  - rotate road instead of just elevate road edges
  - velodrome example video clip: https://youtu.be/vWHJKRoTHOc
  - this might slightly affect XY-coordinates on banked roads
- Report OpenDRIVE object outlines in local coordinates on OSI
  - consumer needs to apply heading from parent object
- Add OSI points in reverse order for linear/open OpenDRIVE outlines
  - to avoid unwanted area/volume by OSI closing any open outline
- Include `parking` lane type in `any_road` lane snap/awareness group

New features:
- Support [OpenDRIVE lane height](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/latest/specification/11_lanes/11_06_lane_geometry.html#sec-d30c9ef9-cb82-4683-9fb6-6487e9dffd2f) (issues [#471](https://github.com/esmini/esmini/issues/471) [#756](https://github.com/esmini/esmini/issues/756))
  - useful for elevated sidewalks and similar
- Add optional XYZ (RGB) axis indicator
  - enable with `--axis_indicator <mode>` (modes 0:off 1:on 2:xray)
  - cycle mode with 'x' key

Improvements and fixes:
- Consider road superelevation in entity pitch calculations
  - video clip: https://youtu.be/zddEvf-vWHQ
- Improve road-mark polygon alignment with road surface
- Add functionality for lookahead along route (issue [#759](https://github.com/esmini/esmini/issues/759))
- Limit grass to outer lanes of `border` or `none` type
  - other lanes of `none` type will get gray material (as border)
- Improve parsing of dataTime in EnvironmentAction
  - properly handle leading zeros
  - any timezone offset format allowed
  - accept missing milliseconds
- Separate visualization of OSI points from road features
- Fix limitation in accumulated lane width calculation
  - accept gaps in lane id numbering
- Clarify [OverrideGear API status type](https://github.com/esmini/esmini/blob/19c5d9e34dd6bd3a829412c05c24f2de85c8ce3b/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L166-L174) and add testcase
- Fix error in calculation of heading wrt lane width and offset
- Fix CI hick-up due to missing esmini version string
- Relax plot_dat.py to run from anywhere
- A few additional minor fixes

### 2026-01-26 Version 2.58.0

Breaking changes:
- Remove scripts/dat2csv.py and scripts/dat.py
  - use updated bin/dat2csv instead

New features:
- Support 2D shape outline
  - for vehicles, pedestrians and misc objects
  - see info in [User guide - 2D shape outline](https://esmini.github.io/#_2d_shape_outline)
  - video clip [here](https://youtu.be/YZX2P4F5Rj8)
- Add default option capability
  - for esmini (`--osc`), odrviewer (`--odr`) and replayer (`--file`)
  - example: `./bin/esmini ./resources/xosc/cut-in.xosc`
  - default option indicated in help by `[]`, e.g: `[--osc] <filename>`

Improvements and fixes:
- Fix and improve replayer dat merge operation
- Update bin/dat2csv
  - supporting same arguments and features as scripts/dat2csv.py
  - run `./bin/dat2csv` or `./bin/dat2csv --help` for usage info
- Truncate negative roadmark `sOffset` values, fixing freeze issue [#766](https://github.com/esmini/esmini/issues/766)
- Fix picture captions in [User guide - Heading behavior in Road vs Lane Position](https://esmini.github.io/#_heading_behavior_in_road_vs_lane_position)
- A few additional minor fixes

### 2026-01-08 Version 2.57.2

Improvements and fixes:
- Ensure global ids also for OpenDRIVE signals ([issue #747](https://github.com/esmini/esmini/issues/747))
- Populate OSI version also to sensor view ([PR #762](https://github.com/esmini/esmini/issues/762))
- Always clear any previous OpenDRIVE data between sessions ([issue #757](https://github.com/esmini/esmini/issues/757))
- Avoid side effects of exit call, use exception instead ([issue #755](https://github.com/esmini/esmini/issues/755))

### 2025-12-18 Version 2.57.1

Improvements and fixes:
- Populate host_vehicle_id in OSI ([issue #747](https://github.com/esmini/esmini/issues/747))
  - assigned to first ScenarioObject listed in [Entities](https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/Entities.html)

### 2025-12-17 Version 2.57.0

New features and behaviors:
- Use global IDs for all OSI elements ([issue #747](https://github.com/esmini/esmini/issues/747))
  - track internal scenario ID in OSI source reference
- Extend AddObject() with model name and obj types
  - add model_3d filename overriding model_id
  - support pedestrians and misc objects ([issue #750](https://github.com/esmini/esmini/issues/750))
- Implement OSI source reference
  - for Lane, StationaryObject, MovingObject, TrafficLight and TrafficSign
- Report added stationary MiscObjects on OSI
  - see updated note in [User guide - OSI data via API call](https://esmini.github.io/#_osi_data_via_api_call)

Improvements and fixes:
- Fix sometimes stuck on loading OpenDRIVE file ([issue #751](https://github.com/esmini/esmini/issues/751))
- Add support for OSI checks in smoke tests

### 2025-12-12 Version 2.56.2

Improvements and fixes:
- Update Unity example and add support for permutations ([issue #743](https://github.com/esmini/esmini/issues/743))
  - brief info and link to package in [User guide - esmini in Unity](https://esmini.github.io/#_esmini_in_unity)
- Add option `--view_ghost_restart` to view also original ghost before restart
  - default mode is now to view only resulting ghost path
  - this option adds the original detour before any restart happens
- Log error message on road vs geometry length mismatch ([issue #753](https://github.com/esmini/esmini/issues/753))
- Fix replayer bug causing crash when running old dat file in newer player
- Fix bug resetting log filename when setting custom dat filename via API

### 2025-12-05 Version 2.56.1

Bugfixes:
- dat2csv extended not saving extended data ([issue #752](https://github.com/esmini/esmini/issues/752))
- sometimes missing infotext and global actions

### 2025-12-04 Version 2.56.0

New features:
- Add support for traffic lights
  - basic functionality and visualization
    - TrafficSignalStateAction
    - TrafficSignalCondition
    - OSI reporting
    - generate 3D model
    - support in .dat/replayer
  - see info and limitations in [User guide - Traffic lights visualization](https://esmini.github.io/#_traffic_lights)
  - example scenario [traffic_lights.xosc](https://github.com/esmini/esmini/blob/dev/resources/xosc/traffic_lights.xosc) video clip [here](https://youtu.be/26BSMblzyO4)
  - Note: Requires update of osg and model packages:
    - remove `externals/osg` and `resources/models` folders
    - run `cmake ..` from build folder
- Add support for moving Vehicle reference point along x-axis
  - see info in [User guide - Shift reference point](https://esmini.github.io/#_shift_reference_point)
  - video clip: https://youtu.be/gvfFI4hgw1M
- Use `model3d` filename in .dat file instead of relying on model_id
  - `model_id` deprecated, support will be removed eventually
  - meanwhile `model_id` overrides any specified `model3d` in replayer
- Write storyboard events to .dat and show in console by replayer
- Add `--wireframe` option

Improvements and fixes:
- Restore support for absolute catalog file path
- Update esminiJS lib and improve WASM [example](https://github.com/esmini/esmini/tree/dev/EnvironmentSimulator/Libraries/esminiJS/example) ([PR #748](https://github.com/esmini/esmini/issues/748))
- Add CI job for building OSG libs
- Restore esmini CI link to original [Vector NCAP scenario repo](https://github.com/vectorgrp/OSC-NCAP-scenarios)
- Log written dat-packet id and size (debug log level)
- Extend search for (osgb) file needed for vehicle shadow
- Restore plot_csv.py (issue introduced in v2.54.0)
- Improve wheel angle and rotation calculations
  - base wheel angle kinematics of the bicycle model
  - limitation: still same rotation on all wheels
- Make .dat playable after crash while recording it
- Prevent bin/dat2csv writing file info to csv
  - aligned behavior with dat2csv.py
- A few additional minor fixes

### 2025-11-21 Version 2.55.0

Breaking changes:
- Bump .dat format from 3.0 to 4.0
  - add esmini version to dat file header
  - Note: Major version bump => previous dat files will not play

Improvements and fixes:
- Fix missing vehicle dynamics with non fixed timestep
- replayer prints which esmini version created the dat file
- Fix replayer issue causing unexpected jump to start/end
- Fix wrong xodr file in [User guide - SUMO integration](https://esmini.github.io/#_sumo_integration)
- Restore support for gcc 7.5 by relaxing need for full fs::filesystem
- A few additional minor fixes in replayer

### 2025-11-12 Version 2.54.0

Breaking changes:
- Total rework of .dat (scenario recording) format
  - package based, enabling more content in future
  - bump to version 3
  - old .dat files < v3 will not play
  - more info in [User guide - Scenario recording (.dat)](https://esmini.github.io/#_scenario_recording_dat)
  - Following related python code removed:
    - csv2dat functionality in dat.py
    - dat2xosc script

New behaviors:
- Change timestep of ghost (re)start phase
  - from hardcoded 0.05 to whatever set fixed timestep
  - fallback to 0.05 when no fixed timestep set
  - will cause any scenarios involving ghost to be slightly different

Improvements and fixes:
- Fix bug writing signal 3D model to wrong OSI package
  - causing crash in some cases
- Fix wrong direction with lanePos and lane 0
- Bump CI and release macOS version to 14
- Ensure init actions ordered wrt entity dependencies
  - sort actions group wise based on object
  - add missing RelativeWorldPosition for sorting consideration
- Fix wrongly positioned and duplicate 3D model bug
- Fix 3D model load failure in swarm action
- Set socket reuse address option avoiding port lockup
- Update catalog search path handling
  - see [User guide - Search locations](https://esmini.github.io/#_search_locations)

### 2025-10-24 Version 2.53.1

New features:
- Add option `vehicle_dynamics` for simple cosmetic vehicle dynamics
  - damped spring model based on lat/long accelerations
  - only affecting visual model
  - video clip: https://youtu.be/QqjhY-euhrY
- Add logger callback and buffer mechanisms
  - see these [test cases](https://github.com/esmini/esmini/blob/8ddfa99d352d94c925fd688830b907be72dbdee2/EnvironmentSimulator/Unittest/CommonMini_test.cpp#L336-L397) for example usage

Improvements and fixes:
- Fix environment model OSI path ([issue #738](https://github.com/esmini/esmini/issues/738))
- Add `view_mode` option for finer entity visualization control
- Fix textures not found issue introduced in v2.53.0

### 2025-10-22 Version 2.53.0

New features:
- Add [Dockerfile](https://github.com/esmini/esmini/tree/dev/resources/dockers) ([issue #735](https://github.com/esmini/esmini/issues/735))
- Add OSI `model_reference` for OpenDRIVE objects and signals

Improvements and fixes:
- Update OSI `model_reference` handling ([issue #738](https://github.com/esmini/esmini/issues/738))
  - put resolved path for found files, regardless loaded or not
- Update file search path handling in general
  - see [User guide - File search paths](https://esmini.github.io/#_file_search_paths)
  - **NOTE:** Might cause files not found, set paths as needed
- Fix incorrect return type of roadmanager GetCountourType() ([PR #740](https://github.com/esmini/esmini/issues/740))
- Some additional minor fixes

### 2025-10-09 Version 2.52.2

New behaviors:
- Restart ghost on overrideControllerAction deactivation
  - e.g. restore scenario from Ego location after avoidance maneuver

Improvements and fixes:
- Add roadmanager ReturnCode indicating reached end of lane
  - potentially snapped to neighbor drivable lane
- Fix incorrect stop on road end in moveAlongS
  - for long steps over multiple roads and lane sections
- Add missing laneType for RM_GetProbeInfo()
- Elaborate return code of RMLib lookahead functions
  - basically propagate roadmanager ReturnCode

### 2025-10-06 Version 2.52.1

Improvements and fixes:
- Ensure ground plane correct size and elevation
- Fix non rotating wheels
- Some additional minor fixes

CI improvements:
- Run wrapper check always, not only nightly

### 2025-10-03 Version 2.52.0

Braking API changes:
- Extend esminiRMLib RoadLaneInfo struct with:
  - lane type
- Extend esminiLib RoadInfo struct with:
  - lane type
  - trail wheel angle, e.g. for ghost tracking
- Extend aggregated lane type "ANY_ROAD" to include SHOULDER
  - as side effect shoulder will be colored as asphalt instead of grass

Note: The above changes might require updates in user applications

Improvements and fixes:
- Make ghost trail sample rate configurable, examples:
  - `--ghost_trail_dt 0.5`
  - `SE_SetOptionValue("ghost_trail_dt", "0.5")`
- Fix esmini lib SE_SetSnapLaneTypes() has no effect
- Use main object lane type mask for trail lookahead
- Some additional minor fixes

### 2025-10-01 Version 2.51.1

Improvements and fixes:
- Fix road tesselation bug introduced in v2.51.0
- Add RM_SetRoadPosition() function, complementing RM_SetLanePosition()
- Update RM unity util wrt return types
- Fully support multiple lane sections in connecting roads
- Fix bug causing GetInLaneType() return wrong lane type
- Fix off-road check to correctly consider lane offset
- Fix lock-on-lane bug

Build improvements:
- Aim for faster package download by switching source order

### 2025-09-24 Version 2.51.0

New features:
- Support initialize road network from OpenDRIVE XML string
  - as alternative to OpenDRIVE filename
- Add option, `--hide_ghost`, to hide any ghost in viewer
- Add follow reference code example and controller
  - based on classic Stanley controller for steering
  - and critically damped spring model for speed control
  - more info in [User guide - FollowReferenceController](https://esmini.github.io/#_followreferencecontroller)
  - controller example scenario: [follow_reference.xosc](https://github.com/esmini/esmini/blob/dev/resources/xosc/follow_reference.xosc)
  - video clip: https://youtu.be/icozF90XZDQ
  - external app code example: [follow_reference](https://github.com/esmini/esmini/tree/dev/EnvironmentSimulator/code-examples/follow_reference)

Improvements and fixes:
- Refactor some esminiRMLib functions making use of out/ref parameter
  - dedicating return value for success/error indication (0/-1)
  - also add lane type(s) argument for following functions:
    - `RM_GetRoadNumberOfLanes()`
    - `RM_GetLaneIdByIndex()`
  - rename old variants, only considering drivable lanes, into:
    - `RM_GetRoadNumberOfDrivableLanes()`
    - `RM_GetDrivableLaneIdByIndex()`
- Add esminiRMLib [C# wrapper](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/Libraries/esminiRMLib/esminiRMWrapper.cs) generator ([CSWrapperGenerator](https://github.com/esmini/esmini/tree/dev/EnvironmentSimulator/Libraries/esminiRMLib/CSWrapperGenerator))
  - add nightly job to check wrapper consistency
- Implement vehicle pool categories and support sumo vClass ([issue #728](https://github.com/esmini/esmini/issues/728))
- Generate texture coords for outline objects, including tunnels
- Make ghost semi-transparent in replayer
- Add enum for options, enabling direct lookup of frequently referred options
- Fix missing ground plane (bug introduced in v2.50.5)
- Fix ghost trailer wrong naming ([PR #731](https://github.com/esmini/esmini/pull/731))
- Fix car color mix-up in [User guide - Heading behavior](https://esmini.github.io/#_heading_behavior_in_road_vs_lane_position)
- Fix ignored name for ActivateControllerAction
  - activateControllerAction without ControllerAction is now deprecated
  - nonetheless fix bug ignoring name in that case
- Fix wrong controller/teleport warning
- Fix wrong type indices in lib header documentation ([PR #727](https://github.com/esmini/esmini/pull/727))
- Fix reverse ordered tunnel roof bottom face
- Bugfix: Ensure correct speed sign for relative time followTrajectoryAction
- Some additional minor fixes

### 2025-09-01 Version 2.50.6

Improvements and fixes:
- Elaborate node names
  - add tunnel component to osg node name
  - add instance index to name (e.g. for object repeats)
  - update [code example](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/road-model/road-model.cpp) to print material names
- Add lib function [`SE_GetOptionValueByIndex()`](https://github.com/esmini/esmini/blob/f52e6b875727c0a1311300e9367ca7c0b0c5a1ab/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L471-L477)
  - useful for options with multiple values
- Fix path handling issue
  - migrate path fully to option handling, skipping separate list

### 2025-08-29 Version 2.50.5

Improvements and fixes:
- Fix bug repeatedly reloading model
    - bug introduced in v2.50.4 (handling of 3D model paths)
- Refactor road 3D model generator into a separate module and lib
    - for standalone use
    - add simple names to osg nodes
    - add code example [road-model](https://github.com/esmini/esmini/tree/refs/heads/dev/EnvironmentSimulator/code-examples/road-model)
    - add documentation [User guide - Export generated 3D model](https://esmini.github.io/#_export_generated_3d_model)
- Fix a few steering angles from deg to radians in vehicle catalog

### 2025-08-26 Version 2.50.4

Improvements and fixes:
- Update OSI model_reference setting
  - set relative path to any loaded 3D model
  - fallback to requested filename as default
- Populate OSI source_reference
  - map scenario object property source_reference
  - both stationary and moving objects
- Constrain stepsize for non fixed timestep mode
  - avoid steps less than 1ms by injecting naps
  - scenarios will run in realtime with less CPU load
- Remove space before comma in csv_logger header ([issue #722](https://github.com/esmini/esmini/issues/722))
  - but keep post comma space, for human readability

### 2025-08-22 Version 2.50.3

New features:
- Support [openx-assets](https://github.com/bounverif/openx-assets) - collection of vehicle 3D models
  - see [User guide - Using OpenX assets library](https://esmini.github.io/#_using_openx_assets_library) for info and [video clip](https://youtu.be/UWjQ8Ai83Cc)

Improvements and fixes:
- Fix some vehicle shadow z issues
- Suppress invalid timestamp in throw log entry

### 2025-08-18 Version 2.50.2

Improvements and fixes:
- Restore .dat backward compatibility
  - fix issue playing a dat file from previous versions in v2.50.1
  - bounding box can look wrong caused by scale mode redefinition
- Add some entries to dat2csv extended mode (see slight usage info [here](https://esmini.github.io/#_scenario_recording_dat))

### 2025-08-15 Version 2.50.1

Improvements and fixes:
- Establish correct bounding box dimensions already first frame
  - by doing viewer frame before OSI
  - for the case where bounding box scales to 3D model
- Add ctrl property ([OverrideVehicleScaleMode](https://github.com/esmini/esmini/blob/6c89429fd28149b460ac1aec18ffd268def7fb35/resources/xosc/Catalogs/Controllers/ControllerCatalog.xosc#L137-L138)) for SUMO vehicles scale mode
  - to adapt model to bounding or the opposite, or not at all
  - or go with setting in the vehicle definition (typically catalog entry)
- Fix missing wheel rotations in OSI ([issue #706](https://github.com/esmini/esmini/issues/706))
- Parse odr object radius attribute ([issue #719](https://github.com/esmini/esmini/issues/719))
  - limited support, affects only rectangular bounding box dimension
- Road id str properly converts to int, supporting expressions ([issue #718](https://github.com/esmini/esmini/issues/718))
- Restore RM_SetLogFilePath correct behavior ([issue #723](https://github.com/esmini/esmini/issues/723))
- Fix bug causing rare infinite loop during road meshing
- Add variable modify demo scenario ([variable_modify.xosc](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/Unittest/xosc/variable_modify.xosc)) and test case
- Fix ghost restart teleport lost route issue
- Fix maxAcceleration not respected by SpeedProfile action
  - for case when dt is too small and more than two entries
- Improve logging of dynamics constraints
  - warn when specified attribute not identified, probably misspelled
  - log all applied values (debug log level)
- Some debug log level cleanup
- Add test case to check and [illustrate](https://youtu.be/fVg10b6lDXY) lane change behavior ([issue #724](https://github.com/esmini/esmini/issues/724))

### 2025-07-08 Version 2.50.0

New features:
- Support LateralDistanceAction ([issue #653](https://github.com/esmini/esmini/issues/653))
  - limited to road and entity coordinates
  - more info and video clips in [User guide - Lateral distance action logics](https://esmini.github.io/#_lateral_distance_action_logics)

Improvements and fixes:
- Update coverage of [odr](https://github.com/esmini/esmini/blob/master/odr_coverage.txt) and [osc](https://github.com/esmini/esmini/blob/master/osc_coverage.txt)
- Improve OSMP FMU ([PR #710](https://github.com/esmini/esmini/pull/710))
  - add AssignRoute and AcquirePosition to traffic commands
  - fix OSI timestamp nanosecond issue
  - more fixes and details documented in the pull request (link above)
- Fix top view bug ([issue #715](https://github.com/esmini/esmini/issues/715))
- Slim logging implementation
  - remove spdlog dependency, only keep fmt lib for formatting
- Improve smoke test performance and include scenario run time in output
- Clarify focus on environment and all entities in overlay info text
- Remove unintended ghost related OSI warning

### 2025-06-24 Version 2.49.0

New features:
- Add support for OpenSCENARIO EnvironmentAction
  - see info in [User guide - Environment conditions](https://esmini.github.io/#_environment_conditions)
  - video clip: https://youtu.be/-mQUJU1rR2M
- Add support for OpenDRIVE Tunnel
  - see info in [User guide - Tunnels](https://esmini.github.io/#_tunnels)
  - video clip: https://youtu.be/pTYcu1yFe5k

Improvements and fixes:
- Fix SUMO controller memory leak
- Add replayer offscreen render support
- Fix replayer ghost stalling application bug
- Add further camera interfaces to esmini lib ([PR #708](https://github.com/esmini/esmini/pull/708))
- Fix steering sensor bug ([PR #702](https://github.com/esmini/esmini/pull/702))
- Fix corrupt OSI dashed/broken roadmarks in curved roads
- osiviewer updates:
  - Make stationary objects semi-transparent to see inside tunnels
  - Add checkbox for hiding stationary objects
  - Plot structure boundaries, e.g. tunnels, with red color
- Fix EndOfRoad condition log entry
- Fix zero rate bug causing entities thrown into space ([issue #711](https://github.com/esmini/esmini/issues/711))

### 2025-06-13 Version 2.48.1

Improvements and fixes:
- Randomize cars for SUMO ctrl ([issue #706](https://github.com/esmini/esmini/issues/706))
  - use cars and vans from available catalogs
- Fix OpenDRIVE_traffic_signals.txt not found by renaming it to pure lowercase
  - esmini (from v2.25.1) assumes prefix/country code to be lower case
- Fix lost delayed triggers during ghost restart
  - consider all condition events, both before and during restart phase
- Improve CI smoke test runner
  - speed up CI smoke tests by reducing watchdog sleep intervals
  - print scenario execution time

### 2025-06-09 Version 2.48.0

Code QA updates:
- Use pre-commit hooks to lint and format code
  - see [User guide - Formatting and code analysis](https://esmini.github.io/#_formatting_and_code_analysis)
  - work inspired by [PR #674](https://github.com/esmini/esmini/pull/674)

Improvements and fixes:
- Add wheel rotation rate, radius and number of wheels ([PR #701](https://github.com/esmini/esmini/pull/701))
- Add two chinese road signs
  - minimum speed 50 and maximum speed 70
  - get updated 3D model pack [here](https://dl.dropboxusercontent.com/s/5gk8bvgzqiaaoco/models.7z?dl=1) <br>
    or remove `resources/models` and run `cmake ..` command again
- 3D geometry updates:
  - add roadmark texture coordinates
  - share material of same color between geometries <br>
    Note: All textured objects will share material. Will be changed in next release.
- Restore pre OSC 1.3 controller domain handling ([issue #703](https://github.com/esmini/esmini/issues/703))
  - for scenarios ≥ v1.3, allow multiple active controllers per entity
  - for scenarios < v1.3, allow only one active controller per entity
- Support unordered (wrt s) OpenDRIVE road type elements
- Improve SynchronizeAction by handling master reversing cases ([issue #705](https://github.com/esmini/esmini/issues/705))
- Incorporate more scenarios from [Vector NCAP scenarios](https://github.com/vectorgrp/OSC-NCAP-scenarios) suite in esmini CI

### 2025-05-28 Version 2.47.0

New features:
- Add support for HID game controllers
  - separate controller, similar to interactiveController
  - properties for device ID and mappings to steering, throttle and brake
  - Windows and Linux only (not macOS)
  - see more info in [User guide - HIDController](https://esmini.github.io/#_hidcontroller)
- Add camera follow modes ALL and ROAD
  - focus on geometric center of all entities or road network
  - for ALL mode, continuously update center point and zoom factor
  - reachable by Tab/shift-Tab (0, 1, 2..., ALL, ROAD) and <br>
    `--follow_object` option, see [User guide - esmini cmd reference](https://esmini.github.io/#_esmini)

Improvements and fixes:
- Nightly job to check for memory leaks with Valgrind
- Document how to work with Blender (see [User guide](https://esmini.github.io/#_convert_osgb_models_for_use_in_blender))
- Fix relative teleport bug with relative position
- Fix bug to ensure not missing ghost final state (at end of trail)
- Add stop triggers to avoid infinite scenarios (needed for memory tests)
- Rework end-of-road condition and handling
  - change EOR duration check from ">" to ">=" to support 0 delay
  - retain speed for the one step reaching end of road (EOR)
  - while stuck at EOR, set speed = 0
- Add missing floor to outline objects (not all are put on ground...)

### 2025-05-15 Version 2.46.4

Improvements and fixes:
- Fix intermittent crash in RoadManager GetLaneSectionIdxByS()
- Add further checks and error return codes in RoadManager
- Fix look-ahead bug in LoomingController

### 2025-05-13 Version 2.46.3

Improvements and fixes:
- Add options for trajectory motion control
  - option and vehicle property for ignoring heading for motion direction
    - see [User guide - Trajectory motion direction control option](https://esmini.github.io/#_trajectory_motion_direction_control_option)
  - option and vehicle property for explicit interpolation mode
    - see [User guide - Trajectory interpolation and alignment](https://esmini.github.io/#_trajectory_interpolation_and_alignment)
  - Note: Previous option `--disable_pline_interpolation` replaced by `--pline_interpolation off`
- Expose object odometer in esminiLib API ([SE_GetObjectOdometer()](https://github.com/esmini/esmini/blob/22f146db3611a0bc02a592be39e51b74e1ba5b99/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1181))
- Handle missing wheel data in [osi2csv.py](https://github.com/esmini/esmini/blob/dev/scripts/osi2csv.py) script, avoid crash
- Fix missed trigger during ghost restart
- Support repeated use of ghost teleport action
- Restore esminiLib C compatibility and add CI regression check  ([issue #697](https://github.com/esmini/esmini/issues/697))
- Fix bad OSI crop printout and adjust OSITrafficCmd log level
- Fix bug showing one (im)plot per object

Minor fixes:
- Fix fbx path in Linux osg app build script
- Fix osgconv example typo in [User guide - Convert osgb models for use in Unity](https://esmini.github.io/#_convert_osgb_models_for_use_in_unity)
- Remove obsolete osi API calls in code examples

### 2025-04-30 Version 2.46.2

Improvements and fixes:
- Fix specified log level having no effect on stdout (bug introduced in v2.46.0)
- Allow setting explicit OSI timestamp before SE_Init (updated doc [here](https://esmini.github.io/#_osi_timestamps))

### 2025-04-28 Version 2.46.1

Improvements and fixes:
- Fix dead-reckoning for udp ctrl state_h mode ([issue #672](https://github.com/esmini/esmini/issues/672))
- Remove trajectory after end of action ([issue #682](https://github.com/esmini/esmini/issues/682))
- Do not collide with ghosts ([issue #687](https://github.com/esmini/esmini/issues/687)) ([PR #688](https://github.com/esmini/esmini/pull/688))
- Add API to get velocity and angular velocity and acceleration ([PR #689](https://github.com/esmini/esmini/pull/689))
- Add support for roadmark wear by material alpha
  - add userData fade attribute to OpenDRIVE lane marking (example in [fade_roadmarks.xodr](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/Unittest/xodr/fade_roadmarks.xodr))
  - add corresponding alpha to roadmark in 3D model
- Fix osg build scripts [compile_osg_apps.sh](https://github.com/esmini/esmini/blob/dev/scripts/compile_osg_apps.sh) and [generate_osg_libs.sh](https://github.com/esmini/esmini/blob/dev/scripts/generate_osg_libs.sh) config for VS 2022
  - also fix broken link to osg 3rd party windows dependency pack
- Fix mixed up print parameters ([issue #694](https://github.com/esmini/esmini/issues/694))
- Improve OSI frequency option handling, any change valid from next step
- Add info on OSI timestamps, see [User guide - OSI timestamps](https://esmini.github.io/#_osi_timestamps)
- Fix lateral distance condition to be non sensitive to road driving direction
  - previous implementation assumes staying in lanes according to driving direction
  - new implementation allows for passing reference line, e.g. during an overtake.
- Fix broken headless window in examples ([issue #699](https://github.com/esmini/esmini/issues/699))
  - window needs to be specified after any headless option
- Always update triggers for ghost acts ([PR #680](https://github.com/esmini/esmini/pull/680))
  - update act trigger even if ghost not involved in events
  - needed for non entity actions such as ParameterSetAction
- Fix unnecessary OSI terminal printouts

### 2025-03-31 Version 2.46.0

API changes:
- Update OSI API, add extended options for static groundtruth data handling
  - see [User guide - OSI data](https://esmini.github.io/#_osi_data) for detailed info
- Extend RoadInfo structs with road type and road rule
  - see example in [esminiLib.hpp](https://github.com/esmini/esmini/blob/a059a432144445b44f54befaa8a40af9d1f9a155/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L101-L102)
  - corresponding changes in esminiRMlib and the C# wrappers

Improvements and fixes:
- Support specifying options in yaml config file
  - see [User guide - About options and config files](https://esmini.github.io/#_about_options_and_config_files) for info and examples
- OSI dynamic groundtruth can be cropped around one or multiple objects
  - API, see [SE_CropOSIDynamicGroundTruth()](https://github.com/esmini/esmini/blob/a059a432144445b44f54befaa8a40af9d1f9a155/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1452-L1459)
  - commandline, see `--osi_crop_dynamic` in [User guide - esmini reference](https://esmini.github.io/#_esmini)
- Refactor route waypoint handling for readability and performance
- Add devcontainer.json ([PR #670](https://github.com/esmini/esmini/pull/670))
- Fix parameter actions to let their parent events terminate ([PR #678](https://github.com/esmini/esmini/pull/678))
- Add efficient distance mapping function, [SE_SimpleGetDistanceToObject()](https://github.com/esmini/esmini/blob/a059a432144445b44f54befaa8a40af9d1f9a155/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1281-L1298)
  - takes distance to target into account when to do updates
  - avoids duplicate calculations
  - used by the [NaturalDriver controller](https://esmini.github.io/#_naturaldrivercontroller)
- Add slice road ([slice_xodr.py](https://github.com/esmini/esmini/blob/dev/scripts/scenario_scripts/slice_xodr.py)) and road utility ([road_helpers.py](https://github.com/esmini/esmini/blob/dev/scripts/scenario_scripts/road_helpers.py)) scripts
- Fix ghost setup, only move triggers for acts including private actions ([PR #680](https://github.com/esmini/esmini/pull/680))
- Reset condition states at event restart ([PR #683](https://github.com/esmini/esmini/issues/683))
- Improve road parser error and warning messages
  - more use of string ID, also check junction to junction connections
- Fix issue when excluding replayer from build, still build osiviewer and dat2csv

CI updates:
- Add CI performance regression test suite
  - the script [performance_test.py](https://github.com/esmini/esmini/blob/dev/test/performance_test.py) can be used for various performance checks
- use standard builds for glxinfo and cppcheck

Build config changes:
- Use Ubuntu xvfb-run instead of setup-xvfb GitHub action
- Bump to Ubuntu 22.04 gcc9 and Windows server 2022
- Add some build options ([PR #673](https://github.com/esmini/esmini/pull/673))
  - skip download external dependencies
  - skip build examples
  - skip build odrplot
  - for all options, see [CMakeLists.txt](https://github.com/esmini/esmini/blob/dev/CMakeLists.txt)
- bump cmake min version required (3.10)
- sync static code analysis enable state with actual state


### 2025-02-21 Version 2.45.3

Improvements and fixes:
- Fix AcquirePosition nullptr ref crash ([issue #662](https://github.com/esmini/esmini/issues/662))
- Fix OSI HW junction connectivity
  - previously dependent on xml road element order of appearance
  - support multiple connections in each direction
- Put actual OSI version in log
  - read from OSI interface instead of hardcoded values
- Fix missing OSI traffic sign country and unit support
- Avoid OSI crash on invalid junction road connections
- Cleanup, remove OSI 3.3.1 support

### 2025-02-06 Version 2.45.2

Improvements and fixes:
- Fix build variant USE_OSG=1 USE_OSI=0 ([issue #661](https://github.com/esmini/esmini/issues/661))
- Bump OpenDRIVE validation schema from v1.6 to v1.6.1
- Fix some typos in OpenDRIVE v1.6.1 xsd ([PR #663](https://github.com/esmini/esmini/pull/663))
- Fix issue with multiple waypoints on single road ([PR #665](https://github.com/esmini/esmini/pull/665))
- Add OSMP FMU build and run checks to CI ([PR #667](https://github.com/esmini/esmini/pull/667))
- Fix cosmetic issues in parameter distribution handling (introduced in v2.45.1)
  - Restore log message in case of missing OpenSCENARIO file
  - Restore correct arguments in window title
- Improve odrviewer car spawning at short roads and end points
- Add odrviewer option to pause at start
- Fix precedence of additional declarations of same parameter (introduced in v2.45.1)
  - keep last value instead of only first

### 2025-01-31 Version 2.45.1

Improvements and fixes:
- Use default argument value in [`SE_UpdateOSIGroundTruth()`](https://github.com/esmini/esmini/blob/b684c2609ee5f1182e2ee48d822275f36fb90c80/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1415-L1420)
  - no need to supply argument (at least not from C++ environment)
  - see updated info in [User guide - OSI data via API call](https://esmini.github.io/#_osi_data_via_api_call)
- Extend parameter distribution support
  - make use of `ScenarioFile` reference in [ParameterValueDistribution](https://publications.pages.asam.net/standards/ASAM_OpenSCENARIO/ASAM_OpenSCENARIO_XML/latest/generated/content/ParameterValueDistribution.html)
  - no need to specify both xosc file and parameter distribution file
  - new option to save scenario with resolved parameter values asap (not executing)
  - see updated info in [User guide - Parameter distributions](https://esmini.github.io/#_parameter_distributions)
- Support parameter and variable modification via callback ([issue #624](https://github.com/esmini/esmini/issues/624))
  - by re-evaluating expressions correctly
  - fix issue preventing variable expressions referring to parameters

### 2025-01-24 Version 2.45.0

New features:
- Compile `replayer` also for [slim esmini](https://esmini.github.io/#_slim_esmini_customize_configration) without graphics support
  - offering limited functionality:
    - collision detection report
    - merge dat files

Improvements and fixes:
- Possible to request static OSI GT anytime ([issue #639](https://github.com/esmini/esmini/issues/639))
  - add flag argument to [`SE_UpdateOSIGroundTruth()`](https://github.com/esmini/esmini/blob/043ba440c809e1cbd850e318bc0bf159b88bab0e/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1419)
  - more info in [User guide - OSI data via API call](https://esmini.github.io/#_osi_data_via_api_call)
- Fix csv logger id order issue ([issue #655](https://github.com/esmini/esmini/issues/655))
  - previous assumed id 0 always first object
  - now object order does not matter
- Pass AcquirePositionAction to ghost vehicle ([PR #654](https://github.com/esmini/esmini/pull/654))
- Refactor RoadManager to comply with enabled CI compile warnings checks
- Improve 3D wheel model discoverability in [User guide](https://esmini.github.io/#_colors_textures_and_wheel_rotations) ([PR #657](https://github.com/esmini/esmini/pull/657))

esminiLib/esminiRMlib API updates:
- Add roadmanager lib functions `RM_SetH*` to set heading alone
  - see [esminiRMLib](https://github.com/esmini/esmini/blob/043ba440c809e1cbd850e318bc0bf159b88bab0e/EnvironmentSimulator/Libraries/esminiRMLib/esminiRMLib.hpp#L401C1-L416C63) and [esminiRMWrapper.cs](https://github.com/esmini/esmini/blob/043ba440c809e1cbd850e318bc0bf159b88bab0e/EnvironmentSimulator/Libraries/esminiRMLib/esminiRMWrapper.cs#L402)
- Update some types from int to unsigned int (including id_t and idx_t)
  - should not affect actual functionality
  - undefined indicated by 0xffff ffff instead of -1 (binary identical)

### 2025-01-16 Version 2.44.2

New features:
- Add Adversarial Highway Driver controller based on IDM and MOBIL
  - see more info in [User guide - NaturalDriverController](https://esmini.github.io/#_naturaldrivercontroller)

Improvements and fixes:
- Bugfix: Avoid potential duplicate OSI points rendering
- Clear also a few missed OSI static data items
- Fix bug sometimes causing missed delayed triggers (introduced in v2.44.0)
  - could happen for non fixed time case
- Reset speed when can't move along s, e.g. end of road
- Apply a few fixes for SVADDS bazel build

### 2025-01-14 Version 2.44.1

New features:
- Add ROS 1 support ([PR #648](https://github.com/esmini/esmini/pull/648))
  - esmini node communicating with ROS 1 using esminiLib
  - see [esminiROS/README.md](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/Libraries/esminiROS/README.md) for info

Improvements and fixes:
- Process also road marks of type none
  - solves missing OSI lane edge (bug introduced in v2.41.1)
- Reduce log verbosity for unresolved signal types and models ([issue #629](https://github.com/esmini/esmini/issues/629))
- Fix osiviewer pan and zoom issues when no selection
- Enable serialization of structs in esminiRMLib C# wrapper

Documentation:
- Add reference to [Truevision Designer](https://github.com/truevisionai/designer) under [Related work](https://github.com/palhimanshu1991/esmini/blob/0fe0c2d5379c5e899ca736d5e79f078290c63dfa/README.md#related-work) ([PR #644](https://github.com/esmini/esmini/pull/644))

### 2025-01-03 Version 2.44.0

Fixed behavior:
- Implement correct condition delay handling ([issue #371](https://github.com/esmini/esmini/issues/371))
  - full condition evaluation history instead of simple timer
  - now works with condition AND/OR grouping
  - see more info [User guide - Condition delay](https://esmini.github.io/#_condition_delay)

Improvements and fixes:
- Fix no-response with some OpenDRIVE files ([issue #652](https://github.com/esmini/esmini/issues/652))

Documentation:
- Add new section [User guide - Scenario construction tips](https://esmini.github.io/#_scenario_construction_tips)

### 2024-12-30 Version 2.43.0

Improvements and fixes:
- Implement correct OpenDRIVE laneOffset handling ([PR #626](https://github.com/esmini/esmini/pull/626))
  - see [User guide - Reference line and center lane](https://esmini.github.io/#_reference_line_and_center_lane)
  - [video clip](https://youtu.be/zGrH35kjyqI) showing intersection utilizing laneOffset
  - for reference, see previous behavior [here](https://htmlpreview.github.io/?https://raw.githubusercontent.com/esmini/esmini.github.io/refs/tags/v2.42.0/index.html#_reference_line_and_center_lane_while_using_laneoffset)
- Restore auto-activate any specified custom camera mode
  - bug introduced in 2.41.0
- Smoothen `flex-orbit` camera mode
  - track simulation time instead of system time
- Fix missing transparency ([issue #651](https://github.com/esmini/esmini/issues/651))
- User guide updates:
  - add info on video concatenation [here](https://esmini.github.io/#_create_video_clip_of_a_scenario)
  - add info on viewing previous versions [here](https://esmini.github.io/#_how_to_view_previous_versions)
- Update [README](https://github.com/esmini/esmini/blob/dev/README.md) clarifying OpenSCENARIO *XML* support

Build improvements:
- Ensure unique UDP ports for CI tests, preventing hick-ups

### 2024-12-22 Version 2.42.1

Improvements and fixes:
- Catch all exceptions and quit more gracefully on error
- Fix issue causing some trajectory actions to end immediately
  - applies to nurbs and clothoidSpline shapes
  - bug introduced in v2.40.4
- Improve osiviewer panning
  - also works when moving object is selected
- Add falling bicycle scenario to demo pack

### 2024-12-20 Version 2.42.0

New features:
- Add simple trajectory filter merging close points
  - add option to set filter radius (0.1 default)
  - disable by setting radius = 0.0

Updated behaviors:
- Ensure unique OSI IDs for all static objects
 - regardless of OpenDRIVE or OpenSCENARIO origin
- Fix OSI z-value to center of bounding box instead of bottom
- Rename OSMP_FMU EsminiOsiSource to esmini ([PR #643](https://github.com/esmini/esmini/pull/643))

Improvements and fixes:
- Disable creation of odrviewer and replayer log files as default
- Set unique default filenames for odrviewer and replayer logs
- Improve followRoute controller
  - Fix missing visualization of some waypoints
  - allow for starting anywhere along the route
- Hand over controllers to ghost (except first, which will stick to "host")
- Add example Misc object catalog
- Fix OSMP_FMU missing static data ([PR #643](https://github.com/esmini/esmini/pull/643))
- Allow any orientation in trajectory road- and lane coordinates
  - old behavior aligned pitch and roll to road surface
  - now entities can fall over (see [video clip](https://youtu.be/4c2av2Y8d1E))

### 2024-12-11 Version 2.41.1

New behaviors:
- Apply any OpenDRIVE offset also to scenario world positions
  - skip with option `--ignore_odr_offset`

Improvements and fixes:
- Improve tesselation of road surfaces and roadmarks
  - dashed roadmarks also bend with lane curvature
  - add support for all combined lane mark variants
  - respect OpenDRIVE inner->outer rule for combined marks
- Fix bug causing wrong trajectory driving direction
  - root cause: when speed is 0, floating point could become -0
- Call InitPostPlayer also when running without viewer
- Fix initial wrong lane id when injecting LaneChangeAction

### 2024-12-04 Version 2.41.0

New features:
- Extend logger functionality:
  - verbosity levels
  - code meta data (module, class, line number)
  - module filter
  - more info in User guide [User guide - Logging](https://esmini.github.io/#_logging)
- Support persistent option setting, remaining across scenario runs
  - applies to lib usage
  - more info in User guide [User guide - Program Options](https://esmini.github.io/#_program_options)
- Support OpenDRIVE Offset and append to OSI proj_string
  - road network is transformed accordingly
  - proj_string will contain any geoRef and offset
  - if both are defined, they are separated by semicolon ";"

Improvements and fixes:
- Fix typo roadmark -> roadMark ([issue #635](https://github.com/esmini/esmini/issues/635))
- Add detailed ghost trail return code
  - see relevant lib API [functions](https://github.com/esmini/esmini/blob/8569e0c71018943ad6b5188db528046c88529b6c/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1219C1-L1238C115)
- Align osi lib path in OSMP_FMU build config to new structure from v2.40.0
- Add spdlog as OSMP_FMU submodule for static build ([issue #638](https://github.com/esmini/esmini/issues/638))

### 2024-11-25 Version 2.40.4

Improvements and fixes:
- Fix direct junction OSI lane pairing issue
  - register correct direction (antecessor/successor)
- Extend support for trajectory reversing
  - considering object heading, speed sign and trajectory headings
  - see more info in [User guide - Trajectory moving and driving direction](https://esmini.github.io/#_trajectory_moving_and_driving_direction)
- Fix OpenDRIVE parsing typos
  - a few object types ([issue #635](https://github.com/esmini/esmini/issues/635))
  - roadWorks lane type ([issue #636](https://github.com/esmini/esmini/issues/636))
- Fix sharp curve bug
  - affects special case when reaching curve with radius near zero
  - maintain relative heading and fix wrong z value being set
- Add loaded filename to osiviewer window title

Build improvements:
  - Add debug info to sanitizer builds for increased report readability
  - Allow sanitizer job to be manually triggered on any pushed branch

### 2024-11-18 Version 2.40.3

Improvements and fixes:
- Add OSI support for direct junctions
- Fix OSI position of wheels ([PR #634](https://github.com/esmini/esmini/pull/634))
- Communicate original OpenDRIVE geoReference string
  - In OSI as `proj_string`
  - In esminiRMLib as [`RM_GeoReference.original_georef_str_`](https://github.com/esmini/esmini/blob/175740cdc1bb153150fb0565cbc035e6ab6a68fb/EnvironmentSimulator/Libraries/esminiRMLib/esminiRMLib.hpp#L127)\
    Note: The struct has been extended with this field

### 2024-11-11 Version 2.40.2

Improvements and fixes:
- Add osiviewer intersection support
- support start time > 0 in osiviewer
- Fix bug causing wrong OSI intersection classification
- Rename cspas controller ([PR #630](https://github.com/esmini/esmini/pull/630))
- Restore sumo-test scenario (bug introduced in v2.40.0)

Build improvements:
- Bump to macOS v13 for github actions

### 2024-10-23 Version 2.40.1

Improvements and fixes:
- Respect ACC Controller set-speed as max speed ([issue #622](https://github.com/esmini/esmini/issues/622))
- Optimize OpenDRIVE loading for densely segmented reference lines ([PR #620](https://github.com/esmini/esmini/pull/620))
- Add missing entry to v2.40.0 release notes (Fix OSI stationary position issues)
- Avoid repeated forced download of external dependencies
  - apply only once by not storing flag in cmake cache

### 2024-10-22 Version 2.40.0

New features:
- Script [run_schema_comply.py](https://github.com/esmini/esmini/blob/dev/scripts/run_schema_comply.py) for validation of OpenSCENARIO XML and OpenDRIVE files
  - See [User guide - Run XML schema validation tests](https://esmini.github.io/#_run_xml_schema_validation_tests) how to use it.
- Script [osiviewer.py](https://github.com/esmini/esmini/blob/dev/scripts/osiviewer.py) for visualizing OSI data
  - experimental and limited, only a small subset of OSI content supported
  - see brief info in [User guide - Scripts/osiviewer](https://esmini.github.io/#osiviewer)
- Script [move_to_origin.py](https://github.com/esmini/esmini/blob/dev/scripts/move_to_origin.py) for translating scenarios to origin
  - useful for scenarios with large coordinates causing precision issues
  - moves the road network and updates scenario coordinates accordingly
- Script [xodr_lines2curves.py](https://github.com/esmini/esmini/blob/dev/scripts/xodr_lines2curves.py) for smoothening polyline roads
  - converts all line geometries into parametric curves achieving full continuity
  - experimental with limitations, e.g. junctions not considered

Improvements and fixes:
- All xodr and xosc scenario files now complies with the XSD of the standards!
- Fix issue preventing debugging of dynamically linked OSI library
  - OSI prebuilt binaries have been updated
  - solves faulty SONAME symbol within dynamic library after lib rename
  - separate release and debug libs by folder structure instead
- Update replayer on-screen info also during pause
- Avoid replayer crash when pressing 'g' while inactive objects
- Fix OSI stationary position issues
  - reset start position of outlines created from repeats
  - deliver outline local coordinates without rotation

NOTE: The OSI lib update requires external dependencies to be updated. See [User guide - Update 3rd party prebuilt libraries](https://esmini.github.io/#_update_3rd_party_prebuilt_libraries).

### 2024-09-24 Version 2.39.0

New features:
- Support VariableAddAction and VariableMultiplyByAction ([PR #597](https://github.com/esmini/esmini/pull/597))

New behaviors:
- Update road and junction ID type and default value ([issue #612](https://github.com/esmini/esmini/issues/612))
  - from signed to unsigned 32 bit integer
  - allows IDs from 0 to 4 294 967 294
  - 0xffffffff (4 294 967 295) is reserved for ID_UNDEFINED
  - ID_UNDEFINED indicates uninitialized or error (previously -1)

Improvements and fixes:
- Take full orientation into account in OSI MovingObject position ([PR #607](https://github.com/esmini/esmini/pull/607))
- Add heading information to [SE_RouteInfo](https://github.com/esmini/esmini/blob/42c0f3c8cc597c1dbe74bca67d90264556378fe5/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L103-L116) ([PR #599](https://github.com/esmini/esmini/pull/599))
- Handle visibility changes for vehicles spawned dynamically ([PR #603](https://github.com/esmini/esmini/pull/603))
- Add a few missing functions to C# wrapper ([PR #604](https://github.com/esmini/esmini/pull/604))
  - SE_RegisterStoryBoardElementStateChangeCallback()
  - SE_ClearOSIGroundTruth()
- Fix Road and Junction ID creator
  - At ID conflict, replace old and keep latest
  - String and conflict IDs replaced internally by generated number
  - Generated number always increments from previous max value

Other:
- Add [link to brief roadmap](https://docs.google.com/spreadsheets/d/e/2PACX-1vS83IWhiCWxVlDlx_51BsIZMihcy1mfZmC7YF-Mm6FyDA-ghMGaoZnmS207MaoxHdVoX2j4XKAH5u4T/pubhtml) to [homepage README](https://github.com/esmini/esmini)
- Add [reference](https://github.com/esmini/esmini/tree/master#ncap-scenarios) to NCAP scenarios
- Add how to run sanitizers locally to [User guide - Sanitizers](https://esmini.github.io/#_sanitizers)

### 2024-09-13 Version 2.38.5

Improvements and fixes:
- Bugfix: Restore segment interpolation mode after ghost restart

### 2024-09-11 Version 2.38.4

Improvements and fixes:
- Support InRoutePosition/FromRoadCoordinates
- Support `pi` and `e` constants in expressions
- Add support for trajectory coordinate system distance measurement
- Add [Vector NCAP scenarios](https://github.com/vectorgrp/OSC-NCAP-scenarios) to CI smoke test suite ([issue #595](https://github.com/esmini/esmini/issues/595))
- Add option to align lateral sign of route positions
  - `--align_routepositions`
  - consider route direction when evaluating sign of `laneId` and `t`
  - use with NCAP scenarios to get it right
- Change synchronize default distance tolerance from 1.0 to 0.0
- Fix crash due to empty wheel data vector
  - affecting e.g. ad_hoc_traffic code example
- Fix crash due to swarm traffic inherit controllers ([issue #601](https://github.com/esmini/esmini/issues/601))
  - when no vehicle catalog available, central object is used for swarm
  - issue was that controller reference was reused and eventually deleted
  - now controller will be unassigned for the swarm copy vehicle
- Bugfix: Restore heading interpolation
  - interpolate heading along segments in object trails
  - unintentionally disabled in v2.38.0
- Bugfix: Avoid route being disabled when position is reported via lib
- Bugfix: Resolve variable value from parameters, not variables
- Bugfix: Resolve position dependencies at synchronize start

Build improvements:
- Add symbols to sanitizer build for increased readability

### 2024-09-02 Version 2.38.3

Improvements and fixes:

- Consider trailers for freespace distance calc ([issue #594](https://github.com/esmini/esmini/issues/594))
- Add lib function ([SE_GetObjectGhostId()](https://github.com/esmini/esmini/blob/707c5b93e6158fca7d6fe0e9d9d6ca2dbc51f06c/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1050-L1055)) to get ghost ID
- Support UserDefinedAction in Init section
  - previously only supported in storyboard
- Adjust order of Init action execution according to OSC xsd sequence
- Fix parsing bug causing wrong bounding box settings
  - bug was introduced in v2.38.2 (previous version)

### 2024-08-29 Version 2.38.2

Improvements and fixes:
- Add further wheel info, not only friction
  - both in lib and OSI
  - see [struct](https://github.com/esmini/esmini/blob/d6308edad511ee37a76009117c834619435960ac/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L55-L71) for what's populated
  - see API [here](https://github.com/esmini/esmini/blob/d6308edad511ee37a76009117c834619435960ac/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1099-L1113)
- Add timestamp to ghost trail info
  - as return argument to [SE_GetRoadInfoAlongGhostTrail()](https://github.com/esmini/esmini/blob/fac842e13679afcbc7897fc10d60ff9721ec2405/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1147-L1156) function
- Improve route handling
  - always map position to closest point on route
  - fix missing route position update, e.g. for externally reported positions
  - refactor and simplify code
  - add visuals for followRouteController waypoints (dim colored)
  - indicate start and end waypoints by larger scaled visuals
- Add a few functions for route info
  - [SE_GetRouteTotalLength()](https://github.com/esmini/esmini/blob/d6308edad511ee37a76009117c834619435960ac/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1703-L1708C60)
  - [SE_GetObjectRouteStatus()](https://github.com/esmini/esmini/blob/d6308edad511ee37a76009117c834619435960ac/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L995C1-L1000C59)
- Fix elevation mapping bug
  - could cause entity to drop to lowest road, e.g. from bridge
  - bug was introduced in v2.37.0 (Feb 2024)

Build improvements:
- Establish nightly sanitizer job
- Add manual CI and sanitizer job triggers
- Fix download issue for build dependent packages
- Fix cmake version deprecation

### 2024-08-16 Version 2.38.1

Improvements and fixes:
- Accept negative values for deceleration parameters in ALKS controller
  - i.e. consider only absolute values of DriverDeceleration and AEBDeceleration
- Increase test coverage by enabling all relevant tests also for non OSG builds in CI

### 2024-08-15 Version 2.38.0

Updated behaviors:
- Polyline trajectory heading interpolation now purely depends on followingMode
  - "position" -> interpolate only around corner
  - "follow" -> interpolate along whole linear segment
  - not considering whether heading was specified or not
  - see updated table in [User guide - Trajectory interpolation and alignment](https://esmini.github.io/#_trajectory_interpolation_and_alignment)
  - new [option](https://esmini.github.io/#_esmini), `--disable_pline_interpolation`, to skip interpolation altogether

- Adapt Trailer implementation to OpenSCENARIO 1.3
  - changes from previous prototype implementation:
    - rename EntityRef element to TrailerRef
    - Add Trailer parent element for inline and catalog trailers
- VehicleCatalog updated to match OpenSCENARIO 1.3
  - trailer configurations updated accordingly

Improvements and fixes:
- Refactor Position implementation and handling
  - simplify and clarify relative/absolute mode handling in code
  - reduce memory leaks

NOTE: Unexpected side effects may have been introduced by the refactorization. Please raise issue if encountered.

### 2024-08-13 Version 2.37.17

Updated behaviors:
- Update target speed for relative speed action continuously
  - previously target speed was sampled only once (at start of action)

Improvements and fixes:
- Fix relative target speed not reset for repeated actions ([issue #589](https://github.com/esmini/esmini/issues/589))
- Fix bug in condition group logging ([PR #587](https://github.com/esmini/esmini/pull/587))
- Add [ref_point.xosc](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/Unittest/xosc/ref_point.xosc) demonstrating effect of various ref points ([issue #590](https://github.com/esmini/esmini/issues/590))
- For injected actions, create unique name and replace ongoing similar action
  - compose name of action type and action counter
  - abort any ongoing action of same type and object
- Fix SUMO controller elevation bug
  - use SUMO z value as is (absolute, not relative)

### 2024-07-01 Version 2.37.16

Fixes of bugs introduced in v2.37.15:
- Fix wrong positioning of waypoint and trajectory visualization
- Fix intermittent coloring issue in viewer (due to scenegraph misconfig)

### 2024-07-01 Version 2.37.15

Improvements and fixes:
- Support (partly) string IDs for roads and junctions
  - integer ID is generated and assigned for internal use
  - the integer ID is the one being logged and reported
- Add arguments to ignore z, pitch and roll inputs ([PR #583](https://github.com/esmini/esmini/pull/583))
  - aligning the vehicle to the road instead
- Improve support for distant road networks
  - very high coordinates (>1+E6) previously resulted in shaky graphics
  - now fixed by coordinate transformation
- Fix outdated C# RMLib wrapper functions ([issue #584](https://github.com/esmini/esmini/issues/584))
- Make road alignment on Controller activation optional
  - previously heading was always aligned at activation/deactivation
  - now behavior is set by each controller
- Fix range for lane Id randomizer in odrviewer
- Fix bug in junction Id check
  - junction ID 0 mistaken for no junction
- Accept and adjust lane section s-value beyond road length

### 2024-06-26 Version 2.37.14

Improvements and fixes:
- Fix ALKS ref driver wrong TimeHeadway edge case calculation
  - was wrongly set to inf (along with ttc) when relative speed < 0
- Add a few parameters to ALKS ref driver
  - Driver brake rate
  - AEB brake rate
  - AEB available (or not)
- Bump CI to macos-12
- Add GetAcceleration components functions
  - [SE_GetObjectAccelerationGlobalXYZ()](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1060)
  - [SE_GetObjectAccelerationLocalLatLong()](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1069)
- Respect road rule for LH or RH traffic ([issue #581](https://github.com/esmini/esmini/issues/581))

### 2024-06-24 Version 2.37.13

Improvements and fixes:
- Embed execution search paths for dynamic libraries on Linux
  - $origin (location of exe file) and expected lib location
  - applies to esmini-dyn and dynamically linked protobuf
- Relax (totally remove) replayer dependency to OSI and protobuf
- Bump OSI lib to Visual Studio toolset v142 and add missing protobuf DLLs
  - remove externals/osi folder and re-run cmake command to get latest
- Fix wrong filename reference for dynamic protobuf lib on Windows
- Add [minimalistic build example](https://esmini.github.io/#_simplest_example) to User guide

### 2024-06-13 Version 2.37.12

Improvements and fixes:
- Make the swarm traffic direction based on RHT or LHT ([PR #575](https://github.com/esmini/esmini/issues/575))
- Add lib [function](https://github.com/esmini/esmini/blob/96f270531c1b4c2d57041a9d783352e256d64427/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L974-L984) to get type of lane object is within
  - ignoring any lane type snap settings
  - add to both esmini and esminiRM libs
- Try any specified scenario file full path first, avoiding wrong file being loaded
- Ensure normalized normals in scaled 3D models
  - previously lighting could be strange for non-uniform scaled objects
- Bugfix: Prevent adding duplicate trailers causing segmentation fault at exit
- Fix FollowRouteController first waypoint to include current position
  - getting rid of "Entity moved out of route" message
- Restore odrviewer even lane distribution for initial spawning

### 2024-05-16 Version 2.37.11

Improvements and fixes:
- Support empty roads, i.e. without geometries and lanes
  - useful for infinite open space with additional properties, e.g. speed limit
  - clarification: esmini also supports skipping OpenDRIVE file altogether
- Fix TTC calculation issue ([issue #569](https://github.com/esmini/esmini/issues/569))
  - Consider direction of motion
  - Relative distance based on entity heading instead of lane direction
- Fix trajectory issues
  - Skip speed update for zero timestep
  - Skip update of non ghost objects during ghost headstart

### 2024-05-08 Version 2.37.10

Improvements and fixes:
- Add brief info on custom camera API to [User guide - Custom camera positions](https://esmini.github.io/#_custom_camera_positions)
- Apply VisbilityAction to connected tow and trailer vehicles ([PR #565](https://github.com/esmini/esmini/issues/565))
- Add missing print camera pose key command ('K') to help
- Bump jinja2 from 3.1.3 to 3.1.4 ([PR #567](https://github.com/esmini/esmini/issues/567))
- Add slight Z offset to road markings, avoiding flickering in exported models
- Fix bug causing generated objects end up at origin (skip destructive optimization)
- Add option to skip textures in generated model (--generate_without_textures)

### 2024-04-26 Version 2.37.9

Improvements and fixes:
- Add missing esminiLib interface for OSC variables
- odrviewer bugfix, avoid crash when respawning left hand traffic ([issue #563](https://github.com/esmini/esmini/issues/563))
  - consider RHT/LFT rule when finding available driving lanes
- odrviewer bugfix, avoid crash for no driving lane road networks ([issue #562](https://github.com/esmini/esmini/issues/562))
  - prevent feeding bad range to randomizer
- Add script for multiple comparison runs ([run_repeat_compare.sh](https://github.com/esmini/esmini/blob/dev/scripts/run_repeat_compare.sh)).

### 2024-04-12 Version 2.37.8

Improvements and fixes:
- Add missing GetObjectAcceleration() declaration to C# wrapper
- Resolve controller domain conflicts
  - when activating a controller, disable any active one on conflicting domains

### 2024-04-10 Version 2.37.7

Improvements and fixes:
- Add missing bounding box to SUMO vehicles
- Fix broken link to osi2read.py script in [User guide - Save OSI data](https://esmini.github.io/#_save_osi_data)

### 2024-04-05 Version 2.37.6

New content:
- Add script, [dat2xosc.py](https://github.com/esmini/esmini/blob/dev/scripts/dat2xosc.py), to convert selected dat content to xosc
  - fixates trajectory for one or several actors
  - brief info in file header comment

Improvements and fixes:
- Add lib [function](https://github.com/esmini/esmini/blob/03a4015a1fe6cd667f1667f7d03fd1fb215497ca/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1019-L1024) to retrieve object acceleration
- Add lib [function](https://github.com/esmini/esmini/blob/db03994adbbece1d7a35f66ec3c26179b234c468/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1312) to close log file ([issue #556](https://github.com/esmini/esmini/issues/556))
  - it will otherwise stay open for concatenating further runs
- Add Python example, [drive.py](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/hello_world/drive.py), using SimpleVehicle model ([issue #555](https://github.com/esmini/esmini/issues/555))
  - run from esmini/bin:<br>
    `python ../EnvironmentSimulator/code-examples/hello_world/drive.py`
  - drive using arrow keys
- Add controller experimental virtual operation mode
  - in this mode the controller only calculates acceleration, not applying it
  - for use cases with an external vehicle dynamics model
  - example code and scenario: [mixed_control](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/mixed_control)
  - brief info in [mixed_control.cpp](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/mixed_control/mixed_control.cpp) file header comment
- Fix ghost trail issue after long standstill
  - at end of standstill phase, add another sample
  - avoiding slightly wrong interpolated speed
- Fix controller type not being updated in object state
  - type now set whenever a controller is activated on longitudinal domain
  - not only for the initially assigned controller, which was the case

### 2024-03-27 Version 2.37.5

New features:
- Support overlay text scaling ([issue #548](https://github.com/esmini/esmini/issues/548))
  - launch argument: `--text_scale <scale factor>`

Improvements and fixes:
- Indicate ClothoidSpline support in [osc_coverage.txt](https://github.com/esmini/esmini/blob/dev/osc_coverage.txt)
- Recommend newer protobuf version for Python scripts
  - root cause is missing builder.py in older versions
  - changing recommended version from 3.19 to 3.20.2
- Align float precision in print statement ([issue #549](https://github.com/esmini/esmini/issues/549))
- Check model id and warn if not matching model3d attribute ([issue #547](https://github.com/esmini/esmini/issues/547))
  - but only if scenario is being recorded (into a dat file)
- Fix typo in CSV Logger, replace "Entitity" label by "Entity" ([PR #552](https://github.com/esmini/esmini/issues/552))
  - might affect processing of files created with `--csv_logger` argument
- Add brief info on ideal sensors ([issue #553](https://github.com/esmini/esmini/issues/553))
  - see [User guide - Simple ideal sensors](https://esmini.github.io/#_simple_ideal_sensors)
- Add another search path for model_id file, solving load failure
- Support trajectory pitch and roll interpolation ([issue #543](https://github.com/esmini/esmini/issues/543))
  - only for floating trajectories, i.e. world pos with explicit z values
  - see [videoclip](https://youtu.be/zlSE6kKN-fc) of [bike_tilt_smooth.xosc](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/Unittest/xosc/bike_tilt_smooth.xosc)
- Establish initial speed from trajectories during initialization
- Avoid crash (null ptr ref) after viewer failure

### 2024-03-22 Version 2.37.4

Updated behaviors:
- Injected action cancels any ongoing action of same type
  - use function `SE_InjectedActionOngoing()` to check for ongoing actions
  - see updated examples [action_injection.cpp](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/hello_world/action_injection.cpp) and [action_injection.py](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/hello_world/action_injection.py)
- Add ground plane when environment is missing, i.e:
  - when no OpenDRIVE specified and specified scenegraph failed to load
  - when neither OpenDRIVE nor scenegraph specified

Improvements and fixes:
- Fix route catalog parameter handling ([issue #544](https://github.com/esmini/esmini/issues/544))
- Add replayer capability to smoke test framework
- Improve replayer collision detection ([issue #483](https://github.com/esmini/esmini/issues/483))
  - detect collision between any objects
  - detect multiple simultaneous overlaps
- Add example how to run all scenarios in a folder
  - see end of [User guide - Replay scenario](https://esmini.github.io/#_replay_scenario)
- Update [coverage](https://github.com/esmini/esmini/blob/dev/osc_coverage.txt) and [osc-extensions](https://github.com/esmini/esmini/blob/dev/docs/osc-extensions.xml) info
  - Trailer support now part of OpenSCENARIO (v1.3), add to coverage
  - SteadyState added to SynchronizeAction (osc v1.1), remove from extensions

### 2024-03-19 Version 2.37.3

Improvements and fixes:
- Fix ghost controller type wrongly reported as default instead of ghost reserved type
  - bug introduced in v2.37.2
- Fix pitch/roll mix-up in relative world- and object positions
  - bug introduced in v2.36.0

### 2024-03-15 Version 2.37.2

New features:
- Support multiple assigned controllers per object (introduced in OpenSCENARIO XML v1.2)
  - example (external app steering while ACC ctrl active): [dual_controllers.cpp](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/hello_world/dual_controllers.cpp) / [acc_with_external_controller.xosc](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/hello_world/acc_with_external_controller.xosc)

Improvements and fixes:
- Fix continuous repeat object double width issue
- Add some info about esmini SUMO integration in [User guide - SUMO integration](https://esmini.github.io/#_sumo_integration)

### 2024-03-04 Version 2.37.1

New features:
- Add action injection capabilities to [esminiLib API](https://github.com/esmini/esmini/blob/8baa199fb3a5b49c7dfb3accf33be5a3410241f7/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1552C1-L1575C1)
  - see example [User guide - Inject actions](https://esmini.github.io/#_inject_actions) (C++ and Python)

Improvements and fixes:
- Fix OSI header file dependency for non OSI builds ([issue #540](https://github.com/esmini/esmini/issues/540))
- Group sign bounding boxes with odr features, toggle key 'o' ([issue #530](https://github.com/esmini/esmini/issues/530))
- In lib header, replace outdated element state description with enum reference ([issue #541](https://github.com/esmini/esmini/issues/541))

### 2024-02-26 Version 2.37.0

New features:
- Add friction support
  - support OpenDRIVE lane material friction
  - populate OSI wheel data ([issue #527](https://github.com/esmini/esmini/issues/527))
  - visualize low friction as blue, high as red
  - see brief info in [User guide - Friction](https://esmini.github.io/#_friction)
- Add duration option (`--duration <time>`) in odrviewer
  - play for specified duration
  - `--duration 0` will quit immediately after one frame
- Flag to skip create signs when external scene-graph is loaded ([issue #530](https://github.com/esmini/esmini/issues/530))
  - always create wire-frame sign bounding boxes (toggle 'O')
  - skip load or create sign model when external scene-graph loaded and flag set

Improvements and fixes:
- New method for controller initialization after player (and viewer) init
  - enables e.g. adding sensors to objects from controller ([issue #532](https://github.com/esmini/esmini/issues/532))
  - see example in [ControllerACC.cpp](https://github.com/esmini/esmini/blob/1bed88b588f77ce230128ccb05554c1453b99382/EnvironmentSimulator/Modules/Controllers/ControllerACC.cpp#L70-L74) (enable line 73, and press 'r' to see frustum)
- Expose visibilityMask in esmini lib ([issue #535](https://github.com/esmini/esmini/issues/535))
- Improve LaneChangeAction and fix edge case
  - fix slightly wrong heading calculation
  - ensure total speed (lat + long) is maintained
  - handle edge case of requested lateral movement exceeding speed
  - support only lateral speed component (edge case above)
- Fix CommonMini version generation ([PR #537](https://github.com/esmini/esmini/issues/537))

### 2024-02-10 Version 2.36.5

New features:
- Support paths in StoryboardElementStateCondition ([issue #529](https://github.com/esmini/esmini/issues/529))
  - allows for prefix element name with parent name
  - e.g. Event2::MyAction
- Add odr signal bounding box ([issue #530](https://github.com/esmini/esmini/issues/530))
  - use as stand-in 3D model if none could be loaded
  - for signal 3D models, optional wire frame bounding box (toggle on key 'O')

New behaviors:
- Replace storyboard element path separator '/' with '::'
- Fix OSI stationary object ids, now start at 1 and not 0 (which is reserved for undefined)

Improvements and fixes:
- Fix issue in calculation of relative orientation components ([issue #496](https://github.com/esmini/esmini/issues/496))
- Add position conversion code example ([convert_position_type](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/convert_position_type))
  - shows ways to print or convert world positions (x,y) to road coordinates
  - [manipulate_positions.py](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/convert_position_type/manipulate_positions.py) makes trajectories portable wrt road characteristics
- Fix link issue with gcc 7.5 on Ubuntu

### 2024-02-02 Version 2.36.4

New features:
- Add OSI Traffic Command to [OSMP](https://github.com/esmini/esmini/tree/dev/OSMP_FMU)
  - initial version with limited coverage
  - only a few actions supported yet (teleport, lane change, speed change)

Improvements and fixes:
- Fix bugs in World (X,Y) to Road/Route position conversion when off route ([issue #523](https://github.com/esmini/esmini/issues/523))
  - don't limit search to route roads only
  - especially when current position is out of route bounds
- Respect full path for permutation artifacts ([issue #526](https://github.com/esmini/esmini/issues/526))
  - e.g. log, dat, osi and csv files
  - include any specified folder path
- Maintain longitudinal speed in step (immediate) lane change actions

### 2024-01-31 Version 2.36.3

New features:
- Add full element path name to the storyboard element state change callback function
  - including all parent names delimited by '/'
  - see updated test case: [StoryBoardElementStateCallbackInstance1](https://github.com/esmini/esmini/blob/da3e8006acddc03fe69d36f7eadace436ef21e21/EnvironmentSimulator/Unittest/ScenarioEngineDll_test.cpp#L3819-L3862)
  - and Python example code: [hello_world/storyboard_state_callback.py](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/hello_world/storyboard_state_callback.py)
- Add OSITrafficCommand embryo ([issue #499](https://github.com/esmini/esmini/issues/499))
  - some initial code outlining a possible approach
  - recent storyboard state handling updates ensures no command is missed
  - code example: [osi-traffic_command](https://github.com/esmini/esmini/tree/dev/EnvironmentSimulator/code-examples/osi-traffic_command)
- Respect and apply jerk settings (acc/dec change rate) in LongitudinalDistanceAction
- Add scooter vehicle
  - add entry in vehicle catalog
  - add simple 3D model to model pack
    - remove esmini/resources/models folder and re-run `cmake ..` to download
    - or fetch latest pack manually from [here](https://esmini.asuscomm.com/AICLOUD779364751/models/models.7z)

Improvements and fixes:
- Skip velocity and acceleration updates after teleport action, avoiding spikes
- Fix bug in DistanceCondition not resolving relative positions correctly
- Fix presets for Linux VSCode including debug support
  - User guide updated as well, see [Run and Debug with Linux and visual studio code](https://esmini.github.io/#_run_and_debug_with_linux_and_visual_studio_code)
-  Major storyboard code refactorization
    - replace mega nested loop with full OO approach
    - improve element state condition handling, no transition misses


### 2024-01-10 Version 2.36.2

New features:
- Add "off-road" follower controller
  - a simple "follow-the-leader" controller for open spaces
  - see more info in [User guide - OffroadFollowerController](https://esmini.github.io/#_offroadfollowercontroller)
  - example scenario [offroad_follower.xosc](https://github.com/esmini/esmini/blob/dev/resources/xosc/offroad_follower.xosc) and [video clip](https://youtu.be/uHqdsORPsGE)
  - code module: [ControllerOffroadFollower.cpp](https://github.com/esmini/esmini/blob/feature/add_offroad_follower/EnvironmentSimulator/Modules/Controllers/ControllerOffroadFollower.cpp)

Improvements and fixes:
- Fix and update ideal sensor API ([issue #514](https://github.com/esmini/esmini/issues/514))
  - Fix wrong return code in SE_AddObjSensor, now returning unique sensor ID
  - Add functions to retrieve number of sensors, both total and per object
- Reset status in SpeedAction for any additional run
  - enables same speed action to be correctly re-triggered
  - example scenario: [drive_when_close.xosc](https://github.com/esmini/esmini/blob/dev/resources/xosc/drive_when_close.xosc), drive while close to another car else stop

Build improvements:
- Automatically attempt download dependent packages from all (3) available sources
- Add implot package to CI cache (faster total build time)
- Exclude implot from [slim esmini](https://esmini.github.io/#_slim_esmini_customize_configration) build (in CI)

### 2024-01-04 Version 2.36.1

Fixes:
- Fix and update esminiRMLib functions for recent positioning updates
- Support trailerRef attribute name in ConnectTrailerAction

Compile and CI updates:
- Restore default setting not treating warnings as errors (still enabled for CI)
- Fix compiler warning in OSI + non-OSG builds
- Run all relevant unit tests also for non-OSG builds

### 2024-01-03 Version 2.36.0

Improvements and fixes:
- Fix relative positioning over connections and junctions ([issue #502](https://github.com/esmini/esmini/issues/502))
  - prioritize own route over referenced object's one
  - if neither entities has assigned route, use default routing
- Improve route handling, don't get stuck outside defined route path ([issue #501](https://github.com/esmini/esmini/issues/501))
- Consider full orientation for relative object positioning ([issue #509](https://github.com/esmini/esmini/issues/509))
  - align the relative delta x, y, z components with reference entity
- Accept zero length NURBS trajectories
- Randomize equal weighting junction choices
  - randomize choice between candidate roads with equal outgoing angle
- Fix outdated list of storyboard element states in [storyboard_state_callback.py](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/hello_world/storyboard_state_callback.py)
- Fix `--threads` mode missing quit request from viewer
- Cosmetics: Limit steering rate for calculated steering angle

New features:
- Add `--pause` launch flag
  - halt after initialization
  - press `Space` or `Enter` to continue or step
- Add runtime control to player server (former action server) ([issue #497](https://github.com/esmini/esmini/issues/497))
    - new feature: support play, pause, step commands
    - see example: [inject_actions.py](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/hello_world/inject_actions.py) and [video clip](https://youtu.be/FmLtwj518No)
    - improvement: avoid duplicate actions on same object

Updated behaviors:
- Treat fixed timestep = 0 as non fixed (realtime)

### 2023-12-16 Version 2.35.0

New features:
- Add support for OpenDRIVE parking space
- Support string concatenation expressions
  - see info in [User guide - Expressions/Strings](https://esmini.github.io/#_strings)
  - example: OvertakerBrakeEvent startTrigger in [cut-in.xosc](https://github.com/esmini/esmini/blob/3a7c3027f93b690bcb49f768a1604dce2cb66d49/resources/xosc/cut-in.xosc#L162-L163)

Improvements and fixes:
- Don't automatically stop when all acts are done
  - continue until storyboard stopTrigger hits
  - or, if stopTrigger is missing, continue "for ever"
  - Note: This change might require moving act stop trigger to storyboard

Additional information:
- Due to a mistake leading to master branch out of synch with dev branch and <br>
  release tag v2.34.1 stuck on dev, it was decided to revert master history back <br>
  to v2.34.0. This might lead to issues when pulling master. If that happens, resync <br>
  your local master branch as described in [User guide - Branch strategy](https://esmini.github.io/#_branch_strategy).

### 2023-12-11 Version 2.34.1

Improvements and fixes:
- Fix bug preventing trigging and evaluation of subsequential stories
  - bug introduced in 2.34.0
- Improve image handling (fetch or save rendered images)  ([issue #173](https://github.com/esmini/esmini/issues/173))
  - off-screen (to RAM) rendering off by default (for performance)
  - API now allows to activate before init
  - Example how to fetch images from multiple cameras: [multiple-cameras.cpp](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/image-capture/multiple-cameras.cpp)
  - Note: Activating off-screen before init is now required to include initial frame

### 2023-12-08 Version 2.34.0

New features:
- Support OpenDRIVE explicit road lines

Improvements and fixes:
- Fix issue with trajectory relative positions not updating
  - too eager optimization skipped initial evaulation of trajectory
  - bug introduced in v2.33.0
- Restore OSI optimization and add documentation
  - static OSI content should only written first step (not always been the case lately)
  - add brief info on OSI API for programmers: [User guide - OSI data](https://esmini.github.io/#_osi_data)
- Improve and simplify storyboard state handling
  - identity act complete state
  - support zero stories (Init actions only)
  - make state transitions immediately
    - Note: Might affect trigger timing slightly
  - some code clean-up, e.g. centralize storyboard update
- Fix low curvature clothoids mistakenly identified as lines
- Fix position calculation issues
  - skip reference lane in relative laneId calculations
  - fix initialization bugs wrt relative/absolute orientation type
  - fix bugs in relative position
  - Note: These changes might affect pose (pos + rot) behavior
- Add code example changing catalog parameter values
  - [override-bounding-box.cpp](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/parametric-init/override-bounding-box.cpp)
- Add rmlib c# example
  - [rm-basic-cs](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/rm-basic-cs)
- Fix FBX SDK download issue in osg apps build and update info to support DAE
  - script: [compile_osg_apps.sh](https://github.com/esmini/esmini/blob/dev/scripts/compile_osg_apps.sh)
  - info: [User guide - Get osgconv](https://esmini.github.io/#_get_osgconv)
- Add Python OSI receiver example ([issue #500](https://github.com/esmini/esmini/issues/500))
  - [osi_groundtruth_from_udp.py](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/hello_world/osi_groundtruth_from_udp.py)
- Add hint how to create .dat files from multiple scenarios in one command
  -- See end of section [User guide - Scenario recording (.dat)](https://esmini.github.io/#_scenario_recording_dat)
- Align trailer actions with updated concept (candidate for OpenSCENARIO 1.3)
  - Introduce separate actions for connect and disconnect trailer
  - Add parent umbrella action TrailerAction
  - See updated example [trailer_connect.xosc](https://github.com/esmini/esmini/blob/dev/resources/xosc/trailer_connect.xosc)
- Handle timestamps in ClothoidSpline prototype implementation
  - ClothoidSpline shape is an OpenSCENARIO 1.3 feature candidate
- Add link to User guide git commit history under [Version](https://esmini.github.io/#_version)

### 2023-11-17 Version 2.33.2

New feature:
- Add clothoid spline trajectory shape
  - Enables sequence of clothoids in one trajectory
  - Prototype implementation of OpenSCENARIO 1.3 feature candidate
  - Example scenario: [lane-change_clothoid_spline_based_trajectory.xosc](https://github.com/esmini/esmini/blob/dev/resources/xosc/lane-change_clothoid_spline_based_trajectory.xosc)

Improvements and fixes:
- Fix graphics flickering of overlapping road, sidewalk, and grass areas
- Identify bidirectional lane type as drivable
- Fix wrong position mode setting in [Hello World external Ego example](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/hello_world/hw4_external_ego_control.cpp)


### 2023-11-10 Version 2.33.1

Updated behaviors:
- Fix wrong t axis for offset in RelativeLanePosition with dsLane
  - ds will alight s/t to road direction, dsLane will align s/t to lane direction
  - for dsLane, negative offset will be right in lane direction and positive left

Improvements and fixes:
- Fix missing polyline length causing unintended trajectory loop
- Improve continuous road objects ([issue #489](https://github.com/esmini/esmini/issues/489))
  - model as 3D outline object instead of individual bounding boxes
  - when 3D model is provided, individual objects are still populated
  - OSI populated accordingly, as polygon vertices or bounding box
- Add Python examples on how to initialize esmini with launch arguments
  - [Initialize esmini by flexible argument list](https://esmini.github.io/#_initialize_esmini_by_flexible_argument_list)
  - [Forward command line arguments](https://esmini.github.io/#_forward_command_line_arguments)
- Add info on how to change fps with ffmpeg ([here](https://esmini.github.io/#_create_video_clip_of_a_scenario))
- Add OSI UDP socket API to C# wrapper

### 2023-10-27 Version 2.33.0

New features:
- Report parking spot on OSI
- Add support for RelativeLanePosition dsLane mode
- Add flexible interpolation and positioning modes
  - introduce mode which can be RELATIVE or ABSOLUTE
  - mode can be set per position component Z (elevation), heading, pitch and roll
  - trajectory orientation interpolation
  - see info in [User guide - Positioning](https://esmini.github.io/#_positioning)

Updated behaviors:

- Camera focus on OSC object bounding box center
  - instead of bounding sphere of object 3D geometry
- Accept zero stories as introduced in OSC 1.2
- Output also relative camera position on 'K' key event

Improvements and fixes:
- Add some more details to the version string
  - most recent tag
  - nr builds from tag
  - indicate local changes (dirty)
  - based on `git describe`
- Fix superelevation calculation
  - project along Z instead of rotating road boundaries
  - harmonizing with object orientation and positioning
- Fix dashed road mark visualization bug
- Fix condition delay issue preventing looped storyboard elements to trigger
- Fix bug in cmake contibuting to sanitizer run failure
- Update recent dropbox links for new policy ([issue #482](https://github.com/esmini/esmini/issues/482))

### 2023-09-29 Version 2.32.1

New features:
- Support UserDefinedAction as wait/noop action
  - see example in [user_defined_action.xosc](https://github.com/esmini/esmini/blob/2af9304767d8b196bb00c6ecc7cb0cb725123b31/EnvironmentSimulator/Unittest/xosc/user_defined_action.xosc#L74)
- Add ConnectTrailerAction prototype
  - action for connecting/disconnecting trailer
  - see [video clip](https://youtu.be/0NOX1we5dZ0) and example scenario [trailer_connect.xosc](https://github.com/esmini/esmini/blob/dev/resources/xosc/trailer_connect.xosc)

Updated behaviors:
- Support activating controller on several domains in mulitple activation steps
  - see example in [distance_test.xosc](https://github.com/esmini/esmini/blob/2af9304767d8b196bb00c6ecc7cb0cb725123b31/resources/xosc/distance_test.xosc#L52)
  - previvious behavior was to reset all domains for each activation

Improvements and fixes:
- Add esmini, OSI and OSMP version to OSMP_FMU modelDescription.xml ([PR #476](https://github.com/esmini/esmini/pull/476))
- Ensure synchronized route position in junctions (scenariogeneration #[179](https://github.com/pyoscx/scenariogeneration/issues/179))
  - solves intermittent intersection border issue leading to wrong road ID and route failure
- Avoid additional step after scenario termination
- Fix plot window crash on added objects
- Add trailer rotating front axle example
  - see [video clip](https://youtu.be/5yud-oiO5AI) and find links in description
- Document optional parameters in [ReportObjectPos](https://github.com/esmini/esmini/blob/2af9304767d8b196bb00c6ecc7cb0cb725123b31/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L759)
  - z, p, r can be set to NAN for road alignment (temporary solution)

### 2023-09-07 Version 2.32.0

New features:
- Runtime plotting feature based on imgui/implot
  - see brief info in [User guide - Runtime plotting](https://esmini.github.io/#_runtime_plotting)
- osi3::TrafficUpdate input to the esmini OSMP FMU ([PR #463](https://github.com/esmini/esmini/pull/463))
  - esmini can now be used in a closed-loop co-simulation with a traffic participant model
- UDP action server ([issue #465](https://github.com/esmini/esmini/issues/465))
  - Inject actions via UDP messages
  - Only a few actions supported so far
  - See [issue](https://github.com/esmini/esmini/issues/465#issuecomment-1693377535) and example [inject_actions.py](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/code-examples/hello_world/inject_actions.py) for more info

Updated behaviors:
- Relative lane-change/offset direction now based on the referenced entity orientation
- Add road ID to trajectory vertices
  - purpose is to preserve road ID for trajectory lookup functions, e.g. follow ghost
  - especially useful in intersections where road id is ambiguous due to overlapping roads
- Reduce lane change jitter
  - apply lateral movement with delta-time of current frame, not previous

Improvements and fixes:

- Code quality improved by const correctness on class methods ([PR #466](https://github.com/esmini/esmini/pull/466))
- Set SUMO vehicles role = CIVIL and category = CAR (previously undefined)
- Add missing road info for esminiLib road lookahead functions
  - junctionId, roadId, laneId, laneOffset, s and t
- Update info how to install clang v15 on Linux in [User guide - Formatting](https://esmini.github.io/#_formatting)
- Fix execution flow bug in the experimental [fix_dae_materials.py](https://github.com/esmini/esmini/blob/dev/scripts/fix_dae_materials.py) script

### 2023-08-04 Version 2.31.10

- Add [SE_AddObjectWithBoundingBox()](https://github.com/esmini/esmini/blob/e76b4cf90f0bf7827617eff587bd41e7388c4dc8/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L741) method to lib API ([PR #460](https://github.com/esmini/esmini/pull/460))
- Fix wrong expected swarm central object attribute name ([issue #459](https://github.com/esmini/esmini/issues/459))
  - correct attribute name is CentralObject
  - previous expected name, CentralSwarmObject, is accepted as well
- Add combined type and vehicle class to osi2csv
- Add missing OSI VAN class
- Add missing fence 3D model to demo package
- For TTC, calc rel speed along triggering entity velocity vector ([issue #445](https://github.com/esmini/esmini/issues/445))


### 2023-07-04 Version 2.31.9

New behaviours:
- Calculate omitted WorldPosition headings also when trajectory following mode is "position"
  - previously omitted heading was set to "0.0" (default according to standard)

Improvements and fixes:
- Expose functions to control snappable lanes ([issue #448](https://github.com/esmini/esmini/issues/448))
  - both in [esminiLib](https://github.com/esmini/esmini/blob/d2f00135a2c43bd6a4430d07caa3d841d9100721/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L841) and [esminiRMLib](https://github.com/esmini/esmini/blob/d2f00135a2c43bd6a4430d07caa3d841d9100721/EnvironmentSimulator/Libraries/esminiRMLib/esminiRMLib.hpp#L229)
- Fix camera lost track of bounding boxes bug

### 2023-06-26 Version 2.31.8

New behaviours:
- Set camera focus on model center instead of referece point ([issue #442](https://github.com/esmini/esmini/issues/442))
- Relative lane id/offset based on entity x-axis instead of road ref system ([issue #444](https://github.com/esmini/esmini/issues/444))
- Align any missing z (elev), r (roll), and p (pitch) to road surface, skipping default value (0)
  - Note: This is a deviation from the OpenSCENARIO standard motivated by common sense
  - Values can still be set to 0 explicitly

Improvements:
- Update prerequisite info on [formatting](https://esmini.github.io/#_formatting)

### 2023-06-09 Version 2.31.7

- Fix left hand traffic going reverse ([issue #441](https://github.com/esmini/esmini/issues/441))
- Fix wrong type in GeoReference.axis_ ([issue #442](https://github.com/esmini/esmini/issues/442))
- Add info on how to install OSI for use with Python on Windows
  - see [User guide - OSI for Python on Windows](https://esmini.github.io/#_osi_for_python_on_windows)
- Add Python example script manipulating the speed of a vehicle
  - see [code-examples/hello_world/change_speed.py](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/code-examples/hello_world/change_speed.py)


### 2023-05-29 Version 2.31.6

New content:
- Add 2+1 road
  - xodr ([resources/xodr/two_plus_one.xodr](https://github.com/esmini/esmini/blob/master/resources/xodr/two_plus_one.xodr))
  - parameterized script: ([scripts/scenario_scripts/create_two_plus_one_road.py](https://github.com/esmini/esmini/blob/master/scripts/scenario_scripts/create_two_plus_one_road.py))
  - scenario ([resources/xosc/two_plus_one_road.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/two_plus_one_road.xosc))
  - run-script, with suitable camera pos ([run/esmini/run_two-plus-one-road.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_two-plus-one-road.bat))
  - video clip: https://youtu.be/aT7I2iBDeR0

Improvements and fixes:

- Clarify esmini lane offset handling deviating from OpenDRIVE
  - see [User guide - Reference line and center lane while using laneOffset](https://esmini.github.io/#_reference_line_and_center_lane_while_using_laneoffset)
- Make sure lane offset definitions starts from s=0
  - if first entry does not start at s=0, add a copy for s=0
- Fix bug in heading calculation wrt lane width and offset
- Fix wrong SE_GetDistanceToObject() return code for case route not found
- Demonstrate position type conversion ([issue #437](https://github.com/esmini/esmini/issues/437))


### 2023-05-25 Version 2.31.5

Improvements and fixes:

- Add some additional object info to csv_logger ([issue #434](https://github.com/esmini/esmini/issues/434))
  - bounding box position (relative obj ref point)
  - bounding box dimensions
  - lane id
  - lane offset
- Add API function [`SE_GetDistanceToObject()`](https://github.com/esmini/esmini/blob/4e45f132f13963a89e4b473c1bb16d9940d5c7cf/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L989) to get delta between objects
  - distance (ds, dt, dx, dy)
  - delta lane id
  - opposite lane flag
  - Note: ds will accumulate over multiple road segments
- Add motorway type to relevant road segments in [Unittest/xodr/highway_exit.xodr](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Unittest/xodr/highway_exit.xodr)
  - affects OSI handling of the junction
- Reduce grid intensity in plot scripts


### 2023-05-15 Version 2.31.4

Project updates:
- Update to macos-11 on github actions

Improvements and fixes:
- Add custom camera option to look at current entity
  - simply specify `--custom_camera <x,y,z>` (skip heading and pitch)
- Fix orientation alignment bug
  - respect individual rotational dimension (h,p,r) alignment setting correctly
- Add parameter to control accepted lateral distance in ACC Controller ([issue #427](https://github.com/esmini/esmini/issues/427))
- Ignore road sign subtypes "none" and "-1" ([issue #430](https://github.com/esmini/esmini/issues/430))
- Fix custom camera position behavior
  - camera position affected only by entity position and heading, not roll and pitch
- Add missing includes to esminiJS CMakeLists.txt

### 2023-05-03 Version 2.31.3

Improvements and fixes:
- Support `freespace` attribute in RelativeClearanceCondition
  - typo in standard, should have been `freeSpace`
  - esmini now supports both `freespace` and `freeSpace`
- Improve RelativeClearanceCondition
  - consider also empty lane of triggering entity
  - consider opposite lane flag for empty lanes as well
- Added OpenMSL to [related work](https://github.com/esmini/esmini#related-work)

### 2023-04-28 Version 2.31.2

Improvements and fixes:
- Fix RelativeClearanceCondition missed case of empty lanes within range
- Support OpenSCENARIO v1.2 `override` priority, renamed from `overwrite`
- Improve LongitudinalDistanceAction
  - make use of any dynamics constraints provided for the action
  - disable dynamics only when all constraints are missing
  - respect vehicle performance settings (limit constraints)
- exclude odr and model refs from csv file converted from .dat
  - fixes plot_csv.py failing to plot csv files created from dat2csv.py
- Add some ghost trigger info to User guide, see [The ghost concept / Actions and triggers](https://esmini.github.io/#_actions_and_triggers)

### 2023-04-19 Version 2.31.1

New features and behaviors:
- Add custom fixed top camera option to odrviewer as well
  - for usage info, see similar last example in [User guide - Camera control](https://esmini.github.io/#_camera_control)

Improvements and fixes:
- Add C# coding example showing how to get and print OSI ground truth
  - see [code-examples/osi-groundtruth-cs](https://github.com/esmini/esmini/tree/master/EnvironmentSimulator/code-examples/osi-groundtruth-cs)
  - or [User guide - Hello World - C# binding](https://esmini.github.io/#_c_binding)
- Fix bug causing camera to be stuck in orthographic mode after switching from custom top view
- Run nightly sanitizer checks on dev branch instead of master

### 2023-04-18 Version 2.31.0

New features and behaviors:
- Add esmini OSMP FMU ([PR #409](https://github.com/esmini/esmini/pull/409) and [issue #341](https://github.com/esmini/esmini/issues/341))
  - based on the [OSMPDummySource](https://github.com/OpenSimulationInterface/osi-sensor-model-packaging/tree/master/examples/OSMPDummySource) example
  - more info here: https://github.com/esmini/esmini/tree/master/OSMP_FMU
- Add support for [OSC 1.2 RelativeClearanceCondition](https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/RelativeClearanceCondition.html)
- Add `--ground_plane` option to replayer
- Add static variant of esminiLib
  - also add code example [hello_world_static](https://github.com/esmini/esmini/tree/master/EnvironmentSimulator/code-examples/hello_world_static)
- Expose subset of OSI API in C# wrapper ([ESMiniWrapper.cs](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Libraries/esminiLib/ESMiniWrapper.cs))

Improvements and fixes:
- Add info on blocked files on Windows (see [here](https://esmini.github.io/#_blocked_by_windows_defender_smartscreen))
- Add some COLLADA info related to osgconv (see [here](https://esmini.github.io/#_get_osgconv))
- Fix issue with stand still phases on trajectories
  - e.g. allow for rotate heading while stationary
- Fix curvature direction dependence in esminiRM Unity Util GetLaneInfo()
- Add info on failed package download to [User guide - Various issues](https://esmini.github.io/#_failed_to_download_3rd_party_assets)
- Fix/update esmini [C# wrapper API](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Libraries/esminiLib/ESMiniWrapper.cs) documentation (header comments/tooltip))
  - AddObject() returns created object ID (not always 0) on success
  - object_id replacing object index as argument in many functions (use GetId(index) to find out)
- Improve delta lane ID calculation in Roadmanager::Position::Delta() method
  - based on connected lane id at target position (compare at same road and s-value)
  - remove reference lane from delta (e.g. difference between lanes -2 and +3 is 4, not 5)
  - include flag for vehicle found in opposite lane or not
- Add another looming controller demo scenario [loomingHW.xosc](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Unittest/xosc/loomingHW.xosc) ([videoclip](https://youtu.be/X1D4b2xZiJc))


### 2023-03-22 Version 2.30.1

Project updates:
- Add brief info for developers and contributors:
  - Code formatting
  - How to run tests
  - Branch strategy
  - see [User guide - For esmini developers and contributors](https://esmini.github.io/#_for_esmini_developers_and_contributors)
- Add [CONTRIBUTING.md](https://github.com/esmini/esmini/blob/master/CONTRIBUTING.md) with brief guidelines for contributions
  - e.g. checklist for reporting issue

New features and behaviors:
- Support OpenSCENARIO 1.2 and OSI 3.5.0 vehicle role attribute
- Add simple police car 3D model ([model pack](https://dl.dropboxusercontent.com/s/5gk8bvgzqiaaoco/models.7z?dl=1) updated)
  - easy way to update: Delete resources/models folder and run `cmake ..` again from build folder
- Add odrviewer option (`--stop_at_end_of_road`) to stop instead of respawn at end of road ([issue #407](https://github.com/esmini/esmini/issues/407))
- Expose SimpleVehicle class (kinematic "bicycle" model) in C# API
  - example code: [simpleVehicleExample.cs](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Libraries/esminiLib/simpleVehicleExample.cs)

Improvements and fixes:
- Set exit code according to esmini success (0) or failure (-1) ([issue #413](https://github.com/esmini/esmini/issues/413))
- Update [odrviewer command reference](https://esmini.github.io/#_odrviewer), add missing Space and Return key commands
- Expose GetId() to C# API and add helloWorld example ([helloWorld.cs](https://github.com/esmini/esmini/blob/dev/EnvironmentSimulator/Libraries/esminiLib/helloWorld.cs))


### 2023-03-16 Version 2.30.0

Project updates:
- Bump C++ version from 14 to 17
- Migrate CI to GitHub actions
  - introducing further checks, e.g. formatting check, build and test more variants
- Introducing "dev" as main development branch while master will be reserved mainly for releases

New features and behaviors:
- Add pause and step feature to odrviewer ([issue #407](https://github.com/esmini/esmini/issues/407))
  - For complete command list, see Key shortcuts in [User guide - odrviewer](https://esmini.github.io/#_odrviewer)
- Add some viewer options to replayer and odrviewer
  - odrviewer: --aa_mode, --headless, --custom_fixed_camera
  - replayer: --aa_mode, --headless
  - For complete and updated launch arguments, see [User guide - Command reference](https://esmini.github.io/#_command_reference)
- Add "Looming" driver model controller
  - experimental implementation, see more info in [ControllerLooming.hpp#L14](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Modules/Controllers/ControllerLooming.hpp#L14)

Improvements and fixes:
- Support pause and quit during ghost head start
- Add driver input reverse example ([issue #400](https://github.com/esmini/esmini/issues/400))
- Add scenario, [one_car_on_hilly_road.xosc](https://github.com/esmini/esmini/blob/master/scripts/udp_driver/one_car_on_hilly_road.xosc), to demonstrate road alignment in UDP driver controller
  - See [testUDPDriver.py](https://github.com/esmini/esmini/blob/master/scripts/udp_driver/testUDPDriver.py) how to run

### 2023-02-20 Version 2.29.3

New features and behaviors:
- Add lookahead options to followGhost controller
  - set separate lookahead time/distance for speed and steering targets
  - see more info [User guide - FollowGhost](https://esmini.github.io/#_followghost)
- Add cmake option (-D OSI_VERSION_3_3_1) to enforce OSI v3.3.1 instead of v3.5.0 (default)

Improvements and fixes:
- Fix incorrect use of timeout parameter in unix ([PR #393](https://github.com/esmini/esmini/pull/393))
- Ghost improvements
  - Support reverse trajectory following ([issue #389](https://github.com/esmini/esmini/issues/389))
  - Bugfix: Skip ghost time offset for relative timing mode causing an initial jump
- Fix distance check messing up route ([issue #395](https://github.com/esmini/esmini/issues/395))
- Fix missing maneuverGroup reset for multiple executions


### 2023-02-13 Version 2.29.2

New features and behaviors:
- Fix wrong ParameterValueSet handling ([issue #390](https://github.com/esmini/esmini/issues/390))
  - For each scenario run one individual parameter value set is now applied
  - Previous behavior was to apply all value sets at once

Improvements and fixes:
- ghost fixes:
  - fix wrong sim time in actions at ghost restart, causing corrupt SpeedProfile
  - add default head-start time in ghost controllers
  - Fix issue with ordinary teleport actions by ghost (no restart)
  - restore ghost bounding box visibility

### 2023-02-09 Version 2.29.1

New features and behaviors:
- Identify road edges and populate OSI accordingly
  - Road edge is currently defined as boundary of outermost lane considered part of the road pavement
  - Lane types currently considered part of the road pavement:
    - DRIVING
    - ENTRY
    - EXIT
    - OFF_RAMP
    - ON_RAMP
    - ANY_DRIVING
    - RESTRICTED
    - STOP
- Count total displacement in SpeedAction dynamics ([issue #384](https://github.com/esmini/esmini/issues/384))
  - for transition dimension = distance (fulfil speed change over specified distance), consider total sum of displacement including motion in opposite directions when speed is changing sign, e.g. from 10 to -5 m/s.

Improvements and fixes:
- Update OSI to v3.5.0 for scripts as well (forgotten about in v2.29.0)
- Add player reference in controllers
  - enables more control of visualization from controllers
  - implemented example: Visualize sensor target point in acc controller (key 'o' / `--road_features on`)
- Fix coloring of stand-in bounding boxes
- Fix issues in ad-hoc traffic example
- Some additional bug fixes

### 2023-02-02 Version 2.29.0

New features and behaviors:

- Support for [DirectionDimension](https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/DirectionalDimension.html) in relevant conditions (introduced in OSC v1.2)
- Make AccelerationCondition aware of deceleration
  - Previously only considering absolute value of acceleration. Now signed.
- Add cmake config presets also for Visual Studio (Windows)

Improvements and fixes:
- Update to OSI v3.5.0 (from v3.3.1)
- Major refactorization cleaning up memory leaks and compiler warnings
- Increase compiler warning sensitivity, add multiple useful warnings
- For CI builds, treat warnings as errors - zero tolerance
- Improve [OSI build script](https://github.com/esmini/esmini/blob/master/scripts/generate_osi_libs.sh)
- Restore OSI demo in esmini-dyn

### 2023-01-24 Version 2.28.0

New features and behaviors:

- Update OverrideControllerValueAction for OpenSCENARIO v1.2
  - support new attributes like rate, force and automatic gear
  - API (data structs) updated, *not backward compatible*, see [esminiLib.hpp](https://github.com/esmini/esmini/blob/732fd19070c865901cd0c662cfeee7643657ea7c/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L125)

- Respect elevation alignment setting
  - enable use cases with external objects not bound to road surface
  - harmonize function prefix, change RM_SetAlignMode\* to SE_SetAlignMode\*
  - see example in [User guide - External control of Ego](https://esmini.github.io/index.html#_external_control_of_ego)

### 2023-01-23 Version 2.27.5

New features and behaviors:
- support OpenSCENARIO variables (introduced in OSC v1.2)
  - Scenario example: [Unittest/xosc/lane_change_trig_by_variable.xosc](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Unittest/xosc/lane_change_trig_by_variable.xosc)
  - limitation: Only SetAction supported, not ModifyAction yet
  - external API for setting and reading variables see [esminiLib.hpp](https://github.com/esmini/esmini/blob/37efb091396a522d1eb1b7daf919bca4e1a03bdc/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L540)
  - deprecated ParameterSetAction still supported
  - API for setting and reading parameters still available as well
- Adjust speed along s to maintain absolute speed
  - In other words: Do not increase total speed during lateral actions
  - Note: This can affect how far an object travels along s for a given duration.
- Update reference driver in ALKS_R157SM controller, e.g:
  - fulfill reference driver action even if critical situation dissolves
  - add overlap tolerance param to avoid edge cases, e.g. equal sized objects
- Embryo solution for esmini.js (esmini lib for web usage), see [Libraries/esminiJS](https://github.com/esmini/esmini/tree/master/EnvironmentSimulator/Libraries/esminiJS) ([PR #356](https://github.com/esmini/esmini/pull/356))
- Add Python OSI example ([issue #367](https://github.com/esmini/esmini/issues/367))
  - see [User guide - OSI groundtruth](https://esmini.github.io/#_osi_groundtruth)
- Add collision detection example to Hello World ([issue #368](https://github.com/esmini/esmini/issues/368))
  - see more info in [User guide - Collision detection](https://esmini.github.io/#_collision_detection)

Improvements and fixes:
- Fix SE_InitWithArgs whitespace bug ([issue #370](https://github.com/esmini/esmini/issues/370))
- Add missing plot.py to demo pack
- Fix scenario initialization return code ([issue #363](https://github.com/esmini/esmini/issues/363))
  - return 0 only on success, else -1
- Fix relative position beyond current road bug ([issue #372](https://github.com/esmini/esmini/issues/372))
- Fix typos in User guide ([PR #375](https://github.com/esmini/esmini/pull/375))
- Some additional minor bug fixes

### 2022-12-17 Version 2.27.4

New features and behaviors:
- Step fwd/back by delta time in replayer instead of fixed nr steps
  - shift => 0.1s
  - ctrl-shift => 1.0s
  - full description see [replayer Command reference](https://esmini.github.io/#_replayer)

Improvements and fixes:
- Fix chopped exe filenames in demo pack for Windows
- Fix orientation alignment bug ([issue #364](https://github.com/esmini/esmini/issues/364))
- Fix visibility mask bug in replayer

### 2022-12-16 Version 2.27.3

New features and behaviors:
- Extend plot_dat and add plot_csv script
  - plot_dat.py support all plottable values
  - added plot_csv.py for plotting any csv file
  - this means that osi tracefiles can be plotted (via osi2csv.py and plot_csv.py)
- Support continuous polyline speed and constant acceleration on segments
  - apply continuous speed / constant acc in FollowMode = follow
  - keep constant speed (zero acc) per segment in FollowMode = position
- Add grid on plots
- Update ALKS Reference Driver controller implementation
  - add support for cut-out and deceleration
  - add experimental support for pedestrians
  - add a few scenario examples (alks_*.xosc)
  - adjusted lateral trigger rule for cutting-in motorbikes
  - add overlap check function and apply for ALKS AEB trigger
  - add a few experimental properties, see `ALKS_R157SM_Controller` in [ControllerCatalog.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/Catalogs/Controllers/ControllerCatalog.xosc)

Improvements and fixes:

- Update equation link in odr elevation example in [straight_500_superelevation_elevation.xodr](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Unittest/xodr/straight_500_superelevation_elevation.xodr)
- Hello World updated to make use of existing library and header file (avoid keeping copies)
- Add info on how to use Visual Studio (and VSCode) with Hello World tutorial
  - see [User guide - How to build, run and debug "Hello World"](https://esmini.github.io/#_how_to_build_run_and_debug_hello_world)
- Fix shaky heading at end of small segment trajectories
- Fix failed quit on close window event ([issue #363](https://github.com/esmini/esmini/issues/363))
- Fix wrong OSI `bbcenter_to_rear` values ([issue #353](https://github.com/esmini/esmini/issues/353))
  - change sign (measure from bb center to read axle)
  - rear axle z relates to actual position, not projected on ground
- Add correct moving obj z coordinate ([issue #355](https://github.com/esmini/esmini/issues/355))
- Fix some additional memory leaks
- Restore lost OSI files in demo pack

### 2022-11-25 Version 2.27.2

- Add script for parallel execution of parameter distributions
  - see more info in [User guide - Parallel execution](https://esmini.github.io/#_parallel_execution)
- Add esmini option to just return nr permutations (also as exit code)
  - see [User guide - Finding out number of permutations](https://esmini.github.io/#_finding_out_number_of_permutations)
- Add acc and orientation vel/acc to [scripts/osi2csv.py](https://github.com/esmini/esmini/blob/master/scripts/osi2csv.py) script
- Fix maxDeceleration parse error ([PR #354](https://github.com/esmini/esmini/pull/354))
- Fix scripts path issues
  - move osi3 folder up one folder for easier reuse
  - add search path to make scrips executable from anywhere
- Move binary demo generator scripts to scripts folder

### 2022-11-18 Version 2.27.1

- Add missing parameter distribution file [cut-in_parameter_set.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/cut-in_parameter_set.xosc) to demo pack
- Fix sensor line elevation bug taking vehicle pitch angle into consideration
- Patch to avoid known VS2019 std::vector issue ([issue #335](https://github.com/esmini/esmini/issues/335))

### 2022-11-18 Version 2.27.0
New feature:
- Add parameter distribution support
  - see info in [User guide - Parameter distributions](https://esmini.github.io/#_parameter_distributions)
  - Note: currently limited to deterministic distributions

Improvements:
- Fix some memory leaks
  - code refactorization for improved memory handling and enabling sanitizer checks
- Unified [test runner script](https://github.com/esmini/esmini/blob/master/scripts/run_tests.sh) (works on all systems)
- Some osi optimization and less print outs
- Extend [testUDPDriver-print-osi-info.py](https://github.com/esmini/esmini/blob/master/scripts/udp_driver/testUDPDriver-print-osi-info.py) example to extract also stationary objects ([issue #350](https://github.com/esmini/esmini/issues/350))
- Add Python example [add_sensor.py](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/code-examples/hello_world/add_sensor.py) showing how to add cull sensors from Python ([issue #351](https://github.com/esmini/esmini/issues/351))

Bug fixes:
- Fix bug messing up auto-calculation of polyline trajectory heading
- Some additional minor fixes

Additional info:
- Move some scripts from root to [scripts](https://github.com/esmini/esmini/blob/master/scripts) folder
- Clarify limited Win32 (16 bit) support in [User guide - Build configurations](https://esmini.github.io/#_build_configurations)

### 2022-11-09 Version 2.26.9
- Make pedestrians snap to sidewalks ([issue #343](https://github.com/esmini/esmini/issues/343))
- Enable visualization of road and lane sensors for pedestrians
- Add GetIdByName function to esminiLib ([issue #345](https://github.com/esmini/esmini/issues/345))
- Remove RoadPosition driving direction dependency ([issue #340](https://github.com/esmini/esmini/issues/340))
- Remove obsolete SE_RegisterEventCallback ([issue #311](https://github.com/esmini/esmini/issues/311))
  - use [SE_RegisterStoryBoardElementStateChangeCallback](https://github.com/esmini/esmini/blob/daaf01d845bc3c6854ae5a92c6b09c15efa30c60/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L881) instead
- Make use of roadmanager detailed return codes in esminiRMLib functions
  - see [roadmanager.hpp::Position::enum class ReturnCode](https://github.com/esmini/esmini/blob/daaf01d845bc3c6854ae5a92c6b09c15efa30c60/EnvironmentSimulator/Modules/RoadManager/RoadManager.hpp#L1857)
- Bugfix: Preserve overlapping road check result until next query
- Bugfix: Reset lane offset after enforced lane switch (e.g. due to lane end or width become zero)
- Bugfix: Fix SE_GetRoadInfoAtDistance along route (avoid accumulating distance)
- Some additional minor fixes

### 2022-10-31 Version 2.26.8

- New feature: Custom lights ([issue #339](https://github.com/esmini/esmini/issues/339))
  - launch option for custom light sources: `--custom_light <x y z intensity>`
  - see more info in [User guide - Lighting](https://esmini.github.io/#_lighting)
  - and [User guide - esmini Launch commands](https://esmini.github.io/#_launch_commands)
- Align `%` and `round` operators to IEEE 754 ([issue #319](https://github.com/esmini/esmini/issues/319))
  - see updated info in [User guide - Supported expression operators and functions](https://esmini.github.io/#_supported_expression_operators_and_functions)
- Prevent snap to zero-width lanes and opposite directed lanes ([issue #337](https://github.com/esmini/esmini/issues/337))
- Fix double speed at FollowRouteCtrl first lane change step ([issue #338](https://github.com/esmini/esmini/issues/338))
- Fix issue with zero width lane in FollowRouteController ([issue #335](https://github.com/esmini/esmini/issues/335))
- Fix SE_GetRoadInfoAtDistance() return codes
  - update to align with internal API codes
  - see [esminiLib.hpp](https://github.com/esmini/esmini/blob/3f5a67c2423fbb660053abbd2472f18d4482f9b7/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L794)
- Fix some memory leaks ([pr #336](https://github.com/esmini/esmini/pull/336))
- Fix bug in LaneChange action potentially resulting in wrong lane offset
- Add note on object not appearing to [User guide - Various issues](https://esmini.github.io/#_entity_does_not_appear)

### 2022-10-17 Version 2.26.7

- Update UDPDriverController
  - add new input mode involving only `heading, speed and wheelAngle`
  - add missing data fields to all example scripts
  - document controller (see [User guide - UDPDriverController](https://esmini.github.io/#_udpdrivercontroller))
- Add external trigger example in Python ([issue #331](https://github.com/esmini/esmini/issues/331))
- Add Python storyboard event callback example ([issue #332](https://github.com/esmini/esmini/issues/332))
  - also restore lost advanced [User guide - Object state callback](https://esmini.github.io/#_object_state_callback) example
- Fix (OSI and tesselation) issue with partial lane segments with intentionally missing/no roadmarks
- Fix OSG lib build script ([generate_osg_libs.sh](https://github.com/esmini/esmini/blob/master/scripts/generate_osg_libs.sh)) issues on Windows

### 2022-10-07 Version 2.26.6

- Support Apple Silicon (e.g. M1)
  - from now on releases includes universal binaries (works on both Intel and Apple Silicon based Macs).
  - see updated info on how to deal with unsigned applications here: [esmini User guide - Mac issues and limitations](https://esmini.github.io/#_mac_issues_and_limitations)
- Fix build error on Ubuntu 22.04
  - enforce OSG lib to use sched_yield instead of deprecated pthread_yield
  - build scripts updated
  - updated osg lib for Ubuntu available here: [osg_linux_glibc_2_31_gcc_7_5_0.7z](https://esmini.asuscomm.com/AICLOUD766065121/libs/osg_linux_glibc_2_31_gcc_7_5_0.7z) (now default)
- Support straight arcs (curvature = 0.0 treated as a line)
- Add option `--extended` to [dat2csv.py](https://github.com/esmini/esmini/blob/master/scripts/dat2csv.py) to extract additional data, e.g. road coordinates
  - `-e` works as well
- Handle empty or missing road IDs ([issue #326](https://github.com/esmini/esmini/issues/326))
  - Generate and assign unique IDs (previously all roads got id=0, which caused trouble)
- Improve road elevation support ([issue #327](https://github.com/esmini/esmini/issues/327))
  - handle discontinuities in elevation profile and lane widths
  - fix bug causing low tesselation resolution wrt elevation slope changes
- Fix dat2csv.py (actually [dat.py](https://github.com/esmini/esmini/blob/master/scripts/dat.py)) issue so that it runs also on Python 2.X
- Fix bugs that could cause a crash when adding/removing entities while co-simulating with SUMO ([issue #308](https://github.com/esmini/esmini/issues/308))
- Add missing box 3D model to demo pack


### 2022-09-22 Version 2.26.5

- API related changes:
  - Wheel angle (steering) and rotation (revolution) populated in the [SE_ScenarioObjectState](https://github.com/esmini/esmini/blob/eaf8e0debdc314c12f3cba4b3624e842e6cc6576/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L23) ([pr #323](https://github.com/esmini/esmini/pull/323))

    **NOTE:** Structure on calling side should be updated accordingly. See example [get_states.py](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/code-examples/hello_world/get_states.py).
  - If velocity (vector) has been [reported](https://github.com/esmini/esmini/blob/eaf8e0debdc314c12f3cba4b3624e842e6cc6576/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L646) but not speed, then speed is calculated based on velocity x,y components (instead of dx, dy movement).

- Bugfixes:
  - Fix end-of-road detection failure for external entities (x, y -> road coord mapping)

### 2022-09-15 Version 2.26.4

- Support some additional expression functions, e.g. sin, sign and atan.
  - NOTE: These functions are extensions to OpenSCENARIO, use with care
  - Complete list see [User guide - Expressions](https://esmini.github.io/#_expressions)
- Fix lane offset, tolerate discontinuities
  - ensure correct lane tesselation for offset steps, e.g. when lane shift left or right
  - cut lane markings at end of lane sections
- Calculate speed and wheel orientations if not reported for external vehicles ([issue #163](https://github.com/esmini/esmini/issues/163))
  - default is to calculate speed and wheel roll and yaw angles based on vehicle motion
  - can be overridden by simply reporting the values, see example in [test-driver.cpp](https://github.com/esmini/esmini/blob/c0d815cda69f74ce9e0e0153476e926bfd84feaf/EnvironmentSimulator/code-examples/test-driver/test-driver.cpp#L105)
- Improve world coordinate (x, y) mapping to road coordinates
  - prioritize connected lanes wrt previous known road coordinate
  - for example when entering an intersection with overlapping lanes

### 2022-09-09 Version 2.26.3

- Add support for [botts' dots](https://en.wikipedia.org/wiki/Botts%27_dots)
  - OpenDRIVE roadMarkType: `botts dots`
  - OSI LaneBoundary/Classification/Type: `TYPE_BOTTS_DOTS`
  - example in [Unittest/xodr/mw_100m.xodr](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Unittest/xodr/mw_100m.xodr)
- Add info on ghost concept to [User guide](https://esmini.github.io/#_the_ghost_concept)

### 2022-09-05 Version 2.26.2

- New features:
  - Add FollowRoute controller
    - improved path finding by incorporating lane changes
    - more info in [User guide - FollowRoute](https://esmini.github.io/#_followroute)
  - Add roadworks/construction sign
    - OSI road sign code: TYPE_ROAD_WORKS = 13
    - Swedish and German signs added to 3D model pack
    - see [User guide - Update 3D model pack](https://esmini.github.io/#_update_3d_model_pack) how to update
  - Extend options for logfile and .dat (recording) location and filename
    - specify either of:
      1. Explicit directory and filename
      1. Just directory (ending '/' or '\\') (use default filename)
      1. Just filename  (use default directory)
    - see details in [esminiLib.hpp](https://github.com/esmini/esmini/blob/71311c7d9b7e25247e040afe4730a07830d3d4d7/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L187)

- Fixes:
  - Fix crash when query objects added by SE_AddObject() ([issue #263](https://github.com/esmini/esmini/issues/263))
  - Fix struct mismatch in Python RoadManager example ([issue #318](https://github.com/esmini/esmini/issues/318))

- Other:
  - Add OpenStreetMap (OSM) to OpenDRIVE tip to User guide  ([issue #317](https://github.com/esmini/esmini/issues/317))
    - see [User guide - OpenStreetMap (OSM) roads in esmini](https://esmini.github.io/#_openstreetmap_osm_roads_in_esmini)

### 2022-08-25 Version 2.26.1

- Road object updates:
  - Support continuous road objects (e.g. fence, railings)
    - examples in [crest-curve.xodr](https://github.com/esmini/esmini/blob/master/resources/xodr/crest-curve.xodr) and [e6mini.xodr](https://github.com/esmini/esmini/blob/master/resources/xodr/e6mini.xodr)
  - Create stand-in bounding boxes for missing 3D object files, instead of skipping
  - Improve alignment of road object wrt road curvature
  - *Note:* For correct alignments the [model pack](https://dl.dropboxusercontent.com/s/5gk8bvgzqiaaoco/models.7z?dl=1) needs to be updated
- Re-add missing files in Mac and Linux demo packages

### 2022-08-24 Version 2.26.0

- New features:
  - Add obj id to legend in [plot_dat.py](https://github.com/esmini/esmini/blob/master/scripts/plot_dat.py) (useful for merged data)
  - Add option to plot dots on top of lines in [plot_dat.py](https://github.com/esmini/esmini/blob/master/scripts/plot_dat.py)
  - Add generic StoryBoardElementStateChange callback functionality ([pr #314](https://github.com/esmini/esmini/pull/314))
  - Add `replayer` merge dat-files option (see [info in User guide](https://esmini.github.io/#_save_merged_dat_files))
  - Add alks reg 157 safety models controller
    - Experimental implementation of four safety models inspired by [JRC-FSM](https://github.com/ec-jrc/JRC-FSM)
    - See more [info in User guide](https://esmini.github.io/#_alks_r157sm)
- Fixes:
  - Support also maneuver element type in StoryboardElementStateCondition
  - Add heading when populating road object to OSI
  - Fix free space distance calc bug ([issue #309](https://github.com/esmini/esmini/issues/309))
  - Check story stopTrigger before stepping Init actions ([issue #313](https://github.com/esmini/esmini/issues/313))
  - Consider only private actions in conflict handling ([issue #295](https://github.com/esmini/esmini/issues/295))
  - Fix Delta function sometimes returning wrong dt/dlane in junction ([pr #283](https://github.com/esmini/esmini/pull/283))
  - Update [OSC coverage](https://github.com/esmini/esmini/blob/pf_rb/osc_coverage.txt), clarify version
  - Add box and cones to [model_id file](https://github.com/esmini/esmini/blob/pf_rb/resources/model_ids.txt)
  - Consider speedProfileAction in ghost setup
  - Fix corrupt search path for [traffic sign files](https://github.com/esmini/esmini/tree/master/resources/traffic_signals)
- Minor updates hopefully not causing side effects:
  - Prevent controller step completely during ghost restart
  - Update Hello World tutorial [OSI example](https://github.com/esmini/esmini/blob/master/Hello-World_coding-example/tutorial.adoc#osi-groundtruth) info regarding platform differences
  - Rework lane change action to preserve position info

### 2022-07-08 Version 2.25.1

- Add info on update road sign framework to [User guide - Road signs](https://esmini.github.io/#_road_signs)
- Relax country code interpretation, support both upper and lower case (e.g. `se` and `SE` will both work)
- Extend search locations, increasing chances to find the traffic signal catalogs
- Add missing speed signs to release package (update cache on build server)

### 2022-07-08 Version 2.25.0

- Update road sign framework
  - clarifies interpretation of country code, type, subtype and value
  - 3D model filename derived from type definition (name now only used as fallback)
  - add embryo catalog for swedish road signs
  - ensure correct lookup when mixing country codes
  - update examples in OpenDRIVE file [straight_500m_signs.xodr](https://github.com/esmini/esmini/blob/master/resources/xodr/straight_500m_signs.xodr)
  - update model pack with new German speed signs and updated and renamed Swedish ones

  **Note**:

  Due to new sign name convention model pack has been updated. Get it from [here](https://dl.dropboxusercontent.com/s/5gk8bvgzqiaaoco/models.7z?dl=1).

  Also, to enable frequent updates of the model pack it has been removed from github. Which means that also cmake build script has been updated. If trouble arise, the easiest solution is to grab a fresh clone of esmini: `git clone https://github.com/esmini/esmini.git`.

### 2022-07-06 Version 2.24.0

- Support lane discontinuities and improve road 3D model tesselation
  - refactorize OSI lane points to support discontinuities, e.g. step laneWidth changes
  - optimize tesselation reducing unnecessary polygons
- Add high precision (64 bit double) GetSimulationTime function
  - See header [here](https://github.com/esmini/esmini/blob/4d82d27b7a613a764d7cf3a5f20c389d5d60f409/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L327)
- Add vehicle 3D model bounding box dimensions info to [vehicle catalog](https://github.com/esmini/esmini/blob/master/resources/xosc/Catalogs/Vehicles/VehicleCatalog.xosc)
- Support inline trailer definitions
  - see updated example ["semi_truck_with_extra_trailer"](https://github.com/esmini/esmini/blob/79c529e6745191c332c8ed879f09273692c49ed5/resources/xosc/Catalogs/Vehicles/VehicleCatalog.xosc#L371) in [vehicle catalog](https://github.com/esmini/esmini/blob/master/resources/xosc/Catalogs/Vehicles/VehicleCatalog.xosc)
  - also see updated draft [trailer XML description](https://docs.google.com/document/u/1/d/e/2PACX-1vR7jXggp_LaEIzNyKsFEpJxWrYdC_W2GEDayQIeeCBU4fzyrEyJ22Ihbq4Ra7JXmbWqgPOydfB6WZ0j/pub)
- Add followingMode to FollowTrajectoryAction
  - followingMode `follow`: Calculate and interpolate heading in case heading is not explicitly set (overriding OpenSCENARIO 1.1+ default values)
  - followingMode `position`: Strictly follow the standard, e.g. missing heading is interpreted as 0.0
- Improve heading interpolation (decrease radius/ duration)
- Populate stationary objects, including outlines, to OSI stream
- Add misc object to example scenario and fix miscObjectCategory typo
  - esmini now expect [`miscObjectCategory`](https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.2.0_Model_Documentation/modelDocumentation/content/MiscObject.html) instead of the former wrong `MiscObjectCategory`
  - obstacle box added to the example scenario [drop-bike.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/drop-bike.xosc)
- Add info to User guide on how to make `osgconv` deal with repeated objects
  - see about FLATTEN_STATIC_TRANSFORMS_DUPLICATING_SHARED_SUBGRAPHS [here](https://esmini.github.io/#_openscenegraph_and_3d_models)
- Add info on sharp braking speed profiles
  - see [User guide - Sharp brake-to-stop profile](https://esmini.github.io/#_sharp_brake_to_stop_profile)
- Add info on how to suppress Windows console ([issue #294](https://github.com/esmini/esmini/issues/294))
  - see [here](https://github.com/esmini/esmini/blob/4d82d27b7a613a764d7cf3a5f20c389d5d60f409/Hello-World_coding-example/CMakeLists.txt#L18)
- Add derivative feature to plot_dat.py
  - plot first derivative (instead of original value) of parameters wrt x axis (typically time)
  - activate with `--derive` argument
- Accept roadMark type `none`
  - This change enables empty road mark segments within same lane section
- Accept unsorted (sOffset) `roadMark` entries
- Fix min time-step glitch causing esmini to be really slow
- Fix error messages to show junction id ([pr #297](https://github.com/esmini/esmini/pull/297))
- Fix invalid warning of unknown argument when using [SE_InitWithArgs()](https://github.com/esmini/esmini/blob/4d82d27b7a613a764d7cf3a5f20c389d5d60f409/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L287)
- Fix elevation jump bug at trajectory end ([issue #293](https://github.com/esmini/esmini/issues/293))
- Fix performance setting log message bug
- Fix crash due to referencing deactivated ghost (nullptr)
  - this fix makes it possible to toggle on/off controllers involving ghost

### 2022-06-20 Version 2.23.4

- Fix wrong successor selection at junction on route ([pr #292](https://github.com/esmini/esmini/issues/292))
- Fix intermittent speed spike at end of synchronize action
- Updated Linux OSG libs for increased compatibility
  - previous version, updated for esmini v2.23.3, was compiled with g++ v9.4
  - current version is compiled with g++ v7.5
  - either pick from [here](https://dl.dropboxusercontent.com/s/3dlev34kj94lir5/OpenSceneGraph_linux.7z?dl=1) or delete externals/OpenSceneGraph and re-run `cmake ..` to download

### 2022-06-15 Version 2.23.3

- Add dead-reckoning option to UDP controller
  - see (or run) updated example [testUDPDriver.py](https://github.com/esmini/esmini/blob/master/scripts/udp_driver/testUDPDriver.py)
- Add optional on-screen info per entity
  - activate with `--info_text 2` or `--info_text 3`
  - toogle info_text mode on key 'i'
  - **Note**: OSG libs for linux update needed. Either pick from [here](https://dl.dropboxusercontent.com/s/3dlev34kj94lir5/OpenSceneGraph_linux.7z?dl=1) or delete externals/OpenSceneGraph and re-run `cmake ..` to download.
- Populate wheel angle in osi2csv.py script
- Disable off-screen rendering by default
  - see more info in [User guide - save_or_grab_images](https://esmini.github.io/#_save_or_grab_images)
- Bugfix: Cancel conflicting actions only for same entity ([issue #289](https://github.com/esmini/esmini/issues/289), [pr #290](https://github.com/esmini/esmini/issues/290))
- Fix esminiRMLib `SetRoadId()` bug sometimes resulting in wrong road
- Fix intermittent crash at viewer initialization  Refactor viewer thread initialization using semaphores
- Fix relative road position issue #253
  - consider road plane in lane change action

### 2022-05-27 Version 2.23.2

- Updated SpeedProfileAction to respect initial acceleration
  - See more info in [User guide - Initial acceleration taken into account](https://esmini.github.io/#_initial_acceleration_taken_into_account)
- Improve multi-replayer support for different timesteps
- Fix direct junction bug missing some counter connections
- Fix route finding failure due to changing lane id
  - consider that lane ids might change over lane sections
- Improve route end checks
  - Detect route end in junctions
  - Fix bug in route length calculation

### 2022-05-13 Version 2.23.1

- Updated SpeedProfileAction handling of single entry cases
  - See more info in [User guide - Special case: Single entry](https://esmini.github.io/#_special_case_single_entry)
- Populate OSI wheel angle (yaw/heading)
- Increase internal precision for time and positions (float -> double)
- Improve stepping, avoid one step diff between external and internal entities
  - info for esmini lib users in [User guide - How to interact with esmini lib API](https://esmini.github.io/#_how_to_interact_with_esmini_lib_api)
- Add API to flush OSI file
- Refactor csv_logger to support multiple runs with different sets of entities
- Fix a few ghost controller bugs
- Fix pline interpolation bug
- Fix collision log bug and align collision timestamps

### 2022-04-29 Version 2.23.0

- New feature: Speed profile
  - specify multiple speed targets over time in one single action
  - optional dynamic constraints
  - more info in [User guide - Speed profile](https://esmini.github.io/#_speed_profile)
  - preliminary and experimental implementation
- Add RMlib method to enforce road ID ([RM_SetRoadId()](https://github.com/esmini/esmini/blob/902dec1ca379d817ca27d065b825d23c262a1bc3/EnvironmentSimulator/Libraries/esminiRMLib/esminiRMLib.hpp#L350))
- Remove `Parking` from set of drivable lane types
- Fix handling of entering non drivable lane
  - when switching lane section and valid driving lane or link is missing, snap to closest valid lane.
- Bugfix: updateObjectSpeed() only affect longitudinal domain
- Bugfix: Fix wrong reported curvature of perfectly straight clothoids

### 2022-04-13 Version 2.22.1

- Fix wrong RelativeLanePos interpretation ([issue #267](https://github.com/esmini/esmini/issues/267))
  - `offset` should be relative to center of new lane, not relative `offset` of the related position
  - in contrast to `dt` in RelativeRoadPosition which is relative `t` of the related position
  **Note:** This can affect existing scenarios. Use RelativeRoadPosition with ds="0.0" and dt="0.0" to get same longitudinal and lateral position as the related entity.
- Support ManeuverGroup multiple executions ([issue #269](https://github.com/esmini/esmini/issues/269))
- Log scenario parameter names and values
- Add event and condition callbacks to C# wrapper ([issue #257](https://github.com/esmini/esmini/issues/257))
- Fix wrong condition delay by relaxing floating point tolerance ([issue #270](https://github.com/esmini/esmini/issues/270))
- Place intermediate waypoints at 1/3 road length instead of at s=0
- Bugfix: Avoid crash on missing incoming road in junctions
- Tolerate init position at waypoint road but outside route s range
  - internal route s value will simply be < 0 or > length_of_route

### 2022-04-08 Version 2.22.0

- User guide published: https://esmini.github.io
  - Initial experimental embryo version
  - Focus on use cases
  - Hello World code tutorial moved to User guide
  - Build guides moved to User guide
- New feature: Custom cameras with fixed position and optionally fixed orientation ([issue #264](https://github.com/esmini/esmini/issues/264))
  ```
  --custom_camera <position>
      Additional custom fixed camera position <x,y,z,h,p> (multiple occurrences supported)
  --custom_fixed_camera <position and optional orientation>
      Additional custom camera position <x,y,z>[,h,p] (multiple occurrences supported)
  --custom_fixed_top_camera <position and rotation>
      Additional custom top camera <x,y,z,rot> (multiple occurrences supported)
  ```
  - see examples in User guide: https://esmini.github.io/#_camera_control
- Add odometer to replayer
- Update RMLib C# wrapper
- Propagate scaled bounding box to .dat file
- Align and change default headstart in ghost controllers to 3s
- Add example scenario [acc-toggle.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/acc-toggle.xosc) to demonstrate switching between controllers ([issue #259](https://github.com/esmini/esmini/issues/259))
- Add odrplot to demo package
- Fix bug in route path calculation ([issue #265](https://github.com/esmini/esmini/issues/265))
- Fix condition edge check bug that in some cases prevented multiple story element executions
- Fix bug in position delta calculation sometimes flipping dLaneId ([PR #266](https://github.com/esmini/esmini/issues/266))
- Bugfix: Add missing initial frame (t=0) to OSI trace file
- Fix malfunctioning replayer start and stop time options
- Fix unintentional toggling of route waypoints (toggle key 'R')
- Remove obsolete EgoSimulator (it was just a duplicate of esmini application)

### 2022-03-28 Version 2.21.2

- Add RMLib methods to get road lane width and type
  - See API [here](https://github.com/esmini/esmini/blob/fe9a582c29af7bb31aed299b517ea3fde451d125/EnvironmentSimulator/Libraries/esminiRMLib/esminiRMLib.hpp#L396).
- Add roadmanager API to fetch all roads overlapping current position (can be useful in junctions)
- Extend roadmanager return code to indicate whenever
  - moved into a new road segment (return code 1)
  - moved into a junction and made a choice (return code 2)
  - See full list of return codes [here](https://github.com/esmini/esmini/blob/fe9a582c29af7bb31aed299b517ea3fde451d125/EnvironmentSimulator/Modules/RoadManager/RoadManager.hpp#L1786)
  - **NOTE:** This change might need code updates. Previously only 0 meant OK and negative return values indicated some error. If `code == 0` is used to check OK, it should be changed to `code >= 0`.
- Add state handling to ManeuverGroups
  - supporting storyBoardState condition on ManeuverGroup level
- Improve xy -> trackpos mapping at road endpoints (using actual road normals instead of OSI points to ensure continuity between roads)
- Fix relativeRoadPos relative heading bug ([issue #253](https://github.com/esmini/esmini/issues/253))
- Log complete path for loaded files (OpenSCENARIO, OpenDRIVE and scenegraph)
- Ghost controller improvements:
  - move any AssignRouteAction to ghost. Enables routing for ghost controllers.
  - restart ghost from Ego after triggered events from other entities
  - parameterize followGhost mode: position (default) or time
  - assign unique route object to ghost, solving issue with FollowRoute actions defined in Init
- Fix wheel-angle calculation
  - also update some car models fixing right front wheel issue
  - grab updated model package from [here](https://dl.dropboxusercontent.com/s/5gk8bvgzqiaaoco/models.7z?dl=1)
- Make road object LOD distance dependent also on object size

### 2022-03-16 Version 2.21.1

- Add ground plane option
  - add launch flag `--ground_plane` to put a large gray surface under the roads
- Fix and improve road object handling
  - Fix repeat object lengthStart/End bug
  - Fix inter-distance affected by road curvature and lateral position
  - Populate length, width and height from bounding box if attributes not set
  - Add hedge and fence models
    Note: To fetch updated models, remove resources/models folder and run `cmake ..` again, or get the package from [here](https://dl.dropboxusercontent.com/s/5gk8bvgzqiaaoco/models.7z?dl=1)
  - Set color of based on type (for objects lacking explicit 3D model)
    - building, barrier: light gray
    - tree, vegetation: green
    - obstacle: red
    - all other: dark gray
  - [xodr/crest-curve.xodr](https://github.com/esmini/esmini/blob/master/resources/xodr/crest-curve.xodr) updated to show some of the features
- Reduce OSI file size (again)
  - Save size by save static data only first frame
  - This was implemented in v2.17.4 but unfortunately disabled by a bug introduced in v2.20.6

### 2022-03-14 Version 2.21.0

- New feature: Trailer support
  - Extend OpenSCENARIO vehicle class with trailer connection elements (experimental)
  - Extension noted in [osg_coverage.txt](https://github.com/esmini/esmini/blob/master/osc_coverage.txt) and briefly described [here](https://docs.google.com/document/u/1/d/e/2PACX-1vR7jXggp_LaEIzNyKsFEpJxWrYdC_W2GEDayQIeeCBU4fzyrEyJ22Ihbq4Ra7JXmbWqgPOydfB6WZ0j/pub)
  - Simplified trailer mathematics and dynamics, good enough for basic scenarios
  - Provided examples:
    - [trailer.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/trailers.xosc) / [video clip](https://youtu.be/15QlPBrF4Ro) / run script: [run/esmini/run_trailers.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_trailers.bat)
    - [parking_lot.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/parking_lot.xosc) / [video clip](https://youtu.be/iDWurUhesSc) / run script: [run/esmini/run_parking_lot.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_parking_lot.bat)
  - Demo showing capabilites and limitations: [video clip](https://youtu.be/9BTNEhU_V9c)
  - New vehicle 3D models: Semi-truck tractor, semi-trailer, truck trailer and car trailer
    **Note:** To fetch updated models, remove resources/models folder and run `cmake ..` again, or just get the package from [here](https://dl.dropboxusercontent.com/s/5gk8bvgzqiaaoco/models.7z?dl=1)
- Support for multiple Repeat objects ([PR #251](https://github.com/esmini/esmini/issues/251))
- Support simple OpenDRIVE bounding box objects (when no osgb filename or outline specified)
  - [xodr/crest-curve.xodr](https://github.com/esmini/esmini/blob/master/resources/xodr/crest-curve.xodr) updated with an example (see object with ID 0)
- SwarmAction updated to spawn only relevant, specified vehicle types
- Respect vehicle acceleration and deceleration constraints in InteractiveDriver controller
- Add lib method to set SimpleVehicle wheel status: [SE_ReportObjectWheelStatus()](https://github.com/esmini/esmini/blob/aed36300e603e25aee9a910af3f6ec7bdc32593a/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L657)
- Add lib API for pause flag: [SE_GetPauseFlag()](https://github.com/esmini/esmini/blob/aed36300e603e25aee9a910af3f6ec7bdc32593a/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L341)
- Parameterize steering rate for InteractiveDriver controller
- Add C# API to get parameter names ([PR #252](https://github.com/esmini/esmini/issues/252))
- Optimize muli-replayer feature, now supporting much larger and/or more scenarios
- Generalize 3D model wheel handling
  - Support any number of front and rear wheels
  - Identify front wheels as wheel_f*
  - Identity rear wheels as wheel_r*
  - All wheels will roll, only front wheels will change heading/steer

### 2022-03-07 Version 2.20.10

- Add methods to [register callbacks](https://github.com/esmini/esmini/blob/4da56f080b8ccdbb41372353d435c4438e4ce394/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L816) for triggered conditions and event start/end ([PR #249](https://github.com/esmini/esmini/issues/249))
- Add Python variant of ad_hoc_traffic example (see [here](https://github.com/esmini/esmini/tree/master/EnvironmentSimulator/code-examples/ad_hoc_traffic))
- Set controller type for externally added objects (via SE_AddObject())
- Fix AcquirePositionAction issues, e.g. when destination is on same road ([PR #247](https://github.com/esmini/esmini/issues/247))
- Fix ghost trail reset issues (at ghost restart)
- Optimize polyline trajectory parsing and ghost trail following
- Fix error in speed calculation for absolute time trajectory

### 2022-02-25 Version 2.20.9

- Expose Add/DeleteObject functions in esminiLib API
  - code example [ad_hoc_traffic](https://github.com/esmini/esmini/tree/master/EnvironmentSimulator/code-examples/ad_hoc_traffic)
- Expose collision detection in esminiLib API ([issue #243](https://github.com/esmini/esmini/issues/243))
- Add OSI related esminiLib functions to C# wrapper
- Update outdated data struct [Python example](https://github.com/esmini/esmini/tree/master/Hello-World_coding-example#example-get-object-states) ([issue #242](https://github.com/esmini/esmini/issues/242))
- Improve 3rd party lib [build scripts](https://github.com/esmini/esmini/tree/master/scripts) ([PR #244](https://github.com/esmini/esmini/issues/244))
- Additional minor bugfixes and improvements

### 2022-02-21 Version 2.20.8

- Support MSYS2 / MinGW-w64 ([issue #239](https://github.com/esmini/esmini/issues/239)). See brief info [here](https://github.com/esmini/esmini/blob/master/docs/BuildInstructions.md#msys2--mingw-w64-support).
- Eliminate need for garbage collection in C# wrapper
  - greatly improves esmini performance e.g. in Unitu
- Fix freeze at time=0 when init with XML ([issue #235](https://github.com/esmini/esmini/issues/235))
- Bugfix: Ensure deleted objects not involved in condition evaluations
- Bugfix: Disregard waypoints with invalid road IDs

### 2022-02-16 Version 2.20.7

- Change lib functions to handle id instead of index
  - **Note**: This can affect user applications making use of esminiLib object query functions. Especially for scenarios making use of Add/DeleteEntityActions.
  - New function SE_GetId(index) to find out id of objects. Example:
    ```c++
    for (int i = 0; i < SE_GetNumberOfObjects(); i++)
	{
    	SE_ScenarioObjectState state;
		SE_GetObjectState(SE_GetId(i), &state);
    }
    ```
  - Complete API see [esminiLib/esminiLib.hpp](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp)
  - More examples, see [Hello-World_coding-example](https://github.com/esmini/esmini/tree/master/Hello-World_coding-example)
- Fix condition timer bug ([issue #237](https://github.com/esmini/esmini/issues/237))
- Improve replayer multi-recording functionality to handle dat files with some variation in terms of timings and number of entities.
- Fix orientation reference for RelativeRoadPosition ([issue #228](https://github.com/esmini/esmini/issues/228))
  - Orientation relates to road s/t system, not driving direction
  - **Note**: RelativeLanePosition still relates to driving direction, but will be changed as well when lane coordinate system has been fully implemented
- CSV logger typo fix and added units to header ([PR #238](https://github.com/esmini/esmini/pull/238))
- Fix issue which could cause simulation to freeze after calling SE_Step()

### 2022-02-11 Version 2.20.6

- Fix wrong ParamPoly3 curvature calculation
- Increase expression precision ([issue #225](https://github.com/esmini/esmini/issues/225))
  - internal representation in double (64 bits) instead of float (32 bits)
- Add option to enforce creation of road 3D model ([issue #227](https://github.com/esmini/esmini/issues/227))
- Fix OSI GT reset between runs in same session ([issue #229](https://github.com/esmini/esmini/issues/229))
- Improve RoadManager OSI robustness ([issue #212](https://github.com/esmini/esmini/issues/212))
- Fix lookahead direction
  - correctly consider driving direction and vehicle heading
- Add lib API to get ghost state by time (as alternative to by distance)
- Fix condition issues
  - Fix timer on falling edge
  - Add precision tolerance to < and > conditions
- Added setting of OSI lane subtype
- Updated [OSG apps build script](https://github.com/esmini/esmini/blob/master/scripts/compile_osg_apps.sh) to support all platforms
- Improve ghost execution
  - e.g. reduce spikes in velocity and acceleration at ghost restarts

### 2022-02-02 Version 2.20.5

- Add route waypoint visualization
  - disable by launch argument `--hide_route_waypoints`
  - toggle on key 'R' (shift 'r')
- Add replayer support for optional external model_id mapping file
  - See default example [resources/model_ids.txt](https://github.com/esmini/esmini/blob/master/resources/model_ids.txt)
- Update ECE ALKS reference driver controller
  - Support some corner cases, e.g. narrow lanes, high lateral velocities and large timesteps.
- Add API to retrieve speed unit (see [esminiLib.hpp](https://github.com/esmini/esmini/blob/24bcac483148cb26136e53198306391ed88398eb/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L689))
- Fix corner case of failed conflict detection ([issue #218](https://github.com/esmini/esmini/issues/218))
- Remove pedestrian performance limits ([issue #224](https://github.com/esmini/esmini/issues/224))
- Bugfix: Remove trajectory visualization of deleted entity ([issue #208](https://github.com/esmini/esmini/issues/208))
- Bugfix: Fix malfuncting left-shift-tab on Linux. It now jump to previous entity.
- Bugfix: Fix quit by close window event issue (Linux)
- Added a [guide](https://github.com/esmini/esmini/blob/master/docs/BuildOSG.md#on-osgb-and-unity) on how to bring osgb 3D models into Unity

### 2022-01-28 Version 2.20.4

- Fix replayer rewind all way to start, including negative ghost headstart time
- Fix crash on missing roadmark color attribute ([issue #221](https://github.com/esmini/esmini/issues/221))
- Add value conditions to cause a ghost teleport
- Fix initial speed and acceleration spikes with ghost


### 2022-01-25 Version 2.20.3

- Fix wrong EntityAction interpretation
  - entityRef an attribute of the EntityAction not its child elements
- Change timer expire condition from > to >=
- Evaluate complete story before stepping it ([issue #216](https://github.com/esmini/esmini/issues/216))
- Change id of OSI objects to be concistent, use scenario entity id
- Prevent default controller (trying to) use invalid routes

### 2022-01-24 Version 2.20.2

- New replayer feature: Play multiple recordings in parallel
  - example: `replayer --window 60 60 800 400 --res_path ./resources --dir ./dat --file variant`
  - above command will load and play all `variant*.dat` files found in `./dat` folder, simultanously
  - Note: Intented for .dat files of same length and timesteps, e.g. using fixed timestep:
    `esmini --headless --fixed_timestep 0.01 --osc variant1.xosc --record variant1.dat`
- Updated behavior: Disable global collision detection by default
  - saving performance for huge scenarios
  - activate with launch flag `--collision`
- Updated roadmark support ([issue #215](https://github.com/esmini/esmini/issues/215)):
  - solid_solid and broken_broken roadmarks
  - yellow colored roadmarks
  - apply <RoadMark> width (optional) and weight (if width is missing)
- Fix LaneOffsetAction final value bug ([issue #213](https://github.com/esmini/esmini/issues/213))
- Optimize evaluation of AND:ed conditions (conditions within same condition group)
- Ensure all timesteps are saved with `--capture_screen` in replayer

### 2022-01-21 Version 2.20.1

- Add replayer option to hide ghost models but still show their trajectories
  - Launch flag: `--no_ghost_model`
  - Toggle on key 'g'
  - See [replayer/readme.txt](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Applications/replayer/readme.txt)
- Add OSI lane id to route info
- Relax replayer, run even if OpenDRIVE file missing
- RoadmarkType rule made optional according to standard (skip scary warnings)
- Hand over also AccelerationCondition to ghost
  - The pattern is to hand over all non relative conditions
  - Acceleration conditions were unintentially not handed over
- Fix SpeedAction continuous mode bug

### 2022-01-19 Version 2.20.0

- Add support for [OpenSCENARIO EntityActions](https://www.asam.net/static_downloads/ASAM_OpenSCENARIO_V1.1.1_Model_Documentation/modelDocumentation/content/EntityAction.html)
  - AddEntityAction and DeleteEntityAction supported (issue [#208](https://github.com/esmini/esmini/issues/208))
  - See example scenario [Unittest/xosc/add_delete_entity.xosc](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Unittest/xosc/add_delete_entity.xosc)
- Fix some replayer issues
  - Repeat works again
  - Don't time-leap over empty periods states (e.g. entities temporarily deleted)

NOTE: To support entity actions, e.g. adding entities at any time, a change of initialization behavior was necessary. Previously all entities were instantiated at start of the scenario, regardless of presence in Init section. Now the following applies: Only entities involved in the Init section, e.g. by a TeleportAction, will be instantiated from start. Other entities can be added later by the AddEntityAction. This change might affect some scenarios and use cases.

### 2022-01-17 Version 2.19.3

- Support OpenDRIVE 1.7 Direct Junctions
- Fix ghost condition handling issues ([issue #210](https://github.com/esmini/esmini/issues/210))
  - Avoid nullptr crash for non event conditions (e.g. act condition)
  - Hand over also SpeedCondition, EndOfRoadCondition, OffRoadCondition and
  StandStillCondition to ghost

### 2022-01-13 Version 2.19.2

- New esmini feature: Pause and step simulation
  - Press space to toggle pause/play
  - Press return to step (forward only)
  Note: In replayer similar feature is space for pause/play and arrow right/left to step forward and backwards.
- Fix maneuver event order dependency
  - When an event overwrites another it could happen that both step methods were applied. Now its first sorted out what events to run, then they are stepped.
- Fix replayer bug when ghost involved, causing really slow progress at negative time
- Update [Python info](https://github.com/esmini/esmini/tree/master/Hello-World_coding-example#python-binding) and scripts
  - A more flexible variant the basic scenario player script [esmini-player.py](https://github.com/esmini/esmini/blob/master/Hello-World_coding-example/esmini-player.py)
    - providing scenario via argument
    - quit at press Escape or end of scenario
    - work on all supported platforms
  - Add example Python launcher for esmini executable: [scripts/esmini-launcher.py](https://github.com/esmini/esmini/blob/master/scripts/esmini-launcher.py)

### 2022-01-09 Version 2.19.1

- Add global collision detection ([issue #206](https://github.com/esmini/esmini/issues/206))
  - Evaluate all entities for collision detection (not only
  those subject to CollisionCondition)
  - Result will be logged in console and log file
  - Collision status added to csv log (activate with --csv_logger
  \<filename\>)
  - Disable the feature with flag --disable_collision_detection

### 2022-01-05 Version 2.19.0

- New feature: Off-screen rendering
  - Fetch rendered images via esminiLib API (synchronous or via callback) ([issue #173](https://github.com/esmini/esmini/issues/173))
  - Example code: [image-capture.cpp](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/code-examples/image-capture/image-capture.cpp)
  - Off-screen rendering without GFX system (e.g. headless Ubuntu):
    - Windows using Mesa3D: Put [opengl32.dll](https://downloads.fdossena.com/geth.php?r=mesa64-latest) in same folder as esmini exec.
    - Linux using xvfb: `Xvfb :99 -screen 0 1920x1080x24+32 & export DISPLAY=:99` then run esmini as ususal.
  - Disable feature with [launch flag](https://github.com/esmini/esmini/blob/master/docs/commands.txt) `--disable_off_screen` (potentially gaining performance)
- Lossless screen-capture in TGA format instead of JPG
  - Disadvantage: larger files
  - Advantage: Videoclips created from uncompressed files better quality/size ratio
  - Anyway it's easy to convert TGA to JPG in a post process step
  - Reminder: Add launch flag `--capture_screen` for continuous capture
- Extended screen-capture control
  - Specify exact number of frames to capture, or continuous mode (-1)
  - Note that esminiLib API for screen-capture has changed, examples (old => new):
    - SE_CaptureContinuously(true) => SE_SaveImagesToFile(-1)
    - SE_CaptureContinuously(false) => SE_SaveImagesToFile(0)
    - SE_CaptureNextFrame() => SE_SaveImagesToFile(1)
- New replayer features:
  - Collision detection. Log events and pause player. Activate with launch flag `--collision`
  - Quit at end of scenario. Activate with launch flag `--quit_at_end`
- Add route [API](https://github.com/esmini/esmini/blob/0c18eaad53eb92bb1859e27ded3c6ddb5a1f11d4/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L1070) to esminiLib.
- Reworked action dynamics. Hopefully more correct math and behavior.
  - e.g. Lane change trajectory affected by (long) speed if and only if Dimension = time
  - LaneOffset maxLateralAcc respected ([issue #201](https://github.com/esmini/esmini/issues/201))
  - Note that behavior might differ from previous versions
  - Changed behavior is either intentional or not, so please report suspected bugs.
  - Respect vehicle performance specification (max speed, acc, dec)
- Improved LaneChangeAction: Handle lane split cases, e.g. highway exit.
- Improved route handling: Relax lane requirement, e.g. maintain route during and after lane changes, when possible.
- Fix lost trajectory during laneChange ([issue #203](https://github.com/esmini/esmini/issues/203))
- Add esmini root folder to binary release packages ([issue #204](https://github.com/esmini/esmini/issues/204))
- Update [ALKS scenarios](https://github.com/asam-oss/OSC-ALKS-scenarios) to [v0.4.1](https://github.com/asam-oss/OSC-ALKS-scenarios/releases/tag/v0.4.1).
- Fix wrong pitch calculation (issue [#197](https://github.com/esmini/esmini/issues/197))
- Improve tesselation wrt elevation and roll rates (issue [#197](https://github.com/esmini/esmini/issues/197))
- Add rounding tolerance to condition rules greaterOrEqual and lessOrEqual
- Updated behavior: Update simulation time after storyboard evalulation (not before)
- Fix typo messing up DistanceCondition ([issue #200](https://github.com/esmini/esmini/issues/200))
- Some additional minor fixes

### 2021-12-09 Version 2.18.2

- Replace plot_csv.py with plot_dat.py
  - No need to create intermediate csv file
  - Increased precision (since not limited by csv file)
- Extend trajectory to complete final step
  - When reaching the end of a trajectory, move the remaning step in the extended line from trajectory end-point and heading.
  - The purpose is to avoid discontinuous velocity (and acceleration spikes)
- Add example scenario ([lane-change_clothoid_based_trajectory.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/lane-change_clothoid_based_trajectory.xosc)) for clothoid based trajectory
- Rename udp-driver subfolder to udp_driver to support Python sub module import

### 2021-12-06 Version 2.18.1

- Improve laneChangeAction behavior with junctions
  - Handle road split case, e.g. highway exit
- Add missing cone model to demo packages
- Add API functions to get object type and 3D model filename ([issue #193](https://github.com/esmini/esmini/issues/193))
- Add launch flag for continuous screen capture in replayer (`--capture_screen`)
- Update dat2csv.py script for current dat version (v2)
- Fix bug causing crash when reset to defaultController

### 2021-12-03 Version 2.18.0

- Support global actions in the Init section of the scenario
  - see example in updated [xosc/swarm.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/swarm.xosc)
- Bump dat file format v1->2. Support object visibility.
- Do not calculate polyline trajectory heading when specified explicitly
- Add cone models cone-45.osgb and cone-100.osgb
  - see example usage in [xodr/crest-curve.xodr](https://github.com/esmini/esmini/blob/8d5e486ccb2ac5d4af2bcdccaa96e53440079260/resources/xodr/crest-curve.xodr#L98)
  - Updated model pack [here](https://drive.google.com/u/1/uc?id=1c3cqRzwY41gWXbg0rmugQkL5I_5L6DH_&export=download)
- Add `--hide_trajectories` launch flag to esmini (already available in replayer)
- Reduce default steering sensitivity of simple vehicle model

### 2021-11-29 Version 2.17.6

- Add optional angular check to ReachPositionCondition
  - If Orientation element is added, then it is checked in addition to position
  - For now using hardcoded tolerance of 0.05 radians (~3 degrees)
  - See examples in scenario [Unittest/xosc/traj-heading-trig.xosc](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Unittest/xosc/traj-heading-trig.xosc).
- Updated behavior: Raise exception/quit on missing required attribute
- Move remaining full step after reaching end of trajectory, avoiding velocity spikes
- Fix replayer bug causing sporadic crash

### 2021-11-26 Version 2.17.5

- Add fixed timestep option to odrviewer (all options see [odrviewer/readme.txt](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Applications/odrviewer/readme.txt))
- Add missing comparison operators (issue [#191](https://github.com/esmini/esmini/issues/191))
- Bugfix: Fix string converted to low precision type (issue [#192](https://github.com/esmini/esmini/issues/192))

### 2021-11-23 Version 2.17.4

- Expose GeoReference data to esminiRM (RoadManager) API
- Add ParameterDeclarationCallback API to C# wrapper ([issue #190](https://github.com/esmini/esmini/issues/190))
  - See [esminiLib/esminiUnityExample.cs](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Libraries/esminiLib/esminiUnityExample.cs) for example usage
- Reduce OSI log size by storing static data only once
- Consider lane offset (and curvature) for steplen calculation ([issue #187](https://github.com/esmini/esmini/issues/187))

### 2021-11-16 Version 2.17.3

- Add more tuneable parameters to SimpleVehicle and expose in API
  - Max speed and acceleration, steering behavior, engine brake
  - See function headers in [esminiLib/esminiLib.hpp](https://github.com/esmini/esmini/blob/4177ac5139db11f899f84b2e9e535eaa5c02ef34/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L888)
- Add framework for country typed traffic signs (from OpenDRIVE to OSI)

### 2021-11-15 Version 2.17.2

- Add missing OSI files to demo packages

### 2021-11-15 Version 2.17.1

- Add API for get and set seed
- Fix ghost on trajectory timing issue
- Fix ECE_ALKS_RefDriverController catalog parameters ([issue #182](https://github.com/esmini/esmini/issues/182))
- Add dependent osi files for UDPDriverController examples

### 2021-11-12 Version 2.17.0

- Support modify parameters during initialization
  - Callback mechanism to change parameter values after
  ParameterDeclaration has been parsed, but before the Init section of
  the scenario.
  - See code example [parametric-init](https://github.com/esmini/esmini/tree/master/EnvironmentSimulator/code-examples/parametric-init).
- Add OSI reading to UDPDriverController examples
  - See [testUDPDriver-minimalistic-example-osi.py](https://github.com/esmini/esmini/blob/master/scripts/udp-driver/testUDPDriver-minimalistic-example-osi.py) and [testUDPDriver-follow-trajectory.py](https://github.com/esmini/esmini/blob/master/scripts/udp-driver/testUDPDriver-follow-trajectory.py).
- Extend SimpleVehicle API in esminiLib
  - Add initial speed to constructor
  - Add SE_SimpleVehicleControlTarget function
  - NOTE: [Vehicle::DrivingControlTarget](https://github.com/esmini/esmini/blob/598b82787c275b5dedd30e1789fa25eb53eb856f/EnvironmentSimulator/Modules/Controllers/vehicle.hpp#L37) arguments changed order
  to harmonize with other SE_SimpleVehicleControl functions.
- Add external controller code example [test-driver](https://github.com/esmini/esmini/tree/master/EnvironmentSimulator/code-examples/test-driver).

Bugfixes
- Fix highway type of junctions to have the correct lane-pairing
- Don't reset ghost on traveledDistance trigger
- Fix steering calculation in Vehicle::DrivingControlTarget

### 2021-11-08 Version 2.16.2

- Ghost vehicle concept updated to handle greater variation of triggers
  - Relative triggers handled using a combination with the teleportAction
  - ResetEvent added to make sure events don't go missing in a teleportation
  - Time handling added to handle the difference between ghost and ego
- Add projection to the geo reference string
- Fix LaneOffsetAction relative target bug (issue [#183](https://github.com/esmini/esmini/issues/183))
- Bugfix: Raise exception on invalid condition instead of crash

### 2021-11-01 Version 2.16.1

- Add ECE ALKS reference driver controller (PR [#181](https://github.com/esmini/esmini/pull/181))
- Separate material for border, sidewalk and biking lane types
  - inner border lanes: dark gray
  - if outmost lane is border: grass
  - sidewalk and biking lanes: light gray
- Fix default mode and init speed issue for ACC/ALKS controllers
- Delete duplicate timestamps in dat files

### 2021-10-28 Version 2.16.0

- Add UDPDriverController
  - Provide an UDP interface to control entities, e.g. for external driver models.
  - Either by explicit position (variants of x,y,z,h,p,r) or driver input (throttle, steering...)
  - Synchronous (blocking) or asynchronous (non-blocking) mode
  - Works both on same or different host (than esmini running on)
  - For more info, see example python scripts in [scripts\udp-driver](https://github.com/esmini/esmini/blob/master/scripts/udp-driver).
- Initial implementation of geo reference
- Add lib API for custom camera positions
- Support additive mode for ACCController
  - In override mode setSpeed is explicitly stated as a property
  - In additive mode setSpeed will adapt to external modifications to speed, e.g. SpeedAction
- Option to remove ghost vehicle info from OSI groundtruth dynamic
- Fix disable_controllers ghost bug
- Fix OSI lane left/right, predecessor/successor and some other OSI related bugs

**Note**: To support the external driver controller some changes to gateway was needed (e.g. extended dirty bit strategy to keep track of what parts of entity states has been updated or not). These changes might affect behavior of existing integrations with esmini, e.g. how and when esmini will align entities to road elevation and/or pitch. Please report any new (undesired) behavior.

### 2021-10-18 Version 2.15.3

- Extend screen capture functionality
  - Support continuous screen capture (individual images though)
  - Add continuous screen capture argument (see [docs/commands.txt](https://github.com/esmini/esmini/blob/master/docs/commands.txt))
  - Add screen capture API to esminiLib
- Support multiple option occurrences
  - e.g. multiple paths can be set by adding several `--path <path to directory>`
- Add support for custom camera positions (partly issue #173)
  - New command line option `--custom_camera <x,y,z,h,p,r>`
  - Multiple custom cameras supported, add any number of custom_camera arguments
  - Make first custom camera default by argument `--camera_mode custom`
  - Switch camera during simulation with key "k"
- Parse vehicle Performance element
  - so far applied only in the ACC controller
- Improve and simplify ACC controller
  - Respect vehicle max acceleration and deceleration Performance properties
  - Simplistic long control based on relative distance and speed
- Add ALKS controller
  - initial dummy implementation inheriting the ACC controller
  - longitudinal domain handled in the same way as the ACC controller
  - lateral domain handled by the default controller (follow lane at current offset)
- Extend use of path(s) for locating files
  - Now both filename with and without any leading relative or absolute
  path will be concatenated with the path entries and tested
- Rework disable controllers strategy
  - Only controllers activated explicitly by the scenario affected
  - Implicit controllers used by e.g. TrafficSwarmAction not affected
- Support OSC 1.1 AssignController activate attributes
  - controllers can be activated when assigned
- Fix missing euclidianDistance support in RelativeDistanceCondition
- Bugfix: Add missing boolean support in setParameterValue()
- Fix missing dummy boundingbox when model failed to load
  - the bug was introduced in v2.14.0
- Catch missing junction error and continue anyway (issue [#174](https://github.com/esmini/esmini/issues/174))

### 2021-10-07 Version 2.15.2

- Fix alternating lane offset issue in routes, e.g. AcquirePosition (PR #[167](https://github.com/esmini/esmini/pull/167))
- Fix random way selector issue resulting in always same choice in intersections
- Add missing bounding box

### 2021-10-05 Version 2.15.1

- Add missing motorbike 3D model

### 2021-10-04 Version 2.15.0

- Add limited support for TrafficSwarmAction
  - TrafficDefinition not supported yet
  - Vehicle models picked randomly from specified catalogs
  - Simplified driver model only based on ACC-Controller, not traffic rules
  - Experimental implementation, expect bugs and shortcomings
  - Example scenario [swarm.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/swarm.xosc)
- Added option `--seed <number>` to specify random seed from a previous runs
  - seed is always printed to console/log file, so it can be grabbed from there
  - seed reuse only per platform (e.g. Windows seed gives different result on Linux)
  - see [docs/commands.txt](https://github.com/esmini/esmini/blob/master/docs/commands.txt) for all launch options
- Support temporary objects in replayer
  - Show only active objects at given time frame
  - Show only active trajectories at given time frame
- Fix some issues in entity freespace distance calculations
- ACC adjustments slightly reducing intersection deadlocks
- Fix crash due to unsupported OpenSCENARIO condition
- Fix wheel rotations in esminiLib

### 2021-09-30 Version 2.14.2

- Fix some action failures in repeated events (maximumExecutionCount > 1)
  - Root cause: Internal timer not reset between runs
- Fix ACC controller issue for scenarios with multiple vehicles

### 2021-09-29 Version 2.14.1

- Add visualization of externally reported OSI sensorView
  - Add argument `--sensors` or just press 'r' to view
- Add support for motorbike modelId=10 in replayer
- mc.osgb added to model pack [models.7z](https://www.dropbox.com/s/5gk8bvgzqiaaoco/models.7z?dl=1)
  - cyclist.osgb updated wrt reference point (rear axle projected on ground)
- Fix OverrideControllerValueAction domain handling
  - Assigned control strategy depends on type of controller value (e.g. steering is lateral, throttle is longitudinal)
- Fix issue with controllers being aborted by conflicting actions
  - ActivateControllerAction itself does not assign any control strategy

### 2021-09-23 Version 2.14.0

- Add entity 3D model and bounding box scale options
  - optional property "scaleMode" for scenario objects. Values:
    - None (default) = Don't scale 3D model or bounding box
    - BBToModel = Scale bounding box to fit loaded 3D model
    - ModelToBB = Scale model to fit specified bounding box
  see examples in [VehicleCatalog](https://github.com/esmini/esmini/blob/master/resources/xosc/Catalogs/Vehicles/VehicleCatalog.xosc)
  - scaleMode added to .dat files as well for scenario replay
  **Note:** .dat file format has been updated, dat files created in previous versions of esmini will not play.
- dat fileformat version control
  - replayer and dat2csv now checking for supported version instead of crashing
- Support ScenarioObject attribute "model3d", added in OpenSCENARIO v1.1
  - "File filepath" property still supported as well (if model3d missing)
- Support Clothoid attribute curvaturePrime (renamed from curvatureDot in OSC v1.1)
  - curvatureDot attribute still supported as well (if curvaturePrime missing)
- Fix bug that in rare cases prevented init actions from reaching complete state

### 2021-09-17 Version 2.13.6

- Fix OSI angle ranges to [-pi, pi]
  - current range [0, 2pi] is not aligned with OSI standard

### 2021-09-15 Version 2.13.5

- Stop conflicting actions when starting new ones (issues [#155](https://github.com/esmini/esmini/issues/155) and [#157](https://github.com/esmini/esmini/issues/157))
- Add velocity to CSV log (issue [#156](https://github.com/esmini/esmini/issues/156))

### 2021-09-10 Version 2.13.4

- Fix SetLanePos s truncation bug
  - reaching end of road or route not handled as error
- Fix trajectory time handling and add timref check
  - Don't assume first timestamp to be 0.0
  - If trajectory duration (from timestamps) is 0, set timeref=NONE
- Fix and sync C# wrappers
- OSI driving side fixed for Right and Left hand traffic
- Separate logfile for roadmanager lib (esminiRM_log.txt)
- Fix trajectory findClosestPoint bug
- Explicitly specify Xcode generator for cmake on macOS
- Some code improvements:
  - Some improved type checking for enum types
  - Some added safeguarding of uninitialized scenario and road objects

### 2021-08-31 Version 2.13.3

- Add build option for dynamic protobuf linking (see more info [here](https://github.com/esmini/esmini/blob/master/docs/BuildInstructions.md#dynamic-protobuf-linking))
- Add junction ID to position object
    - can be used to determine whether in a junction or not (-1 => not in junction)
    - see code example [here](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/code-examples/rm-basic/rm-basic.cpp#L65)
- Parse OpenDRIVE controllers. See code example in test case [TestControllers](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Unittest/RoadManager_test.cpp#L1979).
- Fix odrplot bug with roads < 1m

### 2021-08-27 Version 2.13.2

- Bugfix for osi intersections

### 2021-08-26 Version 2.13.1

- Fix collision condition parsing bug causing crash for the ByType option (issue [#149](https://github.com/esmini/esmini/issues/149))

### 2021-08-24 Version 2.13.0

- Add a simple ACC-Controller
  - and example scenario [acc-test.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/acc-test.xosc) / [run_acc-test.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_acc-test.bat)
- Extend CSV logger with acc and yaw rate (issue [#145](https://github.com/esmini/esmini/issues/145))
- Add support for MiscObject catalogs (not tested yet, issue [#146](https://github.com/esmini/esmini/issues/146))
- Use controller's name for missing esminiController property
  - E.g. by naming the controller "ExternalController", the line:
    `<Property name="esminiController" value="ExternalController" />`
    can be omitted.
- Update roadmanager junction strategy using angle instead of limited discreet choices
  - Road with closest heading direction will be selected (can be randomized as well)
- Add event default trigger if missing (issue [#147](https://github.com/esmini/esmini/issues/147))

### 2021-08-18 Version 2.12.6

- [Fix](https://github.com/esmini/esmini/commit/d74552a462e4449f04bff8606e86f857ae9ec5ab) major bug causing lane width issue for roads with more than one laneSection
  - The bug was introduced 2021-07-02 in commit [4e95a09](https://github.com/esmini/esmini/commit/4e95a09e2aca83fb1fec104f340bf2fbdb262c8d) and release v2.11.3.
- Improve nurbs heading ([#144](https://github.com/esmini/esmini/issues/144))
  - Calculate heading from actual nurbs instead of polyline approximation
- Traffic sign added to OSI ground truth
- Add OSI lane pairing
- Fix condition timer restart issue
  - Old behavior prevents timer from restarting. This commit fix so that timer restarts if condition becomes true again from being false.
- Support halt/pause at end of trajectory
  - When trajectory control points includes time stamps, end of trajectory should be based on time instead of actually reaching full length.

### 2021-08-13 Version 2.12.5

- Fix nurbs heading issues ([#144](https://github.com/esmini/esmini/issues/144))
  - Interpolate heading from polyline approximation
  - Fix bug reusing heading of second vertex to first one
- Add road object lane validity support (issue [#143](https://github.com/esmini/esmini/issues/143))
- Add show version option (--version)

### 2021-08-03 Version 2.12.4

- Fix csv-logger missing first entry (issue [#137](https://github.com/esmini/esmini/issues/137))
- Fix relative distance issue ([#139](https://github.com/esmini/esmini/issues/139))
  - Consider heading when determine entities inter-displacement

### 2021-07-27 Version 2.12.3

- Fix LongitudinalDistanceAction bugs
  - Use correct distance mode
  - Time gap based on referenced enity's speed instead of own speed

### 2021-07-26 Version 2.12.2

- Support OpenSCENARIO 1.1 LongitudinalDistanceAction / displacement attribute
- Improve expression support (e.g. handle "not()" )
- Custom OSI trace filename (see [docs/commands.txt](https://github.com/esmini/esmini/blob/master/docs/commands.txt))
- Center lane logic for broken solid and solid broken
- Some additional minor fixes

### 2021-07-13 Version 2.12.1

- Restore support for "$" prefix in parameter names
- Catch and log expression errors

### 2021-07-13 Version 2.12.0

- Support OpenSCENARIO 1.1 expressions
  - e.g. `<AbsoluteTargetSpeed value="${$EgoSpeed / 3.6}"/>`
- Fix nested parameter declarations issue, ensure limited scope
- Do not reset heading at end of trajectory
  - If alignment is desired, add explicit TeleportAction/RelativeLanePosition with relative h = 0. See example [here](https://github.com/esmini/esmini/blob/f87c354a2f1841e63b02a9f35c1737270b302bfd/resources/xosc/trajectory-test.xosc#L269c).
- Some OSI refactorization and optimization, static data only sent first time

### 2021-07-06 Version 2.11.4

- Fix bug preventing OSC 1.1 controller activation
- Warn for cond edges at simTime 0 (might be missed since edge can not be defined)
- Some additional minor fixes

### 2021-07-05 Version 2.11.3

- Add support for solid/broken roadmark combinations
- Fix issue with multiple waypoints per road
    - Remove any additional waypoints on same road
    - Keep only first on first road, and last on following roads
- Bugfixes:
    - Fix expected location of ActivateControllerAction in OSC 1.1 (issue [#133](https://github.com/esmini/esmini/issues/133))
    - Fix issue with final double entry in .dat files
    - Bugfix causing zero lanewidth reported at edge cases where s > length of road

### 2021-06-24 Version 2.11.2

- Fix issue in LaneSection::GetClosestLaneIdx() sometimes returning wrong lane (PR [#131](https://github.com/esmini/esmini/pull/131))
- Update [ALKS scenarios](https://github.com/asam-oss/OSC-ALKS-scenarios) to [b03cc8a](https://github.com/asam-oss/OSC-ALKS-scenarios/commit/b03cc8a20882a4acffa96b83ae16769834bf0861)
- Fix bug missing ghost coming to a stop
- Some additional minor fixes

### 2021-06-21 Version 2.11.1

- Fix file path issue in sumo-test config
  - Bug prevented [sumo-test.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/sumo-test.xosc) to load on Linux

### 2021-06-18 Version 2.11.0

- Update behavior of the Default Controller
  - Entities will still be aligned to the road direction, but the relative heading will be respected.
    - To have a vehicle heading in driving direction (as defined by lane Id and road rule) simply set relative heading=0, or omit it since 0 is default.
    - To have a vehicle heading in opposite driving direction, set relative heading = 3.14159 (PI/180 deg)
    - To have an entity facing 90 deg left, set relative heading = 1.57 (PI/2)
    - The entity will be moved in the road direction closest to its heading, so
    - To have an entity facing 90 deg left, set relative heading slightly less than PI/2 ~ 1.57. I.e NOT > PI/2 because then it will got the other way.
  - This behaviour allows for flexible control of the orientation more independent of driving direction, see new scenario example [drop-bike.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/drop-bike.xosc) / [run_drop-bike.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_drop-bike.bat)
- Fix some [SUMO](https://www.eclipse.org/sumo/) integration issues
  - add another demo scenario with 100 cars: [sumo-test.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/sumo-test.xosc) / [run_sumo-test.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_sumo-test.bat)
- Fix and improve relative distance calculations in road coordinates
cars.
- Fix grass color when texture not available (from white to green)
- Update ALKS tests, confirming esmini still works with latest [ALKS](https://github.com/asam-oss/OSC-ALKS-scenarios) version.
- Some additional minor fixes

### 2021-06-14 Version 2.10.2

- Fix condition delay issue
  Previous behavior was to return true only once after timer expired. Then, if condition still true, restart the timer.
  Now the behavior is:
  - Timer is started when condition is evaluated to true. No more evaluations will be done from this point.
  - When timer has expired the condition will always return true
- Set correct required cmake version for FILTER feature
- Some additional minor fixes

### 2021-06-14 Version 2.10.1

- Bugfixes:
  - Fix typecast bug in esminiLib GetRoadInfoAtDistance/SE_RoadInfo (roadId and laneId)
  - Avoid crash on road object load failure, e.g. when road sign .osgb file is not found
- Support slimmed esmini by making OSG, OSI, SUMO and googletest optional
  - see more info [here](https://github.com/esmini/esmini/blob/master/docs/BuildInstructions.md#slim-esmini---customize-configration)
- Add relevant modules to non OSG build
  - E.g. odrplot (not OSG dependent) and dat2csv
- Add missing "driver" option to --camera_mode usage help text ([esmini](https://github.com/esmini/esmini/blob/master/docs/commands.txt) and [replayer](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Applications/replayer/readme.txt))
- Add [xodr coverage file](https://github.com/esmini/esmini/blob/master/odr_coverage.txt)

### 2021-06-10 Version 2.10.0

- Support repeat road object
  - Primary use case is to populate copies of imported .osgb files along the road, e.g. guard-rails or guide-posts
  - Dimensions (len, width, height) can be adjusted (even change linearly along the stretch)
  - Add examples in [e6mini.xodr](https://github.com/esmini/esmini/blob/master/resources/xodr/e6mini.xodr) (rails and guide-posts) and
  [curve_r100.xodr](https://github.com/esmini/esmini/blob/master/resources/xodr/curve_r100.xodr) (guide-posts). These roads are used in e.g:
    - [run/odrviewer/run_e6mini.bat](https://github.com/esmini/esmini/blob/master/run/odrviewer/run_e6mini.bat)
    - [run/esmini/run_cut-in.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_cut-in.bat)
    - [run/esmini/run_lane_change_simple.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_lane_change_simple.bat)
    - [run/esmini/run_controller_test.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_controller_test.bat)
- Add railing and guide-post 3D models to the model package.
  - Models are included in the demo package. The complete updated package is found [here](https://www.dropbox.com/s/5gk8bvgzqiaaoco/models.7z?dl=1).
- Add option to NOT include/generate road object models
  (--generate_no_road_objects). Useful IF the objects are already populated in a referred scenegraph file.
- Add return codes to esminiRMLib functions [RM_GetProbeInfo()](https://github.com/esmini/esmini/blob/5f30391896c05e8b9efdcbea1b2c181589067b40/EnvironmentSimulator/Libraries/esminiRMLib/esminiRMLib.hpp#L326) and [RM_SubtractAFromB()](https://github.com/esmini/esmini/blob/5f30391896c05e8b9efdcbea1b2c181589067b40/EnvironmentSimulator/Libraries/esminiRMLib/esminiRMLib.hpp#L335)
- Some additional minor fixes

### 2021-06-04 Version 2.9.0

- Support simpler route definitions
  - Resolve, if possible, any number intermediate waypoints
  - Only start and end is needed, shortest route will be resolved
  - User can specify additional intermediate waypoints for desired explicit routing
  - Shortest path will only be searched for in forward (vehicle heading) direction
- Add AcquirePositionAction
  - Also add example scenario [routing-test.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/routing-test.xosc) (and launch script [run_routing-test.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_routing-test.bat)) demonstrating improved routing and AcquirePositionAction.
- Support OpenDRIVE road "rule" attribute
  - When heading is not explicitly specified, entities will be aligned in the road direction according to the traffic rule (RHT=right-hand traffic, LHT=left-hand traffic).
  - Example [e6mini-lht.xodr](https://github.com/esmini/esmini/blob/master/resources/xodr/e6mini-lht.xodr) and [left-hand-traffic_using_road_rule.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/left-hand-traffic_using_road_rule.xosc)
- Fix bug that could cause SpeedAction to not terminate

### 2021-05-26 Version 2.8.4

- Updated OSI and Protobuf versions (v3.3.1 / v3.15.2)
- End AssignRouteAction immediately according to standard
- Add useful OSG options to [help](https://github.com/esmini/esmini/blob/master/docs/commands.txt) (issue [#119](https://github.com/esmini/esmini/issues/119))
- Some additional minor fixes

### 2021-05-19 Version 2.8.3

- Bugfix: Fix road model generator issue
  - tesselation error could cause application crash or dark model
  - the bug was introduced in 2.8.2, so avoid that release.

### 2021-05-17 Version 2.8.2

- Add first person "driver" view
  - Activate by toggle view on 'k' (press a few times to reach driver view), or
  - launch argument: ```--camera_mode driver```
  - example script: [run/esmini/run_lane_change_crest_driver-view.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_lane_change_crest_driver-view.bat)
- Add reverse junction links for left hand traffic (related issue [#114](https://github.com/esmini/esmini/issues/114))
- Fix nan compile error (issue [#115](https://github.com/esmini/esmini/issues/115))
- Add experimental Abs2Rel Controller
- Some additional minor fixes

### 2021-05-07 Version 2.8.1

- Add some accessors to RMObject, e.g. GetType(), GetLength()... (issue #109)
- Increase precision in csv files (created by dat2csv and dat2csv.py)
- Bugfixes:
  - Fix LaneChange issue over end-to-end/start-to-start road succession (PR #110)
  - Update filename container size in dat2csv.py
  - Fix wrong nurbs trajectory time interpolation
  - Improve action timings (push interpolations one time step)
  - Fix SE_ReportObject\* functions, prevent values getting overwritten
- Some additional minor fixes

### 2021-04-29 Version 2.8.0

- New feature in odrplot: Indicate road ID and direction
- odrviewer update: Respawn vehicles at open road ends (if any)
- Bugfix: Remove 0.5m trajectory end tolerance causing wrong end position
- Add [API](https://github.com/esmini/esmini/blob/f8a0cd739528a1811ab2d595ff47709bdc077377/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L803) for simple vehicle performance
- Improve replayer file path handling for odr and osgb
  - Expand filename container and store complete file paths in .dat file
  NOTE: This change affects .dat file format - old recordings files will not play
  - If absolute path not found, test combinations using res_path argument
- Added mandatory field maxAcceleration to the vehicle catalog

### 2021-04-26 Version 2.7.4

- Add [func](https://github.com/esmini/esmini/blob/1340dad935c52ad955963a484e5d12b3f431a40a/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L748) to set explicit OSI timestamp
- Fix issues in OverrideControllerValueAction
  - Correct wrong override action element name
  - Quit action immediately (as clarified in OSC v1.1 User Guide)
- Improve scroll wheel handling (issue [#105](https://github.com/esmini/esmini/issues/105))
  - inverse wheel in OSG default camera models
  - add scroll wheel zoom to esmini camera models
- Add ideal-sensors [code example](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/code-examples/ideal-sensors/ideal-sensors.cpp)
- Fix odrplot step-length bug (issue [#100](https://github.com/esmini/esmini/issues/100))
- Add brief info on 3D model conversion to [readme](https://github.com/esmini/esmini#3d-model-support) (issue [#63](https://github.com/esmini/esmini/issues/63))
- Some additional minor fixes

### 2021-04-16 Version 2.7.3

- API to get object properties
- Extend road sign API (more attributes, e.g. height), see example use in [rm-basic.cpp](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/code-examples/rm-basic/rm-basic.cpp).
- Fix wrong OverrideControllerValueAction type names
- New feature: Print help text to console, trig on key 'H' (shift 'h')
- Some additional minor fixes

### 2021-04-14 Version 2.7.2

- Fix bug so roads without type can be used with osi intersections

### 2021-04-14 Version 2.7.1

- Fix wrong parameter type preventing synchronize example scenario to run

### 2021-04-13 Version 2.7.0

- Support selected parts of OpenSCENARIO v1.1
  for example:
   - TrajectoryPosition
   - FollowTrajectory with initialDistanceOffset
   - SynchronizeAction with steady state option
   - CoordinateSystem and RelativeDistanceType in distance operations

  Most significant limitations:
   - Logical scenarios not supported yet (e.g. ParameterValueDistribution and StochasticDistribution)
   - Expressions not supported yet (Arithmetic calculations and logical expressions)

   [OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/) v1.1 examples runs (with above limitations)

Other updates:
- Support OSI intersections
- Implement OverrideControllerValueAction
- Add typed Get and Set functions for named parameters
- Add Vehicle ParameterDeclaration support
- Add support for boolean parameter type in conditions
- Add "--disable_stdout" option to prevent log messages being written to console
- Remove debug trace (code module, code line...) as default setting
- Clean up log messages
- Add scenarioEngine unit test module, with one initial test
- Fix road::GetWidth both-sides bug (issue #96)
- Fix trajectory heading interpolation issue
- Add a few basic [code examples](https://github.com/esmini/esmini/tree/master/EnvironmentSimulator/code-examples), e.g. [how to use esminiRMLib](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/code-examples/rm-basic/rm-basic.cpp)

### 2021-04-01 Version 2.6.1

- New feature: Visualize complete driving trajectories in replayer
- Improve dat2csv.py and align to behavior of dat2csv.cpp
- Add object type and category to esminiLib API
- Add funct to get all named parameter names (and type)
- Fix replayer issue with time < 0 (ghost use cases)
- Support parameterRef with and without prefix '$' (in ParameterAssignment)
- Fix heading bug in ghost trail

### 2021-03-26 Version 2.6.0

- Support nurbs trajectories
  - Order, controlpoint Timestamp and weight taken into account
  - Means that now all shape types are supported: polyLine, clothoid and nurbs
  - See updated example [trajectory-test.xosc](https://github.com/esmini/esmini/blob/ec8cfbc70f3360cbf8b167b2c0eb30cb9ba65931/resources/xosc/trajectory-test.xosc#L207). (run script: [run/esmini/run_trajectory-test.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_trajectory-test.bat)) part of the [esmini demo package](https://github.com/esmini/esmini/releases/latest).

- Visualization active trajectory
  - can be toggled with 'n' key (see [docs/readme.txt](https://github.com/esmini/esmini/blob/master/docs/readme.txt) for all key-shortcuts)

- Improve object heading when following polyLine trajectory
  - Instead of interpolating heading all the way between two control points, follow the angle of the line and just interpolate a few meters before and after control point.

- Fix object orientation limitations
  - Fully separate object orientation from road pitch and bank to support correct object rotations on any road and trajectory
  - Correct pitch and roll angles will now also be recorded into .dat files

- Fix precision issue in odrplot

- Fix issue with too large OSI UDP messages
  - Now large messages are split into smaller chunks.
  - Updated [Applications/replayer/osi_receiver.cpp](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Applications/replayer/osi_receiver.cpp) shows how to deal with it on receiver side.

- Add option to remove objects in replayer (see [replayer/readme.txt](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Applications/replayer/readme.txt) for more info)

- Add [dat2csv](https://github.com/esmini/esmini/blob/master/scripts/dat2csv.py) Python script (similar to C++ application [dat2csv.cpp](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Applications/replayer/dat2csv.cpp))

- Update .dat file format to simplify parsing in Python
  - Also ensuring portablility between Windows and Linux
  - NOTE: This change is NOT backward compatible (old .dat file not supported in this and future esmini versions)
  - Hopefully format will now stabilize so .dat files can be used between esmini versions

- Add OSI sensorData to [example code](https://github.com/esmini/esmini/blob/ec8cfbc70f3360cbf8b167b2c0eb30cb9ba65931/EnvironmentSimulator/Applications/esmini-dyn/main.cpp#L245)

- Some additional minor fixes

### 2021-03-18 Version 2.5.2

- Add shortcut keys in replayer app for jumping to start and end of scenario
    - Ctrl + Left (arrow): Jump to start
    - Ctrl + Right (arrow): Jump to end
    - see updated [readme.txt](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Applications/replayer/readme.txt)
- Fix lateralprofile bug
    - All child elements to \<lateralProfile\> was incorrectly assumed to be of
    type \<superelevation\>. Which is wrong, and any \<shape\> element would
    cause broken road and popcorn effect on road users.
    - Now any \<shape\> elements are ignored (until being supported).

### 2021-03-16 Version 2.5.1

- Improve error handling
  - Always log exception messages
  - Handle (quit with log message) invalid object references
- Update Unity wrappers (align to current API)
- Add OSI groundtruth [example](https://github.com/esmini/esmini/tree/master/Hello-World_coding-example#osi-groundtruth) to Hello World tutorial
- Add brief [info](https://github.com/esmini/esmini#OSI-support) on OSI to README, including OSI/Protobuf versions

### 2021-03-12 Version 2.5.0

- Support freespace distance measurement

  Some conditions and actions offer a choice on how to measure distance:
  - freespace = false: Distance between the reference points of objects (low fidelity)
  - freespace = true: Distance between closest bounding box points (high fidelity)

  So far esmini has accepted freespace=true without actually implementing it, leading to no effect but the same result as for freespace=false. Now it is implemented and applied accordingly.

  Conditions with freespace option:
  - TimeHeadwayCondition
  - TimeToCollisionCondition
  - DistanceCondition
  - RelativeDistanceCondition

  Actions with freespace option:
  - LongitudinalDistanceAction
  - (LateralDistanceAction - not implemented in esmini yet)

- Fix road mark bug ([issue #83](https://github.com/esmini/esmini/issues/83))
  The bug could cause visual defects on roads with multiple lane sections.

- Fix relative orientation bug ([PR #85](https://github.com/esmini/esmini/pull/85))
  Could result in wrong vehicle heading caused by referring to road heading before known.


### 2021-03-05 Version 2.4.4

- Further log file improvements
  - Option to disable logfile
  - Support custom log filename
  - These features available both from command line and API
- Stabilize lane matching
  - Increase tension in ZY2Road mapping, add 2m threshold before snapping to
    another lane. Purpose is to stay in current path for driver models.
- Fix bug in Road Manager causing XY2Road to (sometimes) match the wrong road
- Add some recent esminiLib features to esminiRMLib as well

### 2021-03-03 Version 2.4.3

- Improved log file handling
  - If default log file can't be created for some reason, try with system provided temp filename. Last resort is to run without logfile, just logging to console.
- Fix Event and Action life cycle issues
  - Default maximumExecutionCount for Event and ManeuverGroup is now 1 (instead of infinity)
  - Finished Action will move to Complete state instead of Standby, making it
      possible to trig on Action being completed.
  - **Note**: This change CAN have impact on scenarios which might  need to be updated, e.g. excplicitly specifying maximumExecutionCount.

### 2021-03-02 Version 2.4.2

- Improve dat file format
  - Decouple dat format from Position class
  - Reduce file size by factor 4
  - Stabilize format: dat files can be saved and run in later versions of esmini, to greater extent than before.

### 2021-03-01 Version 2.4.1

- Add API for reporting actual acceleration and velocity of external objects
- Improve lane matching in XY2Road position mapping
- Optimize road manager XY2Road mapping (simplify road width calculations)
- Add options related to road lookahead functions
    - Lock object to current lane. Flag that will preserve lane ID regardless of
      lateral position. Useful for driving models, where look-ahead normally should start
      from the original lane.
    - Option in LookAhead function to look along actual driving direction or
      along road primary driving direction.
    - Function returns additional info: roadId, laneId, laneOffset, s, t.
- Update Hello World driver model example, utilizing new options in look-ahead function

### 2021-02-22 Version 2.4.0

- New feature: Support for OpenDRIVE road object outlines
  - support open and closed shapes
  - support both cornerRoad and cornerLocal specifications
  - add a "roof" mesh on closed shapes

  see example OpenDRIVE [crest-curve.xodr](https://github.com/esmini/esmini/blob/master/resources/xodr/crest-curve.xodr) used in scenario [lane_change_crest.xosc]("https://github.com/esmini/esmini/blob/master/resources/xosc/lane_change_crest.xosc").
    To run it, go to esmini/run/esmini and run the script [run_lane_change_crest.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_lane_change_crest.bat).
- Improve OSI performance
  - Static and dynamic groundthruth data updated separately
  - OSI API new functions: SE_UpdateOSIStaticGroundTruth and SE_UpdateOSIDynamicGroundTruth
  - Old function SE_UpdateOSIGroundTruth depricated, will be removed enventually
- Add option to tune OSI tolerances for road points (and 3D model generator)

### 2021-02-17 Version 2.3.2

- Fix issue with normalized ParamPoly3
- Relax StandStillCondition - allow for minor "noise"
- Moved OSI ref point to center of vehicle bounding box (instead of OSC rear axel ref. point)
- Add smoke tests and [ALKS scenarios](https://github.com/arauschert/OSC-ALKS-scenarios) test suite as acceptance steps in the CI service
- Some additional minor fixes

### 2021-02-10 Version 2.3.1

- Fix SpeedAction distance dimension issue
- Fix ParamPoly3 arc length issue

  Motion along paramPoly3 geometries has wrongly been based on the curve
parameter which leads to wrong speed. Now the arc length is calculated
and is used instead as basis for motion along the curve.
- Some additional minor fixes

### 2021-02-08 Version 2.3.0

- Add support for cubic dynamics in Speed, LaneChange and LaneOffset
actions.
- Complete implementation of laneOffset and laneChange actions
  - Obey max_lateral_acc in LaneOffset action (previously ignored)
  - Calculate correct LaneChange duration/distance based on (lateral) rate
- Minor fix: Set default value 250 kph for LongitudinalDistanceAction/maxSpeed

### 2021-02-04 Version 2.2.0

- New feature: Support visualization of OpenDRIVE road signs and objects

  The OpenDRIVE signal attribute name is used for 3D model reference. So far only Swedish speed signs are distributed with esmini. But the concept is generic and allows for customized database of many signs. The pole is separated and handled as an OpenDRIVE object, which is also supported in similar way: Name referring to 3D model.

  Updated complete model pack can be downloaded from [here](https://www.dropbox.com/s/5gk8bvgzqiaaoco/models.7z?dl=1), unpack into esmini/resources/models.

  See example [straight_500m_signs.xodr](https://github.com/esmini/esmini/blob/master/resources/xodr/straight_500m_signs.xodr) which is used in updated scenarios:
  - [distance_test.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/distance_test.xosc) ([run/esmini/run_dist_test.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_dist_test.bat))
  - [slow-lead-vehicle.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/slow-lead-vehicle.xosc) ([run/esmini/run_slow-lead-vehicle.bat](https://github.com/esmini/esmini/blob/master/run/esmini/run_slow-lead-vehicle.bat))

  How to get information on road signs via API, see example usage in [esmini-dyn/main.cpp](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Applications/esmini-dyn/main.cpp#L144)

  Please note that esmini OSI output is not yet propagating signal and object info.

- Add argument for adding search path prefix. (see [launch commands](https://github.com/esmini/esmini/blob/master/docs/commands.txt))
- Calculate pline trajectory headings if Orientation missing
- Add odometer to overlay info text
- Add overlay info text to odrviewer
- Fix pline trajectory heading interpolation bug
- Fix lane change on route issue (not well tested)
- Some additional minor fixes

### 2021-01-26 Version 2.1.5

- Fix typo ParameterSetAction -> SetAction
- Fix pitch and roll initialization issue, now aligned to road as default
- Update and extend the Driver model chapter in [Hello-World examples](https://github.com/esmini/esmini/tree/master/Hello-World_coding-example).
- Some additional minor fixes

### 2021-01-20 Version 2.1.4

- Support for OpenDRIVE road superelevation (lateral inclination) - thanks cfschaller
- Add a few missing modes and shapes to SpeedAction, now aligned with [osg_coverage.txt](https://github.com/esmini/esmini/blob/master/osc_coverage.txt)

### 2021-01-15 Version 2.1.3

- Fix macOS Big Sur issue, avoid crash by moving window position
- Add option for custom steplength in odrplot
- Some additional minor fixes

### 2021-01-11 Version 2.1.2

- Improve collision detection (in CollisionCondition) now based on bounding boxes
- Fix ghost controller bug causing premature stop trigger
- Some additional minor fixes

### 2021-01-08 Version 2.1.1
- Add ground surface textures for road model generator
- Add the textures to demo pack

Updated complete model pack can be downloaded from [here](https://www.dropbox.com/s/5gk8bvgzqiaaoco/models.7z?dl=1)

The road model generator is exercised by the following example scripts:
- run/esmini/run_lane_change.bat
- run/esmini/run_dist_test.bat
- run/odrviewer/run_e6mini.bat

### 2021-01-07 Version 2.1.0
- New feature: Generate simple road 3D model if missing.
- Adapt to compiler warning level 4 (Win/VisualStudio)

### 2021-01-04 Version 2.0.15
- Updates to get RControlStation integration back on feet
- Some additional minor fixes

### 2020-12-22 Version 2.0.14
- Fix OSI output to support all OpenSCENARIO vehicle types
- Option to specify custom OSI tracefile filename and path
- Add plot script to demo pack
- Some additional minor fixes

### 2020-12-17 Version 2.0.13
- All EntityConditions now supported by addition of the following remaining ones (scenario demonstrating the condition in parenthesis):
  - Offroad (lane_change.xosc)
  - Acceleration (cut-in_simple.xosc)
  - StandStill (synchronize.xosc)
  - Speed (ltap-od.xosc)
  - RelativeSpeed (slow-lead-vehicle.xosc)

- All Position types now supported by addition of the following remaining ones:
  - RelativeRoadPosition
  - RoadPosition

### 2020-12-13 Version 2.0.12
- Add SynchronizeAction with SteadyState example scenario to demo package
- Improve world to road coordinate mapping (thanks brifsttar, for input & support)

### 2020-12-11 Version 2.0.11
- Allow storyboard element end transition directly from Standby to Complete state (when no run time needed)
- Added example scenario to demonstrate SynchronizeAction with SteadyState extension
- Fix a link issue that can appear with Visual Studio and vcpkg in Windows
- Some code clean up and other minor fixes

### 2020-12-08 Version 2.0.10
- Changed storyboard element ```maximumExecutionCount``` default value from 1 to infinite
- Improved trigger logging, including all involved conditions and triggering entities
- Updated Unity integration files and added example package, see [here](https://github.com/esmini/esmini#unity-support)
- Added support for CollisionCondition ByType (previously only EntityRef supported)
- Added brief info on OSG tools (e.g. converting 3D models .osgb <-> .fbx), see [here](https://github.com/esmini/esmini/blob/master/docs/BuildInstructions.md#osgconv)
- Fixed an xy2road issue (finding closest road coord from arbitrary x,y position)
- Some additional minor fixes

### 2020-12-04 Version 2.0.9
- Add support for named parameters. See [InnerWorkings/parameters](https://github.com/esmini/esmini/blob/master/docs/InnerWorkings.md#parameters).
- Add experimental steady state to SynchronizeAction, see [osc-extensions.xml](https://github.com/esmini/esmini/blob/master/docs/osc-extensions.xml) and example usage in [pedestrian_traj_synch.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/pedestrian_traj_synch.xosc).
- Visualize all lanes, including non drivable (e.g. border)
- Support space in filenames and file paths (with --osc, enclose with "")
- Some additional minor fixes

### 2020-11-27 Version 2.0.8
- Fix multi-session issues
  - Add method to clear paths between scenarios.
  - Reset controllers between scenario runs (fixes issue with --disable_controllers
    not having effect)
- Fix relative position bug (which caused wrong lateral position in mapping x,y to road coordinates)
- Fix typos in scenario files

### 2020-11-26 Version 2.0.7
- Add position to on-screen info
- Fix end position issue in FollowTrajectory (issue appeared when connecting multiple trajectories)
- odrplot improvements:
  - plot lane boundary instead of lane center
  - plot non drivable lanes in gray
  - maximize axis to simplify panning and zooming
- Fix LaneChange relative lane bug (when relative target lane is on other side of reference line)
- Add search path support for OpenDRIVE and models
- Some additional minor fixes

### 2020-11-17 Version 2.0.6
- Fixed minor issues in Hello World tutorial
- Set correct direction for OSI lanes
- Fix pedestrian catalog path for demo package

### 2020-11-15 Version 2.0.5
- Added a simple vehicle and driver model example to Hello World code guide
- Change interpretation of relative lane in LaneChangeAction. Now sign of relative lane is always according to reference line, regardless of driving direction. To our understanding better complying with OpenSCENARIO 1.0.
- Add analog driver control to the simple vehicle model class
- Improve XYZ2Track method. Now consider overlapping roads (in junctions) when searching best match (not just first hit).
- Add missing pedestrian catalog
- Fix a memory bug potentially causing crash when --disable_controllers flag is set
- Added a small curvy road with and without elevation profile
- Some additional minor fixes

### 2020-11-13 Version 2.0.4
- Align Hello World instructions with updated API
- Fix OpenDRIVE poly3 issues
- Adjust keepDistance tension proportional to MaxAcc (experimental)
- Add position to on-screen info
- Some additional minor fixes

### 2020-11-09 Version 2.0.3
- Fix model ID issue in replayer (caused crash when scenario involves pedestrians)
- Adjusted syntax (XSD) for optional SynchronizeAction tolerances (OpenSCENARIO extension)

### 2020-11-09 Version 2.0.2
- Added pause and step features to replayer
- Solved issue with ghost vehicles in replayer
- Improved end criterias for SynchronizeAction end, detecting when destination gas been passed (increasing dsistance)
- SynchronizeAction: Added tolerances to target positions. Can be set with attribute. Default 1m.
- Corrected some paths in HelloWorld code example
- Some additional minor fixes

### 2020-11-02 Version 2.0.1
- Fixed issue with syncronizeAction and trajectory following entities
- Added correctly dimensioned (according to OSC entity def) bounding box. Toggle show on key ',' (comma)
- Fixed execution order so that controllers and callback are applied AFTER default controller
- Some additional minor fixes

### 2020-10-23 Version 2.0.0
The major functional change is the implementation of the OpenSCENARIO controller concept. A side effect is that much of the functionality such as Ghost concept, interactive driving and external control previously assiciated and hardcoded in different applications now has moved out from the application(s) core and into different controllers which can be activated on demand in a more flexible way.

The actual application code gets much simpler and it makes no sense to have different applications for different use cases. As a result we decided to slim down to only two applications:
1. `esmini` linking internal modules statically providing full access to internal API's
2. `esmini-dyn` demonstrating use of the high-level dynamic shared library.

While doing such major reworks we also took the opportunity to rename applications and some modules, hopefully making things a little bit clearer, at least for newcomers. The changes breaks backward compatibility, calling for the update of major version number.

**Controller details:**
- Controller catalogs supported
- The following controllers, in addition to default controller, have been implemented:
  - **ExternalController**
      Vehicle will not be updated by the scenario. It's state is expected to be reported via gateway.
      The optional Ghost feature will launch a fore-runner vehicle to perform the scenario and create a trajectory for the external driver model to follow.
  - **FollowGhostController**
      An example of a (simple) vehicle and driver model using a Ghost as reference.
  - **InteractiveController**
      A simple vehicle model controlled by the user via keyboard (arrow keys)
  - **SloppyDriverController**
      Another example of a driver model, adding some random speed and lateral deviation to the default road following behavior. This controller is useless, it's pure purpose is to provide an example of separating lontitudinal and lateral control.
  - **SumoController**
      A way to integrate SUMO simulation. OpenSCENARIO vehicles are reported to SUMO, and SUMO vehicles are reported back to esmini.

These controllers can now be utilized also via the shared library. As OpenSCENARIO actions they are activated/deactivated dynamically by means of OSC triggers.

For more information about esmini controllers please see [Controllers.md](https://github.com/esmini/esmini/blob/master/docs/Controllers.md)

**Other new featuers:**
- VisibilityAction - makes object visible or invisible for sensors and/or graphics (visual presentation)

**Structural changes:**
  - Special control modes External, Internal and Hybrid have been removed (replaced by controllers)
  - ScenarioEngineDLL renamed to esminiLib
  - RoadManagerDLL renamed to esminiRMLib
  - EnvironmentSimulator removed (with controllers this got identical with the EgoSimulator)
  - EgoSimulator renamed to esmini (since it now is the recommended application)
  - ScenarioViewer renamed to esmini-dyn (kept as an example of using esminiLib)
  - OpenDriveViewer renamed to odrviewer
  - Replayer renamed to replayer
  - osi_receiver renamed to osireceiver

Note:
  - Previous use of control modes, e.g. external for interactive driving, now has to be specified in terms of controllers in the OpenSCENARIO file. Two steps needed: 1. AssignController and 2. ActivateController. See sloppy-driver.xosc for an example.
  - These changes affects some headerfile names, which might need to be updated in custom code.
  - Scripts referring to executables need updates, e.g. change any "EgoSimulator" or "EnvironmentSimulator" to "esmini" (however EgoSimulator will be provided as a raw copy of esmini executable during a deprecation period of a few releases)

### 2020-10-09 Version 1.7.13
- Added support for lane change dynamics by rate

### 2020-10-06 Version 1.7.12
- Fix pedestrian catalog support

### 2020-10-02 Version 1.7.11
- Fix bug deleting any entity with defined controller
- Add another OpenDriveViewer example run script

### 2020-10-02 Version 1.7.10
- Fix assumption that all controllers are SUMO config type
- Fix parameter name bug
- Add OpenDriveViewer to demo package
- Some additional minor bug fixes

#### 2020-10-01 Version 1.7.9
- Improved OSI performance
- Added pedestrian example scenario

#### 2020-09-30 Version 1.7.8
- Support correct parameter names excluding the "$" prefix (old way still supported as well)
- TimeToCollision condition
- Collision condition
- Improved road and lane connectivity w.r.t. preserve direction
- Additional bugfixes and improvements

#### 2020-09-23 Version 1.7.7
- OSI raw struct output option
- OSI sensor view in local coordinates
- Callback mechanism to override (part of) entity states
- Improved WorldCoordinate(x, y, z) to RoadCoordinate(road, lane, offset) mapping
- Many minor bugfixes and improvements (and probably a few new bugs)

#### 2020-09-04 Version 1.7.6
- Added trajectory clothoid support

2020-08-31 Version 1.7.5
- Added Hello World coding example
- Fixed a bug preventing shared library (ScenarioEngineDLL) to run with viewer on Mac
- Fixed crash when running with only OpenDRIVE road description (i.e. without scenegraph 3D model).

#### 2020-08-26 Version 1.7.2
- CSV logging feature
- EndOfRoad trigger
- Unit test framework based on Google Test
- Cleaned irrelevant error messages

#### 2020-08-21 Version 1.7.0
- SUMO support integrated (via libsumostatic), first limited shot.
  - SUMO vehicles created by means of object controller. See example cut-in_sumo.xosc.
  - NOTE that you need to re-run cmake script in order to fetch SUMO dependency package (including headers and pre-built libraries).
- Further OSI values populated, e.g. velocity and acceleration
- OSI trace file not created by default, activated with argument "--osi_file on"
- Condition/trigger timer now based on simulation time instead of system time

#### 2020-07-24 Version 1.6.3
- OSI support extended with road information (lane and road marks)
- Bugfix: RelativeTargetLane (used in LaneChangeAction) is now calculated correctly, skipping reference lane and considering vehicle orientation so that positive lane changes will go left and negative to the right. Scenarios making use of RelativeTargetLane might need to be updated accordingly.

#### 2020-06-18 Version 1.6.0
- OSI support, initial framework established.
- So far population of OSI global groundtruth moving objects.
- OSI data is populated and provided to user in three ways:
    1. OSI trace file (always created, in folder where application was started)
	2. UDP messages (option to send OSI messages to specified host/IP address)
	3. API to fetch OSI data via function call (in ScenarioEngineDLL)
- osi_receiver is a minimalistic demo-application showing how to receive OSI over UDP

#### 2020-05-19 Version 1.5.0
- Updated to support OpenSCENARIO 1.0. Note: no legacy support for 0.9.1
- Updated demo scenarios to v1.0
- Improved condition handling, fully supporting AND (within ConditionGroup) and OR (Multiple ConditionGroups) combinations
- Refactored runtime model supporting StoryBoardElement states, also obeying nrOfExecutions attribute
- Initial Trajectory support. PolyLine only, so far and limited testing performed.
- Added support for TraveledDistance condition

#### 2020-05-05 Version 1.4.6
- Added heading to sensors so that one entity can have multiple sensors in different directions
- Improved position Delta functionality. It will calculate shortest path and distance between two positions more generically and correctly.
- Added road model (multi_intersections.xodr/osgb) with multiple junctions. Useful for testing the shortest path functionality.
- Added new camera mode: top view (press 'k' multiple times to toggle or specify "--camera_mode top" as command line argument).
- Some bugfixes. E.g. ParamPoly3 with Normalized parameter range.

#### 2020-03-25 Version 1.4
- Demos (and binaries) supplied for Mac (Catalina) and Linux (Ubuntu 18.04 and Kubuntu 18.04) in addition to Windows.
- jpeg screenshot. User can save a screen shot at any time by pressing key 'c'.
- QuitAction. User can specify when to quit the scenario (and application) by means of OSC conditions, just as any OSC action.
- Fixed timestep (--fixed_timestep <sec>). Enabling scenarios to be executed in split of a second and, by using the recording feature, it can be replayed in viewer afterwards.
- Threads (--threads). Put scenario execution into a separate thread, decoupled from the viewer. Example of potential use case: Make it possible to pause scenario while moving camera.
- New keyboard shortcut commands, and some moved to new key. Please see run/readme.txt for complete set.

#### 2020-03-08 Version 1.3
- Anti-Alias filter control
  - EgoSimulator now takes argument aa_mode <number of sub samplings>
      0 means no Anti-Alias. 4 is default.
  - One use case is when running esmini (EgoSimulator) within docker on Linux - which seem to not support AntiAlias.
- Mac CI build environment added. However only RoadManager and ScenarioEngine as shared libraries, and without graphics (OSG) support.
- Linux binaries now includes graphics (OSG) support.
- Demo package for Linux added. Now CI builds and deploy demo for both Windows and Linux (Ubuntu 18.04).

#### 2019-12-06 Version 1.2
- Catalog handling updates
  - Parameter assignment implemented (enabling variants or configuration of re-used catalog items)
  - Name and structure updates:
      When a scenario refers to a catalog entry esmini will first locate the catalog by searching all specified catalog directories for a file named "catalog name".xosc. So a catalog name is defined by the base file name. Filename extension .xosc is mandatory. The actual Catalog XML element attribute "name" is ignored.
      See synchronize.xosc for an example on how to (re-)use manuever catalog entries.
- Trail visualization can be switched off by application argument (--trail <on/off>) and toggled by pressing key "t" in viewer window.

#### 2019-11-20 Version 1.1
- External control flag replaced by enumeration
    Available modes: internal, external, hybrid
    EgoSimulator and EnvironmentSimulator argument syntax changed from --ext_control <on|off> to --control <internal|external|hybrid>
