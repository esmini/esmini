## esmini release notes

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
