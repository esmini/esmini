## esmini release notes

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
