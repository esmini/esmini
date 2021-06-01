## esmini Hello World Tutorial
The simplest preparation is to get the demo package from [the latest relase](https://github.com/esmini/esmini/releases/latest) and then head on to the steps below.

But, of course, if you already have checked out the esmini project from GitHub and compiled it you're basically set. Just copy the runtime shared library to the Hello-World_coding_example folder. Exact filename depends on the platform as follows: 
* Windows: esminiLib.dll
* Linux: libesminiLib.so
* Mac: libesminiLib.dylib

### steps to build the "Hello World" esmini-player application
1. Make sure you have built esminiLib (you should have library files in the esmini/bin folder).
1. Navigate to the folder Hello-World_coding-example.
1. From a command prompt run the following commands to create build scripts and build the provided code example:
	```
	mkdir build
	cd build
	cmake ..
	cmake --build . --config Release --target install
	```
If all works as expected there should be an executable "esmini-player" in the Hello-World_coding-example folder. Try to run it from that directory. 
On Linux you might need to define LD_LIBRARY PATH first, like this: ```export LD_LIBRARY_PATH=.```

If you have any IDE installed, e.g. Visual Studio (Win) or Xcode (Mac), cmake might have produced project files for it, e.g. "build/esmini-player.sln".

> **Note:** On Windows there are two library files: esminiLib.lib and esminiLib.dll. The .lib file (basically only table of contents of the library) is used for compile time linking while the .dll (actual code) for runtime dynamic linking. On other platforms there is only one library file, .so (Shared Object) in Linux and .dylib (Dynamic Library) on Mac, which is used for both compile time and runtime linking.

Here follows a few code examples to try out, e.g. by modifying main.cpp:

### Hello world - load and play a scenario
```C++
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	SE_Init("../resources/xosc/cut-in.xosc", 0, 1, 0, 0);

	for (int i = 0; i < 500; i++)
	{
		SE_Step();
	}

	return 0;
}
```
Exercise: Change scenario to pedestrian.xosc, then compile and run the program again.

### Add optional argument to load any scenario
```C++
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	if (argc > 1)
	{
		SE_InitWithArgs(argc, argv);
	}
	else
	{
		SE_Init("../resources/xosc/cut-in.xosc", 0, 1, 0, 0);
	}

	for (int i = 0; i < 500; i++)
	{
		SE_Step();
	}

	return 0;
}
```
You can now specify esmini arguments according to [esmini launch commands](https://github.com/esmini/esmini/blob/master/docs/commands.txt).

Example:
```
.\esmini-player.exe --window 60 60 1000 500 --osc ..\resources\xosc\pedestrian.xosc --trails
```
### Fetch state of scenario objects
```C++
#include "stdio.h"
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	SE_Init("../resources/xosc/cut-in.xosc", 0, 1, 0, 0);

	for (int i = 0; i < 500; i++)
	{
		SE_Step();

		for (int j = 0; j < SE_GetNumberOfObjects(); j++)
		{
			SE_ScenarioObjectState state;

			SE_GetObjectState(j, &state);
			printf("time [%.2f] object[%d] pos[%.2f, %.2f] \n", state.timestamp, j, state.x, state.y);
		}
	}

	return 0;
}
```

### External control of Ego
A silly example showing how you can just take control over vehicle state via the API. The Ego car will move one meter along the Y-axis for each frame...

Now we will also introduce the quit_flag, which lets you quit by pressing 'Esc' key.
```C++
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	const char* filename = argc > 1 ? argv[1] : "../resources/xosc/cut-in_external.xosc";

	SE_Init(filename, 0, 1, 0, 0);

	for (int i = 0; i < 500 && !(SE_GetQuitFlag() == 1); i++)
	{
		SE_ReportObjectPos(0, 0.0f, 8.0f, (float)i, 0.0f, 1.57f, 0.0f, 0.0f, 15.0f);
		SE_Step();
	}

	return 0;
}

```
Exercise: Change heading with i, e.g: 

```SE_ReportObjectPos(0, 0.0f, 8.0f, (float)i, 0.0f, 1.57f + 0.01f*i, 0.0f, 0.0f, 15.0f);```

Yes, it looks crazy! But it demonstrates how an application totally can take control of a vehicle.

### Control controllers
Try to run cut-in_interactive.xosc, as below.
```C++
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	SE_Init("../resources/xosc/cut-in_interactive.xosc", 0, 1, 0, 0);

	for (int i = 0; i < 2000 && !(SE_GetQuitFlag() == 1); i++)
	{
		SE_Step();
	}

	return 0;
}
```
Control the Ego vehicle with arrow keys. 

To disable controllers and hand over to default scenario behavior set first argument flag (see [headerfile esminiLib.hpp](https://github.com/esmini/esmini/blob/7cf4b6307a203b2f52481d07fb09347ebf5517eb/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L94)):
```C++
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	SE_Init("../resources/xosc/cut-in_interactive.xosc", 1, 1, 0, 0);

	for (int i = 0; i < 2000 && !(SE_GetQuitFlag() == 1); i++)
	{
		SE_Step();
	}

	return 0;
}
```

### Ideal sensors
```C++
#include "stdio.h"
#include "esminiLib.hpp"

#define MAX_HITS 10

int main(int argc, char* argv[])
{
	SE_Init("../resources/xosc/cut-in.xosc", 0, 1, 0, 0);

	SE_AddObjectSensor(0, 2.0, 1.0, 0.5, 1.57, 1.0, 50.0, 1.57, MAX_HITS);
	SE_AddObjectSensor(0, -1.0, 0.0, 0.5, 3.14, 0.5, 20.0, 1.57, MAX_HITS);

	for (int i = 0; i < 2000 && !(SE_GetQuitFlag() == 1); i++)
	{
		SE_Step();

		int objList[MAX_HITS];
		for (int j = 0; j < 2; j++)  // iterate over added sensors
		{
			int nHits = SE_FetchSensorObjectList(j, objList);
			for (int k = 0; k < nHits; k++)
			{
				printf("Sensor[%d] detected obj: %d\n", j, objList[k]);

				// Get some info of that detected object
				SE_ScenarioObjectState state;

				SE_GetObjectState(objList[k], &state);
				printf("object[%d] pos: (%.2f, %.2f) heading: %.2f\n", objList[k], state.x, state.y, state.h);
			}
		}
	}

	return 0;
}
```
Note: If you want M_PI, add on top (before includes): #define _USE_MATH_DEFINES

### Driver model

Using a simple vehicle model this example demonstrates how a driver model can interact with the scenario, once again using the ```ExternalController```. 

Before heading into the application code we will prepare a scenario. Download [test-driver.xosc](https://www.dropbox.com/s/h9uqj2la4sk2t2o/test-driver.xosc?dl=1) and put it in esmini/resources/xosc folder.

Now let's have a look inside it to see how to activate the ExternalController, which will prevent the DefaultController to interfere with the Ego vehicle and instead hand over exclusive control to our application. You can skip this and go to the C++ code example below if you're not interested in the controller setup.
- Open test-driver.xosc 
- Look in the Entities section, under the \<ScenarioObject name="Ego"\> element. Here the controller is defined and assigned to the Ego vehicle. 
	```
    <ObjectController>
        <Controller name="MyExternalControllerWithGhost">
            <Properties>
        	    <Property name="esminiController" value="ExternalController" />
                <Property name="useGhost" value="false" />
                <Property name="headstartTime" value="2" />
            </Properties>
        </Controller>
    </ObjectController>   
	```
- Then the initial position is set. This could instead be done by the application, but it's convenient to specify it in the scenario file.
	```
   <PrivateAction>
        <TeleportAction>
            <Position>
                <LanePosition roadId="1" laneId="-1" offset="0" s="50"/>
            </Position>
        </TeleportAction>
   </PrivateAction>
	```
- And finally we need to activate the controller on both lateral and longitudinal domains. 
	```
    <PrivateAction>
         <ActivateControllerAction longitudinal="true" lateral="true" />
    </PrivateAction>
	```
    The reason for putting it AFTER the positioning is that oterwise the position action would have no effect, since once the controller is activated all scenario actions will be ignored. The controller then having exclusive control.
 
Now let's head into the code example. Copy it into your main.cpp module. Compile and run it. Then study the code and perhaps play around mainpulating some values.

```C++
#include "stdio.h"
#include "math.h"
#include "esminiLib.hpp"

#define TARGET_SPEED 50.0
#define CURVE_WEIGHT 30.0
#define THROTTLE_WEIGHT 0.01
#define GHOST 0

int main(int argc, char* argv[])
{
	void* vehicleHandle = 0;
	SE_SimpleVehicleState vehicleState = { 0, 0, 0, 0, 0, 0 };
	SE_ScenarioObjectState objectState;
	SE_RoadInfo roadInfo;
	float simTime = 0;
	float dt = 0;

	if (SE_Init("../resources/xosc/test-driver.xosc", 0, 1, 0, 0) != 0)
	{
		printf("Failed to initialize the scenario, quit\n");
		return -1;
	}

	// Lock object to the original lane
	// If setting to false, the object road position will snap to closest lane
	SE_SetLockOnLane(0, true);

	// Initialize the vehicle model, fetch initial state from the scenario
	SE_GetObjectState(0, &objectState);
	vehicleHandle = SE_SimpleVehicleCreate(objectState.x, objectState.y, objectState.h, 4.0);

	// show some road features, including road sensor 
	SE_ViewerShowFeature(4, true);

	// Run for 60 seconds or until 'Esc' button is pressed
	while (SE_GetSimulationTime() < 60 && !(SE_GetQuitFlag() == 1))
	{
		// Get simulation delta time since last call (first will be 0)
		dt = SE_GetSimTimeStep();

		// Get road information at a point some speed dependent distance ahead
#if !GHOST
		// Look ahead along the road, to establish target info for the driver model
		SE_GetRoadInfoAtDistance(0, 5 + 0.75f * vehicleState.speed, &roadInfo, 0, true);

		// Slow down when curve ahead - CURVE_WEIGHT is the tuning parameter
		double targetSpeed = TARGET_SPEED / (1 + CURVE_WEIGHT * fabs(roadInfo.angle));
#else
		// ghost version
		float ghost_speed;
		SE_GetRoadInfoAlongGhostTrail(0, 5 + 0.75f * vehicleState.speed, &roadInfo, &ghost_speed);
		double targetSpeed = ghost_speed;
#endif

		// Steer towards where the point 
		double steerAngle = roadInfo.angle;

		// Accelerate or decelerate towards target speed - THROTTLE_WEIGHT tunes magnitude
		double throttle = THROTTLE_WEIGHT * (targetSpeed - vehicleState.speed);

		// Step vehicle model with driver input, but wait until time > 0
		if (SE_GetSimulationTime() > 0)
		{
			SE_SimpleVehicleControlAnalog(vehicleHandle, dt, throttle, steerAngle);
		}

		// Fetch updated state and report to scenario engine
		SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);

		// Report updated vehicle position and heading. z, pitch and roll will be aligned to the road
		SE_ReportObjectPosXYH(0, simTime, vehicleState.x, vehicleState.y, vehicleState.h, vehicleState.speed);

		// Finally, update scenario using same time step as for vehicle model
		SE_StepDT(dt);
	}

	return 0;
}

```
Don't worry about the stationary red car on the road, you'll just drive through it. 

Try experimenting with the driver settings, e.g. increase lookahead distance from 0.75 to 1.75.

**Challenge**: Attach a front looking sensor to detect it and have the driver to brake to avoid collision...

#### Ghost concept
The driver model so far is just capable of driving along the specified lane. It's totally detached from any scenario events. 

There is a solution if you want a driver model to perform scenario actions: The ghost concept. Basically it will launch a fore-runner to perform the scenario actions. The trajectory is registered, including speed, and can then be used as reference for steering and speed target.

To test this you need to make two changes to the previous example:
1. In main.cpp, change line:  
```#define GHOST 0``` to  
```#define GHOST 1```
2. In test-driver.xosc, change line:  
```<Property name="useGhost" value="false" />``` to   
```<Property name="useGhost" value="true" />```

When running the application, press key 'j' to show dots along Ego and Ghost trails.

### OSI groundtruth
For accessing OSI data we first need to complement the Hello-World_coding-example/CMakeLists.cxx with OSI include and library info. Make following modifications:

```
include_directories(. ../include ../EnvironmentSimulator/Libraries/esminiLib ../externals/OSI/v10/include)
```
```
link_directories(. ../lib ../bin ../externals/OSI/v10/lib)
```
```
target_link_libraries(${TARGET} esminiLib libprotobuf open_simulation_interface_pic) 
```
**Note:** 
 - Replace foldername "v10" with linux or mac depending on your platform.
 - If linking with custom applications or libraries: OSI and Google Protobuf versions needs to be consistent (at least major version nr), also with the versions used in esmini (see [here](https://github.com/esmini/esmini#osi-support)).

Then run ``` cmake .. ``` from the build folder to apply the changes in CMakeFiles.cxx.

The following code (put in main.cpp) will update, fetch and print some OSI data each frame.

```C++
#include "esminiLib.hpp"

#include "osi_common.pb.h"
#include "osi_object.pb.h"
#include "osi_groundtruth.pb.h"
#include "osi_version.pb.h"

int main(int argc, char* argv[])
{
	SE_Init("../resources/xosc/cut-in.xosc", 0, 1, 0, 0);

	osi3::GroundTruth* gt;

	// Initial update of complete Ground Truth, including static things
	SE_UpdateOSIGroundTruth();
	// You could now retrieve the initial state of all objects before stepping the scenario

	for (int i = 0; i < 100; i++)
	{
		SE_StepDT(0.01f);

		// Further updates will only affect dynamic OSI stuff
		SE_UpdateOSIGroundTruth();

		// Fetch OSI struct
		gt = (osi3::GroundTruth*)SE_GetOSIGroundTruthRaw();

		// Print timestamp
		printf("Frame %d timestamp: %.2f\n", i, gt->mutable_timestamp()->seconds() +
			1E-9 * gt->mutable_timestamp()->nanos());

		// Print object id, position, orientation and velocity
		for (int j = 0; j < gt->mutable_moving_object()->size(); j++)
		{
			printf(" obj id %lld pos (%.2f, %.2f, %.2f) orientation (%.2f, %.2f, %.2f) velocity (%.2f, %.2f, %.2f) \n",
				gt->mutable_moving_object(j)->mutable_id()->value(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_position()->x(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_position()->y(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_position()->z(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_orientation()->yaw(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_orientation()->pitch(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_orientation()->roll(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_velocity()->x(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_velocity()->y(),
				gt->mutable_moving_object(j)->mutable_base()->mutable_velocity()->z()
			);
		}

		printf("moving objects in GT %d\n", gt->moving_object_size());
		printf("road markings in GT %d\n", gt->road_marking_size());
	}
	return 0;
}
```

### Python binding

A Python wrapper for esmini can easily be created using "ctypes" (thanks David Kaplan for the tip!). Run the following script in a folder where the ScenarioEngineDLL library is present:
```Python
import ctypes

se = ctypes.CDLL("./esminiLib.dll")
se.SE_Init(b"../resources/xosc/cut-in.xosc", 1, 1, 0, 0, 2)
 
for i in range (500):
    se.SE_Step()
```

**Note:** Library name varies with the platform:
* Windows: esminiLib.dll
* Linux: libesminiLib.so
* Mac: libesminiLib.dylib

#### Example get object states
Extend previous example with:
1. Choose lib based on platform (making the script portable)
2. Retrieve the state of all scenario objects

```Python
import ctypes
from sys import platform

if platform == "linux" or platform == "linux2":
    se = ctypes.CDLL("./libesminiLib.so")
elif platform == "darwin":
    se = ctypes.CDLL("./libesminiLib.dylib")
elif platform == "win32":
    se = ctypes.CDLL("./esminiLib.dll")
else:
    print("Unsupported platform: {}".format(platform))
    quit()

# Definition of SE_ScenarioObjectState struct
class SEScenarioObjectState(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_int),
        ("model_id", ctypes.c_int),
        ("control", ctypes.c_int),
        ("timestamp", ctypes.c_float),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("h", ctypes.c_float),
        ("p", ctypes.c_float),
        ("r", ctypes.c_float),
        ("roadId", ctypes.c_int),
        ("t", ctypes.c_float),
        ("laneId", ctypes.c_int),
        ("laneOffset", ctypes.c_float),
        ("s", ctypes.c_float),
        ("speed", ctypes.c_float),
        ("centerOffsetX", ctypes.c_float),
        ("centerOffsetY", ctypes.c_float),
        ("centerOffsetZ", ctypes.c_float),
        ("width", ctypes.c_float),
        ("length", ctypes.c_float),
        ("height", ctypes.c_float),
    ]

se.SE_Init(b"../resources/xosc/cut-in.xosc", 0, 1, 0, 0)

obj_state = SEScenarioObjectState()  # object that will be passed and filled in with object state info
 
for i in range(500):
    for j in range(se.SE_GetNumberOfObjects()):
        se.SE_GetObjectState(j, ctypes.byref(obj_state))
        print('Frame {} Time {:.2f} ObjId {} roadId {} laneId {} laneOffset {:.2f} s {:.2f} x {:.2f} y {:.2f} heading {:.2f}'.format(
            i, obj_state.timestamp, j, obj_state.id, obj_state.roadId, obj_state.laneId, obj_state.laneOffset, 
            obj_state.s, obj_state.x, obj_state.y, obj_state.h))
    se.SE_Step()

```

#### Example use of esminiRMLib (RoadManager)

Just make sure the esminiRMLib is present in the same folder as the script:

```Python
import ctypes as ct
from sys import platform

if platform == "linux" or platform == "linux2":
    rm = ct.CDLL("./libesminiRMLib.so")
elif platform == "darwin":
    rm = ct.CDLL("./libesminiRMLib.dylib")
elif platform == "win32":
    rm = ct.CDLL("./esminiRMLib.dll")
else:
    print("Unsupported platform: {}".format(platform))
    quit()

# Definition of RM_PositionData struct
class RM_PositionData(ct.Structure):
    _fields_ = [
        ("x", ct.c_float),
        ("y", ct.c_float),
        ("z", ct.c_float),
        ("h", ct.c_float),
        ("p", ct.c_float),
        ("r", ct.c_float),
        ("h_relative", ct.c_float),
        ("road_id", ct.c_int),
        ("lane_id", ct.c_int),
        ("lane_offset", ct.c_float),
        ("s", ct.c_float),
    ]

# Specify argument types to a few functions
rm.RM_SetWorldPosition.argtypes = [ct.c_int, ct.c_float, ct.c_float, ct.c_float, ct.c_float, ct.c_float, ct.c_float]
rm.RM_SetLanePosition.argtypes = [ct.c_int, ct.c_int, ct.c_int, ct.c_float, ct.c_float]

## Initialize emsini RoadManger with given OpenDRIVE file
rm.RM_Init(b'../resources/xodr/straight_500m.xodr')

rm_pos = rm.RM_CreatePosition()  # create a position object, returns a handle
rm_pos_data = RM_PositionData()  # object that will be passed and filled in with position info

# test a few positions
x = 65
y = -1.7
rm.RM_SetWorldPosition(rm_pos, x, y, 0.0, 0.0, 0.0, 0.0)
rm.RM_GetPositionData(rm_pos, ct.byref(rm_pos_data))
print('road_id {} lane_id {} lane_offset {:.2f} s {:.2f}'.format(
    rm_pos_data.road_id, rm_pos_data.lane_id, rm_pos_data.lane_offset, rm_pos_data.s))

road_id = 1
lane_id = 1
lane_offset = -0.2
s = 40
rm.RM_SetLanePosition(rm_pos, road_id, lane_id, lane_offset, s)
rm.RM_GetPositionData(rm_pos, ct.byref(rm_pos_data))
print('x {:.2f} y {:.2f} h (yaw) {:.2f}'.format(
    rm_pos_data.x, rm_pos_data.y, rm_pos_data.h))

```

**Note:** Library name varies with the platform:
* Windows: esminiLib.dll
* Linux: libesminiLib.so
* Mac: libesminiLib.dylib
 
#### Advanced use case using callback

Controllers and callbacks can also be utilized via Python. Following code drafts how to grab state of the first scenario object and then, in a callback, manipulate it and report back. If used in combination with ExternalController in mode=additive the scenario action are applied first, while if used with mode=override the scenario actions will not be applied, giving exclusive control to the external callback function.
```Python
# Definition of SE_ScenarioObjectState struct
class SEScenarioObjectState(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_int),
        ("model_id", ctypes.c_int),
        ("control", ctypes.c_int),
        ("timestamp", ctypes.c_float),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("h", ctypes.c_float),
        ("p", ctypes.c_float),
        ("r", ctypes.c_float),
        ("roadId", ctypes.c_int),
        ("t", ctypes.c_float),
        ("laneId", ctypes.c_int),
        ("laneOffset", ctypes.c_float),
        ("s", ctypes.c_float),
        ("speed", ctypes.c_float),
        ("centerOffsetX", ctypes.c_float),
        ("centerOffsetY", ctypes.c_float),
        ("centerOffsetZ", ctypes.c_float),
        ("width", ctypes.c_float),
        ("length", ctypes.c_float),
        ("height", ctypes.c_float),
    ]


# Define callback for scenario object enabling manipulating the state AFTER scenario step but BEFORE OSI output
# Use in combination with ExternalController in mode=additive in order for scenario actions to be applied first
def callback(state_ptr, b):
    state = state_ptr.contents
    print("callback for obj {}: x={} y={}".format(state.id, state.x, state.y))
    se.SE_ReportObjectPos(ctypes.c_int(state.id), ctypes.c_float(state.timestamp),
                          # position
                          ctypes.c_float(state.x + 1.0), ctypes.c_float(state.y + 20.0), ctypes.c_float(0.0),
                          # rotation
                          ctypes.c_float(state.h + 0.5), ctypes.c_float(state.p), ctypes.c_float(state.r),
                          # speed
                          ctypes.c_float(state.speed))



callback_type = ctypes.CFUNCTYPE(None, ctypes.POINTER(SEScenarioObjectState), ctypes.c_void_p)
callback_func = callback_type(callback)

# Intitialize esmini before register the callback
se.SE_Init(b"xosc/my_scenario.xosc", 0, 1, 0, 1)

# register callback for first object (id=0)
se.SE_RegisterObjectCallback(0, callback_func, 0)
```

