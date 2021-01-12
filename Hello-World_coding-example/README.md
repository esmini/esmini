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
.\esmini-player.exe --window 50 50 1000 500 --osc ..\resources\xosc\pedestrian.xosc --trails
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
		SE_Step();
		SE_ReportObjectPos(0, 0.0f, 8.0f, (float)i, 0.0f, 1.57f, 0.0f, 0.0f, 15.0f);
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
			}
		}
	}

	return 0;
}
```
Note: If you want M_PI, add on top (before includes): #define _USE_MATH_DEFINES

### Driver model

Using a simple vehicle model this example demonstrates how a driver model can interact with the scenario, once again using the ```ExternalController```. 

Before heading into the application code we will prepare a scenario. Make a copy of ```slow-lead-vehicle.xosc``` and name it ```test-driver.xosc```.

Do the following steps:

- Open test-driver.xosc for edit. 
- Assign controller to the Ego vehicle. Put the following private action code in the Init section, under Ego
	```
	<PrivateAction>
	    <ControllerAction>
	        <AssignControllerAction>
	            <Controller name="MyController">
	                <Properties>
	                    <Property name="esminiController" value="ExternalController" />
	                </Properties>
	            </Controller>
	        </AssignControllerAction>
	    </ControllerAction>
	</PrivateAction>
	```
- And activate the controller on both lateral and longitudinal domains. Put the following private action right after the above.
	```
    <PrivateAction>
         <ActivateControllerAction longitudinal="true" lateral="true" />
    </PrivateAction>
	```

 - For a more interesting road, change into curves_elevation:
	```
	<RoadNetwork>
	   <LogicFile filepath="../xodr/curves_elevation.xodr"/>
	   <SceneGraphFile filepath="../models/curves_elevation.osgb"/>
	</RoadNetwork>
	```

Now let's head into the code example. 

```C++
#include "stdio.h"
#include "math.h"
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	void* vehicleHandle = 0;
	SE_SimpleVehicleState vehicleState = {0, 0, 0, 0, 0, 0};
	SE_ScenarioObjectState objectState;
	SE_RoadInfo roadInfo;
	float simTime = 0;
	float dt = 0;

	if (SE_Init("../resources/xosc/test-driver.xosc", 0, 1, 0, 0) != 0)
	{
		printf("Failed to initialize the scenario, quit\n");
		return -1;
	}

	// Initialize the vehicle model, fetch initial state from the scenario
	SE_GetObjectState(0, &objectState);
	vehicleHandle = SE_SimpleVehicleCreate(objectState.x, objectState.y, objectState.h, 4.0);

	// show some road features, including road sensor 
	SE_ViewerShowFeature(4, true);

	// Run for 40 seconds or until 'Esc' button is pressed
	while (SE_GetSimulationTime() < 40 && !(SE_GetQuitFlag() == 1))
	{
		SE_Step();
		dt = SE_GetSimulationTime() - simTime;
		simTime = SE_GetSimulationTime();
		
		// Get road information at a point some speed dependent distance ahead
		SE_GetRoadInfoAtDistance(0, 5 + 0.5f * vehicleState.speed, &roadInfo, 0);

		// Steer towards where the point 
		double steerAngle = roadInfo.angle;

		// Accelerate until target speed 25 is reached
		double throttle = vehicleState.speed < 25 ? 1.0 : 0.0;
		
		// Slow down in curves
		throttle /= (1 + 20 * fabs(steerAngle));

		// Step vehicle model with driver input
		SE_SimpleVehicleControlAnalog(vehicleHandle, dt, throttle, steerAngle);

		// Fetch updated state and report to scenario engine
		SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);

		SE_ReportObjectPos(0, simTime, vehicleState.x, vehicleState.y, vehicleState.z,
			vehicleState.h, vehicleState.p, 0, vehicleState.speed);
	}

	return 0;
}
```
Don't worry about the slow vehicle, you'll just drive through it. 

**Challenge**: Attach a front looking sensor to detect it and have the driver to brake to avoid collision...

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
 
##### Advanced use case using callback

Controllers and callbacks can also be utilized via Python. Following code drafts how to grab state of the first scenario object and then, in a callback, manipulate it and report back. If used in combination with ExternalController in mode=additive the scenario action are applied first, while if used with mode=override the scenario actions will not be applied, giving exclusive control to the external callback function.
```
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

