## esmini Hello World Tutorial
The simplest preparation is to get the demo package from [the latest relase](https://github.com/esmini/esmini/releases/latest) and then head on to the steps below.

But, of course, if you already have checked out the esmini project from GitHub and compiled it you're basically set. Just copy the runtime shared library to the Hello-World_coding_example folder. Exact filename depends on the platform as follows: 
* Windows: esminiLib.dll
* Linux: libesminiLib.so
* Mac: libesminiLib.dylib

#### steps
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

#### Hello world - load and play a scenario
```C++
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	SE_Init("../resources/xosc/cut-in.xosc", 0, 1, 0, 0, 0.0f);

	for (int i = 0; i < 500; i++)
	{
		SE_Step();
	}

	return 0;
}
```
#### Add optional argument to load any scenario
```C++
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	const char* filename = argc > 1 ? argv[1] : "../resources/xosc/cut-in.xosc";

	SE_Init(filename, 0, 1, 0, 0, 0.0f);

	for (int i = 0; i < 500; i++)
	{
		SE_Step();
	}

	return 0;
}
```
#### Fetch state of scenario objects
```C++
#include "stdio.h"
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	SE_Init("../resources/xosc/cut-in.xosc", 1, 1, 0, 0, 0.0f);

	for (int i = 0; i < 500; i++)
	{
		SE_Step();

		for (int j = 0; j < SE_GetNumberOfObjects(); j++)
		{
			SE_ScenarioObjectState state;

			SE_GetObjectState(j, &state);
			printf("frame[%d] object[%d] pos[%.2f, %.2f] \n", i, j, state.x, state.y);
		}
	}

	return 0;
}

```
#### External control of Ego
```C++
#include "esminiLib.hpp"

int main(int argc, char* argv[])
{
	const char* filename = argc > 1 ? argv[1] : "../resources/xosc/cut-in.xosc";

	SE_Init(filename, 2, 1, 0, 0, 0);

	for (int i = 0; i < 500; i++)
	{
		SE_Step();
		SE_ReportObjectPos(0, 0.0f, 8.0f, (float)i, 0.0f, 1.57f, 0.0f, 0.0f, 15.0f);
	}

	return 0;
}
```
#### Ideal sensors
```C++
#include "stdio.h"
#include "esminiLib.hpp"

#define MAX_HITS 10

int main(int argc, char* argv[])
{
	const char* filename = argc > 1 ? argv[1] : "../resources/xosc/cut-in.xosc";

	SE_Init(filename, 0, 1, 0, 0, 0.0f);

	SE_AddObjectSensor(0, 2.0, 1.0, 0.5, 1.57, 1.0, 50.0, 1.57, MAX_HITS);
	SE_AddObjectSensor(0, -1.0, 0.0, 0.5, 3.14, 0.5, 20.0, 1.57, MAX_HITS);

	for (int i = 0; i < 500; i++)
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

#### Python binding

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