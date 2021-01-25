# Inner Workings of esmini

## Main modules
Below follows a brief introduction to the main code modules of esmini corresponding to the folders in [`esmini/EnvironmentSimulator/Modules`](https://github.com/esmini/esmini/tree/master/EnvironmentSimulator/Modules) file tree path.

[Here](https://viewer.diagrams.net/?highlight=0000ff&layers=1&nav=1&title=esmini_class_diagram.xml#Uhttps%3A%2F%2Fdrive.google.com%2Fuc%3Fid%3D1z5TM6o-RryOGl1l-lRgiiv9ZMcrw0jBZ%26export%3Ddownload) is a high level (still quite complex) class diagram. Please note that it's not automatically generated, so there's probably some errors and stuff missing. Still it can provide some overview...

### RoadManager
Implementation of [OpenDRIVE](https://www.asam.net/standards/detail/opendrive/) data model and interface. Coverage is not complete at all. The goal is to provide needed and most useful features for esmini. Functionality grows continuously. While being based on 1.4 version it will probably support newer features when needed, so we avoid tagging a specific version.

### ScenarioEngine

This is where the dynamics of the scenario takes place. It parses the [OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/) file, creates a data model of the entities, triggers and actions. And finally steps the scenario, evaluating the triggers and executing the actions accordingly.

It also includes the ScenarioGateway. The purpose of this module is to have a central ground truth representation allowing for asynchronous reporting of entity states. The main driving use case was an external simulator reporting the state of the Ego vehicle (or Vehicle/System Under Test) to esmini via UDP. A separate receiver thread can report latest values to the gateway at any time. See [Server module hpp](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/Server.hpp) / [cpp](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/Server.cpp) for an example.

(The idea was further to use timestamps of each reported states allowing for extrapolation of positions and rotations to synchronize states in time, e.g. when rendering next image or providing data for a radar model. However this functionality has not been implemented yet, since it has not been required by any user.)

ScenarioEngine also takes care of reporting ground truth on OSI standard protocol in various ways:
1. API functions to fetch the OSI structures 
2. Saving to OSI trace file 
3. Sending OSI data over IP/UDP to an external host.

Another feature is ideal sensors, defined by a view frustum positioned anywhere on an entity. It will detect any other entity being inside the view frustum. Please note current limitation that only entity reference point is considered, so an entity might be partly inside the frustum and still not detected. 


### Controllers
Implementation of OpenSCENARIO  controller concept, which is a way of handing over control of individual entities such as vehicles and pedestrians to an external/custom function instead of the Default Controller.

The Default Controller is embedded in the ScenarioEngine (at least for now, but it would make sense to outsource that part to a separate controller module as well - future work).

The base class [`Controller`](https://github.com/esmini/esmini/blob/b996d69d9d84ec66745e6b701e8fc90ab75f998e/EnvironmentSimulator/Modules/Controllers/Controller.hpp#L30) defines the interface (API) for a controller and also implements some common functionality, such as assigning and activating the controller.

Then there is a collection of more or less useful controllers, with purpose varying from providing useful functionality in esmini to just show how to deal with the controller concept.

More information about the controller concept and the provided controllers, please see [Controllers.md](https://github.com/esmini/esmini/blob/master/docs/Controllers.md).


### ViewerBase

Provides a basic 3D viewer to preview scenarios. It's based on the excellent [OpenSceneGraph](http://www.openscenegraph.org/) open source graphics library. Let's say that it prioritizes stability, performance and portability over flashiness - fitting the purposes of esmini.

Briefly the module provides the following features:

- Optional 3D visual representation of the road network and surroundings. `.osgb` (OpenSceneGraph binary) file format is directly supported. But indirectly many file formats are supported by conversion using the [osgconv](http://www.openscenegraph.org/index.php/documentation/user-guides/55-osgconv) tool (NOTE: not included in esmini). Supported file formats are listed [here](http://www.openscenegraph.org/index.php/documentation/user-guides/61-osgplugins).
- 3D visual representation of scenario entities. Either using supplied 3D models or esmini will create stand-in dummy model, e.g. a 3D bounding box according to specified dimensions.
- Road feature visualization. E.g. OpenDRIVE geometries, lanes and road marks. These features are indicated by points and lines in a very simple way. So it does not replace the value of a 3D model, but at least it gives some guidance when 3D model is not available. It can also be useful for debugging issues in the OpenDRIVE road network definition.
- Keyboard input. First OSG provides a set of key shortcuts to control visual features, e.g. enable/disable textures or wireframe/shading mode. Then esmini adds a set of shortcuts to control various functionality such as toggling road features visualization or changing camera behavior. A complete list of keyboard shortcuts is found in [docs/readme.txt](https://github.com/esmini/esmini/blob/master/docs/readme.txt). The keyboard input is also routed to Controllers, which is useful for interactive driving modes.

### PlayerBase

This module ties together the ScenarioEngine with the Viewer also providing high a level API for initializing, stepping and controlling a scenario in a custom player application. 

The initial purpose of this module was to collect some common code from various early example applications.


### CommonMini
Collection of handy functions shared between modules and applications. E.g:
- Argument parser
- Timers
- Threads and Mutex
- Math operations
- Logger

## Parameters
esmini supports three parameter types: integer, double and string. A parameter is declared in the OpenSCENARIO file, in the global ParameterDeclaration. Values can be set by ParameterAction/ParameterSetAction. And it can be used in ParameterCondition to trig storyboard elements. For a simple example see use of "DummyParameter" in [lane_change.xosc](https://github.com/esmini/esmini/blob/master/resources/xosc/lane_change.xosc) and [esmini-dyn/main.cpp](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Applications/esmini-dyn/main.cpp) (enable DEMONSTRATE_PARAMETER to test).

To set and read values via esminiLib/C++ you can do as following examples:

### integer
```
int number = 99;
SE_Parameter param;
param.name = "MyIntParameter";
param.value = &number;
SE_SetParameter(param);
SE_GetParameter(&param);
printf("param value: %d\n", number);
// or by casting the value
printf("param value: %d\n", *((int*)param.value));
```
### double
```
double number = 1.5;
SE_Parameter param;
param.name = "MyDoubleParameter";
param.value = &number;
SE_SetParameter(param);
SE_GetParameter(&param);
printf("param value: %.2f\n", number);
// or by casting the value
printf("param value: %.2f\n", *((double*)param.value));
```
### string
```
std::string myString = "Hello";
SE_Parameter param;
param.name = "MyStringParameter";
param.value = &myString;
SE_SetParameter(param);
SE_GetParameter(&param);
printf("param value: %s\n", myString.c_str());
// or by casting the value
printf("param value: %s\n", (*((std::string*)param.value)).c_str());
```
## How the modules interact

On high level, the sequence of events when stepping the player is:

![diagram](https://github.com/esmini/esmini/blob/master/docs/esmini-frame-basic.png "frame sequence - detailed")

To understand the inner workings of esmini, both for using and developing purposes, let's zoom in one level:

![diagram](https://github.com/esmini/esmini/blob/master/docs/esmini-frame-detailed.png "frame sequence - detailed")
