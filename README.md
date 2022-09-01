# Environment Simulator Minimalistic (esmini)

*esmini* is a basic OpenSCENARIO player

[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)
[![Build Status](https://img.shields.io/appveyor/ci/esmini/esmini/master.svg)](https://ci.appveyor.com/project/esmini/esmini)

![Screenshot](https://github.com/esmini/esmini/blob/master/resources/screenshot.jpg?raw=true "Screenshot")

It contains the following main libraries:

- RoadManager (esminiRMLib). A library providing an interface to road networks described in the OpenDRIVE format.
- ScenarioEngine (esminiLib). The main library providing a viewer and API interface to traffic scenarios described in the OpenSCENARIO format. This library includes RoadManager.

and a few applications that can be used as is or provide ideas for customized solutions:

- esmini. A scenario player application linking esmini modules statically.
- esmini-dyn. A minimalistic example using the esminiLib to play OpenSCENARIO files.
- odrplot. Produces a data file from OpenDRIVE for plotting the road network in Python.
- odrviewer. Visualize OpenDRIVE road network with populated dummy traffic.
- replayer. Re-play previously executed scenarios.
- osireceiver. A simple application receiving OSI messages from esmini over UDP.

Repository: <https://github.com/esmini/esmini>

Pre-built demo packages are available [here](https://github.com/esmini/esmini/releases/latest). Unzip, navigate to "esmini\run\esmini" and run any of the example scripts. See more info below under [Binaries and demos](#binaries-and-demos).

esmini supports OpenSCENARIO v1.1 (from esmini v2.7) and v1.0 (from esmini v1.5). In order to run older versions (i.e. v0.9.1) ASAM provides a transformation scheme (migration0_9_1to1_0.xslt, part of the OpenSCENARIO release bundle) that can be used with tools for automatic migration of XML files.

Please note that the OpenSCENARIO [coverage](https://github.com/esmini/esmini/blob/master/osc_coverage.txt) is limited, which means that not all features of OpenSCENARIO are supported. The functionalty grows slow but steady, based on need and contributions.

The code was initially a result from the Swedish collaborative research project [Simulation Scenarios](https://sites.google.com/view/simulationscenarios), and is now further developed based on users need and OpenSCENARIO development.

User guide: https://esmini.github.io

For more information about the esmini software parts, see [Inner Workings of esmini](https://github.com/esmini/esmini/blob/master/docs/InnerWorkings.md).

## Background

The purpose of this implementation was to explore and get familiar with the emerging [OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/) data format. The development aimed at supporting various platforms such as Windows, Mac, Linux, and Android. Tool integration and portability were high priorities, the project outcomes should be capable of incorporation in native C++ applications as well as other frameworks like Unity3D (C#) and MATLAB/Simulink, among many others.

Although allowed by the license this implementation is not primarily intended for production use. The code was developed ad hoc to answer research questions connected with the ongoing project. Therefore, code quality, as expected from standard production applications, is lacking when it comes to clarity, structure, comments, error handling and coding guidelines.

Since the Simulation Scenarios project is closed, no formal support should be expected from the initial contributors.  

Nevertheless, regarding the above stated limitations, it was decided to release the code as is, as a public outcome from the project. It can hopefully serve as guidance or just inspiration for those aspiring to build similar tools, or even get accustomed with the OpenSCENARIO format. And of course, all contributions to further development are welcome!

## Binaries and demos
Windows, Linux and Mac supported

Latest release including source, binaries and demo packages is found here: https://github.com/esmini/esmini/releases/latest

3D models used by the example scenarios are included in the demo packages. They are also available [here](https://dl.dropboxusercontent.com/s/5gk8bvgzqiaaoco/models.7z?dl=0). Unpack into esmini/resources. These assets works on all platforms.
Environment models (roads, landscape, buildings...) have been created using [VIRES Road Network Editor](https://vires.mscsoftware.com/solutions/3d-environment-road-network).

See [User Guide](https://esmini.github.io) for more information.

## esmini shared library
The easiest way of integrating esmini in your custom application is to link the all inclusive shared library ScenarioEngineDLL. In spite of the name it's available also on Linux and Mac.
See [User Guide "Hello World" tutorial](https://esmini.github.io/#_hello_world_programming_tutorial) on how to create a minimalistic application based on it.

### OSI support
In addition to internal API ([example](https://esmini.github.io/#_fetch_state_of_scenario_objects)) for getting information about the ongoing simulation, e.g. road width and road user position, esmini collects everything into an [OSI](https://opensimulationinterface.github.io/osi-documentation/index.html) [groundtruth structure](https://opensimulationinterface.github.io/open-simulation-interface/structosi3_1_1GroundTruth.html). This structure can be 1. stored in an OSI trace-file, 2. retrieved via UDP and/or 3. retrieved directly via API (see [esminiLib.hpp](https://github.com/esmini/esmini/blob/3af727a3f95825bfcf8b1cbd7becf68ea26cf08e/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L473)).
    
  **Note**: Only parts of OSI groundthruth is populated.

  Hello World tutorial includes an [example of how to fetch OSI groundtruth](https://esmini.github.io/#_osi_groundtruth).

  Current versions used by esmini:  
   - OSI version: v3.3.1 
   - Google Protobuf version: v3.15.2
 
  Script that compiles OSI for Win/Linux/Mac available [here](https://github.com/esmini/esmini/blob/master/scripts/generate_osi_libs.sh). Run in bash, also on Windows, e.g. GIT bash.

### Unity support
esmini shared library works fine also in Unity (Win, Linux, Mac). A simple example can be downloaded from [here](https://www.dropbox.com/s/sj53hz0zesxa681/esmini-player.unitypackage?dl=1). The package contains everything needed to get going:
- esmini library C# wrapper
- a generic scenario player script 
- a few example scenarios (OpenSCENARIO + OpenDRIVE) including 3D models 
- plugins for all platforms (can be updated of course)
- and finally a scene connecting the player script to a game object 

Run:
1. Import package (Import Package -> Custom Package). 
2. Load the scene (Assets/Scenes/esmini-scene). 
3. Then just press play-button.

Select (click on) esmini-player in Hierarchy to show up in Inspector, where you can specify scenario file and a few parameters. Note that scenario file and dependent OpenDRIVE and optional 3D scenegraph file must be present in StreamingAssets folder.

Plugins can be updated, but NOTE that you need to restart Unity to (re)load shared library plugins. And, of course, scenario files and other content can be added or replaced.

### esmini & Python
see [User Guide Hello-World tutorial](https://esmini.github.io/#_python_binding).

## Related work
### scenariogeneration
[scenariogeneration](https://github.com/pyoscx/scenariogeneration) is a Python based scenario creation framework. The idea is to write scenarios in a high-level script format and automatically generate and run linked OpenDRIVE and OpenSCENARIO XML counterparts. It supports parameter sweeps to create multiple variants of a parameterized scenario.

### ALKS scenarios
[OSC-ALKS-scenarios](https://github.com/arauschert/OSC-ALKS-scenarios) is a collection of scenarios for Automated Lane Keeping System testing. "BMW has taken on the task of implementing the test scenarios from the ALKS regulation using OpenSCENARIO and OpenDRIVE resulting in a bundle of XML files executable with standard compliant simulators."

### Online OpenDRIVE viewer
[odrviewer.io](https://odrviewer.io/) is an excellent interactive online OpenDRIVE viewer. Move around, zoom and inspect elements of the road network like roadID, laneID, successor and predecessor.

https://github.com/johschmitz/blender-driving-scenario-creator

### OpenDRIVE plugin for Unreal Engine
[brifsttar/OpenDRIVE](https://github.com/brifsttar/OpenDRIVE) plugin allows you to manipulate OpenDRIVE road networks in Unreal Engine. It's based on esmini RoadManager (esminiRMLib).

### Blender Driving Scenario Creator add-on
https://github.com/johschmitz/blender-driving-scenario-creator

### Editors

[Blender Driving Scenario Creator add-on](https://github.com/johschmitz/blender-driving-scenario-creator) lets you create OpenDRIVE and OpenSCENARIO based scenarios.

[OpenScenarioEditor](https://github.com/ebadi/OpenScenarioEditor) is a simple ASAM OpenSCENARIO editor developed by Infotiv AB under VALU3S project. 

### python-esmini
[python-esmini](https://github.com/ebadi/python-esmini) is a Python wrapper for esmini internal API (not limited by the interfaces of esminiLib and esminiRMLib). Most of the important modules are supported. Currently available for Linux only. 

**Note:** For most use cases the esminiLib basic Python binding is enough, see example in [User guide](https://esmini.github.io/#_python_binding). But if you need access to esmini internal API this project is worth to check out.

### Carla Simulator
[Carla](http://carla.org/) is an [Unreal](https://www.unrealengine.com/) based open source simulator worth to check out.

## Data formats

[OpenDRIVE](https://www.asam.net/standards/detail/opendrive/)
describes the road network, the static part of a scenario.

[OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/)
describes the dynamic content on top of a road network, e.g. traffic maneuvers and weather conditions.
