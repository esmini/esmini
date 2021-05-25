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

Pre-built demo packages are available [here](https://github.com/esmini/esmini/releases/latest). Unzip, navigate to "esmini\run\esmini" and run any of the example scripts. See more info below under "Binaries and demos".

esmini supports OpenSCENARIO v1.1 (from esmini v2.7) and v1.0 (from esmini v1.5). In order to run older versions (i.e. v0.9.1) ASAM provides a transformation scheme (migration0_9_1to1_0.xslt, part of the OpenSCENARIO release bundle) that can be used with tools for automatic migration of XML files.

Please note that the OpenSCENARIO [coverage](https://github.com/esmini/esmini/blob/master/osc_coverage.txt) is limited, which means that not all features of OpenSCENARIO are supported. The functionalty grows slow but steady, based on need and contributions.

The code was initially a result from the Swedish collaborative research project [Simulation Scenarios](https://sites.google.com/view/simulationscenarios), and is now further developed based on users need and OpenSCENARIO development.

For more information about the esmini software parts, please see [Inner Workings of esmini](https://github.com/esmini/esmini/blob/master/docs/InnerWorkings.md).

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

#### Mac specifics tips
On Mac the zip-package might be put in quarantine, to release it:
`xattr -d com.apple.quarantine file.zip`
or even better:
`xattr -c file.zip`

If you get the "damaged file" message, please open a terminal in the folder where the esmini-demo was extracted, and run the following command:
`xattr -c -r esmini-demo`

## Build
If you want to build yourself, please find some instructions [here](https://github.com/esmini/esmini/blob/master/docs/BuildInstructions.md).

## Run esmini
Either get the demo or build yourself. To run demo scenarios:
1. Navigate to run/esmini
2. Run any of the provided batch-script examples (double click on or run from command line)

Or launch esmini from command prompt. There are many options but a few typical examples, assuming current directory is esmini root folder:  

* Just run a scenario:  
```./bin/esmini --window 60 60 800 400 --osc ./resources/xosc/cut-in.xosc```

* Execute and record a scenario with fixed timesteps and no viewer:  
```./bin/esmini --headless --fixed_timestep 0.01 --record sim.dat --osc ./resources/xosc/cut-in.xosc```

* Replay a recorded scenario:  
```./bin/replayer --window 60 60 800 400 --res_path ./resources --file sim.dat```

Further info:
* [esmini launch commands](https://github.com/esmini/esmini/blob/master/docs/commands.txt)
* [esmini runtime control](https://github.com/esmini/esmini/blob/master/docs/readme.txt)
* [replayer launch and key shortcut commands](https://github.com/esmini/esmini/blob/master/EnvironmentSimulator/Applications/replayer/readme.txt)

## Run ASAM OpenSCENARIO examples

With some limitations (see details [here](https://github.com/esmini/esmini/blob/master/osc_coverage.txt)) esmini can play the example scenarios provided with the ASAM OpenSCENARIO v1.1 release bundle.
* If you don't have esmini already, download latest demo package for your platform from [here](https://github.com/esmini/esmini/releases/latest).
* Download the standard from ASAM [here](https://www.asam.net/standards/detail/openscenario/) (register and download is free of charge).
* Extract to any folder.
* Run the examples from command line in esmini root folder, for example:  
```./bin/esmini --window 60 60 800 400 --osc ../../openscenario-v1.1.0/Examples/DoubleLaneChanger.xosc```  
  or with absolute path:  
```./bin/esmini --window 60 60 800 400 --osc c:/stuff/openscenario-v1.1.0/Examples/DoubleLaneChanger.xosc```


## esmini shared library
The easiest way of integrating esmini in your custom application is to link the all inclusive shared library ScenarioEngineDLL. In spite of the name it's available also on Linux and Mac.
See [this "Hello World" tutorial](https://github.com/esmini/esmini/blob/master/Hello-World_coding-example/README.md) on how to create a minimalistic application based on it.

### OSI support
In addition to internal API ([example](https://github.com/esmini/esmini/tree/master/Hello-World_coding-example#fetch-state-of-scenario-objects)) for getting information about the ongoing simulation, e.g. road width and road user position, esmini collects everything into an [OSI](https://opensimulationinterface.github.io/osi-documentation/index.html) [groundtruth structure](https://opensimulationinterface.github.io/open-simulation-interface/structosi3_1_1GroundTruth.html). This structure can be 1. stored in an OSI trace-file, 2. retrieved via UDP and/or 3. retrieved directly via API (see [esminiLib.hpp](https://github.com/esmini/esmini/blob/3af727a3f95825bfcf8b1cbd7becf68ea26cf08e/EnvironmentSimulator/Libraries/esminiLib/esminiLib.hpp#L473)).
    
  **Note**: Only parts of OSI groundthruth is populated.

  Hello World tutorial includes an [example of how to fetch OSI groundtruth](https://github.com/esmini/esmini/tree/master/Hello-World_coding-example#OSI-groundtruth).

  Current versions used by esmini:  
   - OSI version: v3.3.1 
   - Google Protobuf version: v3.15.2
 
  Script that compiles OSI for Win/Linux/Mac available [here](https://github.com/esmini/esmini/blob/master/scripts/generate_osi_libs.sh). Run in bash, also on Windows, e.g. GIT bash.

### Unity support
esmini shared library works fine also in Unity (Win, Linux, Mac). A simple example can be downloaded from [here](https://www.dropbox.com/s/sj53hz0zesxa681/esmini-player.unitypackage?dl=0). The package contains everything needed to get going:
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
see last section in [Hello-World_coding-example](https://github.com/esmini/esmini/tree/master/Hello-World_coding-example#python-binding).

## esmini controllers

esmini comes with a few controllers (ways of controlling individual entities in the scenario):

- DefaultController. Performs actions exactly as specified in the OpenSCENARIO file. Assigned to entities by default.
- InteractiveController. Hand over control to the user via keyboard arrow keys.
- FollowGhost. A ghost-twin is performing the events a few seconds ahead. The entity will then follow its trajectory.
- ExternalController. State (position, rotation ...) expected to be reported from external simulator via API. Ghost trajectory can be created for an external driver model as reference.
- SumoController. A way of integrating SUMO controlled vehicles in a scenario.

More information [here](https://github.com/esmini/esmini/blob/master/docs/Controllers.md).

## 3D model support

esmini make use of OpenSceneGraph (OSG) for visualization of the scenario. The OpenSCENARIO files can optionally refer to existing 3D models of the static environment (scene graph) and dynamic objects (entites). If the scene graph reference is missing, esmini will try to generate a basic model based on the OpenDRIVE road network description.

Currently esmini only supports OSG native .osgb 3D file format. However, there are ways to convert 3D models of some other formats using the OSG tool osgconv. Please see [this issue report](https://github.com/esmini/esmini/issues/63#issuecomment-742273326) for some more info.


## Related work
### pyoscx
[pyoscx](https://github.com/pyoscx/pyoscx) is a Python based scenario creation framework. The idea is to write scenarios in a high-level script format and automatically generate the OpenSCENARIO 1.0 XML counterpart.

### pyodrx
[pyodrx](https://github.com/pyoscx/pyodrx) is a Python based road network creation framework. The idea is to write road networks in a high-level script format and automatically generate the OpenDRIVE (1.4 as of today) XML counterpart.

### pyoscx/scenariogeneration
[pyoscx/scenariogeneration](https://github.com/pyoscx/scenariogeneration) is a simple Python wrapper to combine pyoscx and pyodrx, providing an interface to parametrize and generate linked OpenSCENARIO and OpenDRIVE files to run multiple simulations, including parameter sweeps.

### ALKS scenarios
[OSC-ALKS-scenarios](https://github.com/arauschert/OSC-ALKS-scenarios) is a collection of scenarios for Automated Lane Keeping System testing. "BMW has taken on the task of implementing the test scenarios from the ALKS regulation using OpenSCENARIO and OpenDRIVE resulting in a bundle of XML files executable with standard compliant simulators."

### Scenario video clip generator
[esmini-visualizer](https://github.com/matthewcoyle-cpc/esmini-visualiser) is a tool to automatically generate video visualizations of scenarios.

Note that it does not seem to work with Anti-Alias filtering. Therefore make sure to run `esmini` without Anti-Alias by providing argument `--aa_mode 0`.

### Scenario editors

**[OpenScenarioEditor](https://github.com/ebadi/OpenScenarioEditor).** A simple ASAM OpenSCENARIO editor developed by Infotiv AB under VALU3S project. 

**[RControlStation scenario editor](https://github.com/vedderb/rise_sdvp).** Another embryo to an OpenSCENARIO editor, part of Self-Driving Model Vehicle Platform (SDVP).

The implementation is very limited at this point, but some things can be edited and the scenario can be executed and observed from above, as well as exported to the other tools. You have to start by importing one of the existing scenarios into the editor, as it cannot make a scenario from scratch.

Instruction:

* Get RControlStation source code

```
git clone https://github.com/vedderb/rise_sdvp
```

* Build esmini:
```
cd rise_sdvp/Linux/RControlStation
git clone https://github.com/esmini/esmini
cd esmini
mkdir build
cd build
cmake ../ -DUSE_OSG=true -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release --target install
cd ../../
```
* Edit RControlStation.pro and uncomment the line: #DEFINES += HAS_SIM_SCEN
* Build and run RControlCenter:
```
./build_lin
export LD_LIBRARY_PATH=esmini/bin
./build/lin/RControlStation
```
The editor is the last tab in RControlStation.

### esmini-pybind11
[esmini-pybind11](https://github.com/ebadi/esmini-pybind11) is an ongoing effort to establish a Python wrapper for internal esmini API (not only shared libraries esminiLib and esminiRMLib). 


### Carla Simulator
[Carla](http://carla.org/) is an [Unreal](https://www.unrealengine.com/) based open source simulator worth to check out.

## Data formats

[OpenDRIVE](https://www.asam.net/standards/detail/opendrive/)
describes the road network, the static part of a scenario.

[OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/)
describes the dynamic content on top of a road network, e.g. traffic maneuvers and weather conditions.
