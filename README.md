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

Pre-built demo packages are available [here](https://github.com/esmini/esmini/releases/latest). Unzip, navigate to "esmini\run\EgoSimulator" and run any of the example scripts. See more info below under "Binaries and demos".

**Please note**: From version 1.5 esmini only supports OpenSCENARIO v1.0. All demo scenarios has been updated from 0.9.1 to 1.0. ASAM provides a transformation scheme (migration0_9_1to1_0.xslt, part of the OpenSCENARIO 1.0 release bundle) that can be used with tools for automatic migration of XML files.

The code was initially a result from the Swedish collaborative research project [Simulation Scenarios](https://sites.google.com/view/simulationscenarios), and is now further developed based on users need and OpenSCENARIO development.

## Background

The purpose of this implementation was to explore and get familiar with the emerging [OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/) data format. The development aimed at supporting various platforms such as Windows, Mac, Linux, and Android. Tool integration and portability were high priorities, the project outcomes should be capable of incorporation in native C++ applications as well as other frameworks like Unity3D (C#) and MATLAB/Simulink, among many others.

Although allowed by the license this implementation is not primarily intended for production use. The code was developed ad hoc to answer research questions connected with the ongoing project. Therefore, code quality, as expected from standard production applications, is lacking when it comes to clarity, structure, comments, error handling and coding guidelines.

[OpenSCENARIO coverage](./osc_coverage.txt) is limited as it was developed on demand and defined by the research scope. Moreover, since the Simulation Scenarios project is closed, no formal support should be expected from the initial contributors.  
Nevertheless, regarding the above stated limitations, it was decided to release the code as is, as a public outcome from the project. It can hopefully serve as guidance or just inspiration for those aspiring to build similar tools, or even get accustomed with the OpenSCENARIO format. And of course, all contributions to further development are welcome!

## Binaries and demos
Windows, Linux and Mac supported

Latest release including source, binaries and demo packages is found here: https://github.com/esmini/esmini/releases/latest

On Mac the zip-package might be put in quarantine, to release it:
`xattr -d com.apple.quarantine file.zip`
or even better:
`xattr -c file.zip`

3D models used by the example scenarios are included in the demo packages. They are also available [here](https://drive.google.com/uc?export=download&id=11a8njhkRIjLYMqCsSL9pU-d5_A8pMVhc). Unpack into esmini/resources. These assets works on all platforms.
Environment models (roads, landscape, buildings...) have been created using [VIRES Road Network Editor](https://vires.mscsoftware.com/solutions/3d-environment-road-network).

## Build
If you want to build yourself, please find some instructions [here](https://github.com/esmini/esmini/blob/master/docs/BuildInstructions.md).

## Run esmini
Either get the demo or build yourself. To run demos:
1. Navigate to run/EgoSimulator
2. Run any of the provided batch-script examples (double click on or run from command line)

Further info:
* [Launch commands](https://github.com/esmini/esmini/blob/master/docs/commands.txt)
* [Runtime control](https://github.com/esmini/esmini/blob/master/docs/readme.txt)

## esmini shared library
The easiest way of integrating esmini in your custom application is to link the all inclusive shared library ScenarioEngineDLL. In spite of the name it's available also on Linux and Mac.
See [this "Hello World" tutorial](https://github.com/esmini/esmini/blob/master/Hello-World_coding-example/README.md) on how to create a minimalistic application based on it.

## esmini controllers

esmini comes with a few controllers (ways of controlling individual entities in the scenario):

- DefaultController. Performs actions exactly as specified in the OpenSCENARIO file. Assigned to entities by default.
- InteractiveController. Hand over control to the user via keyboard arrow keys.
- FollowGhost. A ghost-twin is performing the events a few seconds ahead. The entity will then follow its trajectory.
- ExternalController. State (position, rotation ...) expected to be reported from external simulator via API. Ghost trajectory can be created for an external driver model as reference.
- SumoController. A way of integrating SUMO controlled vehicles in a scenario.
 
## Related work
### pyoscx
[pyoscx](https://github.com/pyoscx/pyoscx) is a Python based scenario creation framework. The idea is to write scenarios in a high-level script format and automatically generate the OpenSCENARIO 1.0 XML counterpart.

### pyodrx
[pyodrx](https://github.com/pyoscx/pyodrx) is a Python based road network creation framework. The idea is to write road networks in a high-level script format and automatically generate the OpenDRIVE (1.4 as of today) XML counterpart.

### Scenario video clip generator
[esmini-visualizer](https://github.com/matthewcoyle-cpc/esmini-visualiser) is a tool to automatically generate video visualizations of scenarios.

Note that it does not seem to work with Anti-Alias filtering. Therefore make sure to run EgoSimulator without Anti-Alias by providing argument "--aa_mode 0".

### Scenario editor
[RControlStation scenario editor](https://github.com/vedderb/rise_sdvp). An embryo to an OpenSCENARIO editor, part of Self-Driving Model Vehicle Platform (SDVP).

Instruction:
1. git clone https://github.com/vedderb/rise_sdvp
1. cd rise_sdvp/Linux/RControlStation
1. git clone https://github.com/esmini/esmini esmini
1. Edit RControlStation.pro and uncomment the line: #DEFINES += HAS_SIM_SCEN
1. Build the project. The editor is the last tab in RControlStation.

The implementation is very limited at this point, but some things can be edited and the scenario can be executed and observed from above, as well as exported to the other tools. You have to start by importing one of the existing scenarios into the editor, as it cannot make a scenario from scratch.

### Carla Simulator
[Carla](http://carla.org/) is an [Unreal](https://www.unrealengine.com/) based open source simulator worth to check out.

## Data formats

[OpenDRIVE](https://www.asam.net/standards/detail/opendrive/)
describes the road network, the static part of a scenario.

[OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/)
describes the dynamic content on top of a road network, e.g. traffic maneuvers and weather conditions.
