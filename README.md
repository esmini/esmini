# Environment Simulator Minimalistic (esmini)

*esmini* is a basic OpenSCENARIO player

[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)
[![Build Status](https://img.shields.io/appveyor/ci/esmini/esmini/master.svg)](https://ci.appveyor.com/project/esmini/esmini)

![Screenshot](resources/screenshot.jpg?raw=true "Screenshot")

It contains the following main modules:

- RoadManager. A library providing an interface to road networks described in the OpenDRIVE format.
- ScenarioEngine. A library providing an interface to traffic scenarios described in the OpenSCENARIO format.
- ViewerBase. A library based on [OpenSceneGraph](http://www.openscenegraph.org/) providing a simple visualization of the road and scenario.
- ScenarioEngineDLL. A library packaging the above three modules into a single library with a simplified API.

and a few applications that can be used as is or provide ideas for customized solutions:

- EnvironmentSimulator. A simple scenario player linking ScenarioEngine and Viewer modules statically.
- ScenarioViewer. A minimalistic example using the scenarioengine DLL to play OpenSCENARIO files.
- EgoSimulator. An example of how to integrate a simple Ego vehicle with the scenario engine.
- OdrPlot. Produces a data file from OpenDRIVE for plotting the road network in Python.
- OpenDriveViewer. Visualize OpenDRIVE road network with populated dummy traffic.
- Replayer. Re-play previously executed scenarios.

Repository: <https://github.com/esmini/esmini>

A pre-built demo package for Windows is available [here](https://github.com/esmini/esmini/releases/latest). Unzip, navigate to "esmini\run\EgoSimulator" and run any of the bat-files.

The code is a result from the Swedish collaborative research project [Simulation Scenarios](https://sites.google.com/view/simulationscenarios).

The purpose of this implementation was to explore and get familiar with the emerging [OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/) data format. The development aimed at supporting various platforms such as Windows, Mac, Linux, and Android. Tool integration and portability were high priorities, the project outcomes should be capable of incorporation in native C++ applications as well as other frameworks like Unity3D (C#) and MATLAB/Simulink, among many others.

Although allowed by the license this implementation is not primarily intended for production use. The code was developed ad hoc to answer research questions connected with the ongoing project. Therefore, code quality, as expected from standard production applications, is lacking when it comes to clarity, structure, comments, error handling and coding guidelines.

[OpenSCENARIO coverage](./osc_coverage.txt) is limited as it was developed on demand and defined by the research scope. Moreover, since the Simulation Scenario project is closed, no formal support should be expected from the initial contributors.

Nevertheless, regarding the above stated limitations, it was decided to release the code as is, as a public outcome from the project. It can hopefully serve as guidance or just inspiration for those aspiring to build similar tools, or even get accustomed with the OpenSCENARIO format. All contributions are welcome!

## Build and run

Windows and Linux binary builds and demo packages are available at CI service [AppVeyor/esmini](https://ci.appveyor.com/project/esmini/esmini). If you want to build yourself, here is some instructions:

- Windows instructions are found [here](BuildOnWindows.md).
- Linux instructions are found [here](BuildOnLinux.md).

For Mac, only the shared library variant without OSG has been tested (as a Unity3D native plugin). The script [create_xcode_project.sh](./create_xcode_project.sh) can be used as a starting point.

Even though the code has been tested on Android there are unfortunately no automated scripts or even instructions available for that target platform.

External dependencies: [3D models used by the example scenarios](https://drive.google.com/uc?export=download&id=1RSbyFJoVahX1nGWAsdepsPsznAiNspUc). Unpack into esmini/resources. These assets works on all platforms.
Environment models (roads, landscape, buildings...) have been created using [VIRES Road Network Editor](https://vires.com/vtd-vires-virtual-test-drive/#creation).

## Related work
### Scenario video clip generator
[esmini-visualizer](https://github.com/matthewcoyle-cpc/esmini-visualiser) is a tool to automatically generate video visualizations of scenarios.

### Scenario editor
[RControlStation scenario editor](https://github.com/vedderb/rise_sdvp). An embryo to an OpenSCENARIO editor, part of Self-Driving Model Vehicle Platform (SDVP).

Instruction:
1. git clone https://github.com/vedderb/rise_sdvp
1. cd rise_sdvp/Linux/RControlStation
1. git clone https://github.com/esmini/esmini esmini
1. Edit RControlStation.pro and uncomment the line: #DEFINES += HAS_SIM_SCEN
1. Build the project. The editor is the last tab in RControlStation.

The implementation is very limited at this point, but some things can be edited and the scenario can be executed and observed from above, as well as exported to the other tools. You have to start by importing one of the existing scenarios into the editor, as it cannot make a scenario from scratch.

## Data formats

[OpenDRIVE](https://www.asam.net/standards/detail/opendrive/)
describes the road network, the static part of a scenario.

[OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/)
describes the dynamic content on top of a road network, e.g. traffic maneuvers and weather conditions.
