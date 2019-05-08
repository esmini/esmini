[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)

# Environment Simulator Minimalistic (esmini)

\- a basic OpenSCENARIO player

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

Repository: https://github.com/esmini/esmini

A pre-built package for Windows is available [here](https://drive.google.com/uc?export=download&id=1tS4i2Cik0Ac7Dolp9by4WZsiVY6kpnEz). Unzip, navigate to "esmini\run\EgoSimulator" and run any of the bat-files

The code is a result from the Swedish collaborative research project [Simulation Scenarios](
https://sites.google.com/view/simulationscenarios).

The purpose of this implementation was to explore and get familiar with the emerging [OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/) data format. The development aimed at supporting various platforms such as Windows, Mac, Linux, and Android. Tool integration and portability were high priorities, the project outcomes should be capable of incorporation in native C++ applications as well as other frameworks like Unity3D (C#) and MATLAB/Simulink, among many others.

Although allowed by the license this implementation is not primarily intended for production use. The code was developed ad hoc to answer research questions connected with the ongoing project. Therefore, code quality, as expected from standard production applications, is lacking when it comes to clarity, structure, comments, error handling and coding guidelines.

[OpenSCENARIO coverage](osc_coverage.txt) is limited as it was developed on demand and defined by the research scope. Moreover, since the Simulation Scenario project is closed, no formal support should be expected from the initial contributors.

Nevertheless, regarding the above stated limitations, it was decided to release the code as is, as a public outcome from the project. It can hopefully serve as guidance or just inspiration for those aspiring to build similar tools, or even get accustomed with the OpenSCENARIO format.

## Related work

A second OpenSCENARIO based scenario engine was implemented within the Simulation Scenarios project, reusing the esmini RoadManager. In this case the purpose was to extend an existing simulation software platform with OpenSCENARIO based traffic scenarios. The implementation aims for production use. Repository: https://github.com/esmaxi/esmaxi

## Data formats
[OpenDRIVE](https://www.asam.net/standards/detail/opendrive/)
describes the road network, the static part of a scenario.

[OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/)
describes the dynamic content on top of a road network, e.g. traffic maneuvers and weather conditions.

## Build configurations
The following platforms are supported:
- VisualStudio 2017 / win64 / Windows SDK v10 / Release and Debug (default/preferred)
- VisualStudio 2017 / win32 / Windows SDK v10 / Release and Debug
- VisualStudio 2017 / win64 / Windows SDK v7.1 / Release and Debug (no FBX support)
- MAC Xcode
- Linux ?

[CMake](https://cmake.org/) tool is used to create build configurations, e.g. VisualStudio solutions. A few example "create..." scripts are supplied as examples how to generate desired build setup.

## External dependencies
CMake scripts will download a package of external binary dependencies (OSG) and 3D model resource files. If not using CMake, here are links to those external data packages:

- [OSG for WinSDK v10 x64](https://drive.google.com/uc?export=download&id=1a0HxilPJq2bZrat2le2x-Cscs5JeVAXP)
- [OSG for WinSDK v10 win32](https://drive.google.com/uc?export=download&id=14Xqe_bWGuZQAr69mit4melmnMfmNFOO6)
- [OSG for WinSDK 7.1 x64](https://drive.google.com/uc?export=download&id=1aN88B1_7MnT0OwHt_LOc8FtD7rFEP0Jq)  
Unpack into esmini/externals. Please note that these libraries are for Windows only. For other platforms OSG needs to be downloaded and built separately (set CMake flag DYNAMIC_OPENSCENEGRAPH = true).

- [3D models used by the example scenarios](https://drive.google.com/uc?export=download&id=1RSbyFJoVahX1nGWAsdepsPsznAiNspUc)  
Unpack into esmini/resources. These assets works on all platforms.

Environment models (roads, landscape, buildings...) have been created using [VIRES Road Network Editor](https://vires.com/vtd-vires-virtual-test-drive/#creation).

## Build project
The following guide is for Windows/Visual Studio. Other platforms would be similar.
1. First generate build configuration (see above)
1. Open generated solution, build/EnvironmentSimulator.sln
1. Select configuration, Debug (default) or Release
1. Build CMakePredefinedTargets/INSTALL (right-click and select build)

This will build all projects and copy the binaries into a dedicated folder found by the demo scripts.

## Run demo applications on Windows
- Navigate to the esmini/run folder
- There is a subfolder for each application including a few example batch-files
- Simply doubleclick on any to run
- Usage help, see [EnvironmentSimulator/ViewerBase/readme.txt](EnvironmentSimulator/ViewerBase/readme.txt)
