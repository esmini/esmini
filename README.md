[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)

# Environment Simulator Minimalistic (esmini)

\- a basic OpenSCENARIO player

![Screenshot](resources/screenshot.jpg?raw=true "Screenshot")

It contains the following main modules:
- RoadManager. A library providing an interface to road networks described in the OpenDRIVE format.
- ScenarioEngine. A library providing an interface to traffic scenarios described in the OpenSCENARIO format.
- ViewerBase. A library based on OpenSceneGraph providing a simple visualization of the road and scenario.
- ScenarioEngineDLL. A library packaging the above three modules into a single library with a simplified API.

and a few applications that can be used as is or provide ideas for customized things:
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

The purpose of this implementation was to learn about and explore the emerging [OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/) data format. One goal was to support various platforms such as Windows, Mac and Linux native C++ applications and also frameworks like Unity3D (C#), MATLAB/Simulink and Android. So portability and packaging into a shared library was important aspects.

Although allowed by the license it is not intended for production use. The code was developed ad hoc. Code quality (clarity, structure, comments, error handling, code guidelines) is lacking. OpenSCENARIO coverage is limited. And since the Simulation Scenario project is closed, no formal support should be expected from initial contributors.

Nevertheless it was decided to release the code as is, as a public outcome from the project. Best case it will get used and might improve over time.

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

cmake is used to create build configurations, e.g. VisualStudio solutions.

Use supplied batch scripts or generate desired build setup. Or use them as examples to create your own, e.g. with or without visualization (OSG) support.

## Build project
First generate build configuration (see above). Then follow the steps below, at least for VisualStudio, other platforms would be similar:
1. Open generated solution, build/EnvironmentSimulator.sln
1. Select configuration, Debug (default) or Release.
1. Build CMakePredefinedTargets/INSTALL (right-click and select build)

This will build all projects and copy the binaries into a dedicated folder found by the demo scripts.

## Run demo applications
- Navigate to the esmini/run folder
- There is a subfolder for each application including a few example batch-files
- Simply doubleclick on any to run
- Usage help, see [EnvironmentSimulator/ViewerBase/readme.txt](EnvironmentSimulator/ViewerBase/readme.txt)

Environment models (roads, landscape, buildings...) have been created using [VIRES Road Network Editor](https://vires.com/vtd-vires-virtual-test-drive/#creation).
