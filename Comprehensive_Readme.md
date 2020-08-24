
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

Latest release is found here: https://github.com/esmini/esmini/releases/latest

All builds, successful and unsuccessful ones :), are available at the CI service [AppVeyor/esmini](https://ci.appveyor.com/project/esmini/esmini). Click on a job, then find files under the Artifact tab.

On Mac the zip-package might be put in quarantine, to release it:
`xattr -d com.apple.quarantine file.zip`
or even better:
`xattr -c file.zip`

3D models used by the example scenarios are included in the demo packages. They are also available [here](https://drive.google.com/uc?export=download&id=11a8njhkRIjLYMqCsSL9pU-d5_A8pMVhc). Unpack into esmini/resources. These assets works on all platforms.
Environment models (roads, landscape, buildings...) have been created using [VIRES Road Network Editor](https://vires.com/vtd-vires-virtual-test-drive/#creation).

## Build
If you want to build yourself, please find some instructions [here](docs/BuildInstructions.md).

## Related work
### pyoscx
[pyoscx](https://github.com/pyoscx/pyoscx) is a Python based scenario creation framework. The idea is to write scenarios in a high-level script format and automatically generate the OpenSCENARIO 1.0 XML counterpart. 

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
## Runtime Commands

Run esmini demo:
    1. Navigate to run/EgoSimulator
    2. Run any of the provided batch-script examples (double click on or run from command line)

Key commands:

    Arrow keys is used to drive externally controlled Ego vehicle:
      Up:    Accelerate
      Down:  Brake
      Left:  Steer left
      Right: Steer right


    TAB: Move camera to next vehicle
    Shift-TAB: Move camera to previoius vehicle

    1-9: Camera models acording to:
        1. Custom camera model
        2. Flight
        3. Drive
        4. Terrain
        5. Orbit
        6. FirstPerson
        7. Spherical
        8. NodeTracker
        9. Trackball

    When custom camera model (1) is activated:
        k: Switch between the following sub models:
            - Orbit      (camera facing vehicle, rotating around it)
            - Fixed      (fix rotation, always straight behind vehicle) 
            - Flex       (imagine the camera attached to vehicle via an elastic string)
            - Flex-orbit (Like flex but allows for roatation around vehicle)
            - Top        (top view, fixed rotation, always straight above vehicle) 

    o: Toggle show/hide OpenDRIVE road feature lines
    u: Toggle show/hide OSI road feature lines
    p: Toggle show/hide environment 3D model
    r: Toggle show/hide sensor view frustums
    i: Toggle info text showing time and speed
    j: Toggle show trails after vehicles
    ESC: quit

    Viewer options:
      f: Toggle full screen mode
      t: Toggle textures
      s: Rendering statistics
      l: Toggle light
      w: Toggle geometry mode (shading, wireframe, dots)
      c: Save screenshot in JPEG format - in the folder where the application was started from
      h: Help 


Mouse control:

    Left: Rotate
    Right: Zoom
    Middle: Pan

    This is typical. Exact behaviour depends on active camera model.
## Launch Arguments
Usage : [options]
Options:

	  --osc <filename>
	      OpenSCENARIO filename
	  --control <mode>
	      Ego control ("osc", "internal", "external", "hybrid"
	  --record <filename>
	      Record position data into a file for later replay
	  --csv_logger <filename>
	      Log data for each vehicle in ASCII csv format
	  --info_text <mode>
	      Show info text HUD ("on" (default), "off") (toggle during simulation by press 'i')
	  --trails <mode>
	      Show trails ("on" (default), "off") (toggle during simulation by press 'j')
	  --road_features <mode>
	      Show road features ("on" (default), "off") (toggle during simulation by press 'o')
	  --osi_features <mode>
	      Show OSI road features ("on", "off" (default)) (toggle during simulation by press 'u')
	  --sensors <mode>
	      Show sensor frustums ("on", "off" (default)) (toggle during simulation by press 'r')
	  --camera_mode <mode>
	      Initial camera mode ("orbit" (default), "fixed", "flex", "flex-orbit", "top") (toggle during simulation by press 'k')
	  --aa_mode <mode>
	      Anti-alias mode=number of multisamples (subsamples, 0=off, 4=default)
	  --threads
	      Run viewer in a separate thread, parallel to scenario engine
	  --headless
	      Run without viewer
	  --server
	      Launch server to receive state of external Ego simulator
	  --fixed_timestep <timestep>
	      Run simulation decoupled from realtime, with specified timesteps
	  --osi_receiver_ip <IP address>
	      IP address where to send OSI UDP packages
	  --ghost_headstart <time>
	      Launch Ego ghost at specified headstart time
	  --osi_file <mode>
	      save osi messages in file ("on", "off" (default))
	  --osi_freq <frequence>
	      relative frequence for writing the .osi file e.g. --osi_freq=2 -> we write every two simulation steps
      
