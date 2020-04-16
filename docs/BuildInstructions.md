# Build and Run Instructions

## Build configurations
The following platforms are supported:
- VisualStudio 2017 / win64 / Windows SDK v10 / Release and Debug (default/preferred)
- VisualStudio 2017 / win32 / Windows SDK v10 / Release and Debug
- VisualStudio 2017 / win64 / Windows SDK v7.1 / Release and Debug
- Ubuntu and Kubuntu (tested on 18.04) / gcc / Release
- macOS (Catalina) / Xcode / Release

[CMake](https://cmake.org/) tool is used to create standard make configurations. A few example "create..." batch scripts are supplied as examples how to generate desired build setup.

All configurations defines an "Install" build target that compiles (if needed) and copies relevant binaries into a common "esmini/bin" folder recognized by the example scripts under the "esmini/run" folder.

> Note:
>- For automatic downloading of external dependencies (OSG binaries) and 3D models, CMake version 3.11.4 or above is required (FetchContent_MakeAvailable was introduced).
>- In Windows, if you get an error like "the c compiler identification is unknown", then please make sure to install "Windows Universal CRT SDK" from the Visual Studio Installer tool.

## External dependencies
CMake scripts will download two packages: External binary dependencies OpenSceneGraph (OSG) and 3D model resource files. If not using CMake, here are direct links:

- [OSG for WinSDK v10 x64](https://drive.google.com/uc?export=download&id=1YxLVdQLhKBMGW4HB_ArJglRIpzuDiwhJ)
- [OSG for WinSDK v10 win32](https://drive.google.com/uc?export=download&id=10dV9P0qOeJUgTtsSDld4AlbClE--SivX)
- [OSG for WinSDK 7.1 x64](https://drive.google.com/uc?export=download&id=1NBEvGZiTWmqxk-MEOq7uK1uh_vaUMaEL)  
- [OSG for Linux Ubuntu/Kubuntu 18.04](https://drive.google.com/uc?export=download&id=1OufA3TUQjBTkaRvAuo0rSUwryvoqde8G)
- [OSG for Apple macOS Catalina](https://drive.google.com/uc?export=download&id=1yEFOB8HmNP5lPxYD4VJC-ry7XaBG9dsJ)

Unpack into esmini/externals/OpenSceneGraph. For other platforms OSG needs to be downloaded and built separately (set CMake flags DYNAMIC_OPENSCENEGRAPH and DYNAMIC_OPENTHREADS = false). See further information below.

- [3D models used by the example scenarios](https://drive.google.com/uc?export=download&id=11a8njhkRIjLYMqCsSL9pU-d5_A8pMVhc).

Unpack into esmini/resources.

## Additional platform dependencies

Linux Ubuntu 18.04

```
sudo apt install build-essential git pkg-config libgl1-mesa-dev libpthread-stubs0-dev libjpeg-dev libxml2-dev libpng-dev libtiff5-dev libgdal-dev libpoppler-dev libdcmtk-dev libgstreamer1.0-dev libgtk2.0-dev libcairo2-dev libpoppler-glib-dev libxrandr-dev libxinerama-dev curl cmake
```
Also, g++ version >= 5 is needed for c++14 code support.

Windows and Mac: Install the [cmake](https://cmake.org/) application

## Build project
First generate build configuration (see above)

Windows/Visual Studio
1. Open generated solution, build*/EnvironmentSimulator.sln
1. Select configuration, Debug or Release
1. Build CMakePredefinedTargets/INSTALL (right-click and select build)

macOS
1. cd build
1. xcodebuild -scheme install -configuration Release build

- or open generated project in Xcode, and build from there

Linux
1. cd build
1. make -j4 install

This will build all projects and copy the binaries into a dedicated folder found by the demo batch scripts.

## Run demo applications
- Navigate to the esmini/run folder
- There is a subfolder for each application including a few example batch- or script files
- Simply doubleclick on any to run (might need to run from terminal in Linux/Mac)
- For usage description of available options, start the application with no arguments.
- Key commands (shortcuts) are described briefly in [run/readme.txt](../run/readme.txt)

# Build OSG
Description of how to build OSG from scratch is found [here](BuildOSG.md).
