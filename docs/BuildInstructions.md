# Build and Run Instructions

## In brief

Make sure you have a C++ compiler and [CMake](https://cmake.org/) installed. 
From esmini root folder:
```
mkdir build
cd build
cmake ..
cmake --build . --config Release --target install
```

Binary files will be copied into esmini/bin folder. Run from command line:

```./bin/esmini --window 60 60 800 400 --osc ./resources/xosc/cut-in.xosc```

The above is supported on (some versions of) Windows, Linux and Mac.

## Build configurations
[CMake](https://cmake.org/) tool is used to create standard make configurations. A few example "create..." batch scripts are supplied as examples how to generate desired build setup.
- VisualStudio / win64 / Windows SDK v10 / Release and Debug
- Ubuntu and Kubuntu (tested on 18.04) / gcc / Release and Debug

However, it should be possible to configure custom variants using cmake. For example to use Visual Studio 2019 run the following commands from command prompt (CMD or PowerShell), assuming starting point is esmini root folder:
```
mkdir build
cd build
cmake -G "Visual Studio 16 2019" ..
cmake --build . --config Release --target install
```

This will first generate Visual Studio solution and then compile esmini using MSVC toolset v142 (default with Visual Studio 2019). Default architecture is x64.

If you want to compile with MSVC 2017 toolset just add directive to the generator as follows:
```
cmake -G "Visual Studio 16 2019" -T v141 .. 
```

A complete list of supported toolsets are available [here](https://cmake.org/cmake/help/v3.17/variable/MSVC_TOOLSET_VERSION.html)

If you want to specify architecture you simply add -A x64 or -A Win32. So for example, if you want to compile with MSVC 2015 toolset and for win32 use the following generator command:
```
cmake -G "Visual Studio 16 2019" -T v140 -A Win32 .. 
```
Of course, building with a specific toolset requires it to be installed. Use [Visual Studio Installer](https://docs.microsoft.com/en-us/visualstudio/install/install-visual-studio?view=vs-2019). Steps:
* choose "Modify"
* make sure Desktop Development with C++ is checked
* go to tab "Individual components" 
* scroll down to "Compilers, build tools, and runtimes"
* check the MSVC versions you need, e.g. "MSVC v140 - VS 2015 C++ build tools (v14.00)" and "MSVC v141 - VS 2017 C++ x64/x86 build tools (v14.16)"

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

## Dynamic protobuf linking
When linking esmini with software already dependent on Google protobuf there might be need for dynamic linking of shared protobuf library. This can be achieved by defining cmake symbol DYN_PROTOBUF as following example:

```cmake -D DYN_PROTOBUF=True ..```

Then build as usual. It will link with protobuf shared library instead of linking with a static library.

When running esmini protobuf shared library need to be available. Set LD_LIBRARY_PATH to point to the folder where the library is, example:

```export LD_LIBRARY_PATH=./externals/OSI/linux/lib-dyn```

**Note:** The dynamic versions of protobuf were added Aug 31 2021. So you might need to update the OSI library package. Get the latest from following links:
 - [OSI Windows](https://dl.dropboxusercontent.com/s/an58ckp2qfx5069/osi_v10.7z?dl=0)
 - [OSI Linux](https://dl.dropboxusercontent.com/s/kwtdg0c1c8pawa1/osi_linux.7z?dl=0)
 - [OSI Mac](https://dl.dropboxusercontent.com/s/m62v19gp0m73dte/osi_mac.7z?dl=0)

## Slim esmini - customize configration

The external dependencies OSG, OSI and SUMO are optional. Also the unit test suite is optional, in effect making the dependecy to googletest framework optional as well. All these options are simply controlled by the following cmake options:
- USE_OSG
- USE_OSI
- USE_SUMO
- USE_GTEST

So, for example, to cut dependency to OSG and SUMO, run:  
```cmake -D USE_OSG=False -D USE_SUMO=False ..```

To disable OSG, SUMO, OSI and googletest, run:  
```cmake -D USE_OSG=False -D USE_SUMO=False -D USE_OSI=False -D USE_GTEST=False ..```  

All options are enabled/True as default.

**Note:** Disabling an external dependency will disable corresponding functionality. So, for example, disabling OSI means that no OSI data can be created by esmini. Disabling OSG means that esmini can't visualize the scenario. However it can still run the scenario and create a .dat file, which can be played and visualized later in another esmini build in which OSG is enabled (even on another platform).

## Build project
First generate build configuration (see above)

Then it should work on all platform to build using cmake as follows:
```
cmake --build . --config Release --target install
```

Or you can go with platform specific ways of building:

**Windows/Visual Studio**
1. Open generated solution, build*/EnvironmentSimulator.sln
1. Select configuration, Debug or Release
1. Build CMakePredefinedTargets/INSTALL (right-click and select build)

**macOS**
To generate a Xcode project file, run the initial cmake command as follows:
```
cmake -G Xcode ..
```
Then build as usual:
```
cmake --build . --config Release --target install
```
or using Xcode directly:
```
xcodebuild -scheme install -configuration Release build
```
or open the generated project file in Xcode, and build from there.

To create bundles (shared library container), do from esmini root folder:
```
lipo -create bin/libesminiRMLib.dylib -output bin/esminiRMLib.bundle
lipo -create bin/libesminiLib.dylib -output bin/esminiLib.bundle
```

**Linux**
```
cd build
make -j4 install
```
This will build all projects and copy the binaries into a dedicated folder found by the demo batch scripts.

### CentOS 7 (Linux)

CentOS 7 has some limitations, e.g. old versions of C/C++ compiler toolkits and runtimes. So it's not possible to link with provided 3rd party binary libraries targeting Ubuntu 18++.
However, by disabling some featuers in esmini, e.g. OSI and SUMO, it can still be used for previewing scenarios.

VirtualBox image for Windows host here:
https://www.linuxvmimages.com/images/centos-7/


Follow steps below to build and run esmini on CentOS 7.

```
sudo yum install git
sudo yum install cmake
sudo yum install gcc-c++

sudo yum install freeglut-devel
sudo yum install fontconfig-devel
sudo yum install libXrandr-devel
sudo yum install libXinerama-devel

sudo yum install epel-release
sudo yum install p7zip

git clone https://github.com/esmini/esmini

cd esmini

cd externals
mkdir OpenSceneGraph
cd OpenSceneGraph
curl -L "https://www.dropbox.com/s/mxztf6zbgojyntp/osg_centos.7z?dl=1" -o osg_centos.7z
7za x osg_centos.7z
rm osg_centos.7z

cd ../..
mkdir build
cd build
cmake -D USE_OSG=True -D USE_SUMO=False -D USE_OSI=False -D USE_GTEST=False ..
cmake --build . --target install --config Release
cd ..
./bin/esmini.exe --headless --fixed_timestep 0.01 --record sim.dat --osc ./resources/xosc/cut-in.xosc
```

## Run demo applications
- Navigate to the esmini/run folder
- There is a subfolder for each application including a few example batch- or script files
- Simply doubleclick on any to run (might need to run from terminal in Linux/Mac)
- For usage description of available options, start the application with no arguments.
- Key commands (shortcuts) are described briefly in [run/readme.txt](../run/readme.txt)

# Build OSG
Description of how to build OSG from scratch is found [here](BuildOSG.md).

## osgconv
osgconv is a great command line tool for converting 3D models from and to .osgb format.
You can build it yourself or get a prebuilt package.

### Build osg applications (including osgconv and osgviewer)
You need [cmake](https://cmake.org/download/) (make sure to check "Add to system PATH") and compiler (e.g. Visual Studio C++ on Win10, or g++ on Linux)

Clone code: 
```
git clone --branch OpenSceneGraph-3.6.5 https://github.com/openscenegraph/OpenSceneGraph.git
```
or download code from:
https://github.com/openscenegraph/OpenSceneGraph/archive/OpenSceneGraph-3.6.5.zip

Download and install AUTODESK FBX SDK:
https://www.autodesk.com/content/dam/autodesk/www/adn/fbx/2020-1-1/fbx202011_fbxsdk_vs2017_win.exe

Download OSG dependency package:
https://download.osgvisual.org/3rdParty_VS2017_v141_x64_V11_full.7z

Then set environment variable FBX_DIR to the root directory of FBX SDK, e.g; "C:/Program Files/Autodesk/FBX/FBX SDK/2020.1.1"
In Bash use export: ```export FBX_DIR="C:/Program Files/Autodesk/FBX/FBX SDK/2020.1.1"```
In Win10 Powershell use $env: ```$env:FBX_DIR="C:/Program Files/Autodesk/FBX/FBX SDK/2020.1.1"```

Then configure and generate build files:
```
cmake -G "Visual Studio 16 2019" -A x64 -T v141 -DACTUAL_3RDPARTY_DIR=../../3rdParty_x64/x64 -DBUILD_OSG_EXAMPLES=true -DCMAKE_INSTALL_PREFIX=../install ..
```
Finally build:
```
cmake --build . --config Release --target install --clean-first  
```

### Get binary (pre-built) package
Windows Binaries provided by OBJEXX Engineering: https://objexx.com/OpenSceneGraph.html

Download package, e.g. [3.6.5 for Windows](https://objexx.com/OpenSceneGraph/OpenSceneGraph-3.6.5-VC2019-64-Release.7z)

Then, in order to run the applications, you need to add search paths to PATH environment variable. 
Windows example:
```
$env:PATH+="C:\eknabe1\OpenSceneGraph-3.6.5-VC2019-64-Release\bin;C:\eknabe1\OpenSceneGraph-3.6.5-VC2019-64-Release\bin\osgPlugins-3.6.5""
```
Bash example:
```
export PATH=$PATH:"C:/eknabe1/OpenSceneGraph-3.6.5-VC2019-64-Release/bin;C:/eknabe1/OpenSceneGraph-3.6.5-VC2019-64-Release/bin/osgPlugins-3.6.5"

```
Then just run, e.g:
```
osgconv road.fbx road.osgb
```
and
```
osgviewer road.osgb --window 60 60 1000 500
```
