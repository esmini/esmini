# OpenSceneGraph build instructions
## Preparations
- Check out or download latest stable OSG release from:
http://www.openscenegraph.org/index.php/download-section/stable-releases

- Download OSG 3rd party dependenciy package (small variant is enough) from:
http://www.openscenegraph.org/index.php/download-section/32-third-party.
Pick the version matching your Visual Studio version
Unpack into OpenSceneGraph root folder (same level as src directory)

- Optional: For Autodesk FBX support, download and install fbx-sdk from:
https://www.autodesk.com/developer-network/platform-technologies/fbx-sdk-2019-0. Pick the closest matching version, equal or older, to your Visual Studio version.

## Build configuration
- In OSG root folder, make a directory named "build", and open a command prompt in there and run the following:
```
cmake.exe -G "Visual Studio 15 Win64" -D DYNAMIC_OPENSCENEGRAPH=False -D BUILD_OSG_APPLICATIONS=False -D BUILD_OSG_DEPRECATED_SERIALIZERS=False -D BUILD_OSG_EXAMPLES=False -D DYNAMIC_OPENTHREADS=False -D ACTUAL_3RDPARTY_DIR="../3rdParty_x64/x64" -D CMAKE_INSTALL_PREFIX=install ..
```

## Build
- Open the OpenSceneGraph solution in Visual Studio and build INSTALL project of desired configuration, OR
build from command line:
```
msbuild /m /property:Configuration=Release OpenSceneGraph.sln
```
and/or
```
msbuild /m /property:Configuration=Debug OpenSceneGraph.sln
```
each followed by
```
msbuild INSTALL.vcxproj
```

## Make available for esmini
- Copy (or move) the install folder (should be available in the build folder after previous step) to _esmini/externals/OpenSceneGraph_ and rename it to v10.
> Note: v10 stands for Windows SDK version 10. Default is 64 bit. Replace _v10_ with _win32_ if OSG was compiled for 32 bit. Replace with _winsdk71_ if compiled for Windows SDK 7.1 (only 64 bit is supported by current esmini build configurator).

- Copy zlibstatic.lib and zlibstaticd.lib from OSG 3rdparty lib folder into the _externals/OpenSceneGraph/v10/lib_ folder

- Optional: For FBX support, copy FBX libfbxsdk-md.lib from C:\Program Files\Autodesk\FBX\FBX SDK\2019.0\lib\vs2015\x64 (or wherever located) release and debug folders into _externals/OpenSceneGraph/v10/lib_ folder. Add 'd' as in debug to the debug version of the file, i.e. libfbxsdkd-md.lib
