# OpenSceneGraph build instructions

## OSG code
Either check out
```
git clone https://github.com/openscenegraph/OpenSceneGraph
cd OpenSceneGraph
git checkout OpenSceneGraph-3.6.5
```
or download latest stable release from:
http://www.openscenegraph.org/index.php/download-section/stable-releases

esmini is designed to link all dependencies statically. Main reason is to have a all-inclusive library for easy integration either as a shared library/DLL (e.g. plugin in Unity, or S-function in Simulink) or statically linked into a native application.
> Nothing stops you from going with all dynamic linking, it's just that provided build scripts are not prepared for it.

## Dependencies
See this page for information: http://www.openscenegraph.org/index.php/download-section/32-third-party
- Windows: Download pre-compiled package (small variant is enough) from page above
- For Linux and Mac, see info and links on page above. Probably only zlib and jpeglib are needed, see hints below.
- Optional: For Autodesk FBX support, download and install fbx-sdk from:
https://www.autodesk.com/products/fbx/overview. Pick the closest matching version, equal or older, to your Visual Studio version.

### Some hints for Linux and Mac:
- If zlib.a needed (not needed on Mac, since it's embedded in macOS), more info: http://www.linuxfromscratch.org/lfs/view/6.6/chapter06/zlib.html  
  quick guide: 
  * download [zlib](https://www.zlib.net/) and unzip it into OpenSceneGraph/3rd_party/zlib
  * ```mkdir build; cd Build  ```
  
  For Release config (recommended if you don't need to debug)
  * ```cmake .. -DCMAKE_C_FLAGS=-fPIC -DCMAKE_BUILD_TYPE=Release```
  * ```cmake --build . --config Release```
  
  For Debug config (recommended if you need to debug):
  * ```cmake .. -DCMAKE_C_FLAGS=-fPIC -DCMAKE_BUILD_TYPE=Debug```
  * ```cmake --build . --config Debug```  

  Put the library, zlib.a, in a folder named 3rd_party next to OpenSceneGraph 

- libjpeg (used for screenshots), download http://www.ijg.org/files/jpegsrc.v8d.tar.gz  
  - unpack: ```tar xvf jpegsrc.v8d.tar.gz``` from OpenSceneGraph/3rd_party so it ends up in OpenSceneGraph/3rd_party/jpeg-8d
  - cd jpeg-8d  
  Mac: 
  - ```./configure, make```  
  Linux: 
  - ```./configure CFLAGS='-fPIC', (and/or CFLAGS='-fPIC -g' for debug info); make```   

  The libjpeg.a should end up in .libs folder. Copy 

- On Mac: Currently (Catalina March 2020) there is a problem of rendered image only filling lower left quarter of the window. To avoid it: In OpenSceneGraph/src/osgViewer/GraphicsWindowCocoa.mm,
under the block  
`[_view setAutoresizingMask:  (NSViewWidthSizable | NSViewHeightSizable) ];`  
`[_view setGraphicsWindowCocoa: this];`  
`[_view setOpenGLContext:_context];`  
add the line  
`[_view setWantsBestResolutionOpenGLSurface: NO];`  
Solution found [here](https://github.com/openscenegraph/OpenSceneGraph/issues/926#issuecomment-600080664)

## Build configuration
To build OSG libraries for static linking in esmini, see following examples. All examples assumes you first have created a directory "build" directly under OSG root and moved into it.

- Visual Studio default version (whatever is installed on your system)  
`cmake ../ -DDYNAMIC_OPENSCENEGRAPH=false -DDYNAMIC_OPENTHREADS=false -DCMAKE_INSTALL_PREFIX=../install -DACTUAL_3RDPARTY_DIR=../3rdParty_x64/x64`

    If you have multiple Visual Studio SDK versions installed you can select with -G:

- Visual Studio 15 (2017) / win64  
`cmake -G "Visual Studio 15 Win64" ../ -DDYNAMIC_OPENSCENEGRAPH=false -DDYNAMIC_OPENTHREADS=false -DCMAKE_INSTALL_PREFIX=../install -DACTUAL_3RDPARTY_DIR=../3rdParty_x64/x64`

- Visual Studio 15 (2017) / win32  
`cmake -G "Visual Studio 15" ../ -DDYNAMIC_OPENSCENEGRAPH=false -DDYNAMIC_OPENTHREADS=false -DCMAKE_INSTALL_PREFIX=../install -DACTUAL_3RDPARTY_DIR=../3rdParty_x32/x32`

- Visual Studio 15 (2017) / Win SDK 7.1  
`cmake -G "Visual Studio 15 2017 Win64" -T "Windows7.1SDK" ../ -DDYNAMIC_OPENSCENEGRAPH=false -DDYNAMIC_OPENTHREADS=false -DCMAKE_INSTALL_PREFIX=../install -DACTUAL_3RDPARTY_DIR=../3rdParty_x64/x64`

- Mac / Xcode  
`cmake ../ -DDYNAMIC_OPENSCENEGRAPH=false -DDYNAMIC_OPENTHREADS=false -DJPEG_LIBRARY_RELEASE=../3rd_party/jpeg-8d/.libs/libjpeg.a` -DJPEG_INCLUDE_DIR../3rd_party/jpeg-8d

- Linux / Ubuntu  
`cmake ../ -DDYNAMIC_OPENSCENEGRAPH=false -DDYNAMIC_OPENTHREADS=false -DCMAKE_CXX_FLAGS=-fPIC -DCMAKE_INSTALL_PREFIX=../install`  
For debug info, add -DCMAKE_BUILD_TYPE=Debug
> PIC (Position Independent Code) is needed for static linking

### Compile
All examples assumes you're located in the "build" directory.

Generic way using cmake:  
`cmake --build . --target install --config Release`

Specific ways using compiler toolkit:
- Windows: `msbuild /m /property:Configuration=Release OpenSceneGraph.sln` followed by `msbuild INSTALL.vcxproj`
- Linux/macOS: `make -j4 install`

To build the complete set of OSG applications (e.g. osgview and osgconv) you need to build OSG libraries for dynamic linking. cmake examples for Visual Studio on Windows:

`cmake ../ -DCMAKE_INSTALL_PREFIX=../install -DACTUAL_3RDPARTY_DIR=../../3rdParty_x64/x64 -DBUILD_OSG_EXAMPLES=true`  
and  
`cmake --build . --target install --config Release`

## Make available to esmini
- Copy (or move) the install folder (should be available in the build folder after previous step) into _esmini/externals/OpenSceneGraph_ and rename it to _v10_, _win32_, _winsdk71_, _mac_ or _linux_ respectively.

- Copy _zlibstatic.lib_, _zlibstaticd.lib_, _jpeg.lib_ and _jpegd.lib_ from OSG 3rdparty lib folder, or other source, into the _externals/OpenSceneGraph/*/lib_ folder

- Optional: For FBX support, copy FBX _libfbxsdk-md.lib_ from C:\Program Files\Autodesk\FBX\FBX SDK\2019.0\lib\vs2015\x64 (or wherever located) release and debug folders into _externals/OpenSceneGraph/*/lib_ folder. Add 'd' as in debug to the debug version of the file, i.e. libfbxsdkd-md.lib
