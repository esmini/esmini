mkdir build
cd build
cmake -G "Visual Studio 15" -D USE_ENVSIM_ADAPT=True -D USE_OSG=True ..
cd..