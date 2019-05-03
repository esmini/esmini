mkdir build
cd build
rem cmake -G "Visual Studio 10 2010 Win64" -T "Windows7.1SDK" -D USE_ENVSIM_MINI=True -D USE_OSG=True ..
cmake -G "Visual Studio 15 2017 Win64" -T "Windows7.1SDK" -D WINSDK7_1=True -D USE_OSG=True ..
cd..
