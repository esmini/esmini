mkdir build
cd build
cmake -G "Visual Studio 15 2017 Win64" -T "Windows7.1SDK" -D WINSDK7_1=True -D USE_OSG=True ..
cd..
