mkdir buildVS2019_64_v141
cd buildVS2019_64_v141
cmake.exe -G "Visual Studio 16 2019" -T v141 -A x64 -D USE_OSG=True ..
cd..