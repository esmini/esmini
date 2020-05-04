mkdir buildVS2019_64
cd buildVS2019_64
cmake.exe -G "Visual Studio 16 2019" -A x64 -D USE_OSG=True ..
cd..