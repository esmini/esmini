@rem Run from esmini root folder: .\scripts\create_win_VisualStudio_solution.bat

mkdir build
cd build
cmake.exe -G "Visual Studio 16 2019" -T v141 -A x64 -D USE_OSG=True ..
cd..