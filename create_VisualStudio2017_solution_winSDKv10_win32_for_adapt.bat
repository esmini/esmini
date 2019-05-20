mkdir buildVS15_32_adapt
cd buildVS15_32_adapt
cmake -G "Visual Studio 15" -D USE_ENVSIM_ADAPT=True -D USE_OSG=True ..
cd..