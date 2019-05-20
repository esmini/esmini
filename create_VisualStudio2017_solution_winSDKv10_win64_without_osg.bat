mkdir buildVS15_64_no_OSG
cd buildVS15_64_no_OSG
cmake -G "Visual Studio 15 Win64" -D USE_OSG=False ..
cd..