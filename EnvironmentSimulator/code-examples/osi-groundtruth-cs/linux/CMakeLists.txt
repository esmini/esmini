cmake_minimum_required(VERSION 3.15)
project(
    osi-groundtruth
    NONE) # Csharp not supported in Linux, keep NONE

# Define the path to your .csproj file
set(TARGET
    osi-groundtruth)
set(CSPROJ_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/OsiGroundTruth.csproj")

# Tell CMake to compile the C# project using the dotnet CLI
add_custom_target(
    ${TARGET}
    ALL
    COMMAND dotnet build "${CSPROJ_FILE}" -c $<IF:$<CONFIG:>,Release,$<CONFIG>> -p:AssemblyName=${TARGET}
    COMMENT "Compiling C# project with .NET SDK..."
    VERBATIM)
