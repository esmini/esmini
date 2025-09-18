## Introduction

Generates a C# wrapper for esminiRMLib based on its ANSI C header.

## Simple build for local use

`dotnet build -c Release`

The executable ends up in:

`.\bin\Release\net8.0\win-x64\Cpp2CSWrapperCreator.exe`

Run without argument for usage info.

## Build for publish
To build the distribution files:

For self-contained:
```powershell
dotnet publish -c Release -r win-x64 --self-contained -p:PublishSingleFile=true -o publish
```

For framework-dependent:
```powershell
dotnet publish -c Release -r win-x64 -p:PublishSingleFile=true -p:SelfContained=false -o publish-framework-dependent
```

### 1. Self-Contained Executable (Recommended for most users)
- **Location**: `publish\Cpp2CSWrapperCreator.exe`
- **Size**: ~188 MB
- **Requirements**: None - includes .NET runtime
- **Pros**:
  - Users don't need .NET installed
  - Single file distribution
  - Works on any Windows x64 machine
- **Cons**:
  - Larger file size

### 2. Framework-Dependent Executable
- **Location**: `publish-framework-dependent\Cpp2CSWrapperCreator.exe`
- **Size**: ~121 MB
- **Requirements**: .NET 8.0 Runtime must be installed
- **Pros**:
  - Smaller file size
  - Better for environments where .NET is already available
- **Cons**:
  - Requires users to install .NET 8.0 Runtime

## How to Share

### For Self-Contained Version (Easiest for users):
1. Navigate to the `publish` folder
2. Share the `Cpp2CSWrapperCreator.exe` file
3. Users can run it directly without any additional setup

### For Framework-Dependent Version:
1. Navigate to the `publish-framework-dependent` folder
2. Share the `Cpp2CSWrapperCreator.exe` file
3. Provide users with .NET 8.0 Runtime download link: https://dotnet.microsoft.com/download/dotnet/8.0

## Usage
Both versions work the same way:
```
Cpp2CSWrapperCreator.exe <input.h> <output.cs>
```

## Additional Notes
- Both executables include the ClangSharp native libraries needed for C++ parsing
- The applications are compiled for Windows x64 architecture
- The .pdb files are debug symbols and are not required for distribution
