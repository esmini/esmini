# Copilot Instructions for esmini

esmini is an OpenSCENARIO XML player (v1.0–v1.3) with OpenDRIVE road network support, OSI ground truth output, and optional 3D visualization. Licensed MPL 2.0.

## Build

```bash
# Full build (downloads externals automatically)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release -j 4

# Minimal build (no OSG viewer, no SUMO, no OSI)
cmake -B build -DUSE_OSG=OFF -DUSE_SUMO=OFF -DUSE_OSI=OFF
cmake --build build -j 4

# Install to bin/ directory
cmake --build build --config Release --target install
```

Key CMake options: `USE_OSG`, `USE_OSI`, `USE_SUMO`, `USE_GTEST`, `USE_IMPLOT`, `BUILD_EXAMPLES`, `BUILD_REPLAYER`, `BUILD_ODRPLOT`, `ENABLE_SANITIZERS`, `ENABLE_COVERAGE`, `ENABLE_WARNINGS_AS_ERRORS`.

## Test

```bash
# All tests (unit + smoke)
./scripts/run_tests.sh

# Unit tests only (Google Test)
./build/EnvironmentSimulator/Unittest/esmini_test

# Single Google Test by filter
./build/EnvironmentSimulator/Unittest/esmini_test --gtest_filter="TestSuiteName.TestName"

# Smoke tests (Python, black-box)
pytest test/smoke_test.py

# Single smoke test
pytest test/smoke_test.py -k "test_name"

# ALKS / NCAP regression suites
python test/alks_suite.py
python test/ncap_suite.py

# Memory leak tests (Linux, requires valgrind)
./scripts/run_memory_leak_tests.sh
```

Smoke tests launch esmini as a subprocess, capture log/CSV/dat output, and assert on values. They need a built `bin/esmini` from `--target install`.

## Lint & Format

```bash
# Pre-commit runs all checks (clang-format, cmake-format, black, cppcheck)
pre-commit run --all-files

# Individual tools
clang-format -i <file>           # C/C++ (Google-based, Allman braces, 150 col)
cmake-format -i <file>           # CMake files (150 col)
black <file>                     # Python
cppcheck --enable=all <file>     # Static analysis
```

All PRs must pass formatting checks. Install hooks: `pre-commit install`.

## Architecture

```
EnvironmentSimulator/
├── Modules/           # Core internal modules (dependency order):
│   ├── CommonMini/    #   Utilities, logging, UDP, config parsing
│   ├── RoadManager/   #   OpenDRIVE parsing, lane queries, routing
│   ├── Controllers/   #   15+ vehicle controllers (ACC, ALKS, interactive, SUMO, etc.)
│   ├── ScenarioEngine/#   OpenSCENARIO parsing + execution, OSI reporter
│   ├── PlayerBase/    #   Simulation loop, server, plotting
│   └── ViewerBase/    #   3D visualization (requires OSG)
├── Libraries/         # Public APIs:
│   ├── esminiLib/     #   Full scenario engine API (links all modules)
│   ├── esminiRMLib/   #   Standalone road network API (RoadManager only)
│   └── esminiJS/      #   WebAssembly bindings
├── Applications/      # esmini, esmini-dyn, replayer, odrviewer, odrplot
├── Unittest/          # Google Test unit tests
└── code-examples/     # 17 API usage examples
```

**Module dependency chain:** CommonMini → RoadManager → Controllers → ScenarioEngine → PlayerBase → ViewerBase → esminiLib

**OSMP_FMU/** wraps esmini as an FMI 2.0 Functional Mock-up Unit for co-simulation, accepting OSI TrafficUpdate input and producing OSI SensorView output.

## Conventions

### Branching
- **`master`** — stable releases (tagged `v*.*.*`)
- **`dev`** — integration branch; **PRs target `dev`**, not master
- **`feature/**`** — feature branches

### C++ Style
- Allman braces (opening brace on new line)
- 4-space indentation, 150-character line limit
- Naming: `CamelCase` for classes/functions, `lower_case` for parameters, `lower_case_` with trailing underscore for private/protected members
- IWYU (include-what-you-use) is enforced by default

### ScenarioEngine Internals
- `ScenarioEngine/OSCTypeDefs/` — OpenSCENARIO XML type definitions (conditions, actions, positions)
- `ScenarioEngine/SourceFiles/` — Core execution (Storyboard hierarchy: Story → Act → ManeuverGroup → Maneuver → Event)
- `OSIReporter` handles all OSI ground truth generation
- Controllers are registered via a factory pattern in `Controllers/`

### Test Scenarios
- Road networks: `resources/xodr/` (18 OpenDRIVE maps)
- Scenarios: `resources/xosc/` (80+ OpenSCENARIO files)
- Test-specific scenarios: `EnvironmentSimulator/Unittest/xosc/` and `xodr/`
- Scenario run scripts: `run/esmini/*.bat`

### Python Dependencies
Install from `support/python/requirements.txt`: `pip install -r support/python/requirements.txt`

### External Dependencies
Set `DOWNLOAD_EXTERNALS=ON` (default) to auto-download OSG, OSI, SUMO, ImPlot, GTest, and vehicle/road models. Bundled lightweight deps live in `externals/` (pugixml, fmt, fmi2, dirent, expr, yaml).
