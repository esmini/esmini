# esmini – Copilot instructions

esmini is a C++ OpenSCENARIO (OSC) player and OpenDRIVE (ODR) road manager. User guide / build guide: https://esmini.github.io/

## Build

CMake from the repo root, out-of-tree into `build/`, installing into `bin/`. The `install` target is needed for smoke tests, which expects binaries in `bin/`, and full test runs (`./scripts/run_tests.sh`), since it includes smoke tests.

```
cmake . -B build -DCMAKE_BUILD_TYPE=Release            # Linux/macOS: add -G Ninja if available
cmake . -B build -G "Visual Studio 17 2022" -T v142 -A x64  # Windows
cmake --build build --config Release --target install -j 4  # Full build and install into bin/
```

- Optional features via flags: `USE_OSG` (viewer), `USE_OSI`, `USE_SUMO`, `USE_GTEST` (unit tests), `USE_IMPLOT`, `USE_PROJ`. Prebuilt 3rd-party libs are fetched/used from `externals/` (OSG, OSI, gtest, etc.). Submodules (`externals/fmt`, `externals/fmi2`, `test/OSC-ALKS-scenarios`, `test/OSC-NCAP-scenarios`, `test/ASAM-OSC-XML-scenarios`) must be initialized for full builds/tests.
- CI configures with `-D ENABLE_WARNINGS_AS_ERRORS=ON -D ENABLE_INCLUDE_WHAT_YOU_USE=ON`; new code must be warning-free.

Individual applications, libs and unit-test suites can be built separately using the `--target` option with `cmake --build`. Examples:
```
cmake --build build --config Release --target esmini -j 4
cmake --build build --config Release --target ScenarioEngine_test -j 4
```

## Run

To run an application, use the installed binary from `bin/`:
```
bin/esmini <options> --osc <scenario_file>
```

For quicker iterations, skip building all targets (install) and run the application binary from its original location under the `build/` directory. The path depends on the generator:
```
# Linux/macOS (single-config generator, e.g. Ninja)
build/EnvironmentSimulator/Applications/esmini/esmini --window 60 60 800 400 --osc ./resources/xosc/cut-in.xosc

# Windows (Visual Studio; replace <config> with Release or Debug)
build/EnvironmentSimulator/Applications/esmini/<config>/esmini.exe --window 60 60 800 400 --osc ./resources/xosc/cut-in.xosc
```

## Tests

Two independent suites (see `scripts/run_tests.sh`, which CI runs; run from repo root, e.g. `./scripts/run_tests.sh -b Release [-p] [-u] [-w] [-s]`):

1. **Unit tests (gtest, white-box)** – `EnvironmentSimulator/Unittest/*_test.cpp`, built into `build/EnvironmentSimulator/Unittest/` (needs `USE_GTEST=ON`). Run a single test from that folder:
   `./ScenarioEngine_test --gtest_filter=SuiteName.TestName --disable_stdout` (on Windows: `Release\ScenarioEngine_test.exe`). Binaries: `OperatingSystem_test`, `CommonMini_test`, `RoadManager_test`, `ScenarioEngine_test`, `ScenarioPlayer_test`, `RoadManagerDll_test`, `ScenarioEngineDll_test`, `FollowRoute*_test`.
2. **Smoke tests (Python, black-box)** – `test/smoke_test.py` (plus `alks_suite.py`, `ncap_suite.py`, `asam_osc_examples_suite.py`, `osi_udp_test.py`, `performance_test.py`, etc.). They run the installed `bin/` executables on scenarios in `resources/xosc` and `EnvironmentSimulator/Unittest/xosc|xodr`. Run from `test/`:
   `python smoke_test.py TestSuite.test_cut_in` (standard `unittest` selection). Helpers in `test/test_common.py`.

## Formatting / lint (enforced in CI via pre-commit)

### For iterative development, only consider clang formatting:

`pre-commit run clang-format --files <file ...>` # single file(s)

Example:
```
pre-commit run clang-format --files ./EnvironmentSimulator/Modules/ScenarioEngine/SourceFiles/ScenarioEngine.cpp
```
### Full formatting, e.g. preparing for commits or pull requests

`pre-commit run <hook> --all-files` with hooks: `clang-format` (via `support/script/clang-format-runner.sh`, only `EnvironmentSimulator/`, excludes `externals/`), `cmake-format`, `cppcheck` (pinned 2.17.1 in CI), `black-format` (line length 88; excludes `EnvironmentSimulator/`, `scripts/`, `test/`, `externals/`). Config: `.clang-format`, `.clang-tidy`, `.cmake-format`, `.black-format`.

## Architecture

Source lives in `EnvironmentSimulator/`:

- `Modules/` – core libraries, layered roughly as:
  - `CommonMini` – utilities (math, OS abstraction, logging, string/time helpers).
  - `RoadManager` – OpenDRIVE parsing, road geometry, positions (`roadmanager::Position`), routes, junctions, signals.
  - `ScenarioEngine` – OpenSCENARIO parsing (`OSCTypeDefs`) and execution (`SourceFiles`): storyboard, entities/objects, actions, conditions, triggers, catalogs, parameters/variables.
  - `Controllers` – pluggable vehicle controllers (follow route, interactive, ghost, ACC, etc.).
  - `PlayerBase` – `ScenarioPlayer` ties engine, controllers, logging/replay, and optional viewer/OSI/UDP together.
  - `ViewerBase` – OpenSceneGraph-based visualization (only with `USE_OSG`), `viewer`, `roadgeom` (road geometry) etc.
- `Libraries/` – shared-library APIs built on the modules: `esminiLib` (C API for the scenario player, used by wrappers/co-simulation), `esminiRMLib` (road-manager-only C API), plus `esminiJS`, `esminiROS`.
- `Applications/` – executables: `esmini` (player), `esmini-dyn`, `odrviewer`, `odrplot`, `replayer`.
- `Unittest/` – gtest sources and test scenario/road resources (`xosc/`, `xodr/`).

Other top-level dirs: `OSMP_FMU/` (OSI/OSMP FMU build), `resources/` (xosc/xodr examples, models), `scripts/` (test runners, sanitizer suppressions, plotting tools), `support/` (OSI patches, UDP driver, Python requirements), `Hello-World_coding-example/` and `EnvironmentSimulator/Applications` code examples, `docs/`.

## Standards (OpenSCENARIO XML / OpenDRIVE)

For questions about OSC/ODR semantics, attributes, versions or spec compliance, use the `openx-standards` skill (`.github/skills/openx-standards/SKILL.md`). It gives a low-cost lookup order (local XSDs in `resources/schema`, `osc_coverage.txt`/`odr_coverage.txt`, then targeted spec pages) and maps spec concepts to esmini code. Avoid fetching whole spec pages.

## Conventions

- New or updated scenario behavior is normally covered both by a gtest in `Unittest/` and by a smoke test in `test/smoke_test.py` using a scenario in `resources/xosc` or `EnvironmentSimulator/Unittest/xosc`.
- Pull requests target the `dev` branch (not `master`); CI runs formatting, static analysis, and all test suites on Windows/Linux/macOS.
- Public API changes in `esminiLib`/`esminiRMLib` need matching updates to the C#/Python wrappers and tests (`test/CSharpWrappers`, `*Dll_test.cpp`).
- When adding features, add scenario/road test files under `EnvironmentSimulator/Unittest/{xosc,xodr}` or `resources/` and cover them in the gtest and/or `smoke_test.py` suites; update `release_notes.md` for user-visible changes.
- esmini license applies to all contributions.
