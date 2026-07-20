## esminiJS

This directory contains the standalone Emscripten/WebAssembly wrapper for esmini.

Status:

- intended as the starting point for browser integrations
- not yet a complete web visualization solution
- currently focused on simulation stepping and data access, not rendering

## Points of attention

- Scenario files and referenced assets must be mounted into the Emscripten filesystem before loading them.
- The browser build does not include the native OSG viewer path.
- The wrapper is still evolving and should be treated as a browser-facing engine API, not a finalized SDK.
- The build script applies a repo-owned compatibility patch to a copied `fmt` header tree under `build/`, so the `externals/fmt` submodule does not need to be edited locally.

## Install
Tested on Windows 10 (from CMD), macOS (Catalina) and Linux (Ubuntu 22.04, glic 2.35, gcc/g++ 11.3)

### 1. Install emscripten

* Get the emsdk repo

    ```
    git clone https://github.com/emscripten-core/emsdk.git
    cd emsdk
    ```

* Fetch the latest version of the emsdk (not needed the first time you clone)

    ```
    git pull
    ```

* Download and install the latest SDK tools

    Linux & Mac
    ```
    ./emsdk install latest
    ```
    Windows cmd
    ```
    emsdk install latest
    ```
* Make the "latest" SDK "active" for the current user. (writes .emscripten file)

    Linux & Mac
    ```
    ./emsdk activate latest
    ```
    Windows cmd
    ```
    emsdk activate latest
    ```

* Activate PATH and other environment variables in the current terminal

    Linux & Mac
    ```
    source ./emsdk_env.sh
    ```
    Windows cmd
    ```
    emsdk_env.bat
    ```

### 2. Install other system tools and libraries

Linux
```
sudo apt install cmake  bzip2
```
Windows cmd

* If not already done, install cmake (https://cmake.org/download/)
* Note: bzip2 not needed (it seems)
* Download and extract ninja binary (https://github.com/ninja-build/ninja/releases) to some folder

```
set path=%path%;<folder containing ninja executable>
```

Mac: ? (how do you install cmake and bzip2 on Mac?)

### 3. Build js

```
cd esmini/EnvironmentSimulator/Libraries/esminiJS
```

Linux & Mac
```
./build.sh
```

Windows cmd:
```
mkdir build
cd build
emcmake cmake ..
emmake ninja
copy esmini.js ..\example /y
cd ..
```

### 4. check esmini.js
* Check existence and date

    Linux & Mac
    ```
    ll example/esmini.js
    ```

    Windows cmd
    ```
    dir example\esmini.js
    ```

* Load `example/index.html` in a browser
    - The page should load the `esmini.js` runtime successfully
    - Use the demo buttons to step a scenario and inspect object state output in the page log

## Browser-facing API

The wrapper now exposes a live simulation API in addition to the older bulk snapshot helpers.

Main methods on `OpenScenario`:

- `step(dt)` advances the simulation once and returns the engine status code
- `step_frame(dt)` advances once and returns a frame object with simulation time and current object states
- `reset()` recreates the scenario engine from the original `.xosc`
- `get_road_geometry()` returns sampled lane-surface strips and road-mark polylines for browser rendering
- `get_object_states()` returns the current active objects
- `get_object_state_by_index(index)` returns one current object state
- `get_object_count()` returns the current active object count
- `get_simulation_time()` returns current simulation time in seconds
- `is_quit()` reports whether the scenario has completed
- `time_to_collision(object_a_id, object_b_id, free_space, cs, dist_type)` returns the time to collision in seconds from object A to object B, or `-1` when undefined. `cs` and `dist_type` are the integer values of `roadmanager::CoordinateSystem` and `roadmanager::RelativeDistanceType`

Configuration values exposed through embind `value_object` types are passed from JavaScript as plain object literals, for example:

```js
const scenario = new module.OpenScenario("/example.xosc", {
    max_loop: 0,
    min_time_step: 0,
    max_time_step: 0,
    dt: 0.05,
});
```

Compatibility helpers kept for the existing demo:

- `get_object_state(config)`
- `get_object_state_by_second(seconds, fps)`

These helpers still accumulate snapshots across multiple simulation steps, but new web clients should prefer the live stepping API.

The example directory now also contains a separate three.js viewer page that consumes `get_road_geometry()` and the live frame API.
