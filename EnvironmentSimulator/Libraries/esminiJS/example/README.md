# Esmini WASM Demo

This demonstrates how to run esmini in the browser using WebAssembly. The demo
features both an embedded scenario for quick testing and full file system
integration for loading your own simulation assets.

It is still a data/demo page, not the new three.js visualizer. The intended next
step is to build a dedicated web renderer on top of the live WASM API exposed by
`EnvironmentSimulator/Libraries/esminiJS`.

## Build WASM Application

To build the esmini.js WebAssembly library from source follow the instructions
in the parent directory. The build process compiles the esmini C++ engine to
JavaScript/WebAssembly, resulting in the `esmini.js` file used by this demo. 

## Quick Start

1. **Start the web server from this directory:**
   ```bash
   python3 -m http.server 8080
   ```

2. **Open in browser:**
   - Navigate to `http://localhost:8080/index.html`
   - "Run Embedded Demo Scenario" to use built-in demo content

3. **Load your own files:**
   - Select a directory containing OpenDRIVE (.xodr), OpenSCENARIO (.xosc), and optional GLB/GLTF assets
   - Choose a scenario from the dropdown
   - The viewer will resolve static scene models from `RoadNetwork/SceneGraphFile` and entity models from catalog or object `Properties/File` entries when present
   - When `RoadNetwork/SceneGraphFile` points to a GLB/GLTF road scene, the procedural OpenDRIVE road rendering is suppressed to avoid duplicate road surfaces and z-fighting

## Files in This Example

- `index.html` - Interactive web application for mounting files and exercising the WASM API
- `viewer.js` - renderer, camera, playback logic, and lane-marking mesh generation for the browser viewer
- `viewer.css` - browser viewer styling
- `esmini.js` - The compiled Emscripten runtime module
- `esmini.wasm` - Optional separate WASM binary when single-file packaging is disabled in the future

## Browser Compatibility

| Feature | Chrome | Firefox | Safari | Edge |
|---------|--------|---------|--------|------|
| **Directory Selection** | ✅ 86+ | ❌ | ❌ | ✅ 86+ |
| **File Selection (Modern)** | ✅ 86+ | ❌ | ❌ | ✅ 86+ |
| **File Selection (Legacy)** | ✅ All | ✅ All | ✅ All | ✅ All |
| **WASM/asm.js Runtime** | ✅ All | ✅ All | ✅ All | ✅ All |
