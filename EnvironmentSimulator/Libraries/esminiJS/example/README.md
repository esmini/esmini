# Esmini WASM Demo

This demonstrates how to run esmini in the browser using WebAssembly. The demo
features both an embedded scenario for quick testing and full file system
integration for loading your own simulation assets.

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
   - Select a directory containing OpenDRIVE (.xodr) and OpenSCENARIO (.xosc) files
   - Choose a scenario from the dropdown
   - Click "Run Loaded Scenario" to simulate

## Files in This Example

- `index.html` - Complete interactive demo with file loading, embedded scenarios, and comprehensive UI
- `esmini.js` - The compiled WASM/asm.js esmini library

## Browser Compatibility

| Feature | Chrome | Firefox | Safari | Edge |
|---------|--------|---------|--------|------|
| **Directory Selection** | ✅ 86+ | ❌ | ❌ | ✅ 86+ |
| **File Selection (Modern)** | ✅ 86+ | ❌ | ❌ | ✅ 86+ |
| **File Selection (Legacy)** | ✅ All | ✅ All | ✅ All | ✅ All |
| **WASM/asm.js Runtime** | ✅ All | ✅ All | ✅ All | ✅ All |
