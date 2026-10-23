# esmini WASM web app

This application is published at <https://esmini.github.io/esmini/>. Its HTML,
JavaScript, styles, and bundled sample assets live in this directory. The
Emscripten wrapper lives in `EnvironmentSimulator/Libraries/esminiJS`; its
build script places the generated runtime here so the application stays
self-contained without duplicating the WASM build logic.

## Publish flow

The `Publish esmini WASM app` workflow runs for relevant changes on `master` or
when manually dispatched on `master`. It builds `esmini.js` with Emscripten,
uploads the application directory as a GitHub Pages artifact, and deploys it
from this repository. Generated app files are not committed to either the
esmini repository or the documentation repository, so repeated publishes do
not grow their Git histories. This is a separate project site at
`https://esmini.github.io/esmini/`; alongside the existing documentation site
at `https://esmini.github.io/`.

In the esmini repository settings, enable Pages and select **GitHub Actions**
as the build and deployment source. No cross-repository token is needed. The
workflow deploys the contents of this application directory to the project
site root.

The runtime is built with Emscripten's single-file WASM packaging, so `esmini.js`
contains the WASM payload and is copied with the static viewer files. Three.js
is loaded from unpkg as configured in the viewer's import map.

## Local preview

Build esminiJS using the instructions in
`EnvironmentSimulator/Libraries/esminiJS/README.md`, then serve this directory
over HTTP. From the repository root, run
`python3 -m http.server 8080 --directory EnvironmentSimulator/Applications/esmini-wasm`
and open <http://localhost:8080/>.