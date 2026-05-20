import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { clone as cloneSkinned } from "three/addons/utils/SkeletonUtils.js";

const RESOURCE_ROOT_URL = new URL("./resources/", import.meta.url);
const EXAMPLE_XODR_URL = new URL("xodr/example.xodr", RESOURCE_ROOT_URL).href;
const EXAMPLE_XOSC_URL = new URL("xosc/example.xosc", RESOURCE_ROOT_URL).href;
const EXAMPLE_SCENARIO_DIR = "/scenario";
const EXAMPLE_CATALOGS_URL = new URL("catalogs/", RESOURCE_ROOT_URL);
const EXAMPLE_XOSC_VIRTUAL_PATH = `${EXAMPLE_SCENARIO_DIR}/xosc/example.xosc`;
const EXAMPLE_XODR_VIRTUAL_PATH = `${EXAMPLE_SCENARIO_DIR}/xodr/example.xodr`;
const SCENARIO_ASSET_URL_SCHEME = "esmini://scenario/";
const ROAD_MARK_Z_OFFSET = 0.035;
const MAX_CONSOLE_LINES = 400;
const MODEL_UP_AXIS_ROTATION_X = Math.PI / 2;
const SCENARIO_FILE_PATTERN = /\.(xosc|xodr|gltf|glb|osgb|png|jpg|jpeg|bin)$/i;
const VECTOR_OVERLAY_Z_OFFSET = 0.12;

const VECTOR_OVERLAY_COLORS = {
  referenceLine: "#ffbf1f",
  centerLine: "#1fb6ff",
  boundary: "#f4f7fb",
  steeringTarget: "#2ce67d",
  trailTarget: "#ff5a5f",
  signalBox: "#ff6b4a",
  objectBox: "#ffd166",
  entityBox: "#466b89",
};

const ENTITY_BOUNDING_BOX_GEOMETRY = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));

const CONTROLLER_TYPE = {
  EXTERNAL: 1,
  FOLLOW_GHOST: 2,
};

const LANE_TYPE = {
    DRIVING: 1 << 1,
    STOP: 1 << 2,
    SHOULDER: 1 << 3,
    BIKING: 1 << 4,
    SIDEWALK: 1 << 5,
    BORDER: 1 << 6,
    PARKING: 1 << 8,
    MEDIAN: 1 << 10,
};

  const BUTTON_LABELS = {
    play: "▶",
    pause: "⏸",
    step: "⏭",
    stop: "⏹",
  };

const ui = {
    viewport: document.getElementById("viewport"),
  consoleToggleButton: document.getElementById("consoleToggleButton"),
  consolePanel: document.getElementById("consolePanel"),
  consoleOutput: document.getElementById("consoleOutput"),
  clearConsoleButton: document.getElementById("clearConsoleButton"),
  closeConsoleButton: document.getElementById("closeConsoleButton"),
    helpButton: document.getElementById("helpButton"),
    infoPanel: document.getElementById("infoPanel"),
  closeInfoButton: document.getElementById("closeInfoButton"),
  selectDirectoryButton: document.getElementById("selectDirectoryButton"),
  directorySupport: document.getElementById("directorySupport"),
  scenarioSelect: document.getElementById("scenarioSelect"),
    playPauseButton: document.getElementById("playPauseButton"),
    stepButton: document.getElementById("stepButton"),
    resetButton: document.getElementById("resetButton"),
  followEgoToggle: document.getElementById("followEgoToggle"),
  vectorOverlayToggle: document.getElementById("vectorOverlayToggle"),
    scenarioOverlayToggle: document.getElementById("scenarioOverlayToggle"),
    speedSelect: document.getElementById("speedSelect"),
    hudText: document.getElementById("hudText"),
    statusPill: document.getElementById("statusPill"),
    objectCountStat: document.getElementById("objectCountStat"),
    laneCountStat: document.getElementById("laneCountStat"),
    markCountStat: document.getElementById("markCountStat"),
};

  const CAMERA_NEAR_METERS = 0.1;
  const CAMERA_FAR_METERS = 30000;
  const FOG_NEAR_METERS = 8000;
  const FOG_FAR_METERS = 20000;
    const SKY_COLOR = "#cadcee";
    const FOG_COLOR = "#d4e1ef";

  const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance", logarithmicDepthBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.outputColorSpace = THREE.SRGBColorSpace;
ui.viewport.appendChild(renderer.domElement);

const scene = new THREE.Scene();
  scene.background = new THREE.Color(SKY_COLOR);
    scene.fog = new THREE.Fog(FOG_COLOR, FOG_NEAR_METERS, FOG_FAR_METERS);

  const camera = new THREE.PerspectiveCamera(54, window.innerWidth / window.innerHeight, CAMERA_NEAR_METERS, CAMERA_FAR_METERS);
camera.up.set(0, 0, 1);
  camera.position.set(-20, -18, 12);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
  controls.target.set(40, 0, 0);

const ambientLight = new THREE.HemisphereLight(0xe7f1ff, 0x8a846f, 1.35);
const keyLight = new THREE.DirectionalLight(0xfff4da, 1.6);
keyLight.position.set(120, -80, 150);
scene.add(ambientLight, keyLight);

const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(40000, 40000),
    new THREE.MeshStandardMaterial({ color: "#c8c0aa", roughness: 1.0, metalness: 0.0 })
);
ground.position.z = -0.05;
scene.add(ground);

const roadGroup = new THREE.Group();
const markGroup = new THREE.Group();
const roadObjectGroup = new THREE.Group();
const staticModelGroup = new THREE.Group();
const entityGroup = new THREE.Group();
const vectorOverlayGroup = new THREE.Group();
const vectorBoundaryGroup = new THREE.Group();
const vectorCenterGroup = new THREE.Group();
const vectorFeatureBoxGroup = new THREE.Group();
const vectorFollowGroup = new THREE.Group();
vectorOverlayGroup.add(vectorBoundaryGroup, vectorCenterGroup, vectorFeatureBoxGroup);
scene.add(roadGroup, markGroup, roadObjectGroup, staticModelGroup, entityGroup, vectorOverlayGroup, vectorFollowGroup);

function setProceduralRoadVisibility(visible) {
  roadGroup.visible = visible;
  markGroup.visible = visible;
  roadObjectGroup.visible = visible;
}

function setProceduralRoadMode(enabled) {
  proceduralRoadRenderingEnabled = enabled;
  setProceduralRoadVisibility(enabled);
}

function setVectorOverlayVisibility(visible) {
  vectorOverlayGroup.visible = visible;
}

function setScenarioOverlayVisibility(visible) {
  vectorFollowGroup.visible = visible;

  for (const group of entityMeshes.values()) {
    if (group.userData.boundingBoxOverlay) {
      group.userData.boundingBoxOverlay.visible = visible;
    }
  }
}

let esminiModule = null;
let scenario = null;
let isPlaying = false;
let followEgo = false;
let playbackSpeed = 1.0;
let accumulator = 0;
const fixedStep = 1 / 60;
const esminiRuntimeUrl = "./esmini.js?v=road-geometry-api-5";
let previousTimestamp = 0;
const entityMeshes = new Map();
let isInfoPanelOpen = false;
let isConsoleOpen = false;
let selectedScenarioPath = "";
let loadedScenarioFiles = new Map();
let directoryPickerSupported = false;
let scenarioVisualLoadToken = 0;
let proceduralRoadRenderingEnabled = true;
let vectorOverlayEnabled = false;
let scenarioOverlayEnabled = false;
const consoleLines = [];
const entityModelDefinitions = new Map();
const scenarioAssetUrls = new Map();
const modelTemplateCache = new Map();
const loggedModelFailures = new Set();
const vectorFollowHelpers = new Map();
const laneCenterGeometries = new Map();
const INITIAL_CAMERA_POSITION = new THREE.Vector3(-20, -18, 12);
const INITIAL_CAMERA_TARGET = new THREE.Vector3(40, 0, 0);

setProceduralRoadVisibility(true);
setVectorOverlayVisibility(false);
setScenarioOverlayVisibility(false);

function setStatus(label, kind = "ok") {
    ui.statusPill.textContent = label;
    ui.statusPill.className = kind === "error" ? "status-pill error" : "status-pill";
}

function syncPlaybackUi(updateStatus = true) {
  ui.playPauseButton.textContent = isPlaying ? BUTTON_LABELS.pause : BUTTON_LABELS.play;
  ui.playPauseButton.title = isPlaying ? "Pause" : "Play";
  ui.playPauseButton.setAttribute("aria-label", isPlaying ? "Pause" : "Play");

  if (!updateStatus || !scenario) {
    return;
  }

  setStatus(isPlaying ? "Running" : "Paused");
}

function setInfoPanelOpen(open) {
    isInfoPanelOpen = open;
    ui.helpButton.setAttribute("aria-expanded", String(open));
  ui.helpButton.classList.toggle("hidden", open);
    ui.infoPanel.hidden = !open;
}

function setConsoleOpen(open) {
    isConsoleOpen = open;
    ui.consoleToggleButton.setAttribute("aria-expanded", String(open));
  ui.consoleToggleButton.classList.toggle("hidden", open);
    ui.consolePanel.hidden = !open;
}

function appendConsoleLine(kind, message) {
  if (!message) {
    return;
  }

  const timestamp = new Date().toLocaleTimeString();
  const tag = String(kind || "info").toUpperCase().padEnd(6, " ");
  consoleLines.push(`${timestamp} ${tag} ${message}`);
  if (consoleLines.length > MAX_CONSOLE_LINES) {
    consoleLines.splice(0, consoleLines.length - MAX_CONSOLE_LINES);
  }

  ui.consoleOutput.textContent = consoleLines.join("\n");
  ui.consoleOutput.scrollTop = ui.consoleOutput.scrollHeight;
}

function logMessage(message, kind = "info") {
  if (kind === "error") {
    console.error(message);
  } else if (kind === "warn") {
    console.warn(message);
  } else {
    console.log(message);
  }

  appendConsoleLine(kind, message);
}

function updateDirectorySupportUi() {
  directoryPickerSupported = typeof window.showDirectoryPicker === "function";
  ui.selectDirectoryButton.disabled = !directoryPickerSupported;
  ui.directorySupport.textContent = directoryPickerSupported
    ? "Load a parent directory containing .xodr, .xosc, and optional .glb/.gltf scenario assets. Subdirectories like xodr/, xosc/, catalogs/, and models/ are supported."
    : "Directory loading requires a browser with directory level file system access API support.";
  ui.directorySupport.classList.toggle("hidden", false);
}

function setInitialCameraPose() {
  camera.position.copy(INITIAL_CAMERA_POSITION);
  controls.target.copy(INITIAL_CAMERA_TARGET);
  controls.update();
}

function vectorToArray(vector) {
    const items = [];
    for (let index = 0; index < vector.size(); index += 1) {
        items.push(vector.get(index));
    }
    return items;
}

async function ensureEsminiModule() {
    if (esminiModule) {
        return esminiModule;
    }

    await new Promise((resolve, reject) => {
        const script = document.createElement("script");
      script.src = esminiRuntimeUrl;
        script.async = true;
        script.onload = resolve;
        script.onerror = () => reject(new Error("Failed to load esmini.js. Build the WASM target first."));
        document.head.appendChild(script);
    });

    esminiModule = await window.esmini({
      noInitialRun: true,
      print: (text) => appendConsoleLine("esmini", text),
      printErr: (text) => appendConsoleLine("error", text),
    });
    logMessage("esmini WASM runtime ready.", "info");
    return esminiModule;
}

async function fetchTextAsset(url) {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Failed to load ${url}: ${response.status}`);
  }

  return response.text();
}

function normalizeVirtualPath(path) {
  const segments = [];

  for (const segment of String(path).split("/")) {
    if (!segment || segment === ".") {
      continue;
    }

    if (segment === "..") {
      segments.pop();
      continue;
    }

    segments.push(segment);
  }

  return `/${segments.join("/")}`;
}

function dirname(path) {
  const normalized = normalizeVirtualPath(path);
  const lastSlashIndex = normalized.lastIndexOf("/");

  return lastSlashIndex <= 0 ? "/" : normalized.slice(0, lastSlashIndex);
}

function fileName(path) {
  const normalized = normalizeVirtualPath(path);
  const lastSlashIndex = normalized.lastIndexOf("/");

  return lastSlashIndex < 0 ? normalized : normalized.slice(lastSlashIndex + 1);
}

function resolveVirtualPath(basePath, relativePath) {
  return normalizeVirtualPath(`${dirname(basePath)}/${relativePath}`);
}

function normalizeScenarioAssetPath(path) {
  return normalizeVirtualPath(path).slice(1);
}

function resolveScenarioAssetPath(basePath, relativePath) {
  return resolveVirtualPath(`/${normalizeScenarioAssetPath(basePath)}`, relativePath).slice(1);
}

function getScenarioAssetMimeType(path) {
  const normalized = normalizeScenarioAssetPath(path).toLowerCase();

  if (normalized.endsWith(".glb")) {
    return "model/gltf-binary";
  }
  if (normalized.endsWith(".gltf")) {
    return "model/gltf+json";
  }
  if (normalized.endsWith(".bin")) {
    return "application/octet-stream";
  }
  if (normalized.endsWith(".png")) {
    return "image/png";
  }
  if (normalized.endsWith(".jpg") || normalized.endsWith(".jpeg")) {
    return "image/jpeg";
  }

  return "application/octet-stream";
}

function getLoadedScenarioText(path) {
  const content = loadedScenarioFiles.get(normalizeScenarioAssetPath(path));
  return typeof content === "string" ? content : null;
}

function getLoadedScenarioContent(path) {
  return loadedScenarioFiles.get(normalizeScenarioAssetPath(path)) ?? null;
}

function findExistingScenarioAssetPath(candidates) {
  for (const candidate of candidates) {
    const normalizedCandidate = normalizeScenarioAssetPath(candidate);
    if (loadedScenarioFiles.has(normalizedCandidate)) {
      return normalizedCandidate;
    }
  }

  return "";
}

function locateScenarioAsset(filePath, scenarioPath, additionalSearchPaths = []) {
  const normalizedFilePath = normalizeScenarioAssetPath(filePath);
  const filename = fileName(filePath);
  const candidates = [normalizedFilePath];

  if (filename !== normalizedFilePath) {
    candidates.push(filename);
  }

  for (const searchPath of additionalSearchPaths) {
    candidates.push(resolveScenarioAssetPath(searchPath, normalizedFilePath));
    if (filename !== normalizedFilePath) {
      candidates.push(resolveScenarioAssetPath(searchPath, filename));
    }
  }

  candidates.push(resolveScenarioAssetPath(scenarioPath, normalizedFilePath));
  if (filename !== normalizedFilePath) {
    candidates.push(resolveScenarioAssetPath(scenarioPath, filename));
  }

  return findExistingScenarioAssetPath(candidates);
}

function getScenarioAssetUrl(path) {
  const normalizedPath = normalizeScenarioAssetPath(path);
  if (scenarioAssetUrls.has(normalizedPath)) {
    return scenarioAssetUrls.get(normalizedPath);
  }

  const content = getLoadedScenarioContent(normalizedPath);
  if (!content) {
    return null;
  }

  const blob = new Blob([content], { type: getScenarioAssetMimeType(normalizedPath) });
  const url = URL.createObjectURL(blob);
  scenarioAssetUrls.set(normalizedPath, url);
  return url;
}

function revokeScenarioAssetUrls() {
  for (const url of scenarioAssetUrls.values()) {
    URL.revokeObjectURL(url);
  }
  scenarioAssetUrls.clear();
}

function scenarioAssetBaseUrl(path) {
  const directory = dirname(`/${normalizeScenarioAssetPath(path)}`).slice(1);
  return `${SCENARIO_ASSET_URL_SCHEME}${directory ? `${directory}/` : ""}`;
}

function resolveScenarioAssetRequest(basePath, requestUrl) {
  const normalizedRequest = String(requestUrl || "").split("#")[0].split("?")[0];
  if (!normalizedRequest || /^(blob:|data:|https?:)/i.test(normalizedRequest)) {
    return null;
  }

  const requestPath = normalizedRequest.startsWith(SCENARIO_ASSET_URL_SCHEME)
    ? normalizedRequest.slice(SCENARIO_ASSET_URL_SCHEME.length)
    : normalizedRequest;
  const normalizedPath = normalizeScenarioAssetPath(requestPath);
  if (loadedScenarioFiles.has(normalizedPath)) {
    return normalizedPath;
  }

  return resolveScenarioAssetPath(basePath, requestPath);
}

function createXmlDocument(xmlText, label) {
  const document = new DOMParser().parseFromString(xmlText, "application/xml");
  const parserError = document.querySelector("parsererror");
  if (parserError) {
    throw new Error(`Failed to parse ${label}: ${parserError.textContent.trim()}`);
  }

  return document;
}

function getFirstChildElement(node, tagName) {
  if (!node) {
    return null;
  }

  return Array.from(node.children).find((child) => child.tagName === tagName) ?? null;
}

function getPropertiesFilepath(node) {
  const propertiesNode = getFirstChildElement(node, "Properties");
  return getFirstChildElement(propertiesNode, "File")?.getAttribute("filepath") || "";
}

function collectCatalogLocationMap(scenarioDocument) {
  const catalogLocations = new Map();
  const catalogLocationsNode = scenarioDocument.querySelector("CatalogLocations");
  if (!catalogLocationsNode) {
    return catalogLocations;
  }

  for (const catalogNode of catalogLocationsNode.children) {
    const directoryPath = getFirstChildElement(catalogNode, "Directory")?.getAttribute("path");
    if (!directoryPath) {
      continue;
    }

    catalogLocations.set(catalogNode.tagName, directoryPath);
  }

  return catalogLocations;
}

function findCatalogEntryModelPath(catalogDocument, entryName, scenarioPath) {
  const catalogNode = catalogDocument.querySelector("Catalog");
  if (!catalogNode) {
    return "";
  }

  for (const entryNode of catalogNode.children) {
    if (entryNode.getAttribute("name") !== entryName) {
      continue;
    }

    const filepath = getPropertiesFilepath(entryNode);
    return filepath ? locateScenarioAsset(filepath, scenarioPath) : "";
  }

  return "";
}

function findDirectScenarioObjectModelPath(scenarioObjectNode, scenarioPath) {
  for (const childNode of scenarioObjectNode.children) {
    if (!["Vehicle", "Pedestrian", "MiscObject", "ExternalObject"].includes(childNode.tagName)) {
      continue;
    }

    const filepath = getPropertiesFilepath(childNode);
    if (filepath) {
      return resolveScenarioAssetPath(scenarioPath, filepath);
    }
  }

  return "";
}

function collectScenarioModelReferences(scenarioPath) {
  if (!scenarioPath || loadedScenarioFiles.size === 0) {
    return { staticScenePath: "", entityModels: new Map() };
  }

  const scenarioText = getLoadedScenarioText(scenarioPath);
  if (!scenarioText) {
    return { staticScenePath: "", entityModels: new Map() };
  }

  const scenarioDocument = createXmlDocument(scenarioText, scenarioPath);
  const entityModels = new Map();
  const catalogLocations = collectCatalogLocationMap(scenarioDocument);
  const catalogDocuments = new Map();
  const staticSceneFilepath = scenarioDocument.querySelector("RoadNetwork > SceneGraphFile")?.getAttribute("filepath") || "";
  const staticScenePath = staticSceneFilepath ? resolveScenarioAssetPath(scenarioPath, staticSceneFilepath) : "";

  for (const scenarioObjectNode of scenarioDocument.querySelectorAll("Entities > ScenarioObject")) {
    const entityName = scenarioObjectNode.getAttribute("name") || "";
    if (!entityName) {
      continue;
    }

    const directModelPath = findDirectScenarioObjectModelPath(scenarioObjectNode, scenarioPath);
    if (directModelPath) {
      entityModels.set(entityName, directModelPath);
      continue;
    }

    const catalogReferenceNode = getFirstChildElement(scenarioObjectNode, "CatalogReference");
    if (!catalogReferenceNode) {
      continue;
    }

    const catalogName = catalogReferenceNode.getAttribute("catalogName") || "";
    const entryName = catalogReferenceNode.getAttribute("entryName") || "";
    const directoryPath = catalogLocations.get(catalogName);
    if (!catalogName || !entryName || !directoryPath) {
      continue;
    }

    const catalogPath = resolveScenarioAssetPath(scenarioPath, `${directoryPath}/${catalogName}.xosc`);
    let catalogDocument = catalogDocuments.get(catalogPath);
    if (!catalogDocument) {
      const catalogText = getLoadedScenarioText(catalogPath);
      if (!catalogText) {
        continue;
      }

      catalogDocument = createXmlDocument(catalogText, catalogPath);
      catalogDocuments.set(catalogPath, catalogDocument);
    }

    const catalogModelPath = findCatalogEntryModelPath(catalogDocument, entryName, scenarioPath);
    if (catalogModelPath) {
      entityModels.set(entityName, catalogModelPath);
    }
  }

  return { staticScenePath, entityModels };
}

function createGltfLoader(assetPath) {
  const manager = new THREE.LoadingManager();
  manager.setURLModifier((url) => {
    const resolvedPath = resolveScenarioAssetRequest(assetPath, url);
    return resolvedPath ? getScenarioAssetUrl(resolvedPath) || url : url;
  });
  return new GLTFLoader(manager);
}

function parseGltfAsset(loader, fileContent, assetPath) {
  return new Promise((resolve, reject) => {
    const onLoad = (gltf) => resolve(gltf.scene || gltf.scenes?.[0] || null);
    const onError = (error) => reject(error);

    if (typeof fileContent === "string") {
      loader.parse(fileContent, scenarioAssetBaseUrl(assetPath), onLoad, onError);
      return;
    }

    const buffer = fileContent.buffer.slice(fileContent.byteOffset, fileContent.byteOffset + fileContent.byteLength);
    loader.parse(buffer, scenarioAssetBaseUrl(assetPath), onLoad, onError);
  });
}

async function loadScenarioModelTemplate(assetPath) {
  const normalizedPath = normalizeScenarioAssetPath(assetPath);
  if (!/\.(glb|gltf)$/i.test(normalizedPath)) {
    throw new Error(`Unsupported browser model format: ${normalizedPath}`);
  }

  if (modelTemplateCache.has(normalizedPath)) {
    return modelTemplateCache.get(normalizedPath);
  }

  const loadPromise = (async () => {
    const fileContent = getLoadedScenarioContent(normalizedPath);
    if (!fileContent) {
      throw new Error(`Missing model asset: ${normalizedPath}`);
    }

    const loader = createGltfLoader(normalizedPath);
    const template = await parseGltfAsset(loader, fileContent, normalizedPath);
    if (!template) {
      throw new Error(`Model asset has no renderable scene: ${normalizedPath}`);
    }

    return template;
  })();

  modelTemplateCache.set(normalizedPath, loadPromise);
  return loadPromise;
}

function isBrowserRenderableModelAsset(assetPath) {
  return /\.(glb|gltf)$/i.test(normalizeScenarioAssetPath(assetPath || ""));
}

function resolveScenarioVisualConfig(scenarioPath) {
  const { staticScenePath, entityModels } = collectScenarioModelReferences(scenarioPath);
  return {
    staticScenePath,
    entityModels,
    staticSceneReplacesProceduralRoads: isBrowserRenderableModelAsset(staticScenePath),
  };
}

function createStaticModelInstance(template) {
  const modelRoot = new THREE.Group();
  modelRoot.rotation.x = MODEL_UP_AXIS_ROTATION_X;
  modelRoot.add(cloneSkinned(template));
  return modelRoot;
}

function createEntityModelInstance(template) {
  const anchor = new THREE.Group();
  const orientationRoot = new THREE.Group();
  orientationRoot.rotation.x = MODEL_UP_AXIS_ROTATION_X;
  orientationRoot.add(cloneSkinned(template));
  anchor.add(orientationRoot);
  anchor.updateMatrixWorld(true);

  const bounds = new THREE.Box3().setFromObject(anchor);
  if (bounds.isEmpty()) {
    return { anchor, bounds: null };
  }

  const size = new THREE.Vector3();
  const center = new THREE.Vector3();
  bounds.getSize(size);
  bounds.getCenter(center);
  return {
    anchor,
    bounds: {
      center,
      size,
    },
  };
}

function applyEntityModelTransform(group, state) {
  const modelAnchor = group.userData.modelAnchor;
  const modelBounds = group.userData.modelBounds;
  if (!modelAnchor || !modelBounds) {
    return;
  }

  const targetSize = new THREE.Vector3(
    Math.max(state.length, 0.5),
    Math.max(state.width, 0.5),
    Math.max(state.height, 0.5)
  );
  const scale = new THREE.Vector3(
    modelBounds.size.x > 1e-4 ? targetSize.x / modelBounds.size.x : 1,
    modelBounds.size.y > 1e-4 ? targetSize.y / modelBounds.size.y : 1,
    modelBounds.size.z > 1e-4 ? targetSize.z / modelBounds.size.z : 1
  );
  const scaledCenter = modelBounds.center.clone().multiply(scale);

  modelAnchor.scale.copy(scale);
  modelAnchor.position.set(
    state.center_offset_x - scaledCenter.x,
    state.center_offset_y - scaledCenter.y,
    state.center_offset_z - scaledCenter.z
  );
}

function clearEntityModel(group) {
  if (group.userData.modelAnchor) {
    group.remove(group.userData.modelAnchor);
  }

  group.userData.modelAnchor = null;
  group.userData.modelBounds = null;
  group.userData.modelAssetPath = "";
  group.userData.loadingModelAssetPath = "";

  if (group.userData.fallbackMesh) {
    group.userData.fallbackMesh.visible = true;
  }
}

async function ensureEntityModel(group, state) {
  const assetPath = entityModelDefinitions.get(state.name) || "";
  if (!assetPath) {
    clearEntityModel(group);
    return;
  }

  if (group.userData.modelAssetPath === assetPath && group.userData.modelAnchor) {
    if (group.userData.fallbackMesh) {
      group.userData.fallbackMesh.visible = false;
    }
    applyEntityModelTransform(group, state);
    return;
  }

  if (group.userData.loadingModelAssetPath === assetPath) {
    return;
  }

  group.userData.loadingModelAssetPath = assetPath;
  group.userData.modelRequestToken = (group.userData.modelRequestToken || 0) + 1;
  const requestToken = group.userData.modelRequestToken;

  try {
    const template = await loadScenarioModelTemplate(assetPath);
    if (!group.parent || group.userData.modelRequestToken !== requestToken || group.userData.loadingModelAssetPath !== assetPath) {
      return;
    }

    clearEntityModel(group);
    const { anchor, bounds } = createEntityModelInstance(template);
    group.add(anchor);
    group.userData.modelAnchor = anchor;
    group.userData.modelBounds = bounds;
    group.userData.modelAssetPath = assetPath;
    group.userData.loadingModelAssetPath = "";

    if (group.userData.fallbackMesh) {
      group.userData.fallbackMesh.visible = false;
    }

    applyEntityModelTransform(group, group.userData.state || state);
  } catch (error) {
    if (group.userData.modelRequestToken !== requestToken) {
      return;
    }

    group.userData.loadingModelAssetPath = "";
    if (!loggedModelFailures.has(assetPath)) {
      loggedModelFailures.add(assetPath);
      logMessage(`Failed to load ${assetPath}: ${error.message}`, "warn");
    }

    if (group.userData.fallbackMesh) {
      group.userData.fallbackMesh.visible = true;
    }
  }
}

function disposeFallbackMesh(mesh) {
  if (!mesh) {
    return;
  }

  if (mesh.geometry) {
    mesh.geometry.dispose();
  }

  if (Array.isArray(mesh.material)) {
    for (const material of mesh.material) {
      material?.dispose();
    }
    return;
  }

  mesh.material?.dispose();
}

function removeEntityGroup(id, group) {
  clearEntityModel(group);

  if (group.userData.boundingBoxOverlay) {
    group.remove(group.userData.boundingBoxOverlay);
    group.userData.boundingBoxOverlay.material.dispose();
  }

  if (group.userData.fallbackMesh) {
    group.remove(group.userData.fallbackMesh);
    disposeFallbackMesh(group.userData.fallbackMesh);
  }

  entityGroup.remove(group);
  entityMeshes.delete(id);
  removeFollowHelper(id);
}

function clearAllEntities() {
  for (const [id, group] of entityMeshes) {
    removeEntityGroup(id, group);
  }

  ui.objectCountStat.textContent = "0";
}

async function syncScenarioVisualAssets(scenarioPath) {
  const loadToken = ++scenarioVisualLoadToken;

  clearGroup(staticModelGroup);
  setProceduralRoadMode(true);
  entityModelDefinitions.clear();
  modelTemplateCache.clear();
  loggedModelFailures.clear();
  revokeScenarioAssetUrls();
  for (const group of entityMeshes.values()) {
    clearEntityModel(group);
  }

  if (!scenarioPath || loadedScenarioFiles.size === 0) {
    return;
  }

  const { staticScenePath, entityModels, staticSceneReplacesProceduralRoads } = resolveScenarioVisualConfig(scenarioPath);
  if (scenarioVisualLoadToken !== loadToken) {
    return;
  }

  for (const [entityName, assetPath] of entityModels) {
    entityModelDefinitions.set(entityName, assetPath);
  }

  if (entityModels.size > 0) {
    logMessage(`Discovered ${entityModels.size} entity model asset${entityModels.size === 1 ? "" : "s"} in ${scenarioPath}.`, "info");
  }

  if (staticScenePath) {
    try {
      const template = await loadScenarioModelTemplate(staticScenePath);
      if (scenarioVisualLoadToken !== loadToken) {
        return;
      }

      staticModelGroup.add(createStaticModelInstance(template));
      if (staticSceneReplacesProceduralRoads) {
        setProceduralRoadMode(false);
      }
      logMessage(`Loaded static scene model: ${staticScenePath}`, "info");
    } catch (error) {
      if (staticSceneReplacesProceduralRoads) {
        setProceduralRoadMode(true);
      }
      if (!loggedModelFailures.has(staticScenePath)) {
        loggedModelFailures.add(staticScenePath);
        logMessage(`Failed to load ${staticScenePath}: ${error.message}`, "warn");
      }
    }
  } else {
    setProceduralRoadMode(true);
  }

  for (const state of scenario ? vectorToArray(scenario.get_object_states()) : []) {
    const group = entityMeshes.get(state.id);
    if (group) {
      ensureEntityModel(group, state);
    }
  }
}

function ensureVirtualDirectory(module, path) {
  const normalized = normalizeVirtualPath(path);
  if (normalized === "/") {
    return;
  }

  let currentPath = "";
  for (const segment of normalized.slice(1).split("/")) {
    currentPath += `/${segment}`;
    try {
      module.FS.mkdir(currentPath);
    } catch {
      // Directory already exists.
    }
  }
}

function writeVirtualFile(module, path, contents) {
  const normalized = normalizeVirtualPath(path);
  ensureVirtualDirectory(module, dirname(normalized));

  try {
    module.FS.unlink(normalized);
  } catch {
    // File does not exist yet.
  }

  module.FS.writeFile(normalized, contents);
}

function clearVirtualPath(module, path) {
  const normalized = normalizeVirtualPath(path);

  try {
    const stat = module.FS.stat(normalized);
    if (module.FS.isDir(stat.mode)) {
      const entries = module.FS.readdir(normalized);
      for (const entry of entries) {
        if (entry === "." || entry === "..") {
          continue;
        }
        clearVirtualPath(module, `${normalized}/${entry}`);
      }
      module.FS.rmdir(normalized);
      return;
    }

    module.FS.unlink(normalized);
  } catch {
    // Path does not exist.
  }
}

function resetScenarioMount(module) {
  clearVirtualPath(module, EXAMPLE_SCENARIO_DIR);
  ensureVirtualDirectory(module, EXAMPLE_SCENARIO_DIR);
}

function parseCatalogLocations(scenarioText) {
  return Array.from(
    scenarioText.matchAll(/<([A-Za-z0-9_]+Catalog)>\s*<Directory[^>]*path="([^"]+)"/g),
    ([, catalogName, directoryPath]) => ({ catalogName, directoryPath })
  );
}

async function mountBundledScenarioAssets(module) {
  resetScenarioMount(module);

  const [roadText, scenarioText] = await Promise.all([
    fetchTextAsset(EXAMPLE_XODR_URL),
    fetchTextAsset(EXAMPLE_XOSC_URL),
  ]);

  writeVirtualFile(module, EXAMPLE_XODR_VIRTUAL_PATH, roadText);
  writeVirtualFile(module, EXAMPLE_XOSC_VIRTUAL_PATH, scenarioText);

  const catalogLocations = parseCatalogLocations(scenarioText);
  await Promise.all(
    catalogLocations.map(async ({ catalogName, directoryPath }) => {
      const catalogText = await fetchTextAsset(new URL(`${catalogName}.xosc`, EXAMPLE_CATALOGS_URL).href);
      const catalogPath = resolveVirtualPath(EXAMPLE_XOSC_VIRTUAL_PATH, `${directoryPath}/${catalogName}.xosc`);
      writeVirtualFile(module, catalogPath, catalogText);
    })
  );
}

function isCatalogScenarioFile(path) {
  const normalized = String(path).toLowerCase();
  return normalized.endsWith(".xosc") && (normalized.includes("/catalogs/") || normalized.startsWith("catalogs/") || normalized.includes("catalog"));
}

function getLoadedScenarioOptions() {
  return Array.from(loadedScenarioFiles.keys())
    .filter((path) => path.toLowerCase().endsWith(".xosc") && !isCatalogScenarioFile(path))
    .sort((left, right) => left.localeCompare(right));
}

function updateScenarioSelector() {
  const scenarioOptions = getLoadedScenarioOptions();
  ui.scenarioSelect.innerHTML = "";

  if (scenarioOptions.length === 0) {
    ui.scenarioSelect.disabled = true;
    ui.scenarioSelect.appendChild(new Option("Bundled sample active", ""));
    selectedScenarioPath = "";
    return;
  }

  ui.scenarioSelect.disabled = false;
  for (const scenarioPath of scenarioOptions) {
    ui.scenarioSelect.appendChild(new Option(scenarioPath, scenarioPath));
  }

  if (!scenarioOptions.includes(selectedScenarioPath)) {
    selectedScenarioPath = scenarioOptions[0];
  }
  ui.scenarioSelect.value = selectedScenarioPath;
}

async function loadDirectoryRecursive(dirHandle, basePath, files = new Map()) {
  for await (const entry of dirHandle.values()) {
    const entryPath = basePath ? `${basePath}/${entry.name}` : entry.name;

    if (entry.kind === "directory") {
      await loadDirectoryRecursive(entry, entryPath, files);
      continue;
    }

    if (!SCENARIO_FILE_PATTERN.test(entry.name)) {
      continue;
    }

    const file = await entry.getFile();
    const lowerName = entry.name.toLowerCase();
    const content = lowerName.endsWith(".xosc") || lowerName.endsWith(".xodr") || lowerName.endsWith(".gltf")
      ? await file.text()
      : new Uint8Array(await file.arrayBuffer());
    files.set(entryPath, content);
  }

  return files;
}

function summarizeDirectoryShape(files) {
  const topLevelDirectories = new Set();
  for (const path of files.keys()) {
    const [firstSegment] = String(path).split("/");
    if (firstSegment) {
      topLevelDirectories.add(firstSegment);
    }
  }

  const hasXosc = topLevelDirectories.has("xosc");
  const hasXodr = topLevelDirectories.has("xodr");
  const hasCatalogs = topLevelDirectories.has("catalogs");
  if (hasXosc && hasXodr && hasCatalogs) {
    logMessage("Loaded a complete scenario tree with xosc/, xodr/, and catalogs/.", "info");
    return;
  }

  logMessage(`Loaded directory contents: ${Array.from(topLevelDirectories).sort().join(", ") || "(flat)"}.`, hasXosc || hasXodr ? "warn" : "info");
}

async function mountLoadedScenarioAssets(module) {
  resetScenarioMount(module);

  for (const [relativePath, content] of loadedScenarioFiles.entries()) {
    writeVirtualFile(module, `${EXAMPLE_SCENARIO_DIR}/${relativePath}`, content);
  }
}

function disposeScenario() {
  if (scenario && typeof scenario.delete === "function") {
    scenario.delete();
  }
  scenario = null;
}

function createScenarioConfig() {
  return {
    max_loop: 0,
    min_time_step: 0,
    max_time_step: 0,
    dt: fixedStep,
  };
}

function openScenario(module, scenarioPath) {
  disposeScenario();
  clearAllEntities();

  const { staticScenePath, staticSceneReplacesProceduralRoads } = resolveScenarioVisualConfig(scenarioPath);
  setProceduralRoadMode(!staticSceneReplacesProceduralRoads);

  scenario = new module.OpenScenario(scenarioPath, createScenarioConfig());
  buildRoadScene(scenario.get_road_geometry(), proceduralRoadRenderingEnabled);
  if (!proceduralRoadRenderingEnabled) {
    logMessage(`Skipping procedural road rendering because static scene model ${staticScenePath} provides the road surface.`, "info");
  }
  initializeScenarioState();
  accumulator = 0;
  isPlaying = false;
  setStatus("Ready");
  syncPlaybackUi(false);
}

async function loadScenarioFromDirectorySelection() {
  if (!selectedScenarioPath) {
    return;
  }

  setStatus("Loading");
  ui.hudText.textContent = `Loading ${selectedScenarioPath}...`;

  try {
    const module = await ensureEsminiModule();
    await mountLoadedScenarioAssets(module);
    openScenario(module, `${EXAMPLE_SCENARIO_DIR}/${selectedScenarioPath}`);
    await syncScenarioVisualAssets(selectedScenarioPath);
    logMessage(`Loaded scenario from directory: ${selectedScenarioPath}`, "info");
  } catch (error) {
    console.error(error);
    ui.hudText.textContent = error.message;
    setStatus("Error", "error");
    logMessage(`Failed to load scenario from directory: ${error.message}`, "error");
  }
}

async function selectScenarioDirectory() {
  if (!directoryPickerSupported) {
    return;
  }

  try {
    const directoryHandle = await window.showDirectoryPicker({ mode: "read" });
    logMessage(`Selected scenario directory: ${directoryHandle.name}`, "info");
    loadedScenarioFiles = await loadDirectoryRecursive(directoryHandle, "");
    summarizeDirectoryShape(loadedScenarioFiles);
    updateScenarioSelector();

    if (!selectedScenarioPath) {
      logMessage("No runnable .xosc scenario files found in the selected directory.", "warn");
      return;
    }

    ui.directorySupport.classList.add("hidden");
    await loadScenarioFromDirectorySelection();
  } catch (error) {
    if (error?.name === "AbortError") {
      return;
    }

    logMessage(`Directory load failed: ${error.message}`, "error");
  }
}

function laneMaterialColor(laneType) {
  if (laneType & LANE_TYPE.DRIVING || laneType & LANE_TYPE.STOP || laneType & LANE_TYPE.BORDER) {
        return "#3c4248";
    }

  if (laneType & LANE_TYPE.SHOULDER || laneType & LANE_TYPE.PARKING) {
        return "#707780";
    }

    if (laneType & LANE_TYPE.SIDEWALK || laneType & LANE_TYPE.BIKING) {
        return "#8a8f87";
    }

    if (laneType & LANE_TYPE.MEDIAN) {
        return "#6d6d58";
    }

    return "#59616a";
}

function roadMarkColor(color) {
    const normalized = String(color || "").toLowerCase();
    if (normalized.includes("yellow")) {
        return "#f0c54c";
    }
    if (normalized.includes("red")) {
        return "#d15d47";
    }
    return "#f5e9c8";
}

function laneSurfaceKey(roadId, laneSectionIndex, laneId) {
  return `${roadId}:${laneSectionIndex}:${laneId}`;
}

const geometryOffsetQuaternion = new THREE.Quaternion();
const geometryOffsetVector = new THREE.Vector3();

  function roadMarkWidth(width) {
    return Math.max(Number(width) || 0.15, 0.05);
  }

  function roadMarkGapThreshold(width) {
    return Math.max(1.5, (width || 0.15) * 8);
  }

  function shouldBreakRoadMarkSequence(currentPoint, nextPoint, width) {
    if (!currentPoint || !nextPoint) {
      return false;
    }

    if (currentPoint.endpoint) {
      return true;
    }

    const longitudinalGap = Math.abs((nextPoint.s ?? 0) - (currentPoint.s ?? 0));
    const spatialGap = Math.hypot(
      (nextPoint.x ?? 0) - (currentPoint.x ?? 0),
      (nextPoint.y ?? 0) - (currentPoint.y ?? 0),
      (nextPoint.z ?? 0) - (currentPoint.z ?? 0)
    );
    const threshold = roadMarkGapThreshold(width);

    return longitudinalGap > threshold || spatialGap > threshold;
  }

  function splitRoadMarkSequences(points, width) {
    const sequences = [];
    let currentSequence = [];

    for (let index = 0; index < points.length; index += 1) {
      const point = points[index];
      currentSequence.push(point);

      const nextPoint = index + 1 < points.length ? points[index + 1] : null;
      if (shouldBreakRoadMarkSequence(point, nextPoint, width)) {
        if (currentSequence.length >= 2) {
          sequences.push(currentSequence);
        }
        currentSequence = [];
      }
    }

    if (currentSequence.length >= 2) {
      sequences.push(currentSequence);
    }

    return sequences;
  }

function setRoadMarkOffset(target, point, lateralOffset) {
  setPointOffset(target, point, lateralOffset, ROAD_MARK_Z_OFFSET);
}

function setPointOffset(target, point, lateralOffset = 0, verticalOffset = 0) {
  geometryOffsetQuaternion.setFromEuler(new THREE.Euler(point.r ?? 0, point.p ?? 0, point.h ?? 0, "XYZ"));
  geometryOffsetVector.set(0, lateralOffset, verticalOffset).applyQuaternion(geometryOffsetQuaternion);
  target.set(
    (point.x ?? 0) + geometryOffsetVector.x,
    (point.y ?? 0) + geometryOffsetVector.y,
    (point.z ?? 0) + geometryOffsetVector.z
  );
}

function createRoadMarkStripGeometry(points, width) {
  if (points.length < 2) {
    return null;
  }

  const halfWidth = roadMarkWidth(width) * 0.5;
  const positions = new Float32Array(points.length * 2 * 3);
  const indices = [];
  const leftVertex = new THREE.Vector3();
  const rightVertex = new THREE.Vector3();

  for (let index = 0; index < points.length; index += 1) {
    const point = points[index];
    setRoadMarkOffset(leftVertex, point, halfWidth);
    setRoadMarkOffset(rightVertex, point, -halfWidth);

    const base = index * 6;
    positions[base + 0] = leftVertex.x;
    positions[base + 1] = leftVertex.y;
    positions[base + 2] = leftVertex.z;
    positions[base + 3] = rightVertex.x;
    positions[base + 4] = rightVertex.y;
    positions[base + 5] = rightVertex.z;

    if (index < points.length - 1) {
      const start = index * 2;
      indices.push(start, start + 1, start + 2, start + 1, start + 3, start + 2);
    }
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

function createRoadMarkMesh(points, width, color) {
  const geometry = createRoadMarkStripGeometry(points, width);
  if (!geometry) {
    return null;
  }

  const mesh = new THREE.Mesh(
    geometry,
    new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: 1.0,
      depthWrite: false,
      depthTest: true,
      polygonOffset: true,
      polygonOffsetFactor: -2,
      polygonOffsetUnits: -2,
      side: THREE.DoubleSide,
      toneMapped: false,
    })
  );
  mesh.renderOrder = 1;
  return mesh;
}

function roadObjectWidth(width) {
  return Math.max(Number(width) || 0.05, 0.02);
}

function roadObjectHeight(height) {
  return Math.max(Number(height) || 0.3, 0.1);
}

function profileRatio(points, index) {
  if (points.length <= 1) {
    return 0;
  }

  const startS = points[0]?.s ?? 0;
  const endS = points[points.length - 1]?.s ?? startS;
  const total = endS - startS;
  if (Math.abs(total) > 1e-6) {
    return THREE.MathUtils.clamp(((points[index]?.s ?? startS) - startS) / total, 0, 1);
  }

  return index / (points.length - 1);
}

function createGuardRailGeometry(points, widthStart, widthEnd, heightStart, heightEnd) {
  if (points.length < 2) {
    return null;
  }

  const positions = new Float32Array(points.length * 4 * 3);
  const indices = [];
  const bottomLeft = new THREE.Vector3();
  const bottomRight = new THREE.Vector3();
  const topLeft = new THREE.Vector3();
  const topRight = new THREE.Vector3();

  for (let index = 0; index < points.length; index += 1) {
    const point = points[index];
    const ratio = profileRatio(points, index);
    const halfWidth = THREE.MathUtils.lerp(roadObjectWidth(widthStart), roadObjectWidth(widthEnd), ratio) * 0.5;
    const height = THREE.MathUtils.lerp(roadObjectHeight(heightStart), roadObjectHeight(heightEnd), ratio);

    setPointOffset(bottomLeft, point, halfWidth, 0);
    setPointOffset(bottomRight, point, -halfWidth, 0);
    setPointOffset(topLeft, point, halfWidth, height);
    setPointOffset(topRight, point, -halfWidth, height);

    const base = index * 12;
    positions[base + 0] = bottomLeft.x;
    positions[base + 1] = bottomLeft.y;
    positions[base + 2] = bottomLeft.z;
    positions[base + 3] = bottomRight.x;
    positions[base + 4] = bottomRight.y;
    positions[base + 5] = bottomRight.z;
    positions[base + 6] = topLeft.x;
    positions[base + 7] = topLeft.y;
    positions[base + 8] = topLeft.z;
    positions[base + 9] = topRight.x;
    positions[base + 10] = topRight.y;
    positions[base + 11] = topRight.z;

    if (index < points.length - 1) {
      const start = index * 4;
      const next = start + 4;
      indices.push(
        start, next, start + 2,
        next, next + 2, start + 2,
        start + 1, start + 3, next + 1,
        next + 1, start + 3, next + 3,
        start + 2, next + 2, start + 3,
        next + 2, next + 3, start + 3
      );
    }
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
}

function createGuardRailMesh(roadObject) {
  const geometry = createGuardRailGeometry(
    roadObject.points,
    roadObject.width_start,
    roadObject.width_end,
    roadObject.height_start,
    roadObject.height_end
  );
  if (!geometry) {
    return null;
  }

  return new THREE.Mesh(
    geometry,
    new THREE.MeshStandardMaterial({
      color: "#9aa0a6",
      roughness: 0.72,
      metalness: 0.2,
      side: THREE.DoubleSide,
    })
  );
}

function createPoleOrientation(point) {
  const pointQuaternion = new THREE.Quaternion().setFromEuler(
    new THREE.Euler(point.r ?? 0, point.p ?? 0, point.h ?? 0, "XYZ")
  );
  const verticalAlignment = new THREE.Quaternion().setFromEuler(new THREE.Euler(Math.PI / 2, 0, 0, "XYZ"));
  return pointQuaternion.multiply(verticalAlignment);
}

function createGuardRailPoles(roadObject) {
  if (!Array.isArray(roadObject.points) || roadObject.points.length === 0) {
    return null;
  }

  const postWidth = roadObjectWidth(roadObject.width_start);
  const postDepth = postWidth;
  const geometry = new THREE.BoxGeometry(postWidth, 1, postDepth);
  const material = new THREE.MeshStandardMaterial({
    color: "#767c82",
    roughness: 0.78,
    metalness: 0.18,
  });
  const group = new THREE.Group();
  const center = new THREE.Vector3();

  for (let index = 0; index < roadObject.points.length; index += 1) {
    const point = roadObject.points[index];
    const ratio = profileRatio(roadObject.points, index);
    const height = THREE.MathUtils.lerp(roadObjectHeight(roadObject.height_start), roadObjectHeight(roadObject.height_end), ratio);
    const mesh = new THREE.Mesh(geometry, material.clone());
    setPointOffset(center, point, 0, height * 0.5);
    mesh.position.copy(center);
    mesh.scale.set(1, height, 1);
    mesh.quaternion.copy(createPoleOrientation(point));
    group.add(mesh);
  }

  return group;
}

function interpolateRoadPoint(startPoint, endPoint, targetS) {
  const startS = startPoint.s ?? 0;
  const endS = endPoint.s ?? startS;
  const deltaS = endS - startS;
  if (Math.abs(deltaS) < 1e-6) {
    return { ...startPoint, s: targetS };
  }

  const ratio = (targetS - startS) / deltaS;
  return {
    s: targetS,
    x: startPoint.x + (endPoint.x - startPoint.x) * ratio,
    y: startPoint.y + (endPoint.y - startPoint.y) * ratio,
    z: startPoint.z + (endPoint.z - startPoint.z) * ratio,
    h: startPoint.h + ((endPoint.h ?? startPoint.h) - startPoint.h) * ratio,
    p: startPoint.p + ((endPoint.p ?? startPoint.p) - startPoint.p) * ratio,
    r: startPoint.r + ((endPoint.r ?? startPoint.r) - startPoint.r) * ratio,
    endpoint: false,
  };
}

function clipRoadMarkSequenceToRange(points, rangeStart, rangeEnd) {
  const clipped = [];

  for (let index = 0; index < points.length - 1; index += 1) {
    const startPoint = points[index];
    const endPoint = points[index + 1];
    const startS = startPoint.s ?? 0;
    const endS = endPoint.s ?? startS;
    const segmentStart = Math.min(startS, endS);
    const segmentEnd = Math.max(startS, endS);

    if (segmentEnd < rangeStart || segmentStart > rangeEnd) {
      continue;
    }

    const clippedStart = Math.max(segmentStart, rangeStart);
    const clippedEnd = Math.min(segmentEnd, rangeEnd);
    const firstPoint = Math.abs(clippedStart - startS) < 1e-6 ? startPoint : interpolateRoadPoint(startPoint, endPoint, clippedStart);
    const lastPoint = Math.abs(clippedEnd - endS) < 1e-6 ? endPoint : interpolateRoadPoint(startPoint, endPoint, clippedEnd);

    if (clipped.length === 0 || Math.abs((clipped[clipped.length - 1].s ?? 0) - (firstPoint.s ?? 0)) > 1e-6) {
      clipped.push(firstPoint);
    }
    clipped.push(lastPoint);
  }

  return clipped.length >= 2 ? clipped : null;
}

function splitFallbackRoadMarkByPattern(points, roadMark) {
  const lineLength = Number(roadMark.line_length) || 0;
  const lineSpace = Number(roadMark.line_space) || 0;
  if (lineLength <= 0 || lineSpace <= 0) {
    return [points];
  }

  const cycle = lineLength + lineSpace;
  const startS = points[0]?.s ?? 0;
  const endS = points[points.length - 1]?.s ?? startS;
  const ranges = [];

  let cursor = startS;
  while (cursor < endS - 1e-6) {
    ranges.push([cursor, Math.min(cursor + lineLength, endS)]);
    cursor += cycle;
  }

  return ranges
    .map(([rangeStart, rangeEnd]) => clipRoadMarkSequenceToRange(points, rangeStart, rangeEnd))
    .filter(Boolean);
}

function addBoundaryFallbackRoadMarks(laneSurfaces, roadMarks, laneSurfaceMap) {
  let fallbackCount = 0;

  if (roadMarks.length > 0) {
    for (const roadMark of roadMarks) {
      const normalizedType = String(roadMark.type || "").toLowerCase();
      if (normalizedType.includes("none")) {
        continue;
      }

      const fallbackPoints = fallbackRoadMarkPoints(roadMark, laneSurfaceMap);
      if (fallbackPoints.length < 2) {
        continue;
      }

      const fallbackSequences = splitFallbackRoadMarkByPattern(fallbackPoints, roadMark);
      for (const sequence of fallbackSequences) {
        const fallbackMesh = createRoadMarkMesh(sequence, roadMark.width, roadMarkColor(roadMark.color));
        if (!fallbackMesh) {
          continue;
        }

        markGroup.add(fallbackMesh);
        fallbackCount += 1;
      }
    }

    if (fallbackCount > 0) {
      return fallbackCount;
    }
  }

  for (const laneSurface of laneSurfaces) {
    const isRenderableLane =
      (laneSurface.lane_type & LANE_TYPE.DRIVING) ||
      (laneSurface.lane_type & LANE_TYPE.BORDER) ||
      (laneSurface.lane_type & LANE_TYPE.STOP);

    if (!isRenderableLane) {
      continue;
    }

    const boundaryPoints = laneSurface.lane_id > 0 ? laneSurface.right_boundary : laneSurface.left_boundary;
    const fallbackMesh = createRoadMarkMesh(boundaryPoints, 0.15, "#f5e9c8");
    if (!fallbackMesh) {
      continue;
    }

    markGroup.add(fallbackMesh);
    fallbackCount += 1;
  }

  return fallbackCount;
}

function fallbackRoadMarkPoints(roadMark, laneSurfaceMap) {
  const laneSurface = laneSurfaceMap.get(laneSurfaceKey(roadMark.road_id, roadMark.lane_section_index, roadMark.lane_id));
  if (!laneSurface) {
    return [];
  }

  return roadMark.lane_id >= 0 ? laneSurface.left_boundary : laneSurface.right_boundary;
}

  function createBottsDots(points, width, color) {
    const dotRadius = Math.max(width || 0.15, 0.15) * 0.5;
    const dotHeight = dotRadius * 0.35;
    const geometry = new THREE.CylinderGeometry(dotRadius, dotRadius, dotHeight, 12);
    const material = new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: 0.98,
      polygonOffset: true,
      polygonOffsetFactor: -2,
      polygonOffsetUnits: -2,
    });

    const dots = new THREE.Group();
    for (const point of points) {
      const mesh = new THREE.Mesh(geometry, material.clone());
      mesh.rotation.x = Math.PI / 2;
      mesh.position.set(point.x, point.y, point.z + dotHeight * 0.5 + 0.03);
      dots.add(mesh);
    }

    return dots;
  }

function stripGeometry(leftBoundary, rightBoundary) {
    const count = Math.min(leftBoundary.length, rightBoundary.length);
    if (count < 2) {
        return null;
    }

    const positions = new Float32Array(count * 2 * 3);
    const indices = [];

    for (let index = 0; index < count; index += 1) {
        const left = leftBoundary[index];
        const right = rightBoundary[index];
        const base = index * 6;

        positions[base + 0] = left.x;
        positions[base + 1] = left.y;
        positions[base + 2] = left.z;
        positions[base + 3] = right.x;
        positions[base + 4] = right.y;
        positions[base + 5] = right.z;

        if (index < count - 1) {
            const start = index * 2;
            indices.push(start, start + 1, start + 2, start + 1, start + 3, start + 2);
        }
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();
    return geometry;
}

function clearGroup(group) {
  const disposedGeometries = new Set();
  const disposedMaterials = new Set();

    while (group.children.length > 0) {
    const child = group.children[0];
    group.remove(child);

    child.traverse((node) => {
      if (node.geometry && !disposedGeometries.has(node.geometry)) {
        disposedGeometries.add(node.geometry);
        node.geometry.dispose();
      }

      if (Array.isArray(node.material)) {
        for (const material of node.material) {
          if (material && !disposedMaterials.has(material)) {
            disposedMaterials.add(material);
            material.dispose();
          }
        }
      } else if (node.material && !disposedMaterials.has(node.material)) {
        disposedMaterials.add(node.material);
        node.material.dispose();
      }
    });
    }
}

function createLineStrip(points, color) {
  if (!Array.isArray(points) || points.length < 2) {
    return null;
  }

  const positions = new Float32Array(points.length * 3);
  for (let index = 0; index < points.length; index += 1) {
    const base = index * 3;
    positions[base + 0] = points[index].x;
    positions[base + 1] = points[index].y;
    positions[base + 2] = (points[index].z ?? 0) + VECTOR_OVERLAY_Z_OFFSET;
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  const line = new THREE.Line(
    geometry,
    new THREE.LineBasicMaterial({ color, transparent: true, opacity: 1, toneMapped: false })
  );
  line.renderOrder = 3;
  return line;
}

function createLaneCenterPoints(laneSurface) {
  const count = Math.min(laneSurface.left_boundary.length, laneSurface.right_boundary.length);
  if (count < 2) {
    return [];
  }

  const points = [];
  for (let index = 0; index < count; index += 1) {
    const left = laneSurface.left_boundary[index];
    const right = laneSurface.right_boundary[index];
    points.push({
      x: (left.x + right.x) * 0.5,
      y: (left.y + right.y) * 0.5,
      z: (left.z + right.z) * 0.5,
    });
  }

  return points;
}

function laneCenterKey(roadId, laneId) {
  return `${roadId}:${laneId}`;
}

function rebuildLaneCenterLookup(laneCenters) {
  laneCenterGeometries.clear();

  for (const laneCenter of laneCenters) {
    if (!Array.isArray(laneCenter.points) || laneCenter.points.length < 2) {
      continue;
    }

    const key = laneCenterKey(laneCenter.road_id, laneCenter.lane_id);
    const geometries = laneCenterGeometries.get(key);
    if (geometries) {
      geometries.push(laneCenter);
    } else {
      laneCenterGeometries.set(key, [laneCenter]);
    }
  }
}

function interpolateGeometryPoint(startPoint, endPoint, targetS) {
  const startS = Number(startPoint.s);
  const endS = Number(endPoint.s);
  const deltaS = endS - startS;

  if (!Number.isFinite(startS) || !Number.isFinite(endS) || Math.abs(deltaS) < 1e-6) {
    return {
      x: startPoint.x,
      y: startPoint.y,
      z: startPoint.z,
    };
  }

  const ratio = THREE.MathUtils.clamp((targetS - startS) / deltaS, 0, 1);
  return {
    x: THREE.MathUtils.lerp(startPoint.x, endPoint.x, ratio),
    y: THREE.MathUtils.lerp(startPoint.y, endPoint.y, ratio),
    z: THREE.MathUtils.lerp(startPoint.z ?? 0, endPoint.z ?? 0, ratio),
  };
}

function sampleLaneCenterGeometry(laneCenter, targetS) {
  const points = laneCenter.points;
  if (!Array.isArray(points) || points.length === 0) {
    return null;
  }

  let closestSample = null;
  let closestDistance = Number.POSITIVE_INFINITY;

  for (let index = 0; index < points.length - 1; index += 1) {
    const startPoint = points[index];
    const endPoint = points[index + 1];
    const startS = Number(startPoint.s);
    const endS = Number(endPoint.s);

    if (!Number.isFinite(startS) || !Number.isFinite(endS)) {
      continue;
    }

    const segmentMinS = Math.min(startS, endS);
    const segmentMaxS = Math.max(startS, endS);
    const clampedS = THREE.MathUtils.clamp(targetS, segmentMinS, segmentMaxS);
    const distance = Math.abs(targetS - clampedS);

    if (distance < closestDistance) {
      closestDistance = distance;
      closestSample = interpolateGeometryPoint(startPoint, endPoint, clampedS);
      if (distance < 1e-6) {
        break;
      }
    }
  }

  if (closestSample) {
    return closestSample;
  }

  const firstPoint = points[0];
  return {
    x: firstPoint.x,
    y: firstPoint.y,
    z: firstPoint.z ?? 0,
  };
}

function getLaneCenterTrackingTarget(state) {
  const geometries = laneCenterGeometries.get(laneCenterKey(state.road_id, state.lane_id));
  if (!geometries || geometries.length === 0 || !Number.isFinite(state.s)) {
    return null;
  }

  let bestTarget = null;
  let bestDistance = Number.POSITIVE_INFINITY;

  for (const laneCenter of geometries) {
    const sample = sampleLaneCenterGeometry(laneCenter, state.s);
    if (!sample) {
      continue;
    }

    const distance = Math.hypot(sample.x - state.x, sample.y - state.y, (sample.z ?? 0) - state.z);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestTarget = sample;
    }
  }

  return bestTarget;
}

function createFeatureBoxMesh(feature) {
  const geometry = new THREE.EdgesGeometry(
    new THREE.BoxGeometry(
      Math.max(feature.length || 0.05, 0.05),
      Math.max(feature.width || 0.05, 0.05),
      Math.max(feature.height || 0.05, 0.05)
    )
  );
  const material = new THREE.LineBasicMaterial({
    color: feature.kind === "signal" ? VECTOR_OVERLAY_COLORS.signalBox : VECTOR_OVERLAY_COLORS.objectBox,
    transparent: true,
    opacity: 1,
    toneMapped: false,
  });
  const mesh = new THREE.LineSegments(geometry, material);
  mesh.position.set(feature.x, feature.y, feature.z + VECTOR_OVERLAY_Z_OFFSET);
  mesh.quaternion.setFromEuler(new THREE.Euler(feature.r ?? 0, feature.p ?? 0, feature.h ?? 0, "XYZ"));
  mesh.renderOrder = 3;
  return mesh;
}

function buildVectorOverlay(roadGeometry) {
  clearGroup(vectorBoundaryGroup);
  clearGroup(vectorCenterGroup);
  clearGroup(vectorFeatureBoxGroup);

  const laneSurfaces = roadGeometry.lane_surfaces ?? [];
  const laneCenters = roadGeometry.lane_centers ?? [];
  const featureBoxes = roadGeometry.road_feature_boxes ?? [];

  for (const laneSurface of laneSurfaces) {
    const leftBoundary = createLineStrip(laneSurface.left_boundary, VECTOR_OVERLAY_COLORS.boundary);
    if (leftBoundary) {
      vectorBoundaryGroup.add(leftBoundary);
    }

    const rightBoundary = createLineStrip(laneSurface.right_boundary, VECTOR_OVERLAY_COLORS.boundary);
    if (rightBoundary) {
      vectorBoundaryGroup.add(rightBoundary);
    }
  }

  if (laneCenters.length > 0) {
    for (const laneCenter of laneCenters) {
      const color = laneCenter.reference_line ? VECTOR_OVERLAY_COLORS.referenceLine : VECTOR_OVERLAY_COLORS.centerLine;
      const line = createLineStrip(laneCenter.points, color);
      if (line) {
        vectorCenterGroup.add(line);
      }
    }
  } else {
    for (const laneSurface of laneSurfaces) {
      const line = createLineStrip(createLaneCenterPoints(laneSurface), VECTOR_OVERLAY_COLORS.centerLine);
      if (line) {
        vectorCenterGroup.add(line);
      }
    }
  }

  for (const feature of featureBoxes) {
    vectorFeatureBoxGroup.add(createFeatureBoxMesh(feature));
  }
}

function createFollowHelper(state) {
  const steeringPoint = new THREE.Mesh(
    new THREE.SphereGeometry(0.35, 14, 10),
    new THREE.MeshBasicMaterial({
      color: VECTOR_OVERLAY_COLORS.steeringTarget,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    })
  );
  steeringPoint.renderOrder = 4;

  const steeringLineGeometry = new THREE.BufferGeometry();
  steeringLineGeometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(6), 3));
  const steeringLine = new THREE.Line(
    steeringLineGeometry,
    new THREE.LineBasicMaterial({
      color: VECTOR_OVERLAY_COLORS.steeringTarget,
      transparent: true,
      opacity: 1,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    })
  );
  steeringLine.renderOrder = 4;

  const trailPoint = new THREE.Mesh(
    new THREE.SphereGeometry(0.28, 12, 9),
    new THREE.MeshBasicMaterial({
      color: VECTOR_OVERLAY_COLORS.trailTarget,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    })
  );
  trailPoint.renderOrder = 4;

  const trailLineGeometry = new THREE.BufferGeometry();
  trailLineGeometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(6), 3));
  const trailLine = new THREE.Line(
    trailLineGeometry,
    new THREE.LineBasicMaterial({
      color: VECTOR_OVERLAY_COLORS.trailTarget,
      transparent: true,
      opacity: 1,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    })
  );
  trailLine.renderOrder = 4;

  vectorFollowGroup.add(steeringLine, steeringPoint, trailLine, trailPoint);
  const helper = { steeringPoint, steeringLine, trailPoint, trailLine };
  updateFollowHelper(helper, state);
  return helper;
}

function isFiniteVector3(x, y, z) {
  return Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z);
}

function supportsGhostTrackingOverlay(state) {
  if (state.has_ghost !== true) {
    return false;
  }

  return state.ctrl_type === CONTROLLER_TYPE.FOLLOW_GHOST || state.ctrl_type === CONTROLLER_TYPE.EXTERNAL;
}

function updateFollowHelper(helper, state) {
  const objectPositionValid = isFiniteVector3(state.x, state.y, state.z);
  const ghostTrackingEnabled = supportsGhostTrackingOverlay(state);
  const laneCenterTarget = ghostTrackingEnabled ? null : getLaneCenterTrackingTarget(state);
  const steeringTargetValid = ghostTrackingEnabled
    ? objectPositionValid && isFiniteVector3(state.sensor_x, state.sensor_y, state.sensor_z)
    : objectPositionValid && laneCenterTarget !== null;
  const trailTargetValid = ghostTrackingEnabled
    ? objectPositionValid && isFiniteVector3(state.trail_x, state.trail_y, state.trail_z)
    : false;

  helper.steeringPoint.visible = steeringTargetValid;
  helper.steeringLine.visible = steeringTargetValid;
  helper.trailPoint.visible = trailTargetValid;
  helper.trailLine.visible = trailTargetValid;

  if (steeringTargetValid) {
    const steeringTargetX = ghostTrackingEnabled ? state.sensor_x : laneCenterTarget.x;
    const steeringTargetY = ghostTrackingEnabled ? state.sensor_y : laneCenterTarget.y;
    const steeringTargetZ = ghostTrackingEnabled ? state.sensor_z : laneCenterTarget.z;
    const steeringPositions = helper.steeringLine.geometry.getAttribute("position");
    steeringPositions.setXYZ(0, state.x, state.y, state.z + VECTOR_OVERLAY_Z_OFFSET);
    steeringPositions.setXYZ(1, steeringTargetX, steeringTargetY, steeringTargetZ + VECTOR_OVERLAY_Z_OFFSET);
    steeringPositions.needsUpdate = true;
    helper.steeringLine.geometry.computeBoundingSphere();
    helper.steeringPoint.position.set(steeringTargetX, steeringTargetY, steeringTargetZ + VECTOR_OVERLAY_Z_OFFSET);
  }

  if (trailTargetValid) {
    const trailPositions = helper.trailLine.geometry.getAttribute("position");
    trailPositions.setXYZ(0, state.trail_x, state.trail_y, state.trail_z + VECTOR_OVERLAY_Z_OFFSET);
    trailPositions.setXYZ(1, state.x, state.y, state.z + VECTOR_OVERLAY_Z_OFFSET);
    trailPositions.needsUpdate = true;
    helper.trailLine.geometry.computeBoundingSphere();
    helper.trailPoint.position.set(state.trail_x, state.trail_y, state.trail_z + VECTOR_OVERLAY_Z_OFFSET);
  }
}

function removeFollowHelper(id) {
  const helper = vectorFollowHelpers.get(id);
  if (!helper) {
    return;
  }

  vectorFollowHelpers.delete(id);
  vectorFollowGroup.remove(helper.steeringLine, helper.steeringPoint, helper.trailLine, helper.trailPoint);
  helper.steeringLine.geometry.dispose();
  helper.steeringLine.material.dispose();
  helper.steeringPoint.geometry.dispose();
  helper.steeringPoint.material.dispose();
  helper.trailLine.geometry.dispose();
  helper.trailLine.material.dispose();
  helper.trailPoint.geometry.dispose();
  helper.trailPoint.material.dispose();
}

function applyOrientation(target, state) {
    const yaw = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), state.h);
    const pitch = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), state.p);
    const roll = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), state.r);
    target.quaternion.copy(yaw).multiply(pitch).multiply(roll);
}

function egoFocusPoint(state) {
  return new THREE.Vector3(state.x, state.y, state.z + 1.6);
}

function egoFollowOffset(state) {
  return new THREE.Vector3(-18, 0, 8).applyAxisAngle(new THREE.Vector3(0, 0, 1), state.h);
}

function alignCameraBehindEgo(state) {
  const focusPoint = egoFocusPoint(state);
  camera.position.copy(focusPoint).add(egoFollowOffset(state));
  controls.target.copy(focusPoint);
  controls.update();
}

function trackEgo(state) {
  const focusPoint = egoFocusPoint(state);
  const orbitOffset = camera.position.clone().sub(controls.target);

  controls.target.copy(focusPoint);
  camera.position.copy(focusPoint).add(orbitOffset);
  controls.update();
}

function findEgoState(objectStates) {
  return objectStates.find((state) => state.name === "Ego") || objectStates[0] || null;
}

function applyCameraMode(egoState, resetCamera = false, snapToFollowPose = false) {
  if (followEgo && egoState) {
    if (resetCamera || snapToFollowPose) {
      alignCameraBehindEgo(egoState);
    } else {
      trackEgo(egoState);
    }
    return;
  }

  if (resetCamera) {
    setInitialCameraPose();
  }
}

function refreshScenarioView(objectStates, simulationTime, resetCamera = false, snapToFollowPose = false) {
  const normalizedStates = Array.isArray(objectStates) ? objectStates : vectorToArray(objectStates);
  updateEntitiesFromStates(normalizedStates, simulationTime);
  applyCameraMode(findEgoState(normalizedStates), resetCamera, snapToFollowPose);
}

function initializeScenarioState() {
  if (!scenario) {
    return;
  }

  const frame = scenario.step_frame(0);
  refreshScenarioView(frame.object_states, frame.simulation_time, true);
}

function buildRoadScene(roadGeometry, renderProceduralRoads = true) {
    clearGroup(roadGroup);
    clearGroup(markGroup);
    clearGroup(roadObjectGroup);

    const laneSurfaces = vectorToArray(roadGeometry.lane_surfaces).map((surface) => ({
        ...surface,
        left_boundary: vectorToArray(surface.left_boundary),
        right_boundary: vectorToArray(surface.right_boundary),
    }));
    const roadMarks = vectorToArray(roadGeometry.road_marks).map((mark) => ({
        ...mark,
        points: vectorToArray(mark.points),
    }));
    const laneCenters = roadGeometry.lane_centers
      ? vectorToArray(roadGeometry.lane_centers).map((laneCenter) => ({
          ...laneCenter,
          points: vectorToArray(laneCenter.points),
        }))
      : [];
    const roadObjects = roadGeometry.road_objects
      ? vectorToArray(roadGeometry.road_objects).map((roadObject) => ({
          ...roadObject,
          points: vectorToArray(roadObject.points),
        }))
      : [];
    const roadFeatureBoxes = roadGeometry.road_feature_boxes
      ? vectorToArray(roadGeometry.road_feature_boxes)
      : [];
    const laneSurfaceMap = new Map(
      laneSurfaces.map((surface) => [laneSurfaceKey(surface.road_id, surface.lane_section_index, surface.lane_id), surface])
    );

    rebuildLaneCenterLookup(laneCenters);

    buildVectorOverlay({
      lane_surfaces: laneSurfaces,
      lane_centers: laneCenters,
      road_feature_boxes: roadFeatureBoxes,
    });

    if (!renderProceduralRoads) {
      ui.laneCountStat.textContent = String(laneSurfaces.length);
      ui.markCountStat.textContent = String(roadMarks.length);
      return;
    }

    for (const laneSurface of laneSurfaces) {
        const geometry = stripGeometry(laneSurface.left_boundary, laneSurface.right_boundary);
        if (!geometry) {
            continue;
        }

        const mesh = new THREE.Mesh(
            geometry,
            new THREE.MeshStandardMaterial({
                color: laneMaterialColor(laneSurface.lane_type),
                roughness: 0.92,
                metalness: 0.02,
            })
        );
        roadGroup.add(mesh);
    }

    for (const roadMark of roadMarks) {
      const markPoints = roadMark.points.length >= 2 ? roadMark.points : fallbackRoadMarkPoints(roadMark, laneSurfaceMap);
      if (markPoints.length < 2) {
        continue;
      }

      const color = roadMarkColor(roadMark.color);
      const normalizedType = String(roadMark.type || "").toLowerCase();
      if (normalizedType.includes("botts")) {
        markGroup.add(createBottsDots(markPoints, roadMark.width, color));
        continue;
      }

      const sequences = roadMark.points.length >= 2
        ? splitRoadMarkSequences(markPoints, roadMark.width)
        : [markPoints];
      for (const sequence of sequences) {
        const mesh = createRoadMarkMesh(sequence, roadMark.width, color);
        if (!mesh) {
          continue;
        }
        markGroup.add(mesh);
      }
    }

    const fallbackRoadMarkCount = markGroup.children.length === 0 ? addBoundaryFallbackRoadMarks(laneSurfaces, roadMarks, laneSurfaceMap) : 0;

    for (const roadObject of roadObjects) {
      const normalizedType = String(roadObject.type || "").toLowerCase();
      if (normalizedType.includes("barrier") || normalizedType.includes("railing")) {
        const mesh = createGuardRailMesh(roadObject);
        if (!mesh) {
          continue;
        }

        roadObjectGroup.add(mesh);
        continue;
      }

      if (normalizedType.includes("pole")) {
        const poleGroup = createGuardRailPoles(roadObject);
        if (!poleGroup) {
          continue;
        }

        roadObjectGroup.add(poleGroup);
      }
    }

    ui.laneCountStat.textContent = String(laneSurfaces.length);
    ui.markCountStat.textContent = String(roadMarks.length);
}

function entityColor(state) {
    if (state.name === "Ego") {
        return "#ef8a34";
    }
    return VECTOR_OVERLAY_COLORS.entityBox;
}

function createEntityBoundingBoxOverlay(state) {
  const material = new THREE.LineBasicMaterial({
    color: entityColor(state),
    transparent: true,
    opacity: 0.95,
    depthTest: false,
    depthWrite: false,
    toneMapped: false,
  });
  const overlay = new THREE.LineSegments(ENTITY_BOUNDING_BOX_GEOMETRY, material);
  overlay.renderOrder = 5;
  overlay.visible = scenarioOverlayEnabled;
  updateEntityBoundingBoxOverlay(overlay, state);
  return overlay;
}

function updateEntityBoundingBoxOverlay(overlay, state) {
  overlay.position.set(state.center_offset_x, state.center_offset_y, state.center_offset_z);
  overlay.scale.set(
    Math.max(state.length, 0.5),
    Math.max(state.width, 0.5),
    Math.max(state.height, 0.5)
  );
  overlay.material.color.set(entityColor(state));
}

function createEntityFallbackMesh(state) {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(Math.max(state.length, 0.5), Math.max(state.width, 0.5), Math.max(state.height, 0.5)),
    new THREE.MeshStandardMaterial({ color: entityColor(state), roughness: 0.55, metalness: 0.08 })
  );
  mesh.position.set(state.center_offset_x, state.center_offset_y, state.center_offset_z);
  return mesh;
}

function updateEntities(frame) {
  refreshScenarioView(frame.object_states, frame.simulation_time);
}

function updateEntitiesFromStates(objectStates, simulationTime) {
    const seenIds = new Set();

    for (const state of objectStates) {
        seenIds.add(state.id);

        let group = entityMeshes.get(state.id);
        if (!group) {
            group = new THREE.Group();
      group.userData.fallbackMesh = createEntityFallbackMesh(state);
      group.userData.boundingBoxOverlay = createEntityBoundingBoxOverlay(state);
      group.add(group.userData.fallbackMesh, group.userData.boundingBoxOverlay);
            entityMeshes.set(state.id, group);
            entityGroup.add(group);
        }

        group.position.set(state.x, state.y, state.z);
        applyOrientation(group, state);
        group.userData.state = state;
      if (group.userData.fallbackMesh) {
        group.userData.fallbackMesh.position.set(state.center_offset_x, state.center_offset_y, state.center_offset_z);
      }
      if (group.userData.boundingBoxOverlay) {
        updateEntityBoundingBoxOverlay(group.userData.boundingBoxOverlay, state);
      }
      applyEntityModelTransform(group, state);
      ensureEntityModel(group, state);

      let followHelper = vectorFollowHelpers.get(state.id);
      if (!followHelper) {
        followHelper = createFollowHelper(state);
        vectorFollowHelpers.set(state.id, followHelper);
      } else {
        updateFollowHelper(followHelper, state);
      }
    }

    for (const [id, mesh] of entityMeshes) {
        if (!seenIds.has(id)) {
        removeEntityGroup(id, mesh);
        }
    }

    for (const id of Array.from(vectorFollowHelpers.keys())) {
      if (!seenIds.has(id)) {
        removeFollowHelper(id);
      }
    }

    ui.objectCountStat.textContent = String(objectStates.length);

    const egoState = findEgoState(objectStates);

    ui.hudText.textContent = egoState
        ? [
              `Elapsed time: ${scenario.get_simulation_time().toFixed(2)} s`,
              `Ego road/lane: ${egoState.road_id} / ${egoState.lane_id}`,
              `Ego position: (${egoState.x.toFixed(2)}, ${egoState.y.toFixed(2)}, ${egoState.z.toFixed(2)})`,
              `Ego speed: ${(egoState.speed * 3.6).toFixed(1)} km/h`
          ].join("\n")
        : "No active entities in the current frame.";
}

function stepSimulation(stepDt = fixedStep) {
    if (!scenario) {
        return;
    }

    const frame = scenario.step_frame(stepDt);
    updateEntities(frame);

    if (frame.quit) {
        isPlaying = false;
        setStatus("Complete");
      syncPlaybackUi(false);
    }
}

function syncCurrentState(snapToFollowPose = false) {
  if (!scenario) {
    return;
  }

  const objectStates = vectorToArray(scenario.get_object_states());
  refreshScenarioView(objectStates, scenario.get_simulation_time(), false, snapToFollowPose);
}

async function loadBundledSample() {
    setStatus("Loading");
  ui.hudText.textContent = "Initializing bundled sample...";

    try {
        const module = await ensureEsminiModule();
      await mountBundledScenarioAssets(module);
      openScenario(module, EXAMPLE_XOSC_VIRTUAL_PATH);
      await syncScenarioVisualAssets("");
      logMessage("Loaded bundled sample scenario.", "info");
    } catch (error) {
        console.error(error);
        ui.hudText.textContent = error.message;
        setStatus("Error", "error");
        logMessage(`Failed to load bundled sample: ${error.message}`, "error");
    }
}

ui.consoleToggleButton.addEventListener("click", () => {
    setConsoleOpen(!isConsoleOpen);
});

ui.clearConsoleButton.addEventListener("click", () => {
  consoleLines.length = 0;
  ui.consoleOutput.textContent = "Console cleared.";
});

ui.closeConsoleButton.addEventListener("click", () => {
  setConsoleOpen(false);
});

ui.helpButton.addEventListener("click", () => {
    setInfoPanelOpen(!isInfoPanelOpen);
});

ui.closeInfoButton.addEventListener("click", () => {
  setInfoPanelOpen(false);
});

ui.selectDirectoryButton.addEventListener("click", () => {
  selectScenarioDirectory();
});

ui.scenarioSelect.addEventListener("change", async (event) => {
  selectedScenarioPath = event.target.value;
  if (!selectedScenarioPath) {
    return;
  }

  await loadScenarioFromDirectorySelection();
});

ui.playPauseButton.addEventListener("click", () => {
    isPlaying = !isPlaying;
  syncPlaybackUi();
});

ui.stepButton.addEventListener("click", () => {
  if (!scenario) {
    return;
  }

  isPlaying = false;
  accumulator = 0;
  syncPlaybackUi();

  stepSimulation();
});

ui.resetButton.addEventListener("click", () => {
    if (!scenario) {
        return;
    }

    scenario.reset();
  buildRoadScene(scenario.get_road_geometry(), proceduralRoadRenderingEnabled);
  initializeScenarioState();
    isPlaying = false;
    setStatus("Ready");
    syncPlaybackUi(false);
});

ui.followEgoToggle.addEventListener("change", () => {
  followEgo = ui.followEgoToggle.checked;

  if (!scenario) {
    ui.hudText.textContent = `Follow ego: ${followEgo ? "on" : "off"}`;
    return;
  }

  syncCurrentState(followEgo);
});

ui.vectorOverlayToggle.addEventListener("change", () => {
  vectorOverlayEnabled = ui.vectorOverlayToggle.checked;
  setVectorOverlayVisibility(vectorOverlayEnabled);
});

ui.scenarioOverlayToggle.addEventListener("change", () => {
  scenarioOverlayEnabled = ui.scenarioOverlayToggle.checked;
  setScenarioOverlayVisibility(scenarioOverlayEnabled);
});

ui.speedSelect.addEventListener("change", (event) => {
    playbackSpeed = Number(event.target.value);
});

window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

function animate(timestamp) {
    const deltaSeconds = previousTimestamp === 0 ? 0 : (timestamp - previousTimestamp) / 1000;
    previousTimestamp = timestamp;

    if (isPlaying && scenario) {
        accumulator += deltaSeconds * playbackSpeed;
        while (accumulator >= fixedStep) {
            stepSimulation(fixedStep);
            accumulator -= fixedStep;
        }
    }

    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(animate);
}

setStatus("Idle");
setInfoPanelOpen(false);
setConsoleOpen(false);
ui.vectorOverlayToggle.checked = vectorOverlayEnabled;
setVectorOverlayVisibility(vectorOverlayEnabled);
ui.scenarioOverlayToggle.checked = scenarioOverlayEnabled;
setScenarioOverlayVisibility(scenarioOverlayEnabled);
updateDirectorySupportUi();
loadBundledSample();
requestAnimationFrame(animate);