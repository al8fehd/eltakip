// ============================================================================
// Deneyap Hand Tracker — Phase 1 3D renderer.
// Procedural, anatomically-proportioned hand: palm + 5 two-segment fingers
// with a clean joint hierarchy driven by quaternions from the bridge
// (pose.joints.{finger}_{prox|dist}, pose.hand). Runs at 60 FPS with
// frame-rate-independent slerp smoothing.
// ============================================================================

// --- Three.js: local copy first (offline), CDN fallback ---------------------
let THREE = null;
try {
  THREE = await import("./vendor/three.module.min.js");
} catch {
  try {
    THREE = await import("https://unpkg.com/three@0.168.0/build/three.module.js");
  } catch {
    fail("Three.js could not be loaded — no internet (CDN) and no web/vendor/three.module.min.js. Ask the maintainer to vendor it, or connect the machine.");
  }
}
if (!THREE) fail("Three.js failed to load.");

function fail(msg) {
  const el = document.getElementById("err");
  el.hidden = false;
  el.textContent = msg;
  throw new Error(msg);
}

// --- DOM --------------------------------------------------------------------
const $ = (id) => document.getElementById(id);
const dotEl = $("dot"), statusTxt = $("statusTxt"), hzTxt = $("hzTxt"),
      fpsTxt = $("fpsTxt"), imuCountEl = $("imuCount"), zeroTxt = $("zeroTxt"),
      zeroBtn = $("zeroBtn"), resetBtn = $("resetBtn"), errEl = $("err"),
      imuSelect = $("imuSelect"), deadzoneSlider = $("deadzoneSlider"),
      deadzoneValue = $("deadzoneValue"), jitterReadout = $("jitterReadout"),
      resetDeadzonesBtn = $("resetDeadzonesBtn"), mountYawSelect = $("mountYaw"),
      invertYawInput = $("invertYaw"), invertRollInput = $("invertRoll"),
      invertPitchInput = $("invertPitch");

const WS_URL = "ws://" + location.hostname + ":8765";
const JOINT_NAMES = [
  "thumb_prox", "thumb_dist",
  "index_prox", "index_dist",
  "middle_prox", "middle_dist",
  "ring_prox", "ring_dist",
  "pinky_prox", "pinky_dist",
];

// --- Scene, camera, lights ---------------------------------------------------
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
// Canvas goes directly on <body> in normal flow (renderer.setSize() sizes it
// to the viewport). Static placement composites reliably in every browser
// and embedded webview; fixed-position WebGL canvases are not captured by
// some compositors (e.g. headless Chromium).
renderer.domElement.style.cssText = "position:static;display:block;";
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x24334d);

const camera = new THREE.PerspectiveCamera(42, 1, 1, 4000);
camera.position.set(300, 190, 420);

// Bright, well-balanced lighting: ambient floor + warm key + cool fill.
scene.add(new THREE.AmbientLight(0xffffff, 0.55));
scene.add(new THREE.HemisphereLight(0xffffff, 0x8fa3c8, 1.25));
const key = new THREE.DirectionalLight(0xfff6e8, 2.2);
key.position.set(180, 260, 220);
scene.add(key);
const fill = new THREE.DirectionalLight(0x9fc0ff, 0.7);
fill.position.set(-220, -80, -160);
scene.add(fill);

const grid = new THREE.GridHelper(500, 30, 0x5a7db8, 0x3d5580);
grid.position.y = -14;
scene.add(grid);

// --- Hand model (mm): +Z fingertips, +Y dorsum, +X pinky side ---------------
const PALM = { w: 88, h: 20, len: 102 };
const CFG = {
  thumb:  { x: -44, y: -10, z: 34,  prox: 46, dist: 34, r: 16 },
  index:  { x: -30, y:   2, z: 86,  prox: 44, dist: 29, r: 13 },
  middle: { x: -10, y:   2, z: 96,  prox: 50, dist: 33, r: 14 },
  ring:   { x:  11, y:   2, z: 91,  prox: 47, dist: 31, r: 13 },
  pinky:  { x:  30, y:   0, z: 78,  prox: 36, dist: 25, r: 11 },
};
const C = {
  palm:  0xf2c39b, lower: 0xf9d5ad, upper: 0xfde4c5,
  joint: 0x6f3d22, tip: 0xffead0, wrist: 0xdca97c,
};

const hand = new THREE.Group();       // pose.hand quaternion applied here
scene.add(hand);

// Palm + wrist stub.
const palm = new THREE.Mesh(
  new THREE.BoxGeometry(PALM.w, PALM.h, PALM.len),
  new THREE.MeshStandardMaterial({ color: C.palm, roughness: 0.62, flatShading: true })
);
palm.position.z = PALM.len / 2;
hand.add(palm);
const wrist = new THREE.Mesh(
  new THREE.BoxGeometry(46, 17, 40),
  new THREE.MeshStandardMaterial({ color: C.wrist, roughness: 0.7, flatShading: true })
);
wrist.position.z = -22;
hand.add(wrist);

// Organ capsule: cylinder along +Z, tapered toward the tip.
function segmentMesh(r, len, color) {
  const geo = new THREE.CylinderGeometry(r * 0.78, r, len, 14, 1);
  geo.rotateX(Math.PI / 2);
  return new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color, roughness: 0.6, flatShading: true }));
}

const jointNodes = {};   // joint name -> THREE.Group (rotation targets)

for (const [name, cfg] of Object.entries(CFG)) {
  const mcp = new THREE.Group();
  mcp.position.set(cfg.x, cfg.y, cfg.z);
  hand.add(mcp);

  const lower = segmentMesh(cfg.r, cfg.prox, C.lower);
  lower.position.z = cfg.prox / 2;
  mcp.add(lower);

  const pip = new THREE.Group();                 // dist joint lives at the pip
  pip.position.z = cfg.prox;
  mcp.add(pip);

  const upper = segmentMesh(cfg.r * 0.9, cfg.dist, C.upper);
  upper.position.z = cfg.dist / 2;
  pip.add(upper);

  const tip = new THREE.Mesh(
    new THREE.SphereGeometry(cfg.r * 0.62, 12, 10),
    new THREE.MeshStandardMaterial({ color: C.tip, roughness: 0.55, flatShading: true })
  );
  tip.position.z = cfg.dist;
  pip.add(tip);

  const jr = cfg.r * 1.02;
  const jointBall = new THREE.Mesh(
    new THREE.SphereGeometry(jr * 0.72, 10, 8),
    new THREE.MeshStandardMaterial({ color: C.joint, roughness: 0.55 })
  );
  mcp.add(jointBall);                            // under the fingers

  jointNodes[name + "_prox"] = mcp;
  jointNodes[name + "_dist"] = pip;
}

// --- Camera orbit (no dependencies) ------------------------------------------
let yaw = 0.55, pitch = 0.33, radius = 430;
const TARGET = new THREE.Vector3(0, 62, 40);
let dragging = false, px = 0, py = 0;

renderer.domElement.addEventListener("pointerdown", (e) => {
  dragging = true; px = e.clientX; py = e.clientY;
  renderer.domElement.setPointerCapture(e.pointerId);
});
renderer.domElement.addEventListener("pointermove", (e) => {
  if (!dragging) return;
  yaw -= (e.clientX - px) * 0.005;
  pitch += (e.clientY - py) * 0.005;
  pitch = Math.max(-1.25, Math.min(1.25, pitch));
  px = e.clientX; py = e.clientY;
});
renderer.domElement.addEventListener("pointerup", () => (dragging = false));
renderer.domElement.addEventListener("wheel", (e) => {
  radius *= Math.exp(e.deltaY * 0.0012);
  radius = Math.max(140, Math.min(1200, radius));
  e.preventDefault();
}, { passive: false });

resetBtn.addEventListener("click", () => {
  yaw = 0.55; pitch = 0.33; radius = 430;
});

function updateCamera() {
  camera.position.set(
    TARGET.x + radius * Math.cos(pitch) * Math.sin(yaw),
    TARGET.y + radius * Math.sin(pitch),
    TARGET.z + radius * Math.cos(pitch) * Math.cos(yaw)
  );
  camera.lookAt(TARGET);
}

// --- Sensors HUD --------------------------------------------------------------
const chips = [];
for (let i = 0; i < 11; i++) {
  const d = document.createElement("div");
  d.className = "chip";
  d.dataset.name = "IMU_" + i;
  $("imuChips").appendChild(d);
  chips.push(d);
}
function setSensors(health) {
  const bad = new Set(health?.faulty ?? []);
  let n = health?.n ?? 0;
  chips.forEach((c, i) => {
    c.classList.remove("ok", "bad");
    if (bad.has("IMU_" + i)) c.classList.add("bad");
    else c.classList.add("ok");
  });
  imuCountEl.textContent = n + "/11";
  imuCountEl.className = "count" + (n < 11 ? (n < 9 ? " bad" : " warn") : "");
}

// --- WebSocket ----------------------------------------------------------------
let ws = null;
let lastPoseAt = 0;
let deadzonesDeg = Array(11).fill(0.8);
let lastJitterDeg = Array(11).fill(0);
let mountSettings = {
  yaw_deg: 180,
  invert_yaw: false,
  invert_pitch: false,
  invert_roll: false,
};
const targets = { hand: new THREE.Quaternion(), joints: {} };
let connectionState = "offline";   // offline | connecting | connected

function sendCommand(command) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(command));
}

function updateJitterReadout() {
  const selected = imuSelect.value;
  if (selected === "all") {
    jitterReadout.textContent = `${Math.max(...lastJitterDeg).toFixed(2)}° max`;
  } else {
    jitterReadout.textContent = `${(lastJitterDeg[Number(selected)] ?? 0).toFixed(2)}°`;
  }
}

function syncDeadzoneControls() {
  const selected = imuSelect.value;
  if (selected === "all") {
    const first = deadzonesDeg[0] ?? 0.8;
    const uniform = deadzonesDeg.every((value) => Math.abs(value - first) < 1e-6);
    if (uniform) {
      deadzoneSlider.value = first;
      deadzoneValue.textContent = `${first.toFixed(1)}°`;
    } else {
      deadzoneValue.textContent = "varies";
    }
  } else {
    const value = deadzonesDeg[Number(selected)] ?? 0.8;
    deadzoneSlider.value = value;
    deadzoneValue.textContent = `${value.toFixed(1)}°`;
  }
  updateJitterReadout();
}

function syncMountControls() {
  mountYawSelect.value = String(mountSettings.yaw_deg);
  invertYawInput.checked = mountSettings.invert_yaw;
  invertPitchInput.checked = mountSettings.invert_pitch;
  invertRollInput.checked = mountSettings.invert_roll;
}

function sendMountSettings() {
  mountSettings = {
    yaw_deg: Number(mountYawSelect.value),
    invert_yaw: invertYawInput.checked,
    invert_pitch: invertPitchInput.checked,
    invert_roll: invertRollInput.checked,
  };
  sendCommand({ type: "set_mount", ...mountSettings });
}

function setStatus() {
  if (connectionState === "connected") {
    dotEl.classList.add("on"); dotEl.classList.remove("mid");
    statusTxt.textContent = currentSource === "sim" ? "SIMULATED" : "CONNECTED";
  } else if (connectionState === "connecting") {
    dotEl.classList.remove("on", "mid");
    statusTxt.textContent = "CONNECTING…";
  } else {
    dotEl.classList.remove("on", "mid");
    statusTxt.textContent = "OFFLINE";
  }
}

let currentSource = "hw";

function applyStatus(m) {
  currentSource = m.src || "hw";
  if (Array.isArray(m.deadzones_deg) && m.deadzones_deg.length === 11) {
    deadzonesDeg = m.deadzones_deg;
    syncDeadzoneControls();
  }
  if (m.mount && [0, 90, 180, 270].includes(m.mount.yaw_deg) &&
      typeof m.mount.invert_yaw === "boolean" &&
      typeof m.mount.invert_pitch === "boolean" &&
      typeof m.mount.invert_roll === "boolean") {
    mountSettings = m.mount;
    syncMountControls();
  }
  const t = m.zeroed_at ? new Date(m.zeroed_at * 1000).toLocaleTimeString() : null;
  const who = m.zero_source === "auto" ? "auto" : "manual";
  zeroTxt.textContent = m.zeroed
    ? `captured ${who} · ${t}`
    : "not captured — press Zero with the hand flat";
  setStatus();
}

function applyPose(p) {
  const h = p.hand.q;
  targets.hand.set(h[1], h[2], h[3], h[0]);
  for (const name of JOINT_NAMES) {
    const j = p.joints?.[name]?.q;
    if (!j) continue;
    (targets.joints[name] ??= new THREE.Quaternion()).set(j[1], j[2], j[3], j[0]);
  }
  hzTxt.textContent = p.rate_hz ? p.rate_hz.toFixed(1) : "—";
  setSensors(p.health);
  if (Array.isArray(p.jitter_deg) && p.jitter_deg.length === 11) {
    lastJitterDeg = p.jitter_deg;
    updateJitterReadout();
  }
  lastPoseAt = performance.now();
  if (connectionState !== "connected") {
    connectionState = "connected";
    setStatus();
  }
}

function connect() {
  connectionState = "connecting";
  setStatus();
  try {
    ws = new WebSocket(WS_URL);
  } catch {
    setTimeout(connect, 2000);
    return;
  }
  ws.onopen = () => ws.send(JSON.stringify({ type: "ping" }));
  ws.onmessage = (ev) => {
    let m;
    try { m = JSON.parse(ev.data); } catch { return; }
    if (m.type === "pose") applyPose(m);
    else if (m.type === "status") applyStatus(m);
    else if (m.type === "error") console.warn("Bridge:", m.message);
  };
  ws.onclose = () => {
    connectionState = "offline";
    setStatus();
    setTimeout(connect, 2000);
  };
  ws.onerror = () => { try { ws.close(); } catch {} };
}

zeroBtn.addEventListener("click", () => {
  sendCommand({ type: "zero" });
});
imuSelect.addEventListener("change", syncDeadzoneControls);
deadzoneSlider.addEventListener("input", () => {
  const deg = Number(deadzoneSlider.value);
  deadzoneValue.textContent = `${deg.toFixed(1)}°`;
  const imu = imuSelect.value === "all" ? "all" : Number(imuSelect.value);
  sendCommand({ type: "set_deadzone", imu, deg });
});
resetDeadzonesBtn.addEventListener("click", () => sendCommand({ type: "reset_deadzones" }));
mountYawSelect.addEventListener("change", sendMountSettings);
invertYawInput.addEventListener("change", sendMountSettings);
invertPitchInput.addEventListener("change", sendMountSettings);
invertRollInput.addEventListener("change", sendMountSettings);
syncDeadzoneControls();
syncMountControls();
window.addEventListener("keydown", (e) => {
  if (e.key === "c" || e.key === "C") zeroBtn.click();
});

// --- Animation loop -------------------------------------------------------------
let lastFrame = 0, fpsEma = 0;

function animate(now) {
  requestAnimationFrame(animate);
  const t = now / 1000;
  if (lastFrame) {
    const dt = (now - lastFrame) / 1000;
    fpsEma = fpsEma ? fpsEma + 0.08 * (1 / dt - fpsEma) : 1 / dt;
    if (t % 1 < 0.05) fpsTxt.textContent = fpsEma.toFixed(0);
  }
  const frameDtMs = lastFrame ? Math.min(now - lastFrame, 100) : 16.67;
  lastFrame = now;

  // Frame-rate-independent smoothing; snap on the first pose after a gap.
  const age = now - lastPoseAt;
  const k = age > 600 ? 1 : 1 - Math.exp(-frameDtMs * 0.018);
  if (age > 600) lastPoseAt = now;                 // avoid re-snap every frame

  hand.quaternion.slerp(targets.hand, k);
  for (const [name, node] of Object.entries(jointNodes)) {
    const tg = targets.joints[name];
    if (tg) node.quaternion.slerp(tg, k);
  }

  updateCamera();
  renderer.render(scene, camera);
}

// --- Resize ---------------------------------------------------------------------
function resize() {
  const w = window.innerWidth, h = window.innerHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", resize);
resize();

connect();
requestAnimationFrame(animate);
