/* Skate Commander — frontend.
 * Live mode: talks to the FastAPI server over WebSocket.
 * Preview mode: if window.PREVIEW_DATA is defined (model + recorded frames),
 * runs a playback loop with no backend; meshes degrade to a stick figure. */

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";

const PREVIEW = typeof window.PREVIEW_DATA !== "undefined";
const $ = (id) => document.getElementById(id);

const GROUPS = {
  left:  { label: "LEFT ARM",  idx: [8, 9, 10, 11, 12, 13, 14, 15] },
  right: { label: "RIGHT ARM", idx: [16, 17, 18, 19, 20, 21, 22, 23] },
  head:  { label: "HEAD",      idx: [24, 25] },
  legs:  { label: "LEGS",      idx: [0, 1, 2, 3, 4, 5, 6, 7] },
};
const ROLE = { 9: "abduction", 11: "elbow", 15: "gripper",
               17: "abduction", 19: "elbow", 23: "gripper" };

let model, ws = null, state = null, curGroup = "left";
let jointGroups = {};       // protocol index -> THREE.Group + meta
let limits = {};            // idx -> [lo, hi]
let rows = {};              // idx -> row elements
let eeObjs = {};            // "left"/"right" -> THREE.Object3D (wrist link)
const markers = {};         // "left"/"right" -> gizmo sphere
let draggingArm = null;
let traceOn = true;

// ---------------------------------------------------------------- three.js
const scene = new THREE.Scene();          // transparent: CSS gradient shows
const FLOOR_Z = -0.95;                    // wheel contact plane of skt_v3
const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 50);
camera.up.set(0, 0, 1);
camera.position.set(2.0, -2.0, 0.55);
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
$("viewport").appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, -0.15);         // frame the whole robot

scene.add(new THREE.HemisphereLight(0xffffff, 0x223344, 1.1));
const dir = new THREE.DirectionalLight(0xffffff, 1.4);
dir.position.set(2, -3, 4);
scene.add(dir);
const grid = new THREE.GridHelper(4, 24, 0x2a3240, 0x1b212b);
grid.rotation.x = Math.PI / 2;
grid.position.z = FLOOR_Z;                 // floor under the wheels
scene.add(grid);
const axes = new THREE.AxesHelper(0.22);   // world frame triad on the floor
axes.material.transparent = true;
axes.material.opacity = 0.55;
axes.position.z = FLOOR_Z;
scene.add(axes);

function resize() {
  const w = $("viewport").clientWidth, h = $("viewport").clientHeight;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
}
window.addEventListener("resize", resize);

function rpyToQuat(rpy) {  // URDF fixed-axis rpy -> quaternion
  return new THREE.Quaternion().setFromEuler(
    new THREE.Euler(rpy[0], rpy[1], rpy[2], "ZYX"));
}

function buildRobot() {
  const linkGroup = {};
  const g = (name) => linkGroup[name] ||
    (linkGroup[name] = Object.assign(new THREE.Group(), { name }));
  const children = new Set();
  for (const j of model.joints) {
    const jg = new THREE.Group();
    jg.position.set(...j.xyz);
    const baseQuat = rpyToQuat(j.rpy);
    jg.quaternion.copy(baseQuat);
    jg.add(g(j.child));
    g(j.parent).add(jg);
    children.add(j.child);
    if (j.index !== null && j.index !== undefined) {
      jointGroups[j.index] = { grp: jg, baseQuat,
        axis: new THREE.Vector3(...j.axis).normalize() };
      if (j.lower !== null) limits[j.index] = [j.lower, j.upper];
    }
  }
  for (const name of Object.keys(model.links))
    if (!children.has(name)) scene.add(g(name));   // root link(s)

  // visuals
  const loader = new STLLoader();
  let meshFails = 0;
  for (const [name, link] of Object.entries(model.links)) {
    let fallback = !PREVIEW ? 0 : 1;
    for (const v of link.visuals) {
      if (PREVIEW) continue;
      loader.load(`/meshes/${v.mesh}`, (geo) => {
        const mat = new THREE.MeshStandardMaterial({
          color: v.color ? new THREE.Color(...v.color.slice(0, 3)) : 0x888888,
          metalness: 0.25, roughness: 0.65 });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.position.set(...v.xyz);
        mesh.quaternion.copy(rpyToQuat(v.rpy));
        mesh.scale.set(...v.scale);
        g(name).add(mesh);
      }, undefined, () => {
        meshFails++;
        console.error(`mesh failed: /meshes/${v.mesh}`);
        const ov = $("overlay");
        ov.style.color = "var(--warn)";
        ov.textContent = `⚠ ${meshFails} mesh(es) failed to load — ` +
          "check --model-dir points at skt_v3 with skt_v3_meshes/ " +
          "(stick figure shown instead)";
        if (!fallback++) stickFigure(name, g);
      });
    }
    if (PREVIEW) stickFigure(name, g);
  }

  // wrist (end-effector) objects = child links of the a6 joints
  for (const [arm, lastIdx] of [["left", 14], ["right", 22]]) {
    const j = model.joints.find((j) => j.index === lastIdx);
    if (j) eeObjs[arm] = g(j.child);
  }
  setupGizmo();
  setupTraces();
  resize();
}

// ---- drag-gizmo (cartesian IK teleop) --------------------------------------
let tc = null;
function setupGizmo() {
  if (PREVIEW) return;
  const colors = { left: 0x58a6ff, right: 0xffa657 };
  for (const arm of ["left", "right"]) {
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(0.028, 16, 16),
      new THREE.MeshBasicMaterial({ color: colors[arm], transparent: true,
                                    opacity: 0.65, depthTest: false }));
    m.userData.arm = arm;
    scene.add(m);
    markers[arm] = m;
  }
  tc = new TransformControls(camera, renderer.domElement);
  tc.setMode("translate");
  tc.setSize(0.55);
  tc.setSpace("world");
  scene.add(tc);
  tc.addEventListener("dragging-changed", (e) => {
    controls.enabled = !e.value;
    if (e.value) {
      draggingArm = tc.object ? tc.object.userData.arm : null;
    } else {
      if (draggingArm) send({ type: "ik_clear", arm: draggingArm });
      draggingArm = null;
    }
  });
  let lastSend = 0;
  tc.addEventListener("objectChange", () => {
    if (!draggingArm || !state || !state.live) return;
    const now = performance.now();
    if (now - lastSend < 50) return;          // 20 Hz target stream
    lastSend = now;
    const p = tc.object.position;
    send({ type: "ik_target", arm: draggingArm, pos: [p.x, p.y, p.z] });
  });
  // click a sphere to attach the gizmo; click elsewhere to detach
  const ray = new THREE.Raycaster();
  renderer.domElement.addEventListener("pointerdown", (e) => {
    if (tc.dragging) return;
    const r = renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((e.clientX - r.left) / r.width) * 2 - 1,
      -((e.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ndc, camera);
    const hit = ray.intersectObjects(Object.values(markers), false)[0];
    if (hit) tc.attach(hit.object);
  });
}

// ---- TCP traces -------------------------------------------------------------
const traces = {};
function setupTraces() {
  const colors = { left: 0x58a6ff, right: 0xffa657 };
  for (const arm of ["left", "right"]) {
    const N = 800;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position",
      new THREE.BufferAttribute(new Float32Array(N * 3), 3));
    geo.setDrawRange(0, 0);
    const line = new THREE.Line(geo,
      new THREE.LineBasicMaterial({ color: colors[arm], transparent: true,
                                    opacity: 0.8 }));
    line.frustumCulled = false;
    scene.add(line);
    traces[arm] = { line, n: 0, N, last: new THREE.Vector3(1e9, 0, 0) };
  }
}

function clearTraces() {
  for (const t of Object.values(traces)) {
    t.n = 0;
    t.line.geometry.setDrawRange(0, 0);
    t.last.set(1e9, 0, 0);
  }
}

const _wp = new THREE.Vector3();
function updateEE() {
  for (const arm of ["left", "right"]) {
    if (!eeObjs[arm]) continue;
    eeObjs[arm].getWorldPosition(_wp);
    if (markers[arm] && draggingArm !== arm) markers[arm].position.copy(_wp);
    const t = traces[arm];
    if (t && traceOn && _wp.distanceTo(t.last) > 0.004) {
      const a = t.line.geometry.attributes.position;
      if (t.n < t.N) {
        a.setXYZ(t.n++, _wp.x, _wp.y, _wp.z);
      } else {                                   // ring: shift left
        a.array.copyWithin(0, 3);
        a.setXYZ(t.N - 1, _wp.x, _wp.y, _wp.z);
      }
      a.needsUpdate = true;
      t.line.geometry.setDrawRange(0, t.n);
      t.last.copy(_wp);
    }
  }
}

function stickFigure(name, g) {   // mesh-less degradation (preview mode)
  const mat = new THREE.MeshStandardMaterial({ color: 0x58a6ff,
    metalness: 0.1, roughness: 0.6 });
  g(name).add(new THREE.Mesh(new THREE.SphereGeometry(0.022, 12, 12), mat));
  for (const j of model.joints.filter((j) => j.parent === name)) {
    const v = new THREE.Vector3(...j.xyz);
    if (v.length() < 1e-6) continue;
    const cyl = new THREE.Mesh(
      new THREE.CylinderGeometry(0.012, 0.012, v.length(), 8), mat);
    cyl.position.copy(v.clone().multiplyScalar(0.5));
    cyl.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0),
                                      v.clone().normalize());
    g(name).add(cyl);
  }
}

function setAngles(q) {
  if (!q) return;
  for (const [idx, j] of Object.entries(jointGroups)) {
    const quat = new THREE.Quaternion()
      .setFromAxisAngle(j.axis, q[idx] || 0);
    j.grp.quaternion.copy(j.baseQuat).multiply(quat);
  }
}

(function animate() {
  requestAnimationFrame(animate);
  controls.update();
  updateEE();
  renderer.render(scene, camera);
})();

// ---------------------------------------------------------------- panel
function send(obj) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj)); }

let seqLoopWanted = false;
let lastSeqSig = "";

function buildSeqPanel() {
  const wrap = $("joints");
  rows = {};
  wrap.innerHTML = `
    <div class="panel-head"><span>SEQUENCE</span>
      <small>record poses · play them back</small></div>
    <div class="seq-controls">
      <button id="sq-add" title="Record the current pose">● ADD POSE</button>
      <button id="sq-play">▶ PLAY</button>
      <button id="sq-loop">LOOP: OFF</button>
      <button id="sq-stop">■ STOP</button>
    </div>
    <div class="seq-controls">
      <button id="sq-save">SAVE…</button>
      <select id="sq-files"></select>
      <button id="sq-load">LOAD</button>
      <button id="sq-clear">CLEAR ALL</button>
    </div>
    <div id="seq-list"></div>
    <div class="seq-hint">Jog or drag the robot into a pose, press
    <b>● ADD POSE</b>, repeat. <b>▶ PLAY</b> glides through the list
    (any manual input or E-STOP interrupts playback). Sequences are saved
    on the server in <code>sequences/</code>.</div>`;
  $("sq-add").onclick = () => send({ type: "wp_add" });
  $("sq-play").onclick = () => send({ type: "wp_play", loop: seqLoopWanted });
  $("sq-stop").onclick = () => send({ type: "wp_stop" });
  $("sq-loop").onclick = () => {
    seqLoopWanted = !seqLoopWanted;
    $("sq-loop").textContent = `LOOP: ${seqLoopWanted ? "ON" : "OFF"}`;
  };
  $("sq-clear").onclick = () => send({ type: "wp_clear" });
  $("sq-save").onclick = () => {
    const name = prompt("Sequence name (letters/digits/_-):");
    if (name) { send({ type: "wp_save", name }); setTimeout(refreshSeqFiles, 400); }
  };
  $("sq-load").onclick = () => {
    const sel = $("sq-files");
    if (sel.value) send({ type: "wp_load", name: sel.value });
  };
  refreshSeqFiles();
  lastSeqSig = "";
}

async function refreshSeqFiles() {
  if (PREVIEW) return;
  try {
    const names = await (await fetch("/api/sequences")).json();
    const sel = $("sq-files");
    if (sel) sel.innerHTML =
      names.map((n) => `<option value="${n}">${n}</option>`).join("");
  } catch (e) { /* server down; banner already shows it */ }
}

function updateSeqPanel() {
  const seq = state && state.seq;
  if (!seq) return;
  const sig = JSON.stringify([seq.names, seq.idx, seq.playing, seq.active]);
  if (sig === lastSeqSig) return;
  lastSeqSig = sig;
  const play = $("sq-play");
  if (play) play.className = seq.playing ? "playing" : "";
  const list = $("seq-list");
  if (!list) return;
  list.innerHTML = "";
  if (!seq.names.length) {
    list.innerHTML = `<div class="seq-empty">No poses recorded yet.<br>
      Jog or drag the robot into a pose, then press <b>● ADD POSE</b>.</div>`;
    return;
  }
  seq.names.forEach((nm, i) => {
    const row = document.createElement("div");
    row.className = "seqrow" + (seq.active && seq.idx === i ? " active" : "");
    row.innerHTML = `<span class="nm">${i + 1}. ${nm}</span>
      <button data-a="goto" title="Glide to this pose">▶</button>
      <button data-a="del" title="Delete">✕</button>`;
    row.querySelector('[data-a="goto"]').onclick =
      () => send({ type: "wp_goto", idx: i });
    row.querySelector('[data-a="del"]').onclick =
      () => send({ type: "wp_delete", idx: i });
    list.appendChild(row);
  });
}

function buildPanel() {
  if (curGroup === "seq") { buildSeqPanel(); return; }
  const wrap = $("joints");
  wrap.innerHTML = "";
  rows = {};
  const head = document.createElement("div");
  head.className = "panel-head";
  head.innerHTML = `<span>${GROUPS[curGroup].label}</span>
    <small>${GROUPS[curGroup].idx.length} joints · hold ± or drag the thumb</small>`;
  wrap.appendChild(head);
  const legsLockedReal = curGroup === "legs" && state && state.mode === "real";
  for (const [k, idx] of GROUPS[curGroup].idx.entries()) {
    const jname = model.joint_names[idx];
    const [lo, hi] = limits[idx] || [-3.14, 3.14];
    const row = document.createElement("div");
    row.className = "jrow";
    const human = `J${k + 1}${ROLE[idx] ? " · " + ROLE[idx] : ""}`;
    row.innerHTML = `
      <div class="jname" title="protocol index ${idx}"><b>${human}</b>${jname}</div>
      <button class="jbtn" data-d="-1">−</button>
      <div class="jbar"><div class="jfill"></div><div class="jthumb"></div></div>
      <button class="jbtn" data-d="1">+</button>
      <div class="jval"><span class="ang">—</span><small class="sub">—</small></div>`;
    wrap.appendChild(row);
    const locked = PREVIEW || legsLockedReal;
    for (const b of row.querySelectorAll(".jbtn")) {
      if (locked) {
        b.disabled = true;
        if (PREVIEW) b.title = "preview is a recording — run the local server";
        continue;
      }
      const d = parseInt(b.dataset.d);
      const stop = () => send({ type: "jog_stop", idx });
      b.addEventListener("pointerdown", (e) => {
        e.preventDefault(); send({ type: "jog_start", idx, dir: d }); });
      b.addEventListener("pointerup", stop);
      b.addEventListener("pointerleave", stop);
      b.addEventListener("pointercancel", stop);
    }
    // draggable slider: thumb = commanded target, fill = actual position
    const bar = row.querySelector(".jbar");
    const r = { fill: row.querySelector(".jfill"),
                thumb: row.querySelector(".jthumb"),
                ang: row.querySelector(".ang"),
                sub: row.querySelector(".sub"), lo, hi, dragging: false };
    if (locked) {
      bar.classList.add("disabled");
    } else {
      const valAt = (e) => {
        const rect = bar.getBoundingClientRect();
        const f = Math.min(1, Math.max(0, (e.clientX - rect.left) / rect.width));
        return lo + f * (hi - lo);
      };
      let lastSend = 0;
      const sendVal = (e, force) => {
        const now = performance.now();
        if (!force && now - lastSend < 40) return;
        lastSend = now;
        const v = valAt(e);
        send({ type: "set_joint", idx, value: v });
        r.thumb.style.left = `${(100 * (v - lo)) / (hi - lo)}%`;
      };
      bar.addEventListener("pointerdown", (e) => {
        e.preventDefault();
        r.dragging = true;
        bar.setPointerCapture(e.pointerId);
        sendVal(e, true);
      });
      bar.addEventListener("pointermove", (e) => {
        if (r.dragging) sendVal(e);
      });
      const end = (e) => {
        if (r.dragging) { r.dragging = false; sendVal(e, true); }
      };
      bar.addEventListener("pointerup", end);
      bar.addEventListener("pointercancel", end);
    }
    rows[idx] = r;
  }
  if (legsLockedReal) {
    const note = document.createElement("div");
    note.className = "panel-note";
    note.textContent = "Lower chain locked in REAL mode — balance belongs to the firmware.";
    wrap.prepend(note);
  }
}

const z = (x) => (Math.abs(x) < 0.005 ? 0 : x);   // kill -0.0 flicker
const deg = (r) => (z(r) * 180 / Math.PI).toFixed(1) + "°";

function updatePanel() {
  if (!state) return;
  if (curGroup === "seq") { updateSeqPanel(); return; }
  const q = state.q || [], dq = state.dq || [], temps = state.temps || [];
  for (const [idx, r] of Object.entries(rows)) {
    const a = q[idx], t = temps[idx], targ = state.targ && state.targ[idx];
    if (a === undefined || a === null) continue;
    const span = r.hi - r.lo || 1;
    r.fill.style.width = `${(100 * (a - r.lo)) / span}%`;
    if (!r.dragging && targ !== null && targ !== undefined)
      r.thumb.style.left = `${(100 * (targ - r.lo)) / span}%`;
    r.ang.textContent = deg(a);
    const tcls = t > 50 ? "temp-bad" : t > 40 ? "temp-warn" : "temp-ok";
    r.sub.className = "sub " + tcls;
    r.sub.textContent =
      `${z(dq[idx] || 0).toFixed(2)} r/s · ${t ? t.toFixed(0) : "—"}°C`;
  }
}

// ---------------------------------------------------------------- top bar
function chip(id, on, txtOn, txtOff, badWhenOff = true) {
  const el = $(id);
  el.textContent = on ? txtOn : txtOff;
  el.className = "chip " + (on ? "on" : badWhenOff ? "bad" : "");
}

function updateTop() {
  if (!state) return;
  chip("chip-link", state.connected, "LINK", "NO LINK");
  chip("chip-armed", state.armed, "ARMED", "ARMING…", false);
  chip("chip-live", state.live, "LIVE", "DAMPENED");
  const gc = $("chip-guard");
  if (gc && state.guard) {
    gc.style.display = state.guard.on ? "" : "none";
    gc.textContent = state.guard.blocking ? "LIMIT" : "GUARD";
    gc.className = "chip " + (state.guard.blocking ? "warn" : "on");
  }
  const tmax = state.temps ? Math.max(...state.temps) : 0;
  const tc = $("chip-temp");
  tc.textContent = `T ${tmax.toFixed(0)}°C`;
  tc.className = "chip " + (state.overtemp ? "bad" : tmax > 45 ? "warn" : "");
  $("mode-sim").className = state.mode === "sim" ? "active sim" : "";
  $("mode-real").className = state.mode === "real" ? "active real" : "";
  $("foot-mode").textContent = `mode: ${state.mode}`;
  const es = $("btn-estop");
  es.textContent = state.estop ? "RESUME" : "E-STOP";
  es.className = state.estop ? "resume" : "";
  if (tc && tc.object && !state.live) tc.detach();   // no gizmo while dampened
  const banner = $("banner");
  if (PREVIEW) { banner.className = "preview";
    banner.textContent = "PREVIEW — a recording plays back, controls are " +
      "disabled, meshes are simplified · run the local server for the real " +
      "cockpit";
  } else if (!state.live) { banner.className = "dampened";
    banner.textContent = state.estop
      ? "DAMPENED — press RESUME to enable motion"
      : state.overtemp ? "DAMPENED — overtemp latch"
      : !state.connected ? "waiting for telemetry…" : "DAMPENED";
  } else banner.className = "";
}

$("btn-estop").onclick = () =>
  send({ type: state && state.estop ? "resume" : "estop" });
$("btn-home").onclick = () => send({ type: "home" });
if ($("btn-trace")) {
  $("btn-trace").onclick = () => {
    traceOn = !traceOn;
    $("btn-trace").textContent = `TRACE: ${traceOn ? "ON" : "OFF"}`;
    for (const t of Object.values(traces)) t.line.visible = traceOn;
  };
  $("btn-clear-trace").onclick = clearTraces;
}
$("mode-sim").onclick = () => send({ type: "set_mode", mode: "sim" });
$("mode-real").onclick = () => {
  if (confirm("Switch to REAL robot? It will stay DAMPENED until you press RESUME."))
    send({ type: "set_mode", mode: "real" });
};
for (const tab of document.querySelectorAll("#tabs div")) {
  tab.onclick = () => {
    document.querySelectorAll("#tabs div").forEach((t) =>
      t.classList.remove("active"));
    tab.classList.add("active");
    curGroup = tab.dataset.group;
    buildPanel();
  };
}

// ---------------------------------------------------------------- data in
function onState(s) {
  const modeChanged = state && state.mode !== s.mode;
  state = s;
  setAngles(s.q || s.targ);
  updatePanel();
  updateTop();
  if (modeChanged) buildPanel();
}

function connectWS() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (e) => onState(JSON.parse(e.data));
  ws.onclose = () => {
    const banner = $("banner");
    banner.className = "dampened";
    banner.textContent = "no connection to the server — retrying… " +
      "(server down, or missing: pip install websockets)";
    setTimeout(connectWS, 1000);
  };
}

function startPlayback() {
  const frames = window.PREVIEW_DATA.frames;
  const FRAME_MS = 100;                       // recorded at 10 Hz
  const t0 = performance.now();
  const lerp = (a, b, t) =>
    a && b ? a.map((v, i) => v + (b[i] - v) * t) : a || b;
  setInterval(() => {
    const x = ((performance.now() - t0) / FRAME_MS) % frames.length;
    const i = Math.floor(x), t = x - i, j = (i + 1) % frames.length;
    const a = frames[i], b = frames[j];
    onState({ ...a, q: lerp(a.q, b.q, t), dq: lerp(a.dq, b.dq, t),
              temps: lerp(a.temps, b.temps, t),
              targ: lerp(a.targ, b.targ, t) });
  }, 33);                                     // smooth 30 fps interpolation
}

// ---------------------------------------------------------------- boot
(async function boot() {
  model = PREVIEW ? window.PREVIEW_DATA.model
        : await (await fetch("/api/model")).json();
  buildRobot();
  buildPanel();
  if (PREVIEW) {
    for (const id of ["btn-estop", "btn-home"]) {
      $(id).disabled = true;
      $(id).title = "preview is a recording — run the local server to control";
    }
    startPlayback();
  } else {
    connectWS();
  }
})();
