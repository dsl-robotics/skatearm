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
const CLEAN = !PREVIEW && new URLSearchParams(location.search).has("clean");

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
let wsOnline = false;       // browser <-> server WebSocket up?
let lastMsg = 0;            // perf-time (ms) of the last telemetry frame
let reconnectMs = 1000;     // current reconnect backoff

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
if (CLEAN) { camera.position.set(-0.5, 2.1, 0.72); controls.target.set(0, 0, 0.06); }

scene.add(new THREE.HemisphereLight(0xffffff, 0x223344, 1.1));
const dir = new THREE.DirectionalLight(0xffffff, 1.4);
dir.position.set(2, -3, 4);
scene.add(dir);
const grid = new THREE.GridHelper(4, 24, 0x2a3240, 0x1b212b);
grid.rotation.x = Math.PI / 2;
grid.position.z = FLOOR_Z;                 // floor under the wheels
if (!CLEAN) scene.add(grid);
const axes = new THREE.AxesHelper(0.22);   // world frame triad on the floor
axes.material.transparent = true;
axes.material.opacity = 0.55;
axes.position.z = FLOOR_Z;
if (!CLEAN) scene.add(axes);

function resize() {
  const vp = $("viewport");
  let w = vp.clientWidth, h = vp.clientHeight;
  if (vp.classList.contains("split")) {            // ego+exo: the twin shares width with the work-cam pane
    const cam = $("cam-pip");
    if (cam && cam.classList.contains("show")) w = Math.max(120, w - cam.offsetWidth);
  }
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
}
window.addEventListener("resize", resize);

function rpyToQuat(rpy) {  // URDF fixed-axis rpy -> quaternion
  return new THREE.Quaternion().setFromEuler(
    new THREE.Euler(rpy[0], rpy[1], rpy[2], "ZYX"));
}

let robotRoots = [];
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
  robotRoots = [];
  for (const name of Object.keys(model.links))
    if (!children.has(name)) { const root = g(name); scene.add(root); robotRoots.push(root); }   // root link(s)

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
    if (tc.dragging || overlaysOn()) return;   // no gizmo while overlays declutter the view
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

function tcpWorld(arm, out) {
  // world position of the TCP: wrist link origin + active tool offset
  const t = state && state.tools && state.tools[arm];
  const o = t ? t.offset_mm : null;
  out.set(o ? o[0] / 1000 : 0, o ? o[1] / 1000 : 0, o ? o[2] / 1000 : 0);
  return eeObjs[arm].localToWorld(out);
}

const _wp = new THREE.Vector3();
function updateEE() {
  for (const arm of ["left", "right"]) {
    if (!eeObjs[arm]) continue;
    tcpWorld(arm, _wp);
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
  cartEls = null;
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
      () => previewServer("waypoint", i, "Waypoint " + (i + 1) + " — glide to this pose",
        () => send({ type: "wp_goto", idx: i }));
    row.querySelector('[data-a="del"]').onclick =
      () => send({ type: "wp_delete", idx: i });
    list.appendChild(row);
  });
}

// ---- program tab (python over the bridge) -----------------------------------
const DEMO_PROGRAM = `# Skate program — drives the SAME safe bridge as the panel.
# rbt.movej(joint, deg)   joint = "L4" (left J4) / "R2" / "H1" / index
# rbt.movel(arm, dx=, dy=, dz=)   nudge the TCP in mm, world axes
# rbt.home() · rbt.gripper(arm, deg) · rbt.waypoint(1) · rbt.wait(s)
# rbt.tcp(arm) · rbt.q() · rbt.status() · print(...)
rbt.home()
rbt.movej("L4", 60)              # left elbow up
for d in (40, 80, 40):           # wave the right forearm
    rbt.movej("R4", d)
rbt.movel("right", dz=60)        # square with the right TCP
rbt.movel("right", dy=-80)
rbt.movel("right", dz=-60)
rbt.movel("right", dy=80)
print("tcp right:", rbt.tcp("right"), "mm")
rbt.home()
`;

const EXAMPLES = {
  "demo": DEMO_PROGRAM,
  "wave right arm": `rbt.home()
for d in (30, 80, 30, 80, 30):
    rbt.movej("R4", d)
rbt.home()
`,
  "raise both arms": `# one pose moves several joints together (no mirror needed)
rbt.home()
rbt.pose({"L2": 35, "R2": 35})     # shoulders out
rbt.pose({"L4": 80, "R4": 80})     # elbows up
rbt.wait(0.5)
rbt.home()
`,
  "pick & place (right)": `# absolute world points in mm — same IK + guard as the gizmo
rbt.home()
rbt.gripper("right", 30)             # open
rbt.moveto("right", 130, 350, 35)    # above the part
rbt.moveto("right", 130, 350, -43)   # down to it
rbt.gripper("right", 0)              # close
rbt.moveto("right", 130, 350, 75)    # lift
rbt.moveto("right", 60, 320, 75)     # over the bin
rbt.gripper("right", 30)             # release
rbt.home()
`,
  "gripper cycle": `for _ in range(3):
    rbt.gripper("right", 30)
    rbt.wait(0.4)
    rbt.gripper("right", 0)
    rbt.wait(0.4)
`,
};

// [label shown, text inserted after "rbt.", first arg to auto-select]
const RBT_API = [
  ["movej(joint, deg)", 'movej("L4", 0)', '"L4"'],
  ["pose({joint: deg, ...})", 'pose({"L4": 0})', '"L4"'],
  ["movel(arm, dx=, dy=, dz=)", 'movel("right", dz=0)', '"right"'],
  ["moveto(arm, x, y, z)", 'moveto("right", 0, 0, 0)', '"right"'],
  ["home()", 'home()', null],
  ["gripper(arm, deg)", 'gripper("right", 0)', '"right"'],
  ["waypoint(i_or_name)", 'waypoint(1)', '1'],
  ["wait(seconds)", 'wait(1.0)', '1.0'],
  ["tcp(arm)", 'tcp("right")', '"right"'],
  ["q()", 'q()', null],
  ["status()", 'status()', null],
];

let progCode = DEMO_PROGRAM;
let progSig = "";

let acHide = () => {};        // closes the autocomplete dropdown (set on build)
let progLastErr = null;       // last error log line we highlighted
let progLastStep = null;      // last paused step line we highlighted

function caretCoords(ta) {     // pixel position of the caret (mirror-div trick)
  const div = document.createElement("div");
  const cs = getComputedStyle(ta);
  for (const p of ["fontFamily", "fontSize", "fontWeight", "lineHeight",
      "letterSpacing", "paddingTop", "paddingLeft", "paddingRight",
      "paddingBottom", "borderWidth", "boxSizing", "tabSize"]) div.style[p] = cs[p];
  div.style.position = "absolute"; div.style.visibility = "hidden";
  div.style.whiteSpace = "pre-wrap"; div.style.wordWrap = "break-word";
  div.style.width = ta.clientWidth + "px";
  div.textContent = ta.value.slice(0, ta.selectionStart);
  const span = document.createElement("span");
  span.textContent = "."; div.appendChild(span);
  document.body.appendChild(div);
  const x = span.offsetLeft, y = span.offsetTop;
  document.body.removeChild(div);
  const r = ta.getBoundingClientRect();
  return { left: r.left + x - ta.scrollLeft, top: r.top + y - ta.scrollTop };
}

function setupProgAutocomplete(ta) {
  let items = [], sel = 0, open = false;
  const box = document.createElement("div");
  box.id = "pg-ac"; box.style.display = "none";
  document.body.appendChild(box);
  const close = () => { open = false; box.style.display = "none"; };
  acHide = close;
  const tokenAt = () => {
    const m = ta.value.slice(0, ta.selectionStart).match(/rbt\.(\w*)$/);
    return m ? m[1] : null;
  };
  const render = () => {
    box.innerHTML = "";
    items.forEach((it, i) => {
      const d = document.createElement("div");
      d.className = "ac-item" + (i === sel ? " sel" : "");
      d.textContent = it[0];
      d.onmousedown = (e) => { e.preventDefault(); accept(i); };
      box.appendChild(d);
    });
  };
  const show = (prefix) => {
    items = RBT_API.filter((a) => a[1].startsWith(prefix));
    if (!items.length) { close(); return; }
    sel = 0; render();
    const c = caretCoords(ta);
    box.style.left = c.left + "px"; box.style.top = (c.top + 18) + "px";
    box.style.display = "block"; open = true;
  };
  const accept = (i) => {
    const it = items[i]; if (!it) { close(); return; }
    const start = ta.selectionStart;
    const m = ta.value.slice(0, start).match(/rbt\.(\w*)$/);
    const from = start - (m ? m[1].length : 0);
    ta.value = ta.value.slice(0, from) + it[1] + ta.value.slice(start);
    progCode = ta.value;
    const rel = it[2] ? it[1].indexOf(it[2]) : -1;
    if (rel >= 0) ta.setSelectionRange(from + rel, from + rel + it[2].length);
    else { const c = from + it[1].length; ta.setSelectionRange(c, c); }
    ta.focus(); close();
  };
  ta.addEventListener("input", () => {
    const t = tokenAt(); if (t !== null) show(t); else close();
  });
  ta.addEventListener("keydown", (e) => {
    if (!open) {
      if ((e.ctrlKey || e.metaKey) && e.key === " ") {
        const t = tokenAt(); if (t !== null) { e.preventDefault(); show(t); }
      }
      return;
    }
    if (e.key === "ArrowDown") { e.preventDefault(); sel = (sel + 1) % items.length; render(); }
    else if (e.key === "ArrowUp") { e.preventDefault(); sel = (sel - 1 + items.length) % items.length; render(); }
    else if (e.key === "Enter" || e.key === "Tab") { e.preventDefault(); accept(sel); }
    else if (e.key === "Escape") { e.preventDefault(); close(); }
  });
  ta.addEventListener("blur", () => setTimeout(close, 120));
  ta.addEventListener("scroll", () => { if (open) close(); });
}

function highlightProgLine(ta, lineNo, focus) {
  if (!ta || !lineNo || lineNo < 1) return;
  const lines = ta.value.split("\n");
  if (lineNo > lines.length) return;
  let start = 0;
  for (let i = 0; i < lineNo - 1; i++) start += lines[i].length + 1;
  try {
    ta.setSelectionRange(start, start + lines[lineNo - 1].length);
    if (focus) ta.focus();
  } catch (_) {}
  const lh = parseFloat(getComputedStyle(ta).lineHeight) || 16;
  ta.scrollTop = Math.max(0, (lineNo - 3) * lh);
}

function buildProgPanel() {
  const wrap = $("joints");
  rows = {};
  cartEls = null;
  acHide(); document.getElementById("pg-ac")?.remove();
  wrap.innerHTML = `
    <div class="panel-head"><span>PROGRAM</span>
      <small>python · every move goes through the safe bridge</small></div>
    <div class="nl-row">
      <input id="pg-nl" type="text" autocomplete="off" spellcheck="false"
             placeholder="Describe a task — &ldquo;raise both arms, then home&rdquo;">
      <button id="pg-gen" title="Generate an rbt program from your description">✦ GEN</button>
    </div>
    <div class="prog-controls">
      <button id="pg-run" title="Run the program (releases a paused one)">▶ RUN</button>
      <button id="pg-step" title="Click to Step — execute exactly one motion command">⏭ STEP</button>
      <button id="pg-stop">■ STOP</button>
      <button id="pg-rec" title="Teach-in: move the robot (sliders / gizmo / cartesian), every settled pose becomes a line of code">● REC</button>
      <span id="pg-state" class="prog-state">idle</span>
    </div>
    <textarea id="pg-code" spellcheck="false"></textarea>
    <div class="prog-controls">
      <button id="pg-save">SAVE…</button>
      <select id="pg-files"></select>
      <button id="pg-load">LOAD</button>
      <select id="pg-examples" title="Load an example program"></select>
    </div>
    <pre id="pg-log"></pre>`;
  const ta = $("pg-code");
  ta.value = progCode;
  ta.oninput = () => (progCode = ta.value);
  if (PREVIEW) {
    for (const id of ["pg-run", "pg-step", "pg-stop", "pg-rec", "pg-save",
                      "pg-load", "pg-examples", "pg-nl", "pg-gen"]) {
      $(id).disabled = true;
      $(id).title = "preview is a recording — run the local server";
    }
    ta.readOnly = true;
    $("pg-log").textContent = "> programs need the local server";
  } else {
    $("pg-run").onclick = () => send({ type: "prog_run", code: progCode });
    $("pg-step").onclick = () => send({ type: "prog_step", code: progCode });
    $("pg-stop").onclick = () => send({ type: "prog_stop" });
    {
      const nlIn = $("pg-nl"), gen = $("pg-gen");
      const genNL = async () => {
        const text = nlIn.value.trim();
        if (!text) return;
        gen.disabled = true;
        const label = gen.textContent;
        gen.textContent = "…";
        try {
          const r = await fetch("/api/nl", {
            method: "POST",
            headers: { "content-type": "application/json" },
            body: JSON.stringify({ text }),
          });
          const d = await r.json();
          if (d.code) {
            progCode = d.code;
            ta.value = d.code;
            $("pg-log").textContent =
              "> generated (" + d.engine + ") — review, then ▶ RUN or ⏭ STEP";
          } else {
            $("pg-log").textContent =
              "x " + (d.error || "could not parse") + (d.hint ? "\n" + d.hint : "");
          }
        } catch (e) {
          $("pg-log").textContent = "x generate failed: " + e;
        } finally {
          gen.disabled = false;
          gen.textContent = label;
        }
      };
      gen.onclick = genNL;
      nlIn.onkeydown = (e) => {
        if (e.key === "Enter") { e.preventDefault(); genNL(); }
      };
    }
    $("pg-rec").onclick = async () => {
      const rec = state && state.prog && state.prog.rec;
      if (rec && rec.on) {
        send({ type: "rec_stop" });
        setTimeout(async () => {
          try {
            const code = await (await fetch("/api/recording")).text();
            if (code.trim()) {
              progCode = (progCode.trim() ? progCode.replace(/\s+$/, "")
                + "\n\n" : "") + code;
              ta.value = progCode;
              ta.scrollTop = ta.scrollHeight;
            }
          } catch (e) { /* server down */ }
        }, 400);
      } else {
        send({ type: "rec_start" });
      }
    };
    setupProgAutocomplete(ta);
    const exSel = $("pg-examples");
    exSel.innerHTML = '<option value="">examples…</option>' +
      Object.keys(EXAMPLES).map((k) => `<option>${k}</option>`).join("");
    exSel.onchange = () => {
      if (EXAMPLES[exSel.value]) { progCode = EXAMPLES[exSel.value]; ta.value = progCode; }
      exSel.value = "";
    };
    $("pg-save").onclick = () => {
      const name = prompt("Program name (letters/digits/_-):");
      if (name) {
        send({ type: "prog_save", name, code: progCode });
        setTimeout(refreshProgFiles, 400);
      }
    };
    $("pg-load").onclick = async () => {
      const sel = $("pg-files");
      if (!sel.value) return;
      try {
        const code = await (await fetch(`/api/programs/${sel.value}`)).text();
        progCode = code;
        ta.value = code;
      } catch (e) { /* server down */ }
    };
    refreshProgFiles();
  }
  progSig = "";
}

async function refreshProgFiles() {
  try {
    const names = await (await fetch("/api/programs")).json();
    const sel = $("pg-files");
    if (sel) sel.innerHTML =
      names.map((n) => `<option value="${n}">${n}</option>`).join("");
  } catch (e) { /* server down */ }
}

function updateProgPanel() {
  const p = state && state.prog;
  if (!p) return;
  const sig = JSON.stringify([p.running, p.paused, p.line, p.n,
                              p.log && p.log.length,
                              p.log && p.log[p.log.length - 1],
                              p.rec && p.rec.on, p.rec && p.rec.n]);
  if (sig === progSig) return;
  progSig = sig;
  const rec = $("pg-rec");
  if (rec && p.rec) {
    rec.textContent = p.rec.on ? `■ REC · ${p.rec.n}` : "● REC";
    rec.className = p.rec.on ? "recording" : "";
  }
  const st = $("pg-state");
  if (st) {
    st.textContent = !p.running ? "idle"
      : p.paused ? `paused → line ${p.line ?? "?"}: ${p.current ?? ""}`
      : `running · cmd #${p.n}${p.line ? " · line " + p.line : ""}`;
    st.className = "prog-state" + (p.running ? (p.paused ? " warn" : " on") : "");
  }
  const run = $("pg-run");
  if (run) run.className = p.running && !p.paused ? "playing" : "";
  const ta = $("pg-code");
  if (ta && !PREVIEW) ta.readOnly = p.running;
  const log = $("pg-log");
  if (log && p.log) {
    log.textContent = p.log.join("\n");
    log.scrollTop = log.scrollHeight;
  }
  if (ta && !PREVIEW) {                       // mark the error / current step line
    const lastLog = (p.log && p.log[p.log.length - 1]) || "";
    const m = /line (\d+)/.exec(lastLog);
    if (/^x/.test(lastLog) && m && lastLog !== progLastErr) {
      progLastErr = lastLog; highlightProgLine(ta, +m[1], true);
    } else if (!/^x/.test(lastLog)) {
      progLastErr = null;
      if (p.paused && p.line && p.line !== progLastStep) {
        progLastStep = p.line; highlightProgLine(ta, p.line, false);
      }
      if (!p.paused) progLastStep = null;
    }
  }
}

function buildPanel() {
  if (curGroup === "seq") { buildSeqPanel(); return; }
  if (curGroup === "prog") { buildProgPanel(); return; }
  const wrap = $("joints");
  wrap.innerHTML = "";
  rows = {};
  cartEls = null;
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
      <button class="jlim" data-l="lo" title="Jump to the lower limit (guard permitting)">⇤</button>
      <button class="jbtn" data-d="-1">−</button>
      <div class="jbar" role="slider" tabindex="0" aria-label="${human}" aria-valuemin="${Math.round(lo*180/Math.PI)}" aria-valuemax="${Math.round(hi*180/Math.PI)}" aria-valuenow="0"><div class="jfill"></div><div class="jthumb"></div></div>
      <button class="jbtn" data-d="1">+</button>
      <button class="jlim" data-l="hi" title="Jump to the upper limit (guard permitting)">⇥</button>
      <div class="jval"><span class="ang">—</span><small class="sub">—</small></div>`;
    wrap.appendChild(row);
    const locked = PREVIEW || legsLockedReal;
    for (const b of row.querySelectorAll(".jlim")) {
      if (locked) { b.disabled = true; continue; }
      b.onclick = () => send({ type: "set_joint", idx,
                               value: b.dataset.l === "lo" ? lo : hi });
    }
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
                sub: row.querySelector(".sub"), bar, lo, hi, dragging: false };
    if (locked) {
      bar.classList.add("disabled");
      bar.tabIndex = -1; bar.setAttribute("aria-disabled", "true");
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
      bar.addEventListener("keydown", (e) => {
        const step = (e.shiftKey ? 8 : 2) * Math.PI / 180;
        const cur = (state && state.targ && state.targ[idx] != null)
          ? state.targ[idx] : ((state && state.q && state.q[idx]) ?? lo);
        let v = null;
        if (e.key === "ArrowRight" || e.key === "ArrowUp") v = cur + step;
        else if (e.key === "ArrowLeft" || e.key === "ArrowDown") v = cur - step;
        else if (e.key === "Home") v = lo;
        else if (e.key === "End") v = hi;
        else return;
        e.preventDefault();
        v = Math.min(hi, Math.max(lo, v));
        send({ type: "set_joint", idx, value: v });
        r.thumb.style.left = `${(100 * (v - lo)) / (hi - lo)}%`;
        bar.setAttribute("aria-valuenow", Math.round((v * 180) / Math.PI));
      });
    }
    rows[idx] = r;
  }
  if (legsLockedReal) {
    const note = document.createElement("div");
    note.className = "panel-note";
    note.textContent = "Lower chain locked in REAL mode — balance belongs to the firmware.";
    wrap.prepend(note);
  }
  if (curGroup === "left" || curGroup === "right") buildCartBlock(wrap);
}

// ---- cartesian jog + tool (TCP) block --------------------------------------
let cartEls = null;          // {x,y,z} readout spans for the open arm tab
let cartStepMm = 5;
let toolsCache = null;       // {name: [x,y,z] mm}
let cartTimers = [];

function stopCartTimers() {
  for (const t of cartTimers) clearInterval(t);
  cartTimers = [];
}

async function refreshTools(sel, arm) {
  if (PREVIEW) return;
  try {
    toolsCache = await (await fetch("/api/tools")).json();
  } catch (e) { return; }
  if (!sel) return;
  const cur = state && state.tools && state.tools[arm]
    ? state.tools[arm].name : "flange";
  sel.innerHTML = Object.keys(toolsCache).map((n) =>
    `<option value="${n}"${n === cur ? " selected" : ""}>${n}</option>`).join("");
}

function buildCartBlock(wrap) {
  const arm = curGroup;
  const blk = document.createElement("div");
  blk.innerHTML = `
    <div class="panel-head"><span>CARTESIAN · TCP</span>
      <small>step-jog the tool point · world axes</small></div>
    <div id="cart"></div>
    <div class="cart-foot">
      <label>STEP</label>
      <select id="cart-step">
        ${[1, 5, 20, 50].map((v) => `<option value="${v}"${v === cartStepMm
          ? " selected" : ""}>${v} mm</option>`).join("")}
      </select>
      <span class="vsep"></span>
      <label>TOOL</label>
      <select id="tool-sel"></select>
      <button id="tool-def" title="Define a named TCP offset (mm, wrist frame)">+</button>
      <button id="tool-del" title="Delete the selected tool">✕</button>
    </div>`;
  wrap.appendChild(blk);
  const cart = blk.querySelector("#cart");
  cartEls = {};
  const axes = [["X", 0], ["Y", 1], ["Z", 2]];
  for (const [nm, ax] of axes) {
    const row = document.createElement("div");
    row.className = "cart-row";
    row.innerHTML = `<b class="ax-${nm.toLowerCase()}">${nm}</b>
      <button class="jbtn" data-d="-1">−</button>
      <span class="cart-val">—</span>
      <button class="jbtn" data-d="1">+</button>`;
    cart.appendChild(row);
    cartEls[ax] = row.querySelector(".cart-val");
    for (const b of row.querySelectorAll(".jbtn")) {
      if (PREVIEW) { b.disabled = true; continue; }
      const dir = parseInt(b.dataset.d);
      const fire = () => {
        const delta = [0, 0, 0];
        delta[ax] = (dir * cartStepMm) / 1000;
        send({ type: "cart_step", arm, delta });
      };
      b.addEventListener("pointerdown", (e) => {
        e.preventDefault();
        fire();
        cartTimers.push(setInterval(fire, 150));
      });
      for (const ev of ["pointerup", "pointerleave", "pointercancel"])
        b.addEventListener(ev, stopCartTimers);
    }
  }
  blk.querySelector("#cart-step").onchange = (e) =>
    (cartStepMm = parseInt(e.target.value));
  const sel = blk.querySelector("#tool-sel");
  if (PREVIEW) {
    sel.disabled = true;
    blk.querySelector("#tool-def").disabled = true;
    blk.querySelector("#tool-del").disabled = true;
  } else {
    refreshTools(sel, arm);
    sel.onchange = () => send({ type: "tool_set", arm, name: sel.value });
    blk.querySelector("#tool-def").onclick = () => {
      const name = prompt("Tool name (letters/digits/_-):");
      if (!name) return;
      const xyz = prompt("TCP offset x,y,z in mm (wrist frame):", "0,0,120");
      if (!xyz) return;
      const v = xyz.split(",").map(Number);
      if (v.length !== 3 || v.some(isNaN)) { alert("need three numbers"); return; }
      send({ type: "tool_def", name, xyz_mm: v });
      send({ type: "tool_set", arm, name });
      setTimeout(() => refreshTools(sel, arm), 400);
    };
    blk.querySelector("#tool-del").onclick = () => {
      if (sel.value === "flange") return;
      send({ type: "tool_del", name: sel.value });
      setTimeout(() => refreshTools(sel, arm), 400);
    };
  }
}

const z = (x) => (Math.abs(x) < 0.005 ? 0 : x);   // kill -0.0 flicker
const deg = (r) => (z(r) * 180 / Math.PI).toFixed(1) + "°";

const _tcp = new THREE.Vector3();
function updateCartReadout() {
  if (!cartEls || !eeObjs[curGroup]) return;
  const p = tcpWorld(curGroup, _tcp);
  for (const [ax, el] of Object.entries(cartEls))
    el.textContent = `${(p.getComponent(+ax) * 1000).toFixed(0)} mm`;
}

function updatePanel() {
  if (!state) return;
  if (curGroup === "seq") { updateSeqPanel(); return; }
  if (curGroup === "prog") { updateProgPanel(); return; }
  updateCartReadout();
  const q = state.q || [], dq = state.dq || [], temps = state.temps || [];
  for (const [idx, r] of Object.entries(rows)) {
    const a = q[idx], t = temps[idx], targ = state.targ && state.targ[idx];
    if (a === undefined || a === null) continue;
    const span = r.hi - r.lo || 1;
    r.fill.style.width = `${(100 * (a - r.lo)) / span}%`;
    if (!r.dragging && targ !== null && targ !== undefined)
      r.thumb.style.left = `${(100 * (targ - r.lo)) / span}%`;
    r.ang.textContent = deg(a);
    if (r.bar) r.bar.setAttribute("aria-valuenow",
      Math.round(((targ != null && targ !== undefined ? targ : a) * 180) / Math.PI));
    const tcls = t > 50 ? "temp-bad" : t > 40 ? "temp-warn" : "temp-ok";
    r.sub.className = "sub " + tcls;
    r.sub.textContent =
      `${z(dq[idx] || 0).toFixed(2)} r/s · ${t ? t.toFixed(0) : "—"}°C`;
  }
}

// ---------------------------------------------------------------- top bar
function chip(id, on, txtOn, txtOff, offClass = "bad") {
  const el = $(id);
  el.textContent = on ? txtOn : txtOff;
  el.className = "chip " + (on ? "on" : offClass);
}

function updateTop() {
  if (!state) return;
  const vp = $("viewport");
  if (vp) {
    vp.classList.toggle("cue-estop", !!state.estop);
    vp.classList.toggle("cue-dampened", !state.estop && !state.live);
    vp.classList.toggle("cue-live", !state.estop && !!state.live);
  }
  chip("chip-link", state.connected, "LINK", "NO LINK");
  const lat = $("chip-lat");
  if (lat) {
    const ms = rttMs || frameMs;                   // prefer real RTT, fall back to freshness
    if (wsOnline && ms) {
      lat.style.display = "";
      lat.textContent = Math.round(ms) + "ms";
      lat.className = "chip " + (ms > 260 ? "bad" : ms > 120 ? "warn" : "");
    } else lat.style.display = "none";
  }
  const bw = $("chip-bw");
  if (bw) {
    if (wsOnline && kbps) {
      bw.style.display = "";
      bw.textContent = kbps >= 1000 ? (kbps / 1024).toFixed(1) + " MB/s"
                                    : Math.round(kbps) + " KB/s";
    } else bw.style.display = "none";
  }
  chip("chip-armed", state.armed, "ARMED", "ARMING…", "");
  chip("chip-live", state.live, "LIVE", "DAMPENED", "warn");
  const gc = $("chip-guard");
  if (gc && state.guard) {
    gc.style.display = state.guard.on ? "" : "none";
    gc.textContent = state.guard.blocking ? "LIMIT" : "GUARD";
    gc.className = "chip " + (state.guard.blocking ? "warn" : "on");
  }
  const sc = $("chip-sing");
  if (sc) {
    const mv = state.manip ? Object.values(state.manip).filter(v => v != null) : [];
    const lo = mv.length ? Math.min(...mv) : null;
    sc.style.display = (lo != null && lo < 0.06) ? "" : "none";
    sc.className = "chip warn";
  }
  const rc = $("chip-rec");
  if (rc) {
    const rec = state.prog && state.prog.rec;
    rc.style.display = rec && rec.on ? "" : "none";
    if (rec && rec.on) rc.textContent = `REC · ${rec.n}`;
    rc.className = "chip bad";
  }
  const hc = $("chip-home");
  if (hc) {
    const show = state.homing || state.routing;
    hc.style.display = show ? "" : "none";
    hc.textContent = state.routing ? "ROUTING" : "HOMING";
    hc.className = "chip on";
  }
  const cc = $("chip-contact");
  if (cc) {
    const ct = state.contact && state.contact.tripped;
    cc.style.display = ct ? "" : "none";
    cc.textContent = ct && state.contact.joint != null
      ? `CONTACT · J${state.contact.joint}` : "CONTACT";
    cc.className = "chip bad";
  }
  const tmax = state.temps ? Math.max(...state.temps) : 0;
  const tEl = $("chip-temp");
  tEl.textContent = `T ${tmax.toFixed(0)}°C`;
  tEl.className = "chip " + (state.overtemp ? "bad" : tmax > 45 ? "warn" : "");
  const mb = $("btn-mirror");
  if (mb) mb.classList.toggle("on", !!state.mirror);   // icon + label kept; .on = active
  const cb = $("btn-carry");
  if (cb) {
    cb.classList.toggle("on", !!state.carry);
    const cp = $("carry-pad"); if (cp) cp.style.display = state.carry ? "" : "none";
  }
  $("mode-sim").className = state.mode === "sim" ? "active sim" : "";
  $("mode-real").className = state.mode === "real" ? "active real" : "";
  $("foot-mode").textContent = `mode: ${state.mode}`;
  const es = $("btn-estop");
  es.textContent = state.estop ? "RESUME" : "E-STOP";
  es.className = state.estop ? "resume" : "";
  // v0.4 bug: a local `tc` (temp chip) shadowed the gizmo — detach never ran
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
      : (state.contact && state.contact.tripped)
          ? "DAMPENED — contact reflex: click CONTACT to reset"
      : !state.connected ? "waiting for telemetry…" : "DAMPENED";
  } else banner.className = "";
}

$("btn-estop").onclick = () =>
  send({ type: state && state.estop ? "resume" : "estop" });
let previewViz = null, previewPending = null, ghostGroup = null;
function clearGhost() {
  if (!ghostGroup) return;
  ghostGroup.traverse((o) => { if (o.isMesh && o.material) o.material.dispose(); });
  scene.remove(ghostGroup); ghostGroup = null;
}
function makeGhost(q) {          // translucent robot at the target joint pose
  clearGhost();
  if (!q || !robotRoots.length) return;
  const grp = new THREE.Group(), map = new Map();
  const walk = (a, b) => { map.set(a, b); for (let i = 0; i < a.children.length; i++) walk(a.children[i], b.children[i]); };
  for (const root of robotRoots) { const c = root.clone(true); walk(root, c); grp.add(c); }
  grp.traverse((o) => { if (o.isMesh) {
    o.material = new THREE.MeshBasicMaterial({ color: 0x3fc463, transparent: true, opacity: 0.2, depthWrite: false });
    o.renderOrder = 6;
  } });
  for (const [idx, j] of Object.entries(jointGroups)) {
    const cg = map.get(j.grp); if (!cg) continue;
    cg.quaternion.copy(j.baseQuat).multiply(new THREE.Quaternion().setFromAxisAngle(j.axis, q[idx] || 0));
  }
  scene.add(grp); ghostGroup = grp;
}
function clearPreview() {
  if (previewViz) { scene.remove(previewViz); previewViz = null; }
  clearGhost();
  previewPending = null;
  if ($("approve-bar")) $("approve-bar").style.display = "none";
}
function showPreview(text, opts) {          // opts: {q?, tcp?, onApprove}
  clearPreview();
  opts = opts || {};
  if (opts.q) makeGhost(opts.q);
  if (opts.tcp) {
    const grp = new THREE.Group();
    for (const arm in opts.tcp) {
      const t = opts.tcp[arm];
      const ring = new THREE.Mesh(new THREE.RingGeometry(0.042, 0.052, 28),
        new THREE.MeshBasicMaterial({ color: 0x3fc463, transparent: true, opacity: 0.8,
          side: THREE.DoubleSide, depthTest: false }));
      ring.position.set(t[0], t[1], t[2]); ring.renderOrder = 7; grp.add(ring);
    }
    scene.add(grp); previewViz = grp;
  }
  previewPending = { onApprove: opts.onApprove };
  if ($("approve-text")) $("approve-text").textContent = text;
  if ($("approve-bar")) $("approve-bar").style.display = "flex";
}
async function previewServer(action, idx, text, onApprove) {
  if (PREVIEW || previewPending) return;
  try {
    const d = await fetch("/api/preview?action=" + action + (idx != null ? "&i=" + idx : ""))
      .then((r) => r.json());
    if (d && d.q) showPreview(text, { q: d.q, tcp: d.tcp, onApprove });
    else onApprove();
  } catch (_) { onApprove(); }
}
if ($("btn-home")) $("btn-home").onclick = () =>
  previewServer("home", null, "HOME — glide arms to the safe pose", () => send({ type: "home" }));
if ($("approve-go")) $("approve-go").onclick = () => {
  const p = previewPending; clearPreview();
  if (p && p.onApprove) p.onApprove();
};
if ($("approve-cancel")) $("approve-cancel").onclick = clearPreview;
document.addEventListener("keydown", (e) => {     // Escape cancels a pending move preview
  if (e.key === "Escape" && previewPending) clearPreview();
});
if ($("chip-contact"))
  $("chip-contact").onclick = () => send({ type: "reset_contact" });
$("btn-mirror").onclick = () =>
  send({ type: "mirror", on: !(state && state.mirror) });
if ($("btn-carry")) {
  $("btn-carry").onclick = () =>
    send({ type: state && state.carry ? "carry_release" : "carry_grab" });
  document.querySelectorAll("#carry-pad button").forEach(b => {
    b.onclick = () => {
      const d = b.dataset.cd.split(",").map(Number), s = 0.02;
      send({ type: "carry_step", delta: [d[0] * s, d[1] * s, d[2] * s] });
    };
  });
}
if ($("btn-trace")) {
  $("btn-trace").onclick = () => {
    traceOn = !traceOn;
    $("btn-trace").classList.toggle("on", traceOn);
    for (const t of Object.values(traces)) t.line.visible = traceOn;
  };
  $("btn-clear-trace").onclick = clearTraces;
}

// ---- rendered robot camera (MJPEG panel) --------------------------------
if (!PREVIEW && $("btn-cam")) {
  const pip = $("cam-pip"), img = $("cam-img"), sel = $("cam-sel");
  let camOn = false;
  const start = () =>
    (img.src = "/camstream?cam=" + encodeURIComponent(sel.value) + "&t=" + Date.now());
  const stop = () => img.removeAttribute("src");
  fetch("/api/cameras").then((r) => r.json()).then((d) => {
    sel.innerHTML = "";
    for (const c of d.cameras || []) {
      const o = document.createElement("option");
      o.value = c; o.textContent = c;
      if (c === d.current) o.selected = true;
      sel.appendChild(o);
    }
    if (!(d.cameras && d.cameras.length)) $("btn-cam").disabled = true;
    updateCamActions();
  }).catch(() => ($("btn-cam").disabled = true));
  $("btn-cam").onclick = () => {
    camOn = !camOn;
    $("btn-cam").classList.toggle("on", camOn);
    pip.classList.toggle("show", camOn);
    if (!camOn && $("viewport").classList.contains("split")) {   // turning the camera off exits split
      $("viewport").classList.remove("split");
      if ($("cam-expand")) $("cam-expand").classList.remove("on");
    }
    camOn ? start() : stop();
    resize();
  };
  const mark = $("cam-mark"), info = $("cam-info");
  const CAM_W = 640, CAM_H = 480;
  const showMark = (px) => {
    if (px) {
      mark.style.left = (px[0] / CAM_W * 100) + "%";
      mark.style.top = (px[1] / CAM_H * 100) + "%";
      mark.style.display = "block";
    } else mark.style.display = "none";
  };
  const updateCamActions = () => {                 // pick actions only on the work camera
    const work = sel.value === "cam_work";
    for (const id of ["cam-detect", "cam-pick", "cam-servo", "cam-smart"])
      if ($(id)) $(id).style.display = work ? "" : "none";
    if (!work && $("cam-obj")) $("cam-obj").style.display = "none";
  };
  const toWork = () => { sel.value = "cam_work"; updateCamActions(); if (camOn) start(); };
  sel.onchange = () => { showMark(null); updateCamActions(); if (camOn) start(); };
  if ($("cam-expand")) $("cam-expand").onclick = () => {
    const vp = $("viewport"), willSplit = !vp.classList.contains("split");
    if (willSplit && !camOn) $("btn-cam").click();              // split needs the camera on
    pip.style.left = pip.style.top = pip.style.right = pip.style.bottom = pip.style.width = "";  // drop any dragged position
    vp.classList.toggle("split", willSplit);
    $("cam-expand").classList.toggle("on", willSplit);
    $("cam-expand").title = willSplit ? "Collapse the split (floating camera)"
                                      : "Expand the camera (ego + exo, side by side)";
    $("cam-expand").setAttribute("aria-pressed", willSplit);
    $("cam-expand").setAttribute("aria-label", willSplit ? "Collapse split, float the camera"
                                                         : "Expand to ego + exo split view");
    resize();
  };
  $("cam-detect").onclick = async () => {
    toWork();
    info.textContent = "detecting…";
    try {
      const d = await (await fetch("/api/detect")).json();
      if (d.found) {
        showMark(d.pixel);
        info.textContent = "target  " + d.world_mm.map((v) => v.toFixed(0)).join("  ") + " mm";
      } else {
        showMark(null);
        info.textContent = "no target" + (d.error ? " (" + d.error + ")" : "");
      }
    } catch (e) { info.textContent = "detect failed"; }
  };
  $("cam-pick").onclick = async () => {
    toWork();
    info.textContent = "pick…";
    try {
      const d = await (await fetch("/api/pick", { method: "POST" })).json();
      if (d.found) {
        showMark(d.pixel);
        info.textContent = "picking  " + d.world_mm.map((v) => v.toFixed(0)).join("  ") +
          " mm" + (d.ran ? "" : " — press RESUME");
      } else {
        showMark(null);
        info.textContent = "no target" + (d.error ? " (" + d.error + ")" : "");
      }
    } catch (e) { info.textContent = "pick failed"; }
  };
  $("cam-servo").onclick = async () => {
    toWork();
    info.textContent = "visual servo…";
    try {
      const d = await (await fetch("/api/servo_pick", { method: "POST" })).json();
      if (d.found) {
        info.textContent = "servo pick  " +
          (d.world_mm ? d.world_mm.map((v) => v.toFixed(0)).join("  ") + " mm" : "") +
          " · img " + d.image_err_px + "px" + (d.ran ? "" : " — press RESUME");
      } else {
        showMark(null);
        info.textContent = "no target" + (d.error ? " (" + d.error + ")" : "");
      }
    } catch (e) { info.textContent = "servo failed"; }
  };
  const runSmartPick = async () => {
    info.textContent = "smart pick…";
    try {
      const sel = $("cam-obj");
      const q = sel && sel.value !== "" ? "?target=" + encodeURIComponent(sel.value) : "";
      const d = await (await fetch("/api/smart_pick" + q, { method: "POST" })).json();
      if (d.found && d.ran) {
        info.textContent = "smart pick  " + (d.label ? d.label + " · " : "") +
          (d.center_mm ? d.center_mm.map((v) => v.toFixed(0)).join(" ") + " mm" : "");
        graspOn = true; if ($("btn-grasp")) $("btn-grasp").classList.add("on");
        await buildGrasp();
      } else if (d.feasible === false) {
        info.textContent = d.error || "object too wide for the gripper";
        graspOn = true; await buildGrasp();
      } else {
        info.textContent = d.error ? "smart pick: " + d.error : "no object — armed?";
      }
    } catch (e) { info.textContent = "smart pick failed"; }
  };
  $("cam-smart").onclick = async () => {
    toWork();
    graspOn = true; if ($("btn-grasp")) $("btn-grasp").classList.add("on");
    const ok = await buildGrasp();                // show what we'd grasp first
    if (!ok) { info.textContent = "no object to pick"; return; }
    const sel = $("cam-obj");
    const label = sel && sel.selectedOptions[0] ? sel.selectedOptions[0].textContent : "object";
    showPreview("Smart pick: " + label + " ?", { onApprove: runSmartPick });
  };
} else if ($("btn-cam")) {
  $("btn-cam").disabled = true;
  $("btn-cam").title = "preview is a recording — run the local server";
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

// ---- keyboard shortcuts + legend (operator hotkeys) ----------------------
(function keyboardShortcuts() {
  const MAP = { " ": "btn-estop", h: "btn-home", c: "btn-cam", e: "cam-expand",
                t: "btn-trace", d: "btn-dex", p: "btn-pcl", g: "btn-grasp",
                l: "btn-layers", m: "btn-mirror" };
  const legend = $("keys-legend");
  const setLegend = (show) => { if (legend) legend.style.display = show ? "flex" : "none"; };
  const legendOpen = () => legend && legend.style.display === "flex";
  const typing = (el) => el && (el.tagName === "INPUT" || el.tagName === "TEXTAREA" ||
                                el.tagName === "SELECT" || el.isContentEditable);
  document.addEventListener("keydown", (e) => {
    if (e.ctrlKey || e.metaKey || e.altKey) return;
    if (e.key === "Escape") { setLegend(false); return; }
    if (e.key === "?") { e.preventDefault(); setLegend(!legendOpen()); return; }
    if (typing(document.activeElement)) return;            // don't hijack text fields
    if (legendOpen()) return;                              // legend up: only Esc / ? act
    if (document.activeElement && document.activeElement.tagName === "BUTTON" &&
        (e.key === " " || e.key === "Enter")) return;      // let a focused button self-activate
    if (e.key >= "1" && e.key <= "6") {                    // arm-group tabs
      const t = document.querySelectorAll("#tabs div")[+e.key - 1];
      if (t) { t.click(); e.preventDefault(); }
      return;
    }
    const id = MAP[e.key.toLowerCase()];
    if (id) { const b = $(id); if (b && !b.disabled) { b.click(); e.preventDefault(); } }
  });
  if ($("btn-keys")) $("btn-keys").onclick = () => setLegend(!legendOpen());
  if ($("keys-close")) $("keys-close").onclick = () => setLegend(false);
  if (legend) legend.addEventListener("click", (e) => { if (e.target === legend) setLegend(false); });
})();

// ---------------------------------------------------------------- data in
let lastFrame = 0, frameMs = 0;        // telemetry freshness (inter-frame gap)
let rttMs = 0;                          // real round-trip (ping echo)
let bwBytes = 0, kbps = 0, bwT = 0;     // rolling downlink bandwidth
function onState(s) {
  const _n = performance.now();
  if (lastFrame) frameMs = frameMs ? frameMs * 0.8 + (_n - lastFrame) * 0.2 : (_n - lastFrame);
  lastFrame = _n;
  if (s.pong != null) {                 // real round-trip from the ping echo
    const rtt = _n - s.pong;
    rttMs = rttMs ? rttMs * 0.7 + rtt * 0.3 : rtt;
  }
  const modeChanged = state && state.mode !== s.mode;
  state = s;
  setAngles(s.q || s.targ);
  updatePanel();
  updateTop();
  if (modeChanged) buildPanel();
}

function setOffline(off, msg) {
  document.body.classList.toggle("offline", off);   // dims/freezes stale chips
  if (off) {
    const b = $("banner");
    b.className = "dampened";
    b.textContent = msg || "no connection to the server — retrying…";
  }
}

function connectWS() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => { wsOnline = true; reconnectMs = 1000; lastMsg = performance.now(); };
  ws.onmessage = (e) => {
    lastMsg = performance.now();
    bwBytes += (e.data && e.data.length) || 0;     // rolling downlink bandwidth
    if (document.body.classList.contains("offline")) setOffline(false);
    onState(JSON.parse(e.data));
  };
  ws.onerror = () => { try { ws.close(); } catch (_) {} };   // -> onclose -> retry
  ws.onclose = () => {
    wsOnline = false;
    setOffline(true, "no connection to the server — retrying… " +
      "(server down, or missing: pip install websockets)");
    setTimeout(connectWS, reconnectMs);
    reconnectMs = Math.min(Math.round(reconnectMs * 1.6), 5000);   // gentle backoff
  };
}

// stale-telemetry watchdog: socket looks open but frames stopped -> force reconnect
setInterval(() => {
  if (!PREVIEW && wsOnline && performance.now() - lastMsg > 1500) {
    setOffline(true, "telemetry stalled — reconnecting…");
    try { ws.close(); } catch (_) {}
  }
}, 500);

// link probe: real round-trip ping + downlink bandwidth (1 Hz)
setInterval(() => {
  if (PREVIEW) return;
  if (wsOnline) send({ type: "ping", t: performance.now() });
  const now = performance.now();
  if (bwT) { const dt = (now - bwT) / 1000; if (dt > 0) kbps = (bwBytes / 1024) / dt; }
  bwBytes = 0; bwT = now;
}, 1000);

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
    for (const id of ["btn-estop", "btn-home", "btn-mirror", "btn-carry"]) {
      $(id).disabled = true;
      $(id).title = "preview is a recording — run the local server to control";
    }
    startPlayback();
  } else {
    connectWS();
  }
})();


// ---- manipulability heat-map (dexterity cloud) --------------------------
// A point cloud over each arm's reachable workspace, coloured by
// manipulability (reciprocal Jacobian condition number): blue = near-singular,
// warm = isotropic/dexterous. Fetched once from /api/reachmap, then toggled.
let dexCloud = null, dexOn = false, dexBusy = false;
function manipColor(t) {                    // t in [0,1] -> cold..hot
  t = Math.max(0, Math.min(1, t));
  const stops = [[0.13, 0.20, 0.60], [0.00, 0.62, 0.78], [0.22, 0.80, 0.35],
                 [0.96, 0.85, 0.22], [0.92, 0.28, 0.18]];
  const x = t * (stops.length - 1), i = Math.min(Math.floor(x), stops.length - 2),
        f = x - i, a = stops[i], b = stops[i + 1];
  return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
}
async function buildDexCloud() {
  const pos = [], col = [];
  for (const arm of ["left", "right"]) {
    const r = await fetch(`/api/reachmap?arm=${arm}&n=2500`).then(x => x.json()).catch(() => null);
    if (!r || !r.points) continue;
    for (const p of r.points) {
      pos.push(p[0], p[1], p[2]);
      const c = manipColor(p[3] / 0.45);    // 0.45 ~ top of the manip range
      col.push(c[0], c[1], c[2]);
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
  geo.setAttribute("color", new THREE.Float32BufferAttribute(col, 3));
  dexCloud = new THREE.Points(geo, new THREE.PointsMaterial({
    size: 0.013, vertexColors: true, transparent: true, opacity: 0.85,
    sizeAttenuation: true, depthWrite: false }));
  dexCloud.frustumCulled = false;
  scene.add(dexCloud);
}

// ---- overlay coordination: declutter when heavy overlays are shown --------
// One heavy cloud at a time (DEX <-> PCL), and hide the drag-gizmo + wrist
// handles while any overlay is up so the twin stays readable.
function overlaysOn() { return pclOn || dexOn || graspOn; }
function setOverlayHint() {
  const el = $("overlay-hint"); if (!el) return;
  const parts = [];
  if (dexOn) parts.push("DEX — reach dexterity · warm = agile, blue = near-singular");
  if (pclOn) parts.push("PCL — work-camera depth → 3D points");
  if (graspOn) parts.push("GRASP — synthesised top-down grasps · azure = selected");
  el.textContent = parts.join("      ·      ");
  el.style.display = parts.length ? "block" : "none";
}
function syncGizmo() {
  const hide = overlaysOn();
  for (const m of Object.values(markers)) m.visible = !hide;
  if (hide && tc && tc.object) tc.detach();
  setOverlayHint();
}

if ($("btn-dex")) {
  $("btn-dex").onclick = async () => {
    if (PREVIEW || dexBusy) return;
    dexOn = !dexOn;
    $("btn-dex").classList.toggle("on", dexOn);
    if (dexOn && pclOn) {                 // one heavy cloud at a time
      pclOn = false; if (pclCloud) pclCloud.visible = false;
      $("btn-pcl").classList.remove("on");
    }
    if (dexOn && !dexCloud) {
      dexBusy = true; $("btn-dex").classList.add("busy");
      await buildDexCloud();
      $("btn-dex").classList.remove("busy"); dexBusy = false;
    }
    if (dexCloud) dexCloud.visible = dexOn;
    syncGizmo();
  };
}


// ---- work-camera point cloud --------------------------------------------
// What the work camera sees, back-projected to a coloured 3D cloud in the twin
// (depth render -> world points, each coloured by its RGB pixel).
let pclCloud = null, pclOn = false, pclBusy = false;
async function buildPcl() {
  const r = await fetch("/api/pointcloud?stride=4").then(x => x.json()).catch(() => null);
  if (!r || !r.points || !r.points.length) return;
  const pos = [], col = [];
  for (const p of r.points) { pos.push(p[0], p[1], p[2]); col.push(p[3], p[4], p[5]); }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
  geo.setAttribute("color", new THREE.Float32BufferAttribute(col, 3));
  pclCloud = new THREE.Points(geo, new THREE.PointsMaterial({
    size: 0.008, vertexColors: true, sizeAttenuation: true }));
  pclCloud.frustumCulled = false;
  scene.add(pclCloud);
}
if ($("btn-pcl")) {
  $("btn-pcl").onclick = async () => {
    if (PREVIEW || pclBusy) return;
    pclOn = !pclOn;
    $("btn-pcl").classList.toggle("on", pclOn);
    if (pclOn && dexOn) {                 // one heavy cloud at a time
      dexOn = false; if (dexCloud) dexCloud.visible = false;
      $("btn-dex").classList.remove("on");
    }
    if (pclOn && !pclCloud) {
      pclBusy = true; $("btn-pcl").classList.add("busy");
      await buildPcl();
      $("btn-pcl").classList.remove("busy"); pclBusy = false;
    }
    if (pclCloud) pclCloud.visible = pclOn;
    syncGizmo();
  };
}



// ---- smart-pick grasp overlay -------------------------------------------
// A grasp synthesised on the work-camera cloud (table removed, object
// clustered): the object footprint, the parallel-jaw line (closes across the
// minor axis) and the top-down approach, drawn in the twin. /api/grasp returns
// mm; the twin is world metres. Azure = graspable, amber = too wide for the jaws.
let graspViz = null, graspOn = false, graspBusy = false, graspObjs = [];
function clearGrasp() {
  if (graspViz) { scene.remove(graspViz); graspViz = null; }
}
function graspLine(ptsMm, color, opacity) {
  const pos = [];
  for (const p of ptsMm) pos.push(p[0] / 1000, p[1] / 1000, p[2] / 1000);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
  const ln = new THREE.Line(geo, new THREE.LineBasicMaterial({
    color, transparent: true, opacity: opacity == null ? 0.95 : opacity,
    depthTest: false }));
  ln.renderOrder = 6;
  return ln;
}
function selectedGraspId() {
  const sel = $("cam-obj");
  if (sel && sel.value !== "") return parseInt(sel.value);
  return graspObjs.length ? graspObjs[0].id : -1;
}
const GRASP_COLOUR = {
  magenta: 0xe23bd0, red: 0xe5534b, orange: 0xfb8c00, yellow: 0xf4c430,
  green: 0x43c463, cyan: 0x26c6da, blue: 0x4f86f7, purple: 0x9b59d0,
  white: 0xe8e8ec, grey: 0x9aa3b2, black: 0x5a606c,
};
function objColour(g) {
  return GRASP_COLOUR[g.colour] != null ? GRASP_COLOUR[g.colour] : 0x37c8ff;
}
function graspQuad(cornersMm, hex, opacity) {        // filled footprint (selected)
  const v = cornersMm.map((p) => [p[0] / 1000, p[1] / 1000, p[2] / 1000]);
  const pos = [...v[0], ...v[1], ...v[2], ...v[0], ...v[2], ...v[3]];
  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
  const m = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({
    color: hex, transparent: true, opacity, side: THREE.DoubleSide,
    depthWrite: false, depthTest: false }));
  m.renderOrder = 5;
  return m;
}
function makeLabel(text, hex, bright) {              // floating 3D text sprite
  const cv = document.createElement("canvas"), ctx = cv.getContext("2d");
  const font = "bold 42px Inter, system-ui, sans-serif";
  ctx.font = font;
  cv.width = Math.ceil(ctx.measureText(text).width) + 48; cv.height = 68;
  ctx.font = font; ctx.textBaseline = "middle";
  ctx.fillStyle = bright ? "rgba(10,12,16,0.88)" : "rgba(10,12,16,0.5)";
  ctx.fillRect(0, 0, cv.width, cv.height);
  ctx.fillStyle = "#" + (hex >>> 0).toString(16).padStart(6, "0");
  if (!bright) ctx.globalAlpha = 0.65;
  ctx.fillText(text, 24, cv.height / 2 + 2);
  const tex = new THREE.CanvasTexture(cv);
  tex.minFilter = THREE.LinearFilter;
  const spr = new THREE.Sprite(new THREE.SpriteMaterial({
    map: tex, transparent: true, depthTest: false, depthWrite: false }));
  spr.renderOrder = 8;
  const h = bright ? 0.05 : 0.042;
  spr.scale.set(h * cv.width / cv.height, h, 1);
  return spr;
}
function drawGraspObjs() {
  clearGrasp();
  if (!graspObjs.length) return;
  const selId = selectedGraspId();
  const grp = new THREE.Group();
  for (const g of graspObjs) {
    const on = g.id === selId;
    const col = objColour(g);
    const jaw = g.feasible ? col : 0xff8a3d;        // amber jaws if too wide
    const f = g.footprint, c = g.center_mm;
    if (on) grp.add(graspQuad(f, col, 0.16));
    grp.add(graspLine([f[0], f[1], f[2], f[3], f[0]], col, on ? 0.95 : 0.4));
    grp.add(graspLine([g.jaws[0], g.jaws[1]], jaw, on ? 1.0 : 0.5));
    grp.add(graspLine([c, [c[0], c[1], c[2] + 95]], col, on ? 0.7 : 0.3));
    const lab = makeLabel(g.label + (g.feasible ? "" : " · wide"), col, on);
    lab.position.set(c[0] / 1000, c[1] / 1000, (c[2] + 150) / 1000);
    grp.add(lab);
  }
  scene.add(grp);
  graspViz = grp;
}
async function buildGrasp() {
  const r = await fetch("/api/grasps?stride=4").then((x) => x.json()).catch(() => null);
  const info = $("cam-info"), sel = $("cam-obj");
  graspObjs = (r && r.found && r.objects) ? r.objects : [];
  if (sel) {
    const prev = sel.value;
    sel.innerHTML = "";
    for (const g of graspObjs) {
      const o = document.createElement("option");
      o.value = g.id;
      o.textContent = g.label + (g.feasible ? "" : " (wide)");
      sel.appendChild(o);
    }
    if (prev !== "" && graspObjs.some((g) => String(g.id) === prev)) sel.value = prev;
    sel.style.display = graspObjs.length ? "" : "none";
  }
  if (!graspObjs.length) {
    clearGrasp();
    if (info) info.textContent = "grasp: " + ((r && (r.reason || r.error)) || "no object");
    return false;
  }
  drawGraspObjs();
  if (info) info.textContent = graspObjs.length +
    (graspObjs.length > 1 ? " objects: " : " object: ") +
    graspObjs.map((g) => g.label).join(", ");
  return true;
}
if ($("btn-grasp")) {
  $("btn-grasp").onclick = async () => {
    if (PREVIEW || graspBusy) return;
    graspOn = !graspOn;
    $("btn-grasp").classList.toggle("on", graspOn);
    if (graspOn) {
      graspBusy = true; $("btn-grasp").classList.add("busy");
      const ok = await buildGrasp();
      $("btn-grasp").classList.remove("busy"); graspBusy = false;
      if (!ok) { graspOn = false; $("btn-grasp").classList.remove("on"); }
    } else clearGrasp();
    syncGizmo();
  };
}
if ($("cam-obj")) $("cam-obj").onchange = () => { if (graspOn) drawGraspObjs(); };


// ---- cam PiP: draggable by its bar so it doesn't cover the twin -----------
(function makePipDraggable() {
  const pip = $("cam-pip"), bar = $("cam-bar");
  if (!pip || !bar) return;
  let drag = null;
  bar.addEventListener("pointerdown", (e) => {
    if (e.target.closest("button, select, option")) return;   // leave controls alone
    if ($("viewport").classList.contains("split")) return;     // no dragging while split-docked
    const r = pip.getBoundingClientRect();
    const par = pip.offsetParent ? pip.offsetParent.getBoundingClientRect()
                                 : { left: 0, top: 0 };
    drag = { dx: e.clientX - r.left, dy: e.clientY - r.top, par };
    pip.style.right = "auto"; pip.style.bottom = "auto";
    pip.style.left = (r.left - par.left) + "px";
    pip.style.top = (r.top - par.top) + "px";
    pip.classList.add("dragging");
    try { bar.setPointerCapture(e.pointerId); } catch (_) {}
    e.preventDefault();
  });
  bar.addEventListener("pointermove", (e) => {
    if (!drag) return;
    const par = pip.offsetParent;
    const maxX = (par ? par.clientWidth : window.innerWidth) - pip.offsetWidth - 4;
    const maxY = (par ? par.clientHeight : window.innerHeight) - pip.offsetHeight - 4;
    const x = e.clientX - drag.dx - drag.par.left;
    const y = e.clientY - drag.dy - drag.par.top;
    pip.style.left = Math.max(4, Math.min(x, maxX)) + "px";
    pip.style.top = Math.max(4, Math.min(y, maxY)) + "px";
  });
  const end = (e) => {
    if (!drag) return;
    drag = null; pip.classList.remove("dragging");
    try { bar.releasePointerCapture(e.pointerId); } catch (_) {}
  };
  bar.addEventListener("pointerup", end);
  bar.addEventListener("pointercancel", end);
})();



// ---- overlay layer-tree: eye + opacity per overlay, in one place ---------
(function layersPanel() {
  const pop = $("layers-pop"), btn = $("btn-layers");
  if (!pop || !btn) return;
  const OVL = { trace: "btn-trace", dex: "btn-dex", pcl: "btn-pcl", grasp: "btn-grasp" };
  const refresh = () => {
    for (const r of pop.querySelectorAll(".lyr")) {
      const b = $(OVL[r.dataset.ov]);
      r.classList.toggle("on", !!(b && b.classList.contains("on")));
    }
  };
  const setOpacity = (ov, f) => {
    if (ov === "pcl" && pclCloud) { pclCloud.material.transparent = true; pclCloud.material.opacity = f; }
    if (ov === "dex" && dexCloud) { dexCloud.material.transparent = true; dexCloud.material.opacity = f; }
  };
  btn.onclick = (e) => {
    e.stopPropagation();
    const show = pop.style.display === "none" || !pop.style.display;
    pop.style.display = show ? "block" : "none";
    if (show) refresh();
  };
  for (const r of pop.querySelectorAll(".lyr")) {
    const ov = r.dataset.ov;
    const eye = r.querySelector(".eye");
    if (eye) eye.onclick = () => { const b = $(OVL[ov]); if (b) b.click(); setTimeout(refresh, 40); };
    const opa = r.querySelector(".opa");
    if (opa) opa.oninput = (e) => setOpacity(ov, e.target.value / 100);
  }
  document.addEventListener("click", (e) => {
    if (!e.target.closest("#layers-pop") && !btn.contains(e.target)) pop.style.display = "none";
  });
})();



// ---- view presets: one-click task layouts (overlay combinations) --------
(function viewPresets() {
  const setOv = (id, want) => {
    const b = $(id);
    if (b && b.classList.contains("on") !== want) b.click();
  };
  const P = {
    clean:   { "btn-trace": false, "btn-dex": false, "btn-pcl": false, "btn-grasp": false },
    pick:    { "btn-cam": true, "btn-trace": false, "btn-dex": false, "btn-pcl": false, "btn-grasp": true },
    inspect: { "btn-cam": true, "btn-trace": false, "btn-dex": false, "btn-pcl": true, "btn-grasp": false },
    reach:   { "btn-trace": false, "btn-dex": true, "btn-pcl": false, "btn-grasp": false },
  };
  for (const btn of document.querySelectorAll(".presets button[data-preset]")) {
    btn.onclick = () => { const p = P[btn.dataset.preset]; if (p) for (const id in p) setOv(id, p[id]); };
  }

  // --- saveable custom presets (localStorage) ---
  const TRACK = ["btn-cam", "btn-trace", "btn-dex", "btn-pcl", "btn-grasp"];
  const KEY = "skate.presets.v1";
  const wrap = $("saved-presets"), nameIn = $("preset-name"), saveBtn = $("preset-save");
  if (!wrap || !nameIn || !saveBtn) return;
  const opaInput = (ov) => document.querySelector('.lyr[data-ov="' + ov + '"] .opa');

  const capture = () => {
    const ov = {};
    for (const id of TRACK) { const b = $(id); if (b) ov[id] = b.classList.contains("on"); }
    const opa = {};
    for (const k of ["dex", "pcl"]) { const s = opaInput(k); if (s) opa[k] = +s.value; }
    return { ov, opa };
  };
  const apply = (s) => {
    if (s.ov) for (const id in s.ov) setOv(id, s.ov[id]);
    if (s.opa) for (const k in s.opa) {
      const inp = opaInput(k);
      if (inp) { inp.value = s.opa[k]; inp.dispatchEvent(new Event("input")); }
    }
  };
  const load = () => { try { return JSON.parse(localStorage.getItem(KEY)) || []; } catch (_) { return []; } };
  const store = (a) => { try { localStorage.setItem(KEY, JSON.stringify(a)); } catch (_) {} };

  const render = () => {
    const arr = load();
    wrap.innerHTML = "";
    arr.forEach((p, i) => {
      const chip = document.createElement("div");
      chip.className = "saved-chip";
      const go = document.createElement("button");
      go.className = "chip-apply"; go.textContent = p.name;
      go.title = "apply preset: " + p.name;
      go.setAttribute("aria-label", "Apply preset " + p.name);
      go.onclick = () => apply(p);
      const del = document.createElement("button");
      del.className = "del"; del.textContent = "×";
      del.title = "delete preset";
      del.setAttribute("aria-label", "Delete preset " + p.name);
      del.onclick = () => { const a = load(); a.splice(i, 1); store(a); render(); };
      chip.append(go, del);
      wrap.appendChild(chip);
    });
    wrap.style.display = arr.length ? "grid" : "none";
  };
  const doSave = () => {
    const name = (nameIn.value || "").trim().slice(0, 18);
    if (!name) { nameIn.focus(); return; }
    const arr = load(), st = capture(); st.name = name;
    const ex = arr.findIndex((p) => p.name === name);
    if (ex >= 0) arr[ex] = st; else arr.push(st);
    store(arr); nameIn.value = ""; render();
  };
  saveBtn.onclick = doSave;
  nameIn.onkeydown = (e) => { if (e.key === "Enter") { e.preventDefault(); doSave(); } };
  render();
})();
