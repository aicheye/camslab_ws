'use strict';

// Column order of a sample row. Matches SAMPLE_FIELDS in server.py.
// DIST is ||p* - p||, (XR, YR) is p*. XR and YR are null before a live robot has a reference.
const T = 0, X = 1, Y = 2, QX = 3, QY = 4, V = 5, W = 6, DIST = 7, XR = 8, YR = 9;
const FIELDS = ['t', 'x', 'y', 'qx', 'qy', 'v', 'w', 'dist', 'xr', 'yr'];
const MAX_LIVE_SAMPLES = 20000;

// Robot footprint drawn on the map, in metres.
const ROBOT_LENGTH = 0.30;
const ROBOT_WIDTH = 0.18;

// Parameters shown, in this order; others follow. shape has its own select.
const PARAM_LABELS = {
  x0: 'x0 [m]',
  y0: 'y0 [m]',
  theta0: 'θ0 [rad]',
  circle_speed: 'circle speed [m/s]',
  kappa: 'circle κ [1/m]',
  a: 'gerono a [m]',
  period: 'gerono period [s]',
  k_par: 'k∥ [1/s]',
  k_perp: 'k⊥ [1/m²]',
  k_q: 'k_q [1/s]',
  kappa_max: 'κ max [1/m]',
  w_max: 'ω* max [rad/s]',
  v_max: 'v max [m/s]',
  path_time: 'path time [s]',
  dt: 'dt [s]',
  t_max: 't max [s]',
};

const STATUS_ICONS = { idle: '●', running: '▶', reached: '✓', error: '✕' };

const state = {
  mode: 'batch',
  canSetPose: false,
  robotName: '',  // label of the robot in samples; empty draws none
  samples: [],
  cursor: -1,   // index of the sample the robot is drawn at
  hover: -1,    // index of the sample under the pointer
  playing: false,
  playT: 0,
  lastFrame: 0,
  fleet: {},      // other robots: {name: [row, ...]}, rows as in samples, latest last
  selected: '',   // robot shown in plots and readout: '' is the one in samples
  path: [],       // [[x, y], ...], p*(t) sampled over the run
  initial: null,  // {x, y, theta}
  params: {},
};

const $ = (id) => document.getElementById(id);
const world = $('world');
const plots = [
  { canvas: $('plot-dist'), col: DIST, refs: () => [] },
  { canvas: $('plot-v'), col: V, refs: () => [] },
  { canvas: $('plot-w'), col: W, refs: () => [] },
];

let ws = null;
let dirty = true;
let colors = {};
let view = null;  // world-to-screen transform of the last map draw

// ---------------------------------------------------------------- connection

function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => { $('conn').textContent = 'connected'; };
  ws.onclose = () => {
    $('conn').textContent = 'disconnected, retrying';
    setTimeout(connect, 1000);
  };
  ws.onmessage = (ev) => onMessage(JSON.parse(ev.data));
}

function send(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
}

// Rows and drawn index of the robot selected for plots and readout.
function selRows() {
  return state.selected ? (state.fleet[state.selected] || []) : state.samples;
}
function selCursor() {
  return state.selected ? selRows().length - 1 : state.cursor;
}

// Fixed colour per robot: the one in samples is series-1, others follow in name order.
const FLEET_COLORS = ['series-3', 'series-4', 'series-5'];
function robotColor(name) {
  if (!name) return colors['series-1'];
  const i = Object.keys(state.fleet).sort().indexOf(name);
  return colors[FLEET_COLORS[Math.max(i, 0) % FLEET_COLORS.length]];
}

function syncRobotPicker() {
  const names = Object.keys(state.fleet).sort();
  const select = $('robot-select');
  const wanted = ['', ...names];
  if (select.options.length !== wanted.length) {
    select.replaceChildren(...wanted.map((name) => {
      const opt = document.createElement('option');
      opt.value = name;
      opt.textContent = name || state.robotName || 'robot';
      return opt;
    }));
    select.value = state.selected;
  }
  $('robot-pick').hidden = names.length === 0;
}

$('robot-select').addEventListener('change', (ev) => {
  state.selected = ev.target.value;
  state.hover = -1;
  dirty = true;
});

function onMessage(msg) {
  if (msg.type === 'hello') {
    state.mode = msg.mode;
    state.canSetPose = msg.can_set_pose;
    state.robotName = msg.robot_name || '';
    $('backend').textContent = msg.backend;
    $('playback').hidden = msg.mode !== 'batch';
    $('stop').disabled = msg.mode === 'batch';
    $('initial-set').disabled = !msg.can_set_pose;
    $('initial-set').hidden = !msg.can_set_pose;
  } else if (msg.type === 'config') {
    if (msg.params) Object.assign(state.params, msg.params);
    if (msg.path) state.path = msg.path;
    if (msg.initial) state.initial = msg.initial;
    syncInputs();
  } else if (msg.type === 'samples') {
    onSamples(msg);
  } else if (msg.type === 'fleet') {
    if (msg.reset) state.fleet = {};
    for (const [name, rows] of Object.entries(msg.robots)) {
      const trail = state.fleet[name] || (state.fleet[name] = []);
      for (const row of rows) trail.push(row);
      if (trail.length > MAX_LIVE_SAMPLES) trail.splice(0, trail.length - MAX_LIVE_SAMPLES);
    }
    syncRobotPicker();
  } else if (msg.type === 'status') {
    $('status').dataset.state = msg.state;
    $('status-icon').textContent = STATUS_ICONS[msg.state] || '●';
    $('status-text').textContent = msg.state;
    $('status-message').textContent = msg.message || '';
  }
  dirty = true;
}

function onSamples(msg) {
  if (msg.reset) {
    state.samples = [];
    state.cursor = -1;
    state.hover = -1;
    state.playing = false;
  }
  for (const row of msg.samples) state.samples.push(row);
  if (state.mode === 'live') {
    if (state.samples.length > MAX_LIVE_SAMPLES) {
      state.samples.splice(0, state.samples.length - MAX_LIVE_SAMPLES);
    }
    state.cursor = state.samples.length - 1;
  } else if (msg.reset && state.samples.length > 0) {
    startPlayback();
  }
  $('scrub').max = Math.max(0, state.samples.length - 1);
}

// ------------------------------------------------------------------ playback

function startPlayback() {
  const speed = Number($('speed').value);
  if (speed === 0) {
    state.cursor = state.samples.length - 1;
    state.playing = false;
  } else {
    state.cursor = 0;
    state.playT = state.samples[0][T];
    state.playing = true;
    state.lastFrame = performance.now();
  }
}

function advancePlayback(now) {
  const speed = Number($('speed').value) || 1;
  state.playT += ((now - state.lastFrame) / 1000) * speed;
  state.lastFrame = now;
  const s = state.samples;
  while (state.cursor < s.length - 1 && s[state.cursor + 1][T] <= state.playT) {
    state.cursor++;
  }
  if (state.cursor >= s.length - 1) state.playing = false;
}

$('play').addEventListener('click', () => {
  if (state.samples.length === 0) return;
  if (state.playing) {
    state.playing = false;
  } else if (state.cursor >= state.samples.length - 1) {
    startPlayback();
  } else {
    state.playT = state.samples[state.cursor][T];
    state.lastFrame = performance.now();
    state.playing = true;
  }
  dirty = true;
});

$('scrub').addEventListener('input', (ev) => {
  state.playing = false;
  state.cursor = Number(ev.target.value);
  dirty = true;
});

// ------------------------------------------------------------------ controls

$('start').addEventListener('click', () => {
  if ($('init-random-on-start').checked && state.canSetPose) randomInitial();
  send({ type: 'start' });
});
$('stop').addEventListener('click', () => send({ type: 'stop' }));
$('reset').addEventListener('click', () => send({ type: 'reset' }));
document.addEventListener('keydown', (ev) => {
  if (ev.key === 'Escape') send({ type: 'stop' });
});

function numberOf(id) {
  const v = parseFloat($(id).value);
  return Number.isFinite(v) ? v : null;
}

function sendInitial() {
  const x = numberOf('init-x'), y = numberOf('init-y'), deg = numberOf('init-th');
  if (x === null || y === null || deg === null) return;
  state.initial = { x, y, theta: (deg * Math.PI) / 180 };
  send({ type: 'set_initial', ...state.initial });
  dirty = true;
}
for (const id of ['init-x', 'init-y', 'init-th']) $(id).addEventListener('change', sendInitial);

// Random pose with x, y >= 0 inside the middle 80% of the current map view, then reset to it.
function randomInitial() {
  const r = () => (Math.random() - 0.5) * 0.8 * camera.span;
  $('init-x').value = +Math.max(0, camera.cx + r()).toFixed(2);
  $('init-y').value = +Math.max(0, camera.cy + r()).toFixed(2);
  $('init-th').value = Math.round(Math.random() * 360 - 180);
  sendInitial();
  send({ type: 'reset' });
}
$('init-random').addEventListener('click', randomInitial);

// "random on Start" is remembered per browser. It defaults to on.
try { $('init-random-on-start').checked = localStorage.getItem('gtg-random-on-start') !== 'false'; }
catch (err) { $('init-random-on-start').checked = true; }
$('init-random-on-start').addEventListener('change', (ev) => {
  try { localStorage.setItem('gtg-random-on-start', String(ev.target.checked)); } catch (err) { /* ignore */ }
});

function setValue(input, value) {
  if (document.activeElement !== input) input.value = value;
}

$('shape').addEventListener('change', (ev) => {
  state.params.shape = ev.target.value;
  send({ type: 'set_params', params: { shape: ev.target.value } });
});

function syncInputs() {
  if (typeof state.params.shape === 'string') setValue($('shape'), state.params.shape);
  if (state.initial) {
    setValue($('init-x'), state.initial.x);
    setValue($('init-y'), state.initial.y);
    setValue($('init-th'), +((state.initial.theta * 180) / Math.PI).toFixed(3));
  }
  const box = $('params');
  const keys = Object.keys(state.params).filter((k) => typeof state.params[k] === 'number');
  const order = Object.keys(PARAM_LABELS);
  const rank = (k) => (order.includes(k) ? order.indexOf(k) : order.length);
  keys.sort((a, b) => rank(a) - rank(b));
  for (const key of keys) {
    const value = state.params[key];
    let input = box.querySelector(`input[data-key="${key}"]`);
    if (!input) {
      const label = document.createElement('label');
      label.textContent = (PARAM_LABELS[key] || key) + ' ';
      input = document.createElement('input');
      input.type = 'number';
      input.step = 'any';
      input.dataset.key = key;
      input.addEventListener('change', () => {
        const v = parseFloat(input.value);
        if (!Number.isFinite(v)) return;
        state.params[key] = v;
        send({ type: 'set_params', params: { [key]: v } });
        dirty = true;
      });
      label.appendChild(input);
      box.appendChild(label);
    }
    setValue(input, value);
  }
}

$('csv').addEventListener('click', () => {
  const lines = [FIELDS.join(',')];
  for (const row of selRows()) lines.push(row.join(','));
  const blob = new Blob([lines.join('\n') + '\n'], { type: 'text/csv' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = `tracking-${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
  a.click();
  URL.revokeObjectURL(a.href);
});

// ------------------------------------------------------------ canvas helpers

function readColors() {
  const css = getComputedStyle(document.documentElement);
  for (const name of ['surface', 'border', 'grid', 'text-primary', 'text-secondary',
    'text-muted', 'series-1', 'series-2', 'series-3', 'series-4', 'series-5']) {
    colors[name] = css.getPropertyValue('--' + name).trim();
  }
  dirty = true;
}

function fitCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(rect.width * dpr));
  const h = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);
  return { ctx, w: rect.width, h: rect.height };
}

function niceStep(raw) {
  const pow = Math.pow(10, Math.floor(Math.log10(raw)));
  const frac = raw / pow;
  return (frac <= 1 ? 1 : frac <= 2 ? 2 : frac <= 5 ? 5 : 10) * pow;
}

function fixed(value, digits) {
  const text = value.toFixed(digits);
  return Number(text) === 0 ? (0).toFixed(digits) : text;
}

function formatTick(value, step) {
  const digits = Math.max(0, -Math.floor(Math.log10(step)));
  return value.toFixed(Math.min(digits, 6));
}

// ----------------------------------------------------------------------- map

// The map view is fixed. It changes only on wheel zoom, drag pan, or the Fit button.
// span is the length of the shorter canvas side [m]. While corner is true, the centre
// (cx, cy) is derived so that the origin sits at the bottom-left corner of the map.
const DEFAULT_CAMERA = { cx: 0, cy: 0, span: 6, corner: true };
const CORNER_INSET = 0.08;  // fraction of span left outside the origin for labels
const camera = loadCamera();

function loadCamera() {
  try {
    const saved = JSON.parse(localStorage.getItem('gtg-camera-v2'));
    if (saved && [saved.cx, saved.cy, saved.span].every(Number.isFinite) && saved.span > 0) {
      return saved;
    }
  } catch (err) { /* storage blocked or empty */ }
  return { ...DEFAULT_CAMERA };
}

function saveCamera() {
  try { localStorage.setItem('gtg-camera-v2', JSON.stringify(camera)); } catch (err) { /* ignore */ }
  dirty = true;
}

function fitCamera() {
  let x0 = 0, x1 = 0, y0 = 0, y1 = 0;  // the origin stays in view
  const include = (x, y) => {
    x0 = Math.min(x0, x); x1 = Math.max(x1, x);
    y0 = Math.min(y0, y); y1 = Math.max(y1, y);
  };
  for (const s of state.samples) include(s[X], s[Y]);
  for (const trail of Object.values(state.fleet)) for (const s of trail) include(s[X], s[Y]);
  for (const [x, y] of state.path) include(x, y);
  if (state.initial) include(state.initial.x, state.initial.y);
  camera.cx = (x0 + x1) / 2;
  camera.cy = (y0 + y1) / 2;
  camera.span = Math.max(x1 - x0, y1 - y0, 2) * 1.25;
  camera.corner = false;
  saveCamera();
}

function computeView(w, h) {
  const scale = Math.min(w, h) / camera.span;
  if (camera.corner) {
    camera.cx = w / scale / 2 - CORNER_INSET * camera.span;
    camera.cy = h / scale / 2 - CORNER_INSET * camera.span;
  }
  return {
    scale, span: camera.span,
    sx: (x) => w / 2 + (x - camera.cx) * scale,
    sy: (y) => h / 2 - (y - camera.cy) * scale,
    wx: (px) => camera.cx + (px - w / 2) / scale,
    wy: (py) => camera.cy - (py - h / 2) / scale,
  };
}

function drawArrow(ctx, x0, y0, x1, y1) {
  const a = Math.atan2(y1 - y0, x1 - x0);
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x1, y1);
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - 6 * Math.cos(a - 0.45), y1 - 6 * Math.sin(a - 0.45));
  ctx.moveTo(x1, y1);
  ctx.lineTo(x1 - 6 * Math.cos(a + 0.45), y1 - 6 * Math.sin(a + 0.45));
  ctx.stroke();
}

function drawGrid(ctx, w, h) {
  const step = niceStep(view.span / 8);
  ctx.font = '11px system-ui, sans-serif';
  ctx.fillStyle = colors['text-muted'];
  ctx.strokeStyle = colors.grid;
  ctx.lineWidth = 1;
  for (let x = Math.ceil(view.wx(0) / step) * step; x <= view.wx(w); x += step) {
    const px = Math.round(view.sx(x)) + 0.5;
    ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, h); ctx.stroke();
    ctx.textAlign = 'center';
    ctx.fillText(formatTick(x, step), px, h - 4);
  }
  for (let y = Math.ceil(view.wy(h) / step) * step; y <= view.wy(0); y += step) {
    const py = Math.round(view.sy(y)) + 0.5;
    ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(w, py); ctx.stroke();
    ctx.textAlign = 'left';
    ctx.fillText(formatTick(y, step), 4, py - 3);
  }
  // Global frame axes.
  const ox = view.sx(0), oy = view.sy(0), len = step * view.scale;
  ctx.strokeStyle = colors['text-secondary'];
  ctx.fillStyle = colors['text-secondary'];
  ctx.lineWidth = 1.5;
  drawArrow(ctx, ox, oy, ox + len, oy);
  drawArrow(ctx, ox, oy, ox, oy - len);
  ctx.textAlign = 'left';
  ctx.fillText('x_g', ox + len + 4, oy + 4);
  ctx.fillText('y_g', ox + 4, oy - len - 4);
}

// Reference path p*(t), dashed.
function drawPath(ctx) {
  if (state.path.length < 2) return;
  ctx.strokeStyle = colors['series-2'];
  ctx.lineWidth = 1.5;
  ctx.setLineDash([6, 4]);
  ctx.beginPath();
  state.path.forEach(([x, y], i) => {
    if (i === 0) ctx.moveTo(view.sx(x), view.sy(y));
    else ctx.lineTo(view.sx(x), view.sy(y));
  });
  ctx.stroke();
  ctx.setLineDash([]);
}

// Cross at the reference p* of a sample row, labelled p* or p*<name>.
function drawReference(ctx, row, color, label) {
  if (!row || row[XR] === null || row[XR] === undefined) return;
  const gx = view.sx(row[XR]), gy = view.sy(row[YR]);
  ctx.strokeStyle = colors.surface;
  ctx.lineWidth = 5;
  for (const pass of [0, 1]) {
    ctx.beginPath();
    ctx.moveTo(gx - 5, gy - 5); ctx.lineTo(gx + 5, gy + 5);
    ctx.moveTo(gx - 5, gy + 5); ctx.lineTo(gx + 5, gy - 5);
    ctx.stroke();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
  }
  ctx.font = '11px system-ui, sans-serif';
  ctx.fillStyle = colors['text-secondary'];
  ctx.textAlign = 'left';
  ctx.fillText(label, gx + 8, gy - 8);
}

function drawTrail(ctx, from, to, alpha, rows = state.samples, color = colors['series-1']) {
  if (to - from < 1) return;
  ctx.globalAlpha = alpha;
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.lineJoin = 'round';
  ctx.beginPath();
  for (let i = from; i <= to; i++) {
    const s = rows[i];
    if (i === from) ctx.moveTo(view.sx(s[X]), view.sy(s[Y]));
    else ctx.lineTo(view.sx(s[X]), view.sy(s[Y]));
  }
  ctx.stroke();
  ctx.globalAlpha = 1;
}

function drawRobot(ctx, x, y, qx, qy, ghost, color = colors['series-1'], label = null) {
  const len = Math.max(ROBOT_LENGTH * view.scale, 22);
  const wid = len * (ROBOT_WIDTH / ROBOT_LENGTH);
  ctx.save();
  ctx.translate(view.sx(x), view.sy(y));
  ctx.rotate(-Math.atan2(qy, qx));  // screen y points down
  ctx.lineWidth = 2;
  if (ghost) {
    ctx.strokeStyle = colors['text-muted'];
    ctx.setLineDash([4, 3]);
    ctx.strokeRect(-len / 2, -wid / 2, len, wid);
    ctx.setLineDash([]);
    drawArrow(ctx, 0, 0, len * 0.9, 0);
  } else {
    ctx.fillStyle = color;
    ctx.strokeStyle = colors.surface;
    ctx.beginPath();
    ctx.rect(-len / 2, -wid / 2, len, wid);
    ctx.fill();
    ctx.stroke();
    // Robot frame axes: x_r forward, y_r to the left.
    ctx.strokeStyle = colors['text-primary'];
    ctx.lineWidth = 1.5;
    drawArrow(ctx, 0, 0, len * 1.1, 0);
    drawArrow(ctx, 0, 0, 0, -len * 0.8);
  }
  ctx.restore();
  if (label) {
    ctx.font = '11px system-ui, sans-serif';
    ctx.fillStyle = colors['text-secondary'];
    ctx.textAlign = 'left';
    ctx.fillText(label, view.sx(x) + len * 0.6, view.sy(y) - len * 0.6);
  }
}

// Other robots: trail, latest pose, and reference p*, in its own colour.
function drawFleet(ctx) {
  for (const name of Object.keys(state.fleet).sort()) {
    const trail = state.fleet[name];
    if (trail.length === 0) continue;
    const color = robotColor(name);
    drawTrail(ctx, 0, trail.length - 1, 0.6, trail, color);
    const s = trail[trail.length - 1];
    drawRobot(ctx, s[X], s[Y], s[QX], s[QY], false, color, name);
    drawReference(ctx, s, color, `p*${name}`);
  }
}

function drawWorld() {
  const { ctx, w, h } = fitCanvas(world);
  view = computeView(w, h);
  drawGrid(ctx, w, h);
  drawPath(ctx);
  const n = state.samples.length;
  if (state.initial && state.canSetPose) {
    const th = state.initial.theta;
    drawRobot(ctx, state.initial.x, state.initial.y, Math.cos(th), Math.sin(th), true);
  }
  drawFleet(ctx);
  if (n > 0 && state.cursor >= 0) {
    drawTrail(ctx, state.cursor, n - 1, 0.25);
    drawTrail(ctx, 0, state.cursor, 1);
    const s = state.samples[state.cursor];
    drawRobot(ctx, s[X], s[Y], s[QX], s[QY], false, colors['series-1'], state.robotName || null);
  }
  const sel = selRows();
  if (state.hover >= 0 && state.hover < sel.length) {
    const s = sel[state.hover];
    ctx.strokeStyle = colors['text-primary'];
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(view.sx(s[X]), view.sy(s[Y]), 6, 0, 2 * Math.PI);
    ctx.stroke();
  }
  if (n > 0 && state.cursor >= 0) drawReference(ctx, state.samples[state.cursor], colors['series-2'], 'p*');
}

// A drag longer than 4 px pans the view.
let drag = null;

world.addEventListener('pointerdown', (ev) => {
  if (!view || ev.button !== 0) return;
  drag = { x: ev.clientX, y: ev.clientY, cx: camera.cx, cy: camera.cy, panning: false };
  world.setPointerCapture(ev.pointerId);
});

world.addEventListener('pointerup', (ev) => {
  if (!drag) return;
  drag = null;
  world.style.cursor = '';
});
world.addEventListener('pointercancel', () => { drag = null; world.style.cursor = ''; });

world.addEventListener('wheel', (ev) => {
  if (!view) return;
  ev.preventDefault();
  const rect = world.getBoundingClientRect();
  const px = ev.clientX - rect.left, py = ev.clientY - rect.top;
  const wx = view.wx(px), wy = view.wy(py);
  const factor = Math.exp(ev.deltaY * 0.0015);
  const span = Math.min(Math.max(camera.span * factor, 0.5), 200);
  const k = span / camera.span;
  // Keep the world point under the pointer fixed.
  camera.cx = wx + (camera.cx - wx) * k;
  camera.cy = wy + (camera.cy - wy) * k;
  camera.span = span;
  camera.corner = false;
  saveCamera();
}, { passive: false });

$('fit').addEventListener('click', fitCamera);
$('view-reset').addEventListener('click', () => {
  Object.assign(camera, DEFAULT_CAMERA);
  saveCamera();
});

world.addEventListener('pointermove', (ev) => {
  if (!view) return;
  if (drag) {
    const dx = ev.clientX - drag.x, dy = ev.clientY - drag.y;
    if (!drag.panning && Math.hypot(dx, dy) < 4) return;
    drag.panning = true;
    camera.corner = false;
    world.style.cursor = 'grabbing';
    camera.cx = drag.cx - dx / view.scale;
    camera.cy = drag.cy + dy / view.scale;
    saveCamera();
    setHover(-1);
    return;
  }
  const rect = world.getBoundingClientRect();
  const px = ev.clientX - rect.left, py = ev.clientY - rect.top;
  let best = -1, bestD = 24 * 24;
  selRows().forEach((s, i) => {
    const d = (view.sx(s[X]) - px) ** 2 + (view.sy(s[Y]) - py) ** 2;
    if (d < bestD) { bestD = d; best = i; }
  });
  setHover(best, ev);
});
world.addEventListener('pointerleave', () => setHover(-1));

// --------------------------------------------------------------------- plots

const PLOT_MARGIN = { left: 46, right: 12, top: 8, bottom: 18 };

function timeRange() {
  const s = selRows();
  if (s.length === 0) return [0, 1];
  const t0 = s[0][T], t1 = s[s.length - 1][T];
  return [t0, Math.max(t1, t0 + 1e-6)];
}

function drawPlot(plot) {
  const { ctx, w, h } = fitCanvas(plot.canvas);
  const m = PLOT_MARGIN;
  const pw = w - m.left - m.right, ph = h - m.top - m.bottom;
  const s = selRows();
  const color = robotColor(state.selected);
  const refs = plot.refs();

  let lo = 0, hi = 0;
  for (const row of s) { lo = Math.min(lo, row[plot.col]); hi = Math.max(hi, row[plot.col]); }
  for (const r of refs) hi = Math.max(hi, r.value * 1.15);
  if (hi - lo < 1e-9) { hi = lo + 1; }
  const pad = (hi - lo) * 0.08;
  hi += pad;
  if (lo < 0) lo -= pad;
  const [t0, t1] = timeRange();
  const px = (t) => m.left + ((t - t0) / (t1 - t0)) * pw;
  const py = (v) => m.top + (1 - (v - lo) / (hi - lo)) * ph;

  ctx.font = '11px system-ui, sans-serif';
  ctx.lineWidth = 1;
  const yStep = niceStep((hi - lo) / 3);
  for (let v = Math.ceil(lo / yStep) * yStep; v <= hi; v += yStep) {
    const y = Math.round(py(v)) + 0.5;
    ctx.strokeStyle = Math.abs(v) < yStep / 1e6 ? colors.border : colors.grid;
    ctx.beginPath(); ctx.moveTo(m.left, y); ctx.lineTo(w - m.right, y); ctx.stroke();
    ctx.fillStyle = colors['text-muted'];
    ctx.textAlign = 'right';
    ctx.fillText(formatTick(Math.abs(v) < yStep / 1e6 ? 0 : v, yStep), m.left - 6, y + 4);
  }
  const tStep = niceStep((t1 - t0) / 6);
  ctx.textAlign = 'center';
  for (let t = Math.ceil(t0 / tStep) * tStep; t <= t1; t += tStep) {
    ctx.fillStyle = colors['text-muted'];
    ctx.fillText(formatTick(t, tStep), px(t), h - 4);
  }

  for (const r of refs) {
    const y = Math.round(py(r.value)) + 0.5;
    ctx.strokeStyle = colors['series-2'];
    ctx.setLineDash([5, 4]);
    ctx.beginPath(); ctx.moveTo(m.left, y); ctx.lineTo(w - m.right, y); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = colors['text-secondary'];
    ctx.textAlign = 'right';
    ctx.fillText(r.label, w - m.right - 2, y - 3);
  }

  if (s.length > 0) {
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.beginPath();
    s.forEach((row, i) => {
      if (i === 0) ctx.moveTo(px(row[T]), py(row[plot.col]));
      else ctx.lineTo(px(row[T]), py(row[plot.col]));
    });
    ctx.stroke();
  }

  const mark = (index, lineColor) => {
    if (index < 0 || index >= s.length) return;
    const row = s[index];
    const x = Math.round(px(row[T])) + 0.5;
    ctx.strokeStyle = lineColor;
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x, m.top); ctx.lineTo(x, m.top + ph); ctx.stroke();
    ctx.fillStyle = color;
    ctx.strokeStyle = colors.surface;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, py(row[plot.col]), 4, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
  };
  if (state.mode === 'batch') mark(selCursor(), colors.border);
  mark(state.hover, colors['text-secondary']);

  plot.invert = (clientX) => {
    const rect = plot.canvas.getBoundingClientRect();
    return t0 + ((clientX - rect.left - m.left) / pw) * (t1 - t0);
  };
}

function nearestIndex(t) {
  const s = selRows();
  if (s.length === 0) return -1;
  let lo = 0, hi = s.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (s[mid][T] < t) lo = mid; else hi = mid;
  }
  return Math.abs(s[lo][T] - t) <= Math.abs(s[hi][T] - t) ? lo : hi;
}

for (const plot of plots) {
  plot.canvas.addEventListener('pointermove', (ev) => {
    if (plot.invert) setHover(nearestIndex(plot.invert(ev.clientX)), ev);
  });
  plot.canvas.addEventListener('pointerleave', () => setHover(-1));
}

// ------------------------------------------------------- hover and readouts

function setHover(index, ev) {
  state.hover = index;
  const tip = $('tooltip');
  if (index < 0 || !ev) {
    tip.hidden = true;
  } else {
    const s = selRows()[index];
    const rows = [
      ['t', fixed(s[T], 2) + ' s'],
      ['‖p* − p‖', fixed(s[DIST], 3) + ' m'],
      ['v', fixed(s[V], 3) + ' m/s'],
      ['ω', fixed(s[W], 3) + ' rad/s'],
      ['x, y', fixed(s[X], 3) + ', ' + fixed(s[Y], 3)],
    ];
    tip.replaceChildren(...rows.map(([name, value]) => {
      const row = document.createElement('div');
      row.className = 'row';
      const a = document.createElement('span'); a.textContent = name;
      const b = document.createElement('span'); b.textContent = value;
      row.append(a, b);
      return row;
    }));
    tip.hidden = false;
    const flip = ev.clientX > window.innerWidth - 200;
    tip.style.left = (flip ? ev.clientX - tip.offsetWidth - 14 : ev.clientX + 14) + 'px';
    tip.style.top = (ev.clientY + 14) + 'px';
  }
  dirty = true;
}

function updateReadout() {
  const index = state.hover >= 0 ? state.hover : selCursor();
  const s = selRows()[index];
  const set = (id, text) => { $(id).textContent = text; };
  if (!s) {
    for (const id of ['r-t', 'r-x', 'r-y', 'r-th', 'r-d', 'r-v', 'r-w']) set(id, '-');
    return;
  }
  set('r-t', fixed(s[T], 2) + ' s');
  set('r-x', fixed(s[X], 3) + ' m');
  set('r-y', fixed(s[Y], 3) + ' m');
  set('r-th', ((Math.atan2(s[QY], s[QX]) * 180) / Math.PI).toFixed(1) + '°');
  set('r-d', fixed(s[DIST], 3) + ' m');
  set('r-v', fixed(s[V], 3) + ' m/s');
  set('r-w', fixed(s[W], 3) + ' rad/s');
}

// ---------------------------------------------------------------- main loop

function frame(now) {
  if (state.playing) {
    advancePlayback(now);
    dirty = true;
  }
  if (dirty) {
    dirty = false;
    drawWorld();
    for (const plot of plots) drawPlot(plot);
    updateReadout();
    $('play').textContent = state.playing ? 'Pause' : 'Play';
    if (document.activeElement !== $('scrub')) $('scrub').value = Math.max(0, state.cursor);
  }
  requestAnimationFrame(frame);
}

new ResizeObserver(() => { dirty = true; }).observe(document.body);
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', readColors);
readColors();
connect();
requestAnimationFrame(frame);
