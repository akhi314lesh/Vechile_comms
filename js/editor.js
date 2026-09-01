import * as THREE from 'three';
import { trackDef, roadMetrics, T, buildTrack, wrapS } from './track.js';

export const ed = { open: false, view: null, samples: [], curve: null, L: 1, drag: null };
const ED_W = 720, ED_H = 520, ED_N = 420, MIN_PT_DIST = 18;
let ctx2, rebuild, getSpawnU, setSpawnU, scenario, toastFn;

export function initEditor(opts) {
  ({ rebuild, getSpawnU, setSpawnU, scenario, toast: toastFn } = opts);
  const c = document.getElementById('edCanvas');
  const d = Math.min(devicePixelRatio, 2);
  c.width = ED_W * d; c.height = ED_H * d;
  ctx2 = c.getContext('2d');
  ctx2.setTransform(d, 0, 0, d, 0, 0);
  c.addEventListener('pointerdown', onDown);
  c.addEventListener('pointermove', onMove);
  c.addEventListener('pointerup', onUp);
  c.addEventListener('contextmenu', e => e.preventDefault());
  document.getElementById('edClose').onclick = closeEditor;
}
function recompute() {
  ed.curve = new THREE.CatmullRomCurve3(
    trackDef.points.map(p => new THREE.Vector3(p[0], 0, p[1])), true, 'centripetal');
  ed.curve.arcLengthDivisions = 800;
  ed.L = ed.curve.getLength();
  ed.samples = [];
  for (let i = 0; i < ED_N; i++) {
    const u = i / ED_N, p = ed.curve.getPointAt(u), t = ed.curve.getTangentAt(u);
    ed.samples.push({ x: p.x, z: p.z, tx: t.x, tz: t.z });
  }
}
function computeView() {
  let x0 = 1e9, x1 = -1e9, z0 = 1e9, z1 = -1e9;
  for (const [x, z] of trackDef.points) {
    x0 = Math.min(x0, x); x1 = Math.max(x1, x);
    z0 = Math.min(z0, z); z1 = Math.max(z1, z);
  }
  const k = Math.min(ED_W / (x1 - x0 + 90), ED_H / (z1 - z0 + 90));
  ed.view = { k, cx: (x0 + x1) / 2, cz: (z0 + z1) / 2 };
}
const W2S = (x, z) => { const v = ed.view; return [(x - v.cx) * v.k + ED_W / 2, (v.cz - z) * v.k + ED_H / 2]; };
const S2W = (px, py) => { const v = ed.view; return [(px - ED_W / 2) / v.k + v.cx, v.cz - (py - ED_H / 2) / v.k]; };

function tooClose(x, z, skip = -1) {
  for (let i = 0; i < trackDef.points.length; i++) {
    if (i === skip) continue;
    if (Math.hypot(trackDef.points[i][0] - x, trackDef.points[i][1] - z) < MIN_PT_DIST) return true;
  }
  return false;
}
function drawChevron(u, lat, color, label, reverse) {
  u = ((u % 1) + 1) % 1;
  const p = ed.curve.getPointAt(u), t = ed.curve.getTangentAt(u);
  let dx = t.x, dz = t.z;
  if (reverse) { dx = -dx; dz = -dz; }
  const x = p.x - t.z * lat, z = p.z + t.x * lat;
  const [X, Y] = W2S(x, z);
  const ang = Math.atan2(-dz, dx);
  ctx2.save();
  ctx2.translate(X, Y); ctx2.rotate(ang);
  ctx2.fillStyle = color;
  ctx2.beginPath(); ctx2.moveTo(9, 0); ctx2.lineTo(-6, 5.5); ctx2.lineTo(-6, -5.5); ctx2.closePath(); ctx2.fill();
  if (label) {
    ctx2.rotate(-ang);
    ctx2.fillStyle = color; ctx2.font = '9px IBM Plex Mono';
    ctx2.fillText(label, 8, -8);
  }
  ctx2.restore();
}
function redraw() {
  if (!ed.view) return;
  const v = ed.view, met = roadMetrics(trackDef);
  ctx2.clearRect(0, 0, ED_W, ED_H);
  ctx2.strokeStyle = 'rgba(90,120,160,0.10)'; ctx2.lineWidth = 1;
  const [wx0, wz0] = S2W(0, 0), [wx1, wz1] = S2W(ED_W, ED_H);
  ctx2.beginPath();
  for (let gx = Math.floor(wx0 / 50) * 50; gx <= wx1; gx += 50) {
    const [sx] = W2S(gx, 0); ctx2.moveTo(sx, 0); ctx2.lineTo(sx, ED_H);
  }
  for (let gz = Math.floor(wz1 / 50) * 50; gz <= wz0; gz += 50) {
    const [, sy] = W2S(0, gz); ctx2.moveTo(0, sy); ctx2.lineTo(ED_W, sy);
  }
  ctx2.stroke();
  ctx2.beginPath();
  ed.samples.forEach((s, i) => {
    const [px, py] = W2S(s.x, s.z);
    i ? ctx2.lineTo(px, py) : ctx2.moveTo(px, py);
  });
  ctx2.closePath();
  ctx2.strokeStyle = '#1d2229';
  ctx2.lineWidth = Math.max(4, met.deckW * v.k);
  ctx2.lineJoin = 'round';
  ctx2.stroke();
  const line = (off, col, w, dash) => {
    ctx2.beginPath();
    ed.samples.forEach((s, i) => {
      const [X, Y] = W2S(s.x - s.tz * off, s.z + s.tx * off);
      i ? ctx2.lineTo(X, Y) : ctx2.moveTo(X, Y);
    });
    ctx2.closePath();
    ctx2.strokeStyle = col; ctx2.lineWidth = w;
    ctx2.setLineDash(dash || []);
    ctx2.stroke();
    ctx2.setLineDash([]);
  };
  line(met.pavedHalf, 'rgba(226,229,233,0.85)', 1.2);
  line(-met.pavedHalf, 'rgba(226,229,233,0.85)', 1.2);
  if (trackDef.lanesO > 0) {
    line(0.2, 'rgba(214,171,64,0.9)', 1.4); line(-0.2, 'rgba(214,171,64,0.9)', 1.4);
    for (let i = 1; i < trackDef.lanesF; i++) line(i * 3.7, 'rgba(226,229,233,0.5)', 1, [6, 6]);
    for (let i = 1; i < trackDef.lanesO; i++) line(-i * 3.7, 'rgba(226,229,233,0.5)', 1, [6, 6]);
  } else {
    for (let i = 1; i < trackDef.lanesF; i++) line(-met.pavedHalf + i * 3.7, 'rgba(226,229,233,0.5)', 1, [6, 6]);
  }
  /* barrier preview */
  if (trackDef.barrier !== 'off') {
    const kap = [];
    const N = 300, ds = ed.L / N;
    const tans = [];
    for (let i = 0; i < N; i++) tans.push(ed.curve.getTangentAt(i / N));
    for (let i = 0; i < N; i++) {
      const a = tans[i], b = tans[(i + 1) % N];
      kap.push(Math.acos(THREE.MathUtils.clamp(a.dot(b), -1, 1)) / ds);
    }
    const sm = kap.map((_, i) => (kap[(i + N - 2) % N] + kap[(i + N - 1) % N] + kap[i] + kap[(i + 1) % N] + kap[(i + 2) % N]) / 5);
    let iMax = 0;
    for (let i = 1; i < N; i++) if (sm[i] > sm[iMax]) iMax = i;
    const thr = sm[iMax] * 0.45;
    let a = iMax, b = iMax, g = 0;
    while (g++ < N && sm[(a + N - 1) % N] > thr) a = (a + N - 1) % N;
    g = 0;
    while (g++ < N && sm[(b + 1) % N] > thr) b = (b + 1) % N;
    if (b < a) b += N;
    const cy = tans[iMax].z * tans[(iMax + 1) % N].x - tans[iMax].x * tans[(iMax + 1) % N].z;
    let side = cy < 0 ? 1 : -1;
    if (trackDef.barrier === 'outside') side = -side;
    const span = (b - a) * ds;
    ctx2.beginPath();
    let started = false;
    for (let d = 0; d <= span; d += 3) {
      const u = ((((a * ds + d) / ed.L) % 1) + 1) % 1;
      const p = ed.curve.getPointAt(u), t = ed.curve.getTangentAt(u);
      const [X, Y] = W2S(p.x - t.z * side * (met.deckHalf + 0.7), p.z + t.x * side * (met.deckHalf + 0.7));
      started ? ctx2.lineTo(X, Y) : ctx2.moveTo(X, Y);
      started = true;
    }
    ctx2.strokeStyle = 'rgba(255,90,64,0.9)'; ctx2.lineWidth = 4;
    ctx2.setLineDash([10, 5]);
    ctx2.stroke();
    ctx2.setLineDash([]);
  }
  drawChevron(getSpawnU(), T.laneLat('fwd', 0), '#45e6ff', 'EGO');
  for (const ai of scenario)
    drawChevron(ai.spawnU, T.laneLat(ai.dir, ai.lane), ai.dir === 'onc' ? '#dfe8f0' : '#ffa23c', ai.name, ai.dir === 'onc');
  trackDef.points.forEach((p, i) => {
    const [X, Y] = W2S(p[0], p[1]);
    const active = ed.drag && ed.drag.type === 'pt' && ed.drag.i === i;
    ctx2.fillStyle = active ? '#ffffff' : '#ffb454';
    ctx2.fillRect(X - 4, Y - 4, 8, 8);
    ctx2.strokeStyle = 'rgba(0,0,0,0.6)';
    ctx2.strokeRect(X - 4.5, Y - 4.5, 9, 9);
  });
  const sbl = 50 * v.k;
  ctx2.strokeStyle = 'rgba(180,215,240,0.7)'; ctx2.lineWidth = 2;
  ctx2.beginPath();
  ctx2.moveTo(14, ED_H - 16); ctx2.lineTo(14 + sbl, ED_H - 16);
  ctx2.moveTo(14, ED_H - 20); ctx2.lineTo(14, ED_H - 12);
  ctx2.moveTo(14 + sbl, ED_H - 20); ctx2.lineTo(14 + sbl, ED_H - 12);
  ctx2.stroke();
  ctx2.fillStyle = 'rgba(180,215,240,0.8)'; ctx2.font = '10px IBM Plex Mono';
  ctx2.fillText('50 M', 16, ED_H - 24);
}
export function openEditor() {
  ed.open = true;
  recompute(); computeView(); redraw();
  document.getElementById('editor').style.display = 'flex';
  toastFn('TRACK EDITOR — DRAG POINTS · CLICK TO INSERT · RIGHT-CLICK TO DELETE');
}
export function closeEditor() {
  ed.open = false;
  document.getElementById('editor').style.display = 'none';
  rebuild();
}
function edPos(e) {
  const r = document.getElementById('edCanvas').getBoundingClientRect();
  return [(e.clientX - r.left) * (ED_W / r.width), (e.clientY - r.top) * (ED_H / r.height)];
}
function nearestSample(px, py) {
  const [wx, wz] = S2W(px, py);
  let bi = 0, bd = 1e18;
  ed.samples.forEach((s, i) => {
    const d = (s.x - wx) ** 2 + (s.z - wz) ** 2;
    if (d < bd) { bd = d; bi = i; }
  });
  return bi;
}
function insertPointAt(px, py) {
  const [wx, wz] = S2W(px, py);
  if (tooClose(wx, wz)) { toastFn('TOO CLOSE TO AN EXISTING POINT (MIN ' + MIN_PT_DIST + ' M)'); return; }
  const u = nearestSample(px, py) / ED_N;
  const t = ed.curve.getUtoTmapping(u);
  const n = trackDef.points.length;
  const idx = Math.floor(t * n) % n;
  trackDef.points.splice(idx + 1, 0, [clampC(wx), clampC(wz)]);
  recompute(); computeView();
  rebuild();
}
const clampC = v => THREE.MathUtils.clamp(v, -480, 480);
function deletePoint(i) {
  if (trackDef.points.length <= 4) { toastFn('MINIMUM 4 CONTROL POINTS'); return; }
  trackDef.points.splice(i, 1);
  recompute(); computeView();
  rebuild();
}
function onDown(e) {
  e.preventDefault();
  const [px, py] = edPos(e);
  for (let i = 0; i < trackDef.points.length; i++) {
    const [X, Y] = W2S(trackDef.points[i][0], trackDef.points[i][1]);
    if (Math.hypot(px - X, py - Y) < 12) {
      if (e.button === 2 || e.altKey) { deletePoint(i); return; }
      ed.drag = { type: 'pt', i };
      e.target.setPointerCapture(e.pointerId);
      return;
    }
  }
  const ep = ed.curve.getPointAt(getSpawnU()), et = ed.curve.getTangentAt(getSpawnU());
  const [eX, eY] = W2S(ep.x - et.z * T.laneLat('fwd', 0), ep.z + et.x * T.laneLat('fwd', 0));
  if (Math.hypot(px - eX, py - eY) < 16) {
    ed.drag = { type: 'ego' };
    e.target.setPointerCapture(e.pointerId);
    return;
  }
  for (const ai of scenario) {
    const ap = ed.curve.getPointAt(ai.spawnU), at = ed.curve.getTangentAt(ai.spawnU);
    const lat = T.laneLat(ai.dir, ai.lane);
    const [aX, aY] = W2S(ap.x - at.z * lat, ap.z + at.x * lat);
    if (Math.hypot(px - aX, py - aY) < 14) {
      ed.drag = { type: 'ai', ai };
      e.target.setPointerCapture(e.pointerId);
      return;
    }
  }
  if (e.button === 0) insertPointAt(px, py);
}
function onMove(e) {
  if (!ed.drag) return;
  const [px, py] = edPos(e);
  if (ed.drag.type === 'pt') {
    const [wx, wz] = S2W(px, py);
    if (!tooClose(wx, wz, ed.drag.i)) {
      trackDef.points[ed.drag.i] = [clampC(wx), clampC(wz)];
      recompute(); redraw();
    }
  } else if (ed.drag.type === 'ego') {
    setSpawnU(nearestSample(px, py) / ED_N);
    redraw();
  } else if (ed.drag.type === 'ai') {
    ed.drag.ai.spawnU = nearestSample(px, py) / ED_N;
    ed.drag.ai.s = ed.drag.ai.spawnU * T.L;
    redraw();
  }
}
function onUp() {
  if (ed.drag && ed.drag.type === 'pt') rebuild();
  ed.drag = null;
}
export { recompute as recomputeEd, redraw as redrawEditor, computeView as computeEdView };