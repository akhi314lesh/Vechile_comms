import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { scene, fog, skyGroup, makeTexture, speckle, bermTex, concTex, cityTexs, glareTex, poolTex, chevTex, signTex } from './env.js';

const UP = new THREE.Vector3(0, 1, 0);
export const CFG = { laneW: 3.7, shoulder: 1.1 };

export const DEFAULT_POINTS = [
  [0, 120], [0, 50], [0, -20], [0, -90], [0, -150],
  [6, -192], [30, -224], [68, -239], [108, -244],
  [150, -246], [196, -244], [232, -228], [252, -196],
  [258, -150], [258, -80], [258, -10], [258, 60], [258, 120],
  [254, 172], [228, 205], [185, 222], [120, 224],
  [60, 222], [16, 214], [-12, 186], [-2, 152]
];
export const trackDef = {
  points: DEFAULT_POINTS.map(p => p.slice()),
  lanesF: 2, lanesO: 2, barrier: 'inside'
};
export let T = null;

export function roadMetrics(def) {
  const paved = (def.lanesF + def.lanesO) * CFG.laneW;
  return { paved, deckW: paved + 2 * CFG.shoulder, deckHalf: (paved + 2 * CFG.shoulder) / 2, pavedHalf: paved / 2 };
}
export function laneLatOf(def, dir, idx) {
  const { pavedHalf } = roadMetrics(def);
  if (def.lanesO > 0) return dir === 'fwd' ? (idx + 0.5) * CFG.laneW : -(idx + 0.5) * CFG.laneW;
  return -pavedHalf + (idx + 0.5) * CFG.laneW;
}
export function wrapS(x) {
  if (!T) return x;
  x = ((x % T.L) + T.L) % T.L;
  return x > T.L / 2 ? x - T.L : x;
}

/* asphalt texture baked for the current lane configuration */
function makeRoadTex(def) {
  const { paved, deckW, pavedHalf } = roadMetrics(def);
  const t = makeTexture(512, 512, (ctx, w, h) => {
    const m2px = w / deckW, X = m => (m + deckW / 2) * m2px;
    ctx.fillStyle = '#22262b'; ctx.fillRect(0, 0, w, h);
    const nL = def.lanesF + def.lanesO;
    for (let ln = 0; ln < nL; ln++) {
      const c = -pavedHalf + (ln + 0.5) * CFG.laneW;
      for (const o of [-CFG.laneW * 0.26, CFG.laneW * 0.26]) {
        const g = ctx.createLinearGradient(X(c + o) - 14, 0, X(c + o) + 14, 0);
        g.addColorStop(0, 'rgba(12,13,15,0)'); g.addColorStop(0.5, 'rgba(12,13,15,0.32)'); g.addColorStop(1, 'rgba(12,13,15,0)');
        ctx.fillStyle = g; ctx.fillRect(X(c + o) - 14, 0, 28, h);
      }
    }
    speckle(ctx, w, h, 9000, 0.5, 26);
    ctx.strokeStyle = 'rgba(10,10,12,0.3)'; ctx.lineWidth = 1;
    for (let i = 0; i < 8; i++) {
      ctx.beginPath(); let x = Math.random() * w, y = Math.random() * h; ctx.moveTo(x, y);
      for (let k = 0; k < 6; k++) { x += (Math.random() - .5) * 60; y += (Math.random() - .5) * 80; ctx.lineTo(x, y); }
      ctx.stroke();
    }
    const W = 'rgba(226,229,233,0.92)', Y = 'rgba(214,171,64,0.95)';
    const solid = (m, col) => { ctx.fillStyle = col; ctx.fillRect(X(m - 0.06), 0, 0.12 * m2px, h); };
    const dash = (m, col) => { ctx.fillStyle = col; ctx.fillRect(X(m - 0.06), 0, 0.12 * m2px, 170); };
    solid(-pavedHalf + 0.25, W); solid(pavedHalf - 0.25, W);
    if (def.lanesO > 0) {
      solid(-0.2, Y); solid(0.2, Y);
      for (let i = 1; i < def.lanesF; i++) dash(i * CFG.laneW, W);
      for (let i = 1; i < def.lanesO; i++) dash(-i * CFG.laneW, W);
    } else {
      for (let i = 1; i < def.lanesF; i++) dash(-pavedHalf + i * CFG.laneW, W);
    }
    speckle(ctx, w, h, 2500, 0.3, 30);
  });
  t.wrapS = THREE.ClampToEdgeWrapping;
  return t;
}

/* berm profile as a SHARED function: used by the sweep, the terrain
   heightfield and the car's ground height — one source of truth */
const BERM_U = [0, 0.2, 1.55, 6, 12, 22, 34, 46, 54, 56];
const BERM_H = [0, 0.02, 0.32, 1.05, 1.35, 1.05, 0.6, 0.3, 0.02, -0.45];
export function bermH(u) {
  if (u <= 0) return 0;
  if (u >= BERM_U[BERM_U.length - 1]) return BERM_H[BERM_H.length - 1];
  for (let i = 1; i < BERM_U.length; i++) {
    if (u <= BERM_U[i]) {
      const k = (u - BERM_U[i - 1]) / (BERM_U[i] - BERM_U[i - 1]);
      return BERM_H[i - 1] + (BERM_H[i] - BERM_H[i - 1]) * k;
    }
  }
  return 0;
}

/* ============================================================
   SWEEP with curvature clamping — THE fix for irregular tracks.
   For each sample we know the local radius of curvature R and which
   side the curve centre is on. Profile offsets on the inside of the
   turn are clamped to fac*R so the ribbon can never sweep past the
   centre and fold back over the road ("road buried under ground").
   ============================================================ */
function sweep(curve, L, profile, { segments = 1000, uPer = 1, vPer = 1, filterS = null, clampFac = 0.9 } = {}) {
  const m = profile.length, cum = [0];
  for (let j = 1; j < m; j++)
    cum.push(cum[j - 1] + Math.hypot(profile[j][0] - profile[j - 1][0], profile[j][1] - profile[j - 1][1]));
  const pos = new Float32Array((segments + 1) * m * 3);
  const uvA = new Float32Array((segments + 1) * m * 2);
  const idx = [], inc = new Array(segments + 1);
  const P = new THREE.Vector3(), Tn = new THREE.Vector3(), R = new THREE.Vector3();
  const tan = [], kap = [];
  const ds = L / segments;
  for (let i = 0; i <= segments; i++) tan.push(curve.getTangentAt(i / segments));
  for (let i = 0; i <= segments; i++) {           // smoothed curvature, side sign
    const a = tan[i], b = tan[(i + 1) % (segments + 1)] || tan[i];
    const cy = a.z * b.x - a.x * b.z, dot = a.x * b.x + a.z * b.z;
    const ang = Math.atan2(cy, dot);
    kap.push({ s: Math.sign(cy) || 0, k: ang / ds });
  }
  for (let i = 0; i <= segments; i++) {
    let k = 0, sgn = 0;
    for (let o = -2; o <= 2; o++) {
      const j = Math.min(segments, Math.max(0, i + o));
      k += kap[j].k; sgn ||= kap[j].s;
    }
    k /= 5;
    const Rc = Math.min(1 / Math.max(Math.abs(k), 1e-5), 5000);
    kap[i] = { r: Rc, side: sgn };
  }
  for (let i = 0; i <= segments; i++) {
    const u = i / segments, s = u * L;
    inc[i] = !filterS || filterS(s);
    curve.getPointAt(u, P);
    Tn.copy(tan[i]);
    R.crossVectors(Tn, UP).normalize();
    const { r, side } = kap[i];
    for (let j = 0; j < m; j++) {
      let o = profile[j][0];
      if (side < 0) o = Math.min(o, r * clampFac);        // centre on +r side → clamp + offsets
      else if (side > 0) o = Math.max(o, -r * clampFac);  // centre on -r side → clamp - offsets
      const k = i * m + j;
      pos[k * 3] = P.x + R.x * o;
      pos[k * 3 + 1] = profile[j][1];
      pos[k * 3 + 2] = P.z + R.z * o;
      uvA[k * 2] = cum[j] / uPer;
      uvA[k * 2 + 1] = s / vPer;
    }
  }
  for (let i = 0; i < segments; i++) {
    if (!inc[i] || !inc[i + 1]) continue;
    for (let j = 0; j < m - 1; j++) {
      const a = i * m + j, b = i * m + j + 1, c = (i + 1) * m + j + 1, d = (i + 1) * m + j;
      idx.push(a, b, c, a, c, d);
    }
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setAttribute('uv', new THREE.BufferAttribute(uvA, 2));
  geo.setIndex(idx);
  geo.computeVertexNormals();
  if (!filterS) {
    const n = geo.getAttribute('normal');
    for (let j = 0; j < m; j++) {
      const a = j, b = segments * m + j;
      n.setXYZ(a, (n.getX(a) + n.getX(b)) / 2, (n.getY(a) + n.getY(b)) / 2, (n.getZ(a) + n.getZ(b)) / 2);
      n.setXYZ(b, n.getX(a), n.getY(a), n.getZ(a));
    }
  }
  return geo;
}

/* tightest-curve detection → auto barrier window + side */
function detectBarrierWindow(curve, L) {
  const N = 600, ds = L / N, tans = [];
  for (let i = 0; i < N; i++) tans.push(curve.getTangentAt(i / N));
  const kap = [];
  for (let i = 0; i < N; i++) {
    const a = tans[i], b = tans[(i + 1) % N];
    kap.push(Math.acos(THREE.MathUtils.clamp(a.dot(b), -1, 1)) / ds);
  }
  const sm = kap.map((_, i) =>
    (kap[(i + N - 2) % N] + kap[(i + N - 1) % N] + kap[i] + kap[(i + 1) % N] + kap[(i + 2) % N]) / 5);
  let iMax = 0;
  for (let i = 1; i < N; i++) if (sm[i] > sm[iMax]) iMax = i;
  const thr = sm[iMax] * 0.45;
  let a = iMax, b = iMax, g = 0;
  while (g++ < N && sm[(a + N - 1) % N] > thr) a = (a + N - 1) % N;
  g = 0;
  while (g++ < N && sm[(b + 1) % N] > thr) b = (b + 1) % N;
  if (b < a) b += N;
  const minS = Math.ceil(70 / ds), maxS = Math.floor(240 / ds);
  if (b - a < minS) { const e = (minS - (b - a)) / 2; a -= Math.floor(e); b += Math.ceil(e); }
  if (b - a > maxS) b = a + maxS;
  const cY = tans[iMax].z * tans[(iMax + 1) % N].x - tans[iMax].x * tans[(iMax + 1) % N].z;
  return { s0: (a / N) * L, s1: (b / N) * L, side: cY < 0 ? 1 : -1 };
}

/* spatial hash over curve samples → fast nearest-point queries for
   the terrain heightfield, surface grip and world→arc projection */
function buildCurveHash(curve, L) {
  const N = THREE.MathUtils.clamp(Math.round(L / 1.3), 400, 6000);
  const samples = [];
  for (let i = 0; i < N; i++) {
    const u = i / N;
    const p = curve.getPointAt(u), t = curve.getTangentAt(u);
    samples.push({ x: p.x, z: p.z, tx: t.x, tz: t.z, s: u * L });
  }
  const CS = 14, cells = new Map();
  samples.forEach((sm, i) => {
    const key = Math.floor(sm.x / CS) + ',' + Math.floor(sm.z / CS);
    (cells.get(key) || cells.set(key, []).get(key)).push(i);
  });
  return {
    N, samples, CS,
    nearest(x, z) {
      const cx = Math.floor(x / CS), cz = Math.floor(z / CS);
      let best = 1e18, bi = -1;
      for (let ring = 0; ring < 40; ring++) {
        if (ring > 1 && best < (ring - 1) * CS * (ring - 1) * CS) break;
        for (let dx = -ring; dx <= ring; dx++) for (let dz = -ring; dz <= ring; dz++) {
          if (Math.max(Math.abs(dx), Math.abs(dz)) !== ring) continue;
          const c = cells.get((cx + dx) + ',' + (cz + dz));
          if (!c) continue;
          for (const i of c) {
            const sm = samples[i];
            const d = (sm.x - x) * (sm.x - x) + (sm.z - z) * (sm.z - z);
            if (d < best) { best = d; bi = i; }
          }
        }
      }
      return bi >= 0 ? { i: bi, d: Math.sqrt(best), ...samples[bi] } : null;
    }
  };
}

/* rolling-hills noise for the far terrain */
function terrainNoise(x, z) {
  const h2 = (a, b) => { const v = Math.sin(a * 127.1 + b * 311.7) * 43758.5453; return v - Math.floor(v); };
  const vn = (a, b) => {
    const xi = Math.floor(a), zi = Math.floor(b), xf = a - xi, zf = b - zi;
    const u = xf * xf * (3 - 2 * xf), w = zf * zf * (3 - 2 * zf);
    return h2(xi, zi) * (1 - u) * (1 - w) + h2(xi + 1, zi) * u * (1 - w) + h2(xi, zi + 1) * (1 - u) * w + h2(xi + 1, zi + 1) * u * w;
  };
  return 0.6 * vn(x * 0.011, z * 0.011) + 0.4 * vn(x * 0.037, z * 0.037);
}

const roadMat = new THREE.MeshStandardMaterial({ roughness: 0.88, metalness: 0.03, side: THREE.DoubleSide });
const radarRoadMat = new THREE.MeshBasicMaterial({ color: 0xc4d8ee, fog: false });
const radarBarMat = new THREE.MeshBasicMaterial({ color: new THREE.Color(0xff5a40).multiplyScalar(1.7), fog: false, side: THREE.DoubleSide });
const bermMat = new THREE.MeshStandardMaterial({ map: bermTex, roughness: 1, metalness: 0, side: THREE.DoubleSide });
const railMat = new THREE.MeshStandardMaterial({ color: 0x97a0aa, metalness: 0.85, roughness: 0.38, side: THREE.DoubleSide });
const poleMat = new THREE.MeshStandardMaterial({ color: 0x565d66, metalness: 0.8, roughness: 0.45 });
const bulbMat = new THREE.MeshStandardMaterial({ color: 0x221607, emissive: 0xffd9a4, emissiveIntensity: 3.2, roughness: 0.4 });
const postMat = new THREE.MeshStandardMaterial({ color: 0x494f57, metalness: 0.7, roughness: 0.5 });
const poolMat = new THREE.MeshBasicMaterial({ map: poolTex, transparent: true, opacity: 0.55, blending: THREE.AdditiveBlending, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -4 });
const chevMat = new THREE.MeshStandardMaterial({ map: chevTex, color: 0x999999, emissive: 0xffffff, emissiveMap: chevTex, emissiveIntensity: 0.5, roughness: 0.6, side: THREE.DoubleSide });
const signMat = new THREE.MeshStandardMaterial({ map: signTex, emissive: 0xffffff, emissiveMap: signTex, emissiveIntensity: 0.28, roughness: 0.5, side: THREE.DoubleSide });
const steelMat = new THREE.MeshStandardMaterial({ color: 0x3a3f46, metalness: 0.75, roughness: 0.5 });
const catEyeMat = new THREE.PointsMaterial({ map: glareTex, color: 0xffb156, size: 0.16, transparent: true, blending: THREE.AdditiveBlending, depthWrite: false });
const lampGlowMat = new THREE.PointsMaterial({ map: glareTex, color: 0xffd2a0, size: 1.6, transparent: true, blending: THREE.AdditiveBlending, depthWrite: false });
const terrainMat = new THREE.MeshStandardMaterial({ map: bermTex, roughness: 1, metalness: 0, vertexColors: true });

/* ground height of any world point (car body + city placement + chase cam) */
export function groundHeightAt(x, z) {
  if (!T) return 0;
  const q = T.hash.nearest(x, z);
  if (!q) return 0;
  return terrainHeight(q.d, x, z);
}
function terrainHeight(d, x, z) {
  const u = d - T.deckHalf;
  if (u <= 0) return -0.12;                       // strictly under the deck
  if (u <= 54) return bermH(u) - 0.16;            // embankment, just below the berm mesh
  const t = THREE.MathUtils.smoothstep(u, 54, 120);
  return (0.02 - 0.16) * (1 - t) + t * ((terrainNoise(x, z) * 2 - 0.75) * 5);
}

/* surface query for physics: grip + lateral position + station */
export function surfaceAt(x, z) {
  const q = T.hash.nearest(x, z);
  if (!q) return { d: 1e9, lat: 1e9, s: 0, mu: 0.5, off: true, tx: 1, tz: 0 };
  const lat = (x - q.x) * (-q.tz) + (z - q.z) * q.tx;   // signed lateral offset (+ = right of travel dir)
  const al = Math.abs(lat);
  const mu = al <= T.pavedHalf + 0.4 ? 1.05 : al <= T.deckHalf + 2.5 ? 0.85 : al <= T.deckHalf + 9 ? 0.5 : 0.45;
  return { d: q.d, lat, s: q.s, mu, off: al > T.pavedHalf + 0.2, tx: q.tx, tz: q.tz };
}
export function latHeight(lat) {
  const u = Math.abs(lat) - T.deckHalf;
  return u <= 0 ? 0 : Math.min(bermH(u), 1.35);
}

/* ---------- the world rebuild ---------- */
export function buildTrack(def) {
  const old = T;
  T = null;
  if (old) {
    scene.remove(old.group);
    old.group.traverse(o => { if (o.geometry) o.geometry.dispose(); });
    old.roadTex.dispose();
    old.mats.forEach(m => m.dispose());
  }
  const curve = new THREE.CatmullRomCurve3(
    def.points.map(p => new THREE.Vector3(p[0], 0, p[1])), true, 'centripetal');   // no overshoot on irregular spacing
  curve.arcLengthDivisions = 1600;
  const L = curve.getLength();
  const { paved, deckW, deckHalf, pavedHalf } = roadMetrics(def);
  const laneLat = (dir, idx) => laneLatOf(def, dir, idx);

  const roadTex = makeRoadTex(def);
  roadMat.map = roadTex; roadMat.needsUpdate = true;
  radarRoadMat.map = roadTex; radarRoadMat.needsUpdate = true;

  let bar = def.barrier !== 'off' ? detectBarrierWindow(curve, L) : null;
  if (bar && def.barrier === 'outside') bar.side *= -1;
  const inWin = s => {
    if (!bar) return false;
    const span = (((bar.s1 - bar.s0) % L) + L) % L;
    const d = ((s - bar.s0) % L + L) % L;
    return span > 0 ? d <= span : false;
  };

  const group = new THREE.Group();
  scene.add(group);
  const mats = [roadMat, radarRoadMat, radarBarMat, bermMat, railMat, poleMat, bulbMat, postMat, poolMat, chevMat, signMat, steelMat, catEyeMat, lampGlowMat, terrainMat];
  const qL = v => L / Math.round(L / v);
  const at = s => {
    const u = (((s / L) % 1) + 1) % 1;
    const p = curve.getPointAt(u), t = curve.getTangentAt(u);
    return { p, t, r: new THREE.Vector3(-t.z, 0, t.x) };
  };
  const hash = buildCurveHash(curve, L);

  /* road deck + radar twin — deck clamp 0.97: only bites on pathological radii */
  const roadGeo = sweep(curve, L, [[-deckHalf, 0.005], [deckHalf, 0.005]], { uPer: deckW, vPer: qL(12), clampFac: 0.97 });
  const road = new THREE.Mesh(roadGeo, roadMat);
  road.receiveShadow = true;
  group.add(road);
  const rRoad = new THREE.Mesh(roadGeo, radarRoadMat);
  rRoad.layers.set(1);
  rRoad.position.y = 0.09;
  group.add(rRoad);

  /* curbs (extended skirt tucks under terrain) */
  for (const sg of [1, -1]) {
    const prof = [[deckHalf - 0.7, 0], [deckHalf - 0.7, 0.15], [deckHalf + 0.15, 0.15], [deckHalf + 0.55, -0.25]]
      .map(([o, h]) => [sg * o, h]);
    const curb = new THREE.Mesh(sweep(curve, L, prof, { uPer: 1, vPer: qL(6) }), concTex ? new THREE.MeshStandardMaterial({ map: concTex, roughness: 0.92, metalness: 0.02, side: THREE.DoubleSide }) : bermMat);
    curb.receiveShadow = true;
    mats.push(curb.material);
    group.add(curb);
  }
  /* berms */
  const bOff = [0, 0.2, 1.55, 6, 12, 22, 34, 46, 54, 56];
  const bH = [0.02, 0.02, 0.32, 1.05, 1.35, 1.05, 0.6, 0.3, 0.02, -0.45];
  for (const sg of [1, -1]) {
    const berm = new THREE.Mesh(sweep(curve, L, bOff.map((o, i) => [sg * (deckHalf + o), bH[i]]), { uPer: 7, vPer: qLoop(qL, 14) }), bermMat);
    berm.receiveShadow = true;
    group.add(berm);
  }
  function qLoop(_q, v) { return v; }

  /* guardrails + posts (skipped where the wall takes over) */
  const skipR = bar && bar.side === 1, skipL = bar && bar.side === -1;
  for (const sg of [1, -1]) {
    const skip = sg === 1 ? skipR : skipL;
    const rail = new THREE.Mesh(sweep(curve, L,
      [[sg * (deckHalf + 0.05), 0.5], [sg * (deckHalf + 0.18), 0.66], [sg * (deckHalf + 0.05), 0.84]],
      { uPer: 1, vPer: qL(3), filterS: s => !(skip && inWin(s)) }), railMat);
    rail.castShadow = rail.receiveShadow = true;
    group.add(rail);
  }
  {
    const dummy = new THREE.Object3D(), pm = [];
    for (let s = 0; s < L; s += 4) {
      const f = at(s);
      for (const sg of [1, -1]) {
        if ((sg === 1 && skipR || sg === -1 && skipL) && inWin(s)) continue;
        dummy.position.set(f.p.x + f.r.x * (deckHalf + 0.35) * sg, 0.33, f.p.z + f.r.z * (deckHalf + 0.35) * sg);
        dummy.rotation.set(0, Math.atan2(f.t.x, f.t.z), 0);
        dummy.updateMatrix();
        pm.push(dummy.matrix.clone());
      }
    }
    const posts = new THREE.InstancedMesh(new THREE.BoxGeometry(0.1, 0.64, 0.1), postMat, pm.length);
    pm.forEach((m, i) => posts.setMatrixAt(i, m));
    posts.castShadow = true;
    group.add(posts);
  }

  /* sound barrier + chevrons + caps + gantry + radar ribbon */
  if (bar) {
    const side = bar.side;
    const wallProf = [[deckHalf - 0.05, 0], [deckHalf - 0.05, 5.3], [deckHalf + 0.45, 5.5],
      [deckHalf + 0.95, 5.5], [deckHalf + 1.35, 5.3], [deckHalf + 1.35, 0]]
      .map(([o, h]) => [side * o, h]);
    const wall = new THREE.Mesh(sweep(curve, L, wallProf, { filterS: inWin, uPer: 5.5, vPer: 3.4, segments: 700 }), mats[15] || wallMatOf());
    function wallMatOf() { const m = new THREE.MeshStandardMaterial({ map: concTex, roughness: 0.92, metalness: 0.02, side: THREE.DoubleSide }); mats.push(m); return m; }
    wall.castShadow = wall.receiveShadow = true;
    group.add(wall);
    for (const ss of [bar.s0, bar.s1]) {
      const f = at(ss);
      const cap = new THREE.Mesh(new THREE.BoxGeometry(1.3, 5.6, 1.3), wall.material);
      cap.position.set(f.p.x + f.r.x * side * (deckHalf + 0.65), 2.75, f.p.z + f.r.z * side * (deckHalf + 0.65));
      cap.rotation.y = Math.atan2(f.t.x, f.t.z);
      cap.castShadow = cap.receiveShadow = true;
      group.add(cap);
    }
    const chevs = [], q = new THREE.Quaternion(), m4 = new THREE.Matrix4(), one = new THREE.Vector3(1, 1, 1);
    const span = ((bar.s1 - bar.s0) % L + L) % L;
    for (let d = 6; d < span - 4; d += 10) {
      const f = at(bar.s0 + d);
      const g = new THREE.PlaneGeometry(1.5, 0.95);
      q.setFromAxisAngle(UP, Math.atan2(-f.t.x, -f.t.z));
      m4.compose(new THREE.Vector3(f.p.x + f.r.x * side * (deckHalf - 0.04), 2.55, f.p.z + f.r.z * side * (deckHalf - 0.04)), q, one);
      g.applyMatrix4(m4);
      chevs.push(g);
    }
    if (chevs.length) group.add(new THREE.Mesh(mergeGeometries(chevs), chevMat));
    const rb = new THREE.Mesh(sweep(curve, L,
      [[side * (deckHalf + 0.55), 0.3], [side * (deckHalf + 0.55), 2.2]],
      { filterS: inWin, uPer: 1, vPer: 5, segments: 300 }), radarBarMat);
    rb.layers.set(1);
    group.add(rb);
    const f = at(bar.s0 - 45);
    for (const sg of [-1, 1]) {
      const post = new THREE.Mesh(new THREE.BoxGeometry(0.26, 6.6, 0.26), steelMat);
      post.position.set(f.p.x + f.r.x * (deckHalf + 1.4) * sg, 3.3, f.p.z + f.r.z * (deckHalf + 1.4) * sg);
      post.castShadow = true;
      group.add(post);
    }
    const beam = new THREE.Mesh(new THREE.BoxGeometry(deckW + 2.8, 0.4, 0.4), steelMat);
    beam.position.set(f.p.x, 6.5, f.p.z);
    beam.rotation.y = Math.atan2(-f.r.z, f.r.x);
    beam.castShadow = true;
    const sign = new THREE.Mesh(new THREE.PlaneGeometry(4.6, 1.8), signMat);
    sign.position.set(f.p.x + f.r.x * pavedHalf * 0.35, 5.4, f.p.z + f.r.z * pavedHalf * 0.35);
    sign.rotation.y = Math.atan2(-f.t.x, -f.t.z);
    group.add(beam, sign);
  }

  /* centreline cat-eyes */
  if (def.lanesO > 0) {
    const pts = [];
    for (let s = 0; s < L; s += 8) {
      const f = at(s);
      for (const o of [-0.42, 0.42]) pts.push(f.p.x + f.r.x * o, 0.035, f.p.z + f.r.z * o);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
    group.add(new THREE.Points(g, catEyeMat));
  }

  /* streetlights — instanced + 5 roaming real SpotLights */
  const lamps = [];
  {
    const poleGeo = mergeGeometries([
      new THREE.CylinderGeometry(0.09, 0.12, 7.6, 10).translate(0, 3.8, 0),
      new THREE.CylinderGeometry(0.05, 0.07, 2.7, 8).rotateZ(Math.PI / 2).translate(1.25, 7.5, 0),
      new THREE.BoxGeometry(0.6, 0.16, 0.28).translate(2.55, 7.42, 0)
    ]);
    const bulbGeo = new THREE.BoxGeometry(0.42, 0.07, 0.2).translate(2.55, 7.31, 0);
    const poolGeo = new THREE.PlaneGeometry(7, 11).rotateX(-Math.PI / 2);
    const dummy = new THREE.Object3D(), poleMats = [], poolMats = [], glowPts = [];
    for (let s = 0, k = 0; s < L; s += 38, k++) {
      const side = k % 2 ? 1 : -1;
      const f = at(s);
      const Dx = -f.r.x * side, Dz = -f.r.z * side;
      const bx = f.p.x + f.r.x * (deckHalf + 2.6) * side, bz = f.p.z + f.r.z * (deckHalf + 2.6) * side;
      const bulbX = bx + Dx * 2.55, bulbZ = bz + Dz * 2.55;
      dummy.position.set(bx, 0, bz);
      dummy.rotation.set(0, Math.atan2(-Dz, Dx), 0);
      dummy.updateMatrix(); poleMats.push(dummy.matrix.clone());
      dummy.position.set(bulbX + Dx * 1.5, 0.05, bulbZ + Dz * 1.5);
      dummy.rotation.set(0, Math.atan2(f.t.x, f.t.z), 0);
      dummy.updateMatrix(); poolMats.push(dummy.matrix.clone());
      glowPts.push(bulbX, 7.28, bulbZ);
      lamps.push({ s, bulbPos: new THREE.Vector3(bulbX, 7.28, bulbZ), groundPos: new THREE.Vector3(bulbX + Dx * 1.2, 0, bulbZ + Dz * 1.2) });
    }
    const poles = new THREE.InstancedMesh(poleGeo, poleMat, poleMats.length);
    poleMats.forEach((m, i) => poles.setMatrixAt(i, m));
    poles.castShadow = true;
    const bulbs = new THREE.InstancedMesh(bulbGeo, bulbMat, poleMats.length);
    poleMats.forEach((m, i) => bulbs.setMatrixAt(i, m));
    const pools = new THREE.InstancedMesh(poolGeo, poolMat, poolMats.length);
    poolMats.forEach((m, i) => pools.setMatrixAt(i, m));
    group.add(poles, bulbs, pools);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(glowPts, 3));
    group.add(new THREE.Points(g, lampGlowMat));
  }

  /* ---------- adaptive terrain heightfield ----------
     invariant: h ≤ -0.12 wherever within the deck corridor → the road
     can never end up under the ground, on ANY track shape */
  let cx = 0, cz = 0, x0 = 1e9, x1 = -1e9, z0 = 1e9, z1 = -1e9, tr = 0;
  {
    const lut = hash.samples;
    for (const p of lut) {
      cx += p.x; cz += p.z;
      x0 = Math.min(x0, p.x); x1 = Math.max(x1, p.x);
      z0 = Math.min(z0, p.z); z1 = Math.max(z1, p.z);
    }
    cx /= lut.length; cz /= lut.length;
    for (const p of lut) tr = Math.max(tr, Math.hypot(p.x - cx, p.z - cz));
    skyGroup.position.set(cx, 0, cz);
    const M = 300, res = 4;
    const gw = Math.ceil((x1 - x0 + 2 * M) / res) + 1, gh = Math.ceil((z1 - z0 + 2 * M) / res) + 1;
    const pos = new Float32Array(gw * gh * 3);
    const col = new Float32Array(gw * gh * 3);
    const idx = [];
    for (let j = 0; j < gh; j++) for (let i = 0; i < gw; i++) {
      const x = x0 - M + i * res, z = z0 - M + j * res;
      const q = hash.nearest(x, z);
      const h = terrainHeight(q ? q.d : 1e9, x, z);
      const k = (j * gw + i) * 3;
      pos[k] = x; pos[k + 1] = h; pos[k + 2] = z;
      const n = terrainNoise(x * 0.05, z * 0.05);
      const b = 0.55 + 0.45 * n;
      col[k] = 0.42 * b; col[k + 1] = 0.46 * b; col[k + 2] = 0.34 * b;
    }
    for (let j = 0; j < gh - 1; j++) for (let i = 0; i < gw - 1; i++) {
      const a = j * gw + i, b = a + 1, c = a + gw + 1, d = a + gw;
      idx.push(a, b, c, a, c, d);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(col, 3));
    geo.setIndex(new THREE.BufferAttribute(new Uint32Array(idx), 1));
    geo.computeVertexNormals();
    const terrain = new THREE.Mesh(geo, terrainMat);
    terrainMat.map.repeat.set(90, 90);
    terrain.receiveShadow = true;
    group.add(terrain);
  }

  /* city — inner cluster follows the terrain, far ring floats above the fog */
  {
    const lut = hash.samples;
    const inner = [];
    const ir = Math.max(26, Math.min(x1 - x0, z1 - z0) * 0.32);
    for (let t = 0; t < 200 && inner.length < 14; t++) {
      const a = Math.random() * 6.283, rr = Math.random() * ir;
      const x = cx + Math.cos(a) * rr, z = cz + Math.sin(a) * rr * 0.8;
      let ok = true;
      for (const p of lut) if ((p.x - x) ** 2 + (p.z - z) ** 2 < (deckHalf + 18) ** 2) { ok = false; break; }
      if (ok) inner.push([x, z]);
    }
    const ring = () => {
      const a = Math.random() * 6.283, r = tr + 180 + Math.random() * 240;
      return [cx + Math.cos(a) * r, cz + Math.sin(a) * r];
    };
    cityTexs.forEach((tex, ti) => {
      const geos = [];
      const spots = inner.filter((_, i) => i % 3 === ti);
      const n = spots.length + 30;
      for (let i = 0; i < n; i++) {
        const [x, z] = i < spots.length ? spots[i] : ring();
        const innerB = i < spots.length;
        const w = 10 + Math.random() * 26, d = 10 + Math.random() * 26;
        const h = innerB ? 12 + Math.random() * 24 : 22 + Math.random() * 66;
        const g = new THREE.BoxGeometry(w, h, d);
        const uv = g.getAttribute('uv');
        const ox = Math.random() * 3, oy = Math.random() * 3;
        for (let k = 0; k < uv.count; k++) uv.setXY(k, uv.getX(k) + ox, uv.getY(k) + oy);
        g.translate(x, (innerB ? groundHeight(x, z) : 0) + h / 2, z);
        geos.push(g);
      }
      const mat = new THREE.MeshStandardMaterial({
        color: 0x04060c, emissive: 0xffffff, emissiveMap: tex,
        emissiveIntensity: ti === 0 ? 1.15 : 0.55, roughness: 1, fog: ti === 0 });
      mats.push(mat);
      group.add(new THREE.Mesh(mergeGeometries(geos), mat));
    });
  }

  T = { def, curve, L, hash, laneLat, paved, pavedHalf, deckHalf, deckW, bar, inWin, lamps, group, mats, roadTex };
  return T;
}

/* the 5 roaming streetlight SpotLights (light pooling) */
export const lampSpots = [];
for (let i = 0; i < 5; i++) {
  const sp = new THREE.SpotLight(0xffbd77, 0, 60, 1.02, 0.72, 2);
  scene.add(sp, sp.target);
  lampSpots.push({ light: sp, lamp: -1, power: 0 });
}
let lampTimer = 0, lastNear = [];
export function updateLampLights(dt, sEgo) {
  if (!T || !T.lamps.length) return;
  lampTimer -= dt;
  if (lampTimer <= 0) {
    lampTimer = 0.35;
    lastNear = T.lamps.map((l, i) => [i, Math.abs(wrapS(l.s - sEgo))])
      .sort((a, b) => a[1] - b[1]).slice(0, 5).map(a => a[0]);
  }
  for (const d of lampSpots) {
    const wanted = d.lamp >= 0 && lastNear.includes(d.lamp);
    d.power = THREE.MathUtils.clamp(d.power + (wanted ? 2.2 : -5) * dt, 0, 1);
    d.light.intensity = 360 * d.power * d.power;
    if (!wanted && d.power <= 0.02) {
      const free = lastNear.find(n => !lampSpots.some(o => o !== d && o.lamp === n));
      if (free !== undefined) {
        d.lamp = free;
        d.light.position.copy(T.lamps[free].bulbPos);
        d.light.target.position.copy(T.lamps[free].groundPos);
      } else d.lamp = -1;
    }
  }
} 