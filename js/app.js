import * as THREE from 'three';
import { renderer, scene, fog, moonLight, moonDir } from './env.js';
import { trackDef, DEFAULT_POINTS, T, buildTrack, wrapS, updateLampLights } from './track.js';
import { buildEgo, buildAI, disposeAI, egoTailMat, egoHeadMat } from './cars.js';
import { VehicleSim, SkidTrail, Smoke } from './physics.js';
import { AudioFX } from './audio.js';
import { initEditor, ed, openEditor, closeEditor, recomputeEd, computeEdView, redrawEditor } from './editor.js';

const $ = id => document.getElementById(id);
const MAP_PX = 240, MAP_MARG = 20;
const clamp = THREE.MathUtils.clamp;
const UP = new THREE.Vector3(0, 1, 0);

/* ego + fx systems */
const ego = buildEgo();
const carB = ego.root;
scene.add(carB);
const skids = new SkidTrail();
scene.add(skids.mesh);
const smoke = new Smoke();
scene.add(smoke.points);
const sim = new VehicleSim();
const audio = new AudioFX();

/* cameras */
const fpsCam = new THREE.PerspectiveCamera(65, innerWidth / innerHeight, 0.1, 1000);
fpsCam.position.set(-0.35, 1.25, 0.2);
ego.body.add(fpsCam);
const chaseCam = new THREE.PerspectiveCamera(62, innerWidth / innerHeight, 0.3, 1000);
chaseCam.position.set(0, 4, 10);
let camMode = 'cockpit';
const minimapCam = new THREE.OrthographicCamera(-50, 50, 50, -50, 1, 500);
minimapCam.position.set(0, 120, 0);
minimapCam.up.set(0, 0, -1);
minimapCam.lookAt(0, 0, 0);
minimapCam.layers.enable(1);

/* scenario vehicles */
const PAINTS = [0x39404a, 0x9aa2ac, 0xc9ced4, 0x59626e, 0x7f8894];
const scenario = [];
let aiSeq = 0;
const player = { spawnU: 0.08, cruise: 13.9 };
let mode = 'manual', crashed = null, graceT = 1, shakeT = 0, headlightsOn = true;
const keys = {};

const FR = { p: new THREE.Vector3(), t: new THREE.Vector3(), r: new THREE.Vector3() };
function placeOnRail(car, s, lat, reversed) {
  const u = (((s / T.L) % 1) + 1) % 1;
  T.curve.getPointAt(u, FR.p);
  T.curve.getTangentAt(u, FR.t);
  FR.r.crossVectors(FR.t, UP).normalize();
  car.position.set(FR.p.x + FR.r.x * lat, 0, FR.p.z + FR.r.z * lat);
  car.rotation.y = reversed ? Math.atan2(FR.t.x, FR.t.z) : Math.atan2(-FR.t.x, -FR.t.z);
}

function addAI(p = {}) {
  let dir = p.dir === 'onc' && trackDef.lanesO > 0 ? 'onc' : 'fwd';
  const built = buildAI({
    hatch: !!p.hatch, paint: p.paint ?? PAINTS[aiSeq % PAINTS.length],
    onc: dir === 'onc', bigLights: p.bigLights ?? scenario.length === 0
  });
  const ai = {
    id: ++aiSeq, name: p.name || 'CAR ' + (aiSeq + 1), dir,
    lane: p.lane || 0,
    spawnU: p.spawnU ?? (((player.spawnU + 40 / T.L) % 1) + 1) % 1,
    speed: p.speed ?? 13, behavior: p.behavior || 'cruise',
    hatch: !!p.hatch, paint: p.paint ?? null,
    ...built,
    s: 0, v: 0, braking: false, timer: 4 + Math.random() * 5, forceT: 0, blink: Math.random() * 2,
    lat: 0, latV: 0, yawOff: 0, yawV: 0
  };
  ai.lat = T.laneLat(ai.dir, ai.lane);
  ai.s = ai.spawnU * T.L;
  ai.v = ai.behavior === 'stopped' ? 0 : ai.speed;
  scenario.push(ai);
  scene.add(ai.root);
  placeOnRail(ai.root, ai.s, ai.lat, ai.dir === 'onc');
  return ai;
}
function removeAI(ai) {
  scene.remove(ai.root);
  disposeAI(ai);
}
function clearScenario() { scenario.forEach(removeAI); scenario.length = 0; }
function defaultScenario() {
  clearScenario();
  const rel = m => (((player.spawnU + m / T.L) % 1) + 1) % 1;
  addAI({ name: 'CAR A', hatch: true, paint: 0xe8631e, dir: 'fwd', lane: 0, spawnU: rel(30), speed: 14, behavior: 'brake' });
  addAI({ name: 'CAR 2', dir: 'onc', lane: 0, spawnU: rel(180), speed: 15, behavior: 'cruise' });
  addAI({ name: 'CAR 3', dir: 'onc', lane: 1, spawnU: rel(260), speed: 13, behavior: 'cruise' });
  addAI({ name: 'CAR 4', dir: 'fwd', lane: 1, spawnU: rel(55), speed: 10.5, behavior: 'cruise' });
}
function setAIBrake(ai, on) {
  ai.tailM.emissiveIntensity = on ? 5 : 1.1;
  ai.glows.forEach(g => g.scale.setScalar(on ? 1.4 : 0.5));
  ai.lights.forEach(l => l.intensity = on ? 30 : 3.4);
}

/* ---------- rebuild / respawn (ego spawns IN ITS LANE — the fix that
   made every V2V lateral check sane) ---------- */
function respawnAll() {
  const u = ((player.spawnU % 1) + 1) % 1;
  const sp = T.curve.getPointAt(u);
  const st = T.curve.getTangentAt(u);
  const r = new THREE.Vector3().crossVectors(st, UP).normalize();
  const lat = T.laneLat('fwd', 0);                       // lane centre, not the centerline
  sim.place(sp.x + r.x * lat, sp.z + r.z * lat, Math.atan2(-st.x, -st.z));
  for (const ai of scenario) {
    ai.s = ai.spawnU * T.L;
    ai.v = ai.behavior === 'stopped' ? 0 : ai.speed;
    ai.braking = false; ai.forceT = 0; ai.latV = 0; ai.yawOff = 0; ai.yawV = 0;
    ai.lat = T.laneLat(ai.dir, ai.lane);
    placeOnRail(ai.root, ai.s, ai.lat, ai.dir === 'onc');
  }
  chaseCam.position.set(sim.pos.x - sim.fw.x * 9, 3.6, sim.pos.z - sim.fw.z * 9);
}
function rebuild() {
  buildTrack(trackDef);
  scenario.forEach(ai => {
    if (ai.dir === 'onc' && trackDef.lanesO === 0) ai.dir = 'fwd';
    const maxL = ai.dir === 'fwd' ? trackDef.lanesF : trackDef.lanesO;
    ai.lane = clamp(ai.lane, 0, Math.max(0, maxL - 1));
    ai.lat = T.laneLat(ai.dir, ai.lane);
  });
  skids.reset();
  smoke.reset();
  respawnAll();
  renderVehicles();
  if (ed.open) { recomputeEd(); computeEdView(); redrawEditor(); }
}

/* ---------- V2V ASSIST — ACC / FCW / AEB / EVASIVE STEER ----------
   Interventions are physics-level inputs (throttle/brake/steer), and
   the BSM data propagates through the sound barrier, so occluded
   brake events are warned even when invisible. Every state surfaces
   on a telltale chip so you can SEE the system working. */
const assist = {
  alert: null, sev: 'amber', target: 0, brake: 0, steer: 0,
  accActive: false, aeb: false, fcw: null, evasive: false, evT: 0
};
function occludedByBarrier(targetS) {
  const b = T.bar;
  if (!b) return false;
  const g = ((targetS - sim.s) % T.L + T.L) % T.L;
  if (g < 15) return false;
  let m = 0;
  for (let d = 4; d < g - 4; d += 5) if (T.inWin(sim.s + d)) m += 5;
  return m > 22;
}
function computeAssist(dt) {
  assist.target = player.cruise;
  assist.brake = 0;
  assist.steer = 0;
  assist.accActive = false;
  assist.fcw = null;
  assist.sev = 'amber';
  const wasAeb = assist.aeb;
  assist.aeb = false;

  let lead = null, lg = 1e9;
  for (const ai of scenario) {
    if (ai.dir !== 'fwd') continue;
    const g = wrapS(ai.s - sim.s);
    if (g <= 1 || g > 170) continue;
    if (Math.abs(T.laneLat('fwd', ai.lane) - sim.lat) < 2.6 && g < lg) { lg = g; lead = ai; }
  }
  if (lead) {
    const closing = sim.vx - lead.v;
    const ttc = closing > 0.4 ? lg / closing : 1e9;
    const headway = sim.vx * 1.5 + 9;
    const leadBraking = lead.braking || lead.forceT > 0 || lead.behavior === 'stopped';
    if (lg < 12 || ttc < 1.6) {                            /* AEB — full stop */
      assist.aeb = true;
      assist.target = 0; assist.brake = 1;
      assist.fcw = 'AEB · AUTOMATIC EMERGENCY BRAKE';
      assist.sev = 'red';
    } else if (leadBraking && ttc < 6) {                   /* FCW */
      assist.target = Math.max(0, lead.v - 2.5);
      assist.brake = 0.8;
      assist.fcw = occludedByBarrier(lead.s)
        ? 'FCW · OCCLUDED VEHICLE BRAKING — RADIO ALERT'
        : 'FCW · LEAD VEHICLE BRAKING';
    } else if (lg < headway) {                             /* ACC headway */
      assist.target = Math.max(0, lead.v);
      assist.brake = clamp((sim.vx - assist.target) / 5, 0, 0.5);
      assist.accActive = true;
      if (lg < headway * 0.65) assist.fcw = 'ACC · CLOSING ON LEAD — ADJUSTING';
    }
  }

  /* oncoming intrusion → evasive steer, with release hysteresis */
  let threat = false;
  if (trackDef.lanesO > 0 && sim.lat < 1.2) {
    let worst = 1e9;
    for (const ai of scenario) {
      if (ai.dir !== 'onc') continue;
      const g = wrapS(ai.s - sim.s);
      if (g < 0 || g > 260) continue;
      if (Math.abs(T.laneLat('onc', ai.lane) - sim.lat) > 2.8) continue;
      worst = Math.min(worst, g / Math.max(sim.vx + ai.v, 2));
    }
    threat = worst < 3.5;
  }
  if (threat) { assist.evasive = true; assist.evT = 0.8; }
  else if (assist.evasive) {
    assist.evT -= dt;
    if (assist.evT <= 0) assist.evasive = false;
  }
  if (assist.evasive) {
    assist.steer = 0.85;                                   // hard right, away from intrusion
    assist.target = Math.min(assist.target, 7);
    assist.brake = Math.max(assist.brake, 0.9);
    assist.fcw = 'DANGER · ONCOMING TRAFFIC — EVASIVE STEER ENGAGED';
    assist.sev = 'red';
  }
  assist.alert = assist.fcw;
  if (assist.aeb && !wasAeb) toast('AEB — AUTOMATIC EMERGENCY BRAKE ENGAGED');
}

/* ---------- player driving ---------- */
function driveInputs(dt) {
  const up = keys.KeyW || keys.ArrowUp;
  const dn = keys.KeyS || keys.ArrowDown;
  const lf = keys.KeyA || keys.ArrowLeft;
  const rt = keys.KeyD || keys.ArrowRight;
  const sIn = (rt ? 1 : 0) - (lf ? 1 : 0);
  if (mode === 'manual') {
    sim.inThrottle = up ? 1 : 0;
    sim.inBrake = dn ? 1 : 0;
    sim.inSteer = sIn;
  } else {                                          /* V2V: pedals command the cruise */
    if (up) player.cruise = Math.min(player.cruise + 8 * dt, 44);
    if (dn) player.cruise = Math.max(player.cruise - 12 * dt, 0);
    computeAssist(dt);
    const acc = clamp((assist.target - sim.vx) * 0.3, 0, 1);
    sim.inThrottle = Math.max(0, acc);
    sim.inBrake = Math.max(assist.brake, clamp((sim.vx - assist.target) / 6, 0, 1));
    sim.inSteer = clamp(sIn * 0.55 + assist.steer, -1, 1);
  }
  sim.inHand = !!keys.Space;
}
function applyEgoVisual(dt) {
  carB.position.set(sim.pos.x, sim.groundY, sim.pos.z);
  carB.rotation.y = sim.heading;
  const roll = clamp(sim.latAcc * 0.09, -0.14, 0.14);
  const pitch = clamp(sim.lonAcc * 0.05, -0.09, 0.09);
  ego.body.rotation.z += (roll - ego.body.rotation.z) * Math.min(1, dt * 7);
  ego.body.rotation.x += (pitch - ego.body.rotation.x) * Math.min(1, dt * 7);
  for (const w of ego.wheels) {
    if (w.front) w.group.rotation.y += (-sim.steer - w.group.rotation.y) * Math.min(1, dt * 12);
    w.spin.rotation.x -= (sim.wv / 0.32) * dt;       // wheels follow WHEEL speed (burnouts!)
  }
  egoTailMat.emissiveIntensity = sim.inBrake > 0.2 || (mode === 'v2v' && assist.brake > 0.3) ? 4.5 : 0.9;
}

/* ---------- AI update ---------- */
const tmpF = new THREE.Vector3(), tmpW = new THREE.Vector3();
function updateAI(dt) {
  for (const ai of scenario) {
    if (ai.forceT > 0) ai.forceT -= dt;
    if (ai.behavior === 'brake') {
      ai.timer -= dt;
      if (ai.timer <= 0) {
        ai.braking = !ai.braking;
        ai.timer = ai.braking ? 1.6 + Math.random() * 1.7 : 7 + Math.random() * 9;
      }
    }
    let braking = ai.forceT > 0 || ai.braking || ai.behavior === 'stopped';
    let want = ai.behavior === 'stopped' ? 0 : braking ? Math.max(ai.speed * 0.22, 2.5) : ai.speed;
    if (ai.dir === 'fwd') {
      let g = 1e9, lv = 0;
      const myLat = T.laneLat('fwd', ai.lane);
      for (const o of scenario) {
        if (o === ai || o.dir !== 'fwd' || o.lane !== ai.lane) continue;
        const d = wrapS(o.s - ai.s);
        if (d > 0.5 && d < g) { g = d; lv = o.v; }
      }
      if (Math.abs(sim.lat - myLat) < 2.6 && Math.abs(sim.lat) < T.pavedHalf + 1) {
        const d = wrapS(sim.s - ai.s);
        if (d > 0.5 && d < g) { g = d; lv = sim.vx; }
      }
      if (g < 15) want = Math.min(want, lv * Math.max(0, (g - 5.5) / 9.5));
    }
    if (Math.abs(ai.yawOff) > 0.4) want = Math.min(want, 3);
    const rate = want < ai.v ? 6.5 : 2.2;
    ai.v = Math.max(0, ai.v + clamp(want - ai.v, -rate * dt, rate * dt));
    ai.s = ai.dir === 'fwd' ? (ai.s + ai.v * dt) % T.L : (ai.s - ai.v * dt + T.L) % T.L;
    ai.lat += ai.latV * dt;
    ai.latV *= Math.max(0, 1 - 2.2 * dt);
    ai.yawOff += ai.yawV * dt;
    ai.yawV *= Math.max(0, 1 - 1.6 * dt);
    ai.yawOff *= Math.max(0, 1 - 0.8 * dt);
    ai.lat = clamp(ai.lat, -T.pavedHalf - 1.5, T.pavedHalf + 1.5);
    placeOnRail(ai.root, ai.s, ai.lat, ai.dir === 'onc');
    if (ai.dir === 'onc') ai.root.rotation.y += ai.yawOff; else ai.root.rotation.y -= ai.yawOff;
    for (const w of ai.wheels) w.spin.rotation.x -= (ai.dir === 'fwd' ? ai.v : -ai.v) / 0.32 * dt;
    if (ai.behavior === 'stopped') { ai.blink += dt; braking = (ai.blink % 1.4) < 0.7; }
    setAIBrake(ai, braking || ai.v < ai.speed - 1.5);
    if (ai.v < ai.speed - 3 && ai.dir === 'fwd' && ai.v > 2) {
      const cs = Math.cos(ai.root.rotation.y), sn = Math.sin(ai.root.rotation.y);
      for (const lz of [1.46]) for (const lx of [-0.8, 0.8]) {
        skids.wheel('ai' + ai.id + lz + lx,
          ai.root.position.x + lx * cs + lz * sn, 0.02, ai.root.position.z - lx * sn + lz * cs, true, 0.6);
      }
    }
  }
}

/* ---------- car ↔ car collisions ---------- */
function aiWorldFwd(ai) { return tmpW.set(0, 0, -1).applyQuaternion(ai.root.quaternion); }
function carCircles(pos, fwd) {
  return [
    { x: pos.x + fwd.x * 1.1, z: pos.z + fwd.z * 1.1 },
    { x: pos.x - fwd.x * 1.1, z: pos.z - fwd.z * 1.1 }
  ];
}
function carCollisions() {
  if (graceT > 0) return;
  const fw = sim.fw;
  const pc = carCircles(sim.pos, fw);
  for (const ai of scenario) {
    const af = { x: aiWorldFwd(ai).x, z: aiWorldFwd(ai).z };
    const ac = carCircles(ai.root.position, af);
    for (const p of pc) for (const a of ac) {
      const dx = p.x - a.x, dz = p.z - a.z;
      const d = Math.hypot(dx, dz);
      if (d > 2.25 || d < 1e-4) continue;
      const nx = dx / d, nz = dz / d;
      const overlap = 2.25 - d;
      sim.pos.x += nx * overlap * 0.5;
      sim.pos.z += nz * overlap * 0.5;
      const pvx = sim.vx * fw.x + sim.vy * sim.rt.x, pvz = sim.vx * fw.z + sim.vy * sim.rt.z;
      const avx = af.x * ai.v * (ai.dir === 'fwd' ? 1 : -1), avz = af.z * ai.v * (ai.dir === 'fwd' ? 1 : -1);
      const rvn = (pvx - avx) * nx + (pvz - avz) * nz;
      if (rvn < 0) {
        const j = -rvn * 1.3;
        const wvx = pvx - nx * j, wvz = pvz - nz * j;
        sim.vx = wvx * fw.x + wvz * fw.z;
        sim.vy = wvx * sim.rt.x + wvz * sim.rt.z;
        sim.omega += j * 0.25 * Math.sign(nx * fw.z - nz * fw.x);
        ai.latV += (nx * -af.x + nz * -af.z) * j * 0.9;
        ai.yawV += j * 0.3 * (Math.random() > 0.5 ? 1 : -1);
        ai.v = Math.max(0, ai.v - j * 0.5);
        sim.impact = Math.max(sim.impact, j);
        shakeT = Math.max(shakeT, 0.6);
        audio.init(); audio.impact(j);
        for (let k = 0; k < 6; k++) smoke.spawn(a.x, 0.6, a.z, 1.2);
        if (j > 6.5) doCrash(ai, j);
      }
    }
  }
}
function doCrash(ai, dv) {
  crashed = { name: ai.name, dv, headOn: ai.dir === 'onc' };
  sim.vx = sim.vy = 0;
  shakeT = 1.4;
  $('flash').classList.add('on');
  $('crash').style.display = 'flex';
  $('crashInfo').textContent =
    (crashed.headOn ? 'HEAD-ON — ONCOMING LANE' : 'REAR-END COLLISION') +
    ` · ${ai.name} · IMPACT ${Math.round(dv * 3.6)} KM/H · ` +
    (mode === 'v2v' ? 'V2V ASSIST — LIMITS EXCEEDED' : 'MANUAL — NO V2V LINK');
  toast('COLLISION — PRESS R TO RESET THE RUN');
}
function resetRun() {
  crashed = null;
  $('crash').style.display = 'none';
  $('flash').classList.remove('on');
  graceT = 1.6;
  skids.reset();
  smoke.reset();
  if (mode === 'v2v' && player.cruise < 6) player.cruise = 13.9;
  respawnAll();
  toast('RUN RESET — SCENARIO RESTARTED');
}

/* ---------- player skids & smoke ---------- */
function playerFX(dt) {
  const contacts = sim.wheelContacts();
  let maxSlip = 0;
  contacts.forEach((c, i) => {
    const front = c.front;
    /* rear marks/smoke key on REAL slip: locked, or wheelspin past 2.2 m/s
       (below that it's just a firm launch, not a burnout) */
    const slip = front
      ? (sim.lockF ? 1 : clamp((sim.slipF - 0.14) * 4, 0, 1))
      : (sim.lockR ? 1 : sim.spin > 2.2 ? 1 : clamp((sim.slipR - 0.14) * 4, 0, 1));
    maxSlip = Math.max(maxSlip, slip);
    const onAsphalt = Math.abs(sim.lat) < T.pavedHalf + 0.4 && sim.vAbs > 2;
    skids.wheel('p' + i, c.x, c.y + 0.01, c.z, slip > 0.05 && onAsphalt, 0.4 + slip * 0.6);
    const hot = front ? (sim.lockF || sim.slipF > 0.18) : (sim.lockR || sim.spin > 2.4);
    if (slip > 0.3 && hot && onAsphalt && Math.random() < slip * 0.6) smoke.spawn(c.x, c.y + 0.1, c.z, 0.7 + slip * 0.6);
  });
  if (sim.scrape > 0) {
    sim.scrape -= dt;
    if (sim.scrapePos && Math.random() < 0.5) {
      smoke.spawn(sim.scrapePos.x, 0.5, sim.scrapePos.z, 0.9);
      maxSlip = Math.max(maxSlip, 0.7);
    }
  }
  sim.impact = 0;
  skids.update(dt);
  smoke.update(dt);
  audio.update(sim.rpm, sim.inThrottle, sim.vAbs, maxSlip);
}

/* ---------- HUD ---------- */
const tele = $('tele'), mapTick = $('mapTick'), alertEl = $('alert');
/* extra telltale chips for the V2V interventions (created in JS so the
   html doesn't need another edit) */
{
  const tells = $('tells');
  [['tAcc', 'ACC'], ['tFcw', 'FCW'], ['tAeb', 'AEB']].forEach(([id, lab]) => {
    const s = document.createElement('span');
    s.id = id;
    s.textContent = lab;
    tells.appendChild(s);
  });
}
let toastTimer = 0, hudT = 0, alertTxt = '', lastChirpT = 0;
function toast(msg) {
  const t = $('toast');
  t.textContent = msg;
  t.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove('show'), 1700);
}
function updateHUD() {
  $('spd').textContent = Math.round(sim.vAbs * 3.6);
  $('gearbox').textContent = sim.reverse ? 'R' : (sim.vAbs < 0.4 && sim.inThrottle < 0.05 ? 'N' : sim.gear);
  const rp = clamp((sim.rpm - 900) / 5700, 0, 1);
  const bar = $('rpmBar');
  bar.style.width = (rp * 100).toFixed(1) + '%';
  bar.classList.toggle('red', rp > 0.85);
  $('gdot').style.transform = `translate(${clamp(sim.latAcc, -1.2, 1.2) * 20}px, ${clamp(-sim.lonAcc, -1.2, 1.2) * 20}px)`;
  $('tEsc').classList.toggle('lit', sim.escActive);
  $('tEsc').classList.toggle('red', sim.escActive);
  $('tAbs').classList.toggle('lit', sim.absActive);
  $('tTc').classList.toggle('lit', sim.tcActive);
  $('tV2v').classList.toggle('lit', mode === 'v2v');
  $('tAcc').classList.toggle('lit', mode === 'v2v' && assist.accActive);
  $('tFcw').classList.toggle('lit', mode === 'v2v' && !!assist.fcw);
  $('tFcw').classList.toggle('red', mode === 'v2v' && assist.sev === 'red');
  $('tAeb').classList.toggle('lit', mode === 'v2v' && (assist.aeb || assist.evasive));
  $('tAeb').classList.toggle('red', mode === 'v2v' && (assist.aeb || assist.evasive));
  let txt = (mode === 'v2v' ? `V2V LINK · ${scenario.length} BSM` : 'FULLY MANUAL · NO LINK') +
    (sim.off ? ' · OFF PAVEMENT' : '') +
    (sim.lockF || sim.lockR ? ' · WHEEL LOCK' : '') +
    (sim.spin > 2.5 ? ' · WHEELSPIN' : '');
  let warn = sim.off;
  let lead = null, lg = 1e9;
  for (const ai of scenario) {
    if (ai.dir !== 'fwd') continue;
    const g = wrapS(ai.s - sim.s);
    if (g > 1 && g < 250 && Math.abs(T.laneLat('fwd', ai.lane) - sim.lat) < 4.5 && g < lg) { lg = g; lead = ai; }
  }
  if (lead) {
    const occ = occludedByBarrier(lead.s);
    txt += ` · LEAD ${Math.round(lg)} M · ${occ ? 'OCCLUDED — RADAR ONLY' : 'VISUAL'}`;
    warn = warn || occ;
  }
  tele.textContent = txt;
  tele.classList.toggle('warn', warn);
  tmpF.set(0, 0, -1).applyQuaternion(carB.quaternion);
  mapTick.style.transform = `rotate(${(Math.atan2(tmpF.x, -tmpF.z) * 180 / Math.PI).toFixed(1)}deg)`;
}
function updateAlertDOM() {
  const t = mode === 'v2v' ? assist.alert : '';
  if (t !== alertTxt) {
    alertTxt = t;
    alertEl.textContent = t;
    alertEl.className = t ? 'on ' + assist.sev : '';
    const now = performance.now();                        // audible FCW / AEB warning
    if (t && now - lastChirpT > 1100) {
      audio.chirp(assist.sev === 'red');
      lastChirpT = now;
    }
  }
}

/* ---------- camera ---------- */
let mx = 0, my = 0, yaw = 0, pitch = -0.045;
addEventListener('pointermove', e => {
  mx = (e.clientX / innerWidth) * 2 - 1;
  my = (e.clientY / innerHeight) * 2 - 1;
});
function updateCamera(dt, et) {
  const driving = keys.KeyW || keys.KeyS || keys.KeyA || keys.KeyD ||
    keys.ArrowUp || keys.ArrowDown || keys.ArrowLeft || keys.ArrowRight;
  const k = 1 - Math.exp(-dt * 3.2);
  yaw += ((driving ? 0 : -mx * 0.5) - yaw) * k;
  pitch += (-0.045 - (driving ? 0 : my * 0.16) - pitch) * k;
  let vib = (sim.lockF || sim.lockR || sim.spin > 2) && sim.vAbs > 4 ? 0.012 : 0;
  if (sim.off && sim.vAbs > 1) vib += 0.02;
  let py = 1.25 + Math.sin(et * 8.5) * 0.006 * (sim.vAbs / 20) + (Math.random() - 0.5) * vib;
  if (shakeT > 0) {
    shakeT -= dt;
    py += (Math.random() - 0.5) * 0.09 * shakeT;
    yaw += (Math.random() - 0.5) * 0.05 * shakeT;
  }
  fpsCam.rotation.set(pitch, yaw, -yaw * 0.05 - sim.latAcc * 0.03);
  fpsCam.position.set(-0.35, py, 0.2);
  const fw = sim.fw;
  const tx = sim.pos.x - fw.x * 8.2, tz = sim.pos.z - fw.z * 8.2;
  const ty = Math.max(sim.groundY + 3.1, chaseCam.position.y - 1);
  const ck = 1 - Math.exp(-dt * 4);
  chaseCam.position.x += (tx - chaseCam.position.x) * ck;
  chaseCam.position.y += (ty - chaseCam.position.y) * ck;
  chaseCam.position.z += (tz - chaseCam.position.z) * ck;
  chaseCam.lookAt(sim.pos.x + fw.x * 4, sim.groundY + 1.0, sim.pos.z + fw.z * 4);
}

/* ---------- modes / toggles ---------- */
function setMode(m, silent) {
  mode = m;
  $('mManual').classList.toggle('on', m === 'manual');
  $('mV2v').classList.toggle('on', m === 'v2v');
  $('modeHint').textContent = m === 'manual'
    ? 'NO LINK — RAW PHYSICS, EYES ONLY'
    : 'BSM LINK · ACC + FCW + AEB + EVASIVE STEER · W/S = SET CRUISE';
  if (m === 'v2v' && sim.vAbs > 2) player.cruise = Math.max(8, sim.vx);
  if (!silent) toast(m === 'manual' ? 'FULLY MANUAL — V2V LINK OFF, AIDS AS CONFIGURED' : 'V2V ASSIST ENGAGED');
}
function setHeadlights(on) {
  headlightsOn = on;
  ego.headSpots.forEach(sp => sp.intensity = on ? 1200 : 0);
  egoHeadMat.emissiveIntensity = on ? 5 : 0.15;
  ego.beams.forEach(b => b.visible = on);
}
function setFogDensity(d) {
  fog.density = Math.max(0.0005, +d || 0.02);
  $('fogRange').value = fog.density;
}

/* ---------- console UI ---------- */
function renderVehicles() {
  const box = $('vlist');
  box.innerHTML = '';
  const oncAllowed = trackDef.lanesO > 0;
  scenario.forEach(ai => {
    const row = document.createElement('div');
    row.className = 'vrow' + (ai.dir === 'onc' ? ' onc' : '');
    const l1 = document.createElement('div'); l1.className = 'l1';
    const l2 = document.createElement('div'); l2.className = 'l2';
    const name = document.createElement('span'); name.className = 'vname'; name.textContent = ai.name;
    const dirSel = document.createElement('select');
    [['fwd', 'FWD'], ...(oncAllowed ? [['onc', 'ONC']] : [])].forEach(([val, lab]) => {
      const o = document.createElement('option'); o.value = val; o.textContent = lab; dirSel.appendChild(o);
    });
    dirSel.value = ai.dir;
    dirSel.onchange = () => { ai.dir = dirSel.value; ai.lane = 0; ai.lat = T.laneLat(ai.dir, 0); renderVehicles(); };
    const laneSel = document.createElement('select');
    const maxL = ai.dir === 'fwd' ? trackDef.lanesF : trackDef.lanesO;
    for (let i = 0; i < maxL; i++) {
      const o = document.createElement('option'); o.value = i; o.textContent = 'L' + (i + 1); laneSel.appendChild(o);
    }
    ai.lane = Math.min(ai.lane, maxL - 1);
    laneSel.value = ai.lane;
    laneSel.onchange = () => { ai.lane = +laneSel.value; ai.lat = T.laneLat(ai.dir, ai.lane); };
    const behSel = document.createElement('select');
    [['cruise', 'CRUISE'], ['brake', 'BRAKE EVT'], ['stopped', 'STOPPED']].forEach(([val, lab]) => {
      const o = document.createElement('option'); o.value = val; o.textContent = lab; behSel.appendChild(o);
    });
    behSel.value = ai.behavior;
    behSel.onchange = () => { ai.behavior = behSel.value; if (ai.behavior === 'stopped') ai.v = 0; };
    const spd = document.createElement('input');
    spd.type = 'range'; spd.min = 0; spd.max = 120; spd.value = Math.round(ai.speed * 3.6);
    const spdl = document.createElement('span'); spdl.className = 'vkmh'; spdl.textContent = spd.value + ' KMH';
    spd.oninput = () => { ai.speed = spd.value / 3.6; spdl.textContent = spd.value + ' KMH'; };
    const del = document.createElement('button');
    del.className = 'vdel'; del.textContent = '×';
    del.onclick = () => { removeAI(ai); scenario.splice(scenario.indexOf(ai), 1); renderVehicles(); };
    l1.append(name, dirSel, laneSel, del);
    l2.append(behSel, spd, spdl);
    row.append(l1, l2);
    box.appendChild(row);
  });
}
function syncTrackUI() {
  $('lfVal').textContent = trackDef.lanesF;
  $('loVal').textContent = trackDef.lanesO;
  $('barrierSel').value = trackDef.barrier;
  $('fogRange').value = fog.density;
}
 $('lfMinus').onclick = () => { trackDef.lanesF = Math.max(1, trackDef.lanesF - 1); syncTrackUI(); rebuild(); };
 $('lfPlus').onclick = () => { trackDef.lanesF = Math.min(3, trackDef.lanesF + 1); syncTrackUI(); rebuild(); };
 $('loMinus').onclick = () => { trackDef.lanesO = Math.max(0, trackDef.lanesO - 1); syncTrackUI(); rebuild(); };
 $('loPlus').onclick = () => { trackDef.lanesO = Math.min(3, trackDef.lanesO + 1); syncTrackUI(); rebuild(); };
 $('barrierSel').onchange = () => { trackDef.barrier = $('barrierSel').value; rebuild(); };
 $('fogRange').oninput = () => setFogDensity(+$('fogRange').value);
 $('btnEdit').onclick = () => ed.open ? closeEditor() : openEditor();
 $('btnDefault').onclick = restoreDefault;
 $('addVeh').onclick = () => { addAI({}); renderVehicles(); toast('VEHICLE ADDED — DRAG ITS MARKER IN THE EDITOR'); };
 $('btnReset').onclick = resetRun;
 $('crashReset').onclick = resetRun;
 $('mManual').onclick = () => setMode('manual');
 $('mV2v').onclick = () => setMode('v2v');
 $('conToggle').onclick = () => {
  const c = $('console');
  c.classList.toggle('collapsed');
  $('conToggle').textContent = c.classList.contains('collapsed') ? '+' : '–';
};
function aidBtn(id, key) {
  $(id).onclick = () => {
    sim.aids[key] = !sim.aids[key];
    $(id).classList.toggle('on', sim.aids[key]);
    toast(key.toUpperCase() + (sim.aids[key] ? ' ON' : ' OFF'));
  };
}
aidBtn('tAbsB', 'abs'); aidBtn('tTcB', 'tc'); aidBtn('tEscB', 'esc');
 $('tSndB').onclick = () => {
  audio.init(); audio.resume();
  audio.setMuted(!audio.muted);
  $('tSndB').classList.toggle('on', !audio.muted);
};
function restoreDefault() {
  trackDef.points = DEFAULT_POINTS.map(p => p.slice());
  trackDef.lanesF = 2; trackDef.lanesO = 2; trackDef.barrier = 'inside';
  player.spawnU = 0.08;
  sim.aids = { esc: true, abs: true, tc: true };
  $('tAbsB').classList.add('on'); $('tTcB').classList.add('on'); $('tEscB').classList.add('on');
  syncTrackUI();
  rebuild();
  defaultScenario();
  resetRun();
  toast('DEFAULT TRACK RESTORED');
}
function saveScenario() {
  const data = {
    points: trackDef.points, lanesF: trackDef.lanesF, lanesO: trackDef.lanesO,
    barrier: trackDef.barrier, egoU: player.spawnU, fog: fog.density, mode,
    aids: sim.aids, vehicles: scenario.map(a => ({
      name: a.name, dir: a.dir, lane: a.lane, spawnU: a.spawnU,
      speed: a.speed, behavior: a.behavior, hatch: a.hatch, paint: a.paint
    }))
  };
  try { localStorage.setItem('v2v-scenario-v2', JSON.stringify(data)); toast('SCENARIO SAVED'); }
  catch { toast('SAVE FAILED (STORAGE BLOCKED)'); }
}
function loadScenario() {
  let data = null;
  try { data = JSON.parse(localStorage.getItem('v2v-scenario-v2')); } catch { }
  if (!data || !Array.isArray(data.points)) { toast('NO SAVED SCENARIO FOUND'); return; }
  trackDef.points = data.points.map(p => p.slice());
  trackDef.lanesF = data.lanesF || 2;
  trackDef.lanesO = data.lanesO ?? 2;
  trackDef.barrier = data.barrier || 'off';
  player.spawnU = data.egoU ?? 0.08;
  setFogDensity(data.fog ?? 0.02);
  Object.assign(sim.aids, data.aids || {});
  $('tAbsB').classList.toggle('on', sim.aids.abs);
  $('tTcB').classList.toggle('on', sim.aids.tc);
  $('tEscB').classList.toggle('on', sim.aids.esc);
  syncTrackUI();
  buildTrack(trackDef);
  clearScenario();
  (data.vehicles || []).forEach(v => addAI(v));
  setMode(data.mode === 'v2v' ? 'v2v' : 'manual', true);
  resetRun();
  toast('SCENARIO LOADED');
}
 $('btnSave').onclick = saveScenario;
 $('btnLoad').onclick = loadScenario;

/* ---------- input ---------- */
addEventListener('keydown', e => {
  audio.init(); audio.resume();
  keys[e.code] = true;
  if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'].includes(e.code)) e.preventDefault();
  const tag = (e.target && e.target.tagName) || '';
  if (tag === 'SELECT' || tag === 'INPUT') return;
  if (e.repeat) return;
  switch (e.code) {
    case 'KeyE': ed.open ? closeEditor() : openEditor(); break;
    case 'KeyR': resetRun(); break;
    case 'KeyV': setMode(mode === 'manual' ? 'v2v' : 'manual'); break;
    case 'KeyC':
      camMode = camMode === 'cockpit' ? 'chase' : 'cockpit';
      toast(camMode === 'cockpit' ? 'COCKPIT CAMERA' : 'CHASE CAMERA');
      break;
    case 'KeyB':
      scenario.forEach(a => { if (a.dir === 'fwd') a.forceT = 3; });
      toast('HAZARD EVENT — FORWARD VEHICLES BRAKING');
      break;
    case 'KeyH':
      setHeadlights(!headlightsOn);
      toast(headlightsOn ? 'HEADLIGHTS ON' : 'HEADLIGHTS OFF');
      break;
    case 'KeyM':
      audio.init(); audio.resume(); audio.setMuted(!audio.muted);
      $('tSndB').classList.toggle('on', !audio.muted);
      toast(audio.muted ? 'SOUND OFF' : 'SOUND ON');
      break;
  }
});
addEventListener('keyup', e => { keys[e.code] = false; });
addEventListener('blur', () => { for (const k in keys) keys[k] = false; });
addEventListener('resize', () => {
  fpsCam.aspect = innerWidth / innerHeight;
  fpsCam.updateProjectionMatrix();
  chaseCam.aspect = innerWidth / innerHeight;
  chaseCam.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

/* ---------- fixed-step simulation + render loop ---------- */
const clock = new THREE.Clock();
const FIXED = 1 / 120;
let acc = 0;
function update(dt, et) {
  if (ed.open) return;
  if (!crashed) {
    driveInputs(dt);
    acc = Math.min(acc + dt, FIXED * 5);
    while (acc >= FIXED) {
      sim.step(FIXED);
      acc -= FIXED;
    }
    carCollisions();
    if (graceT > 0) graceT -= dt;
    updateAI(dt);
  }
  applyEgoVisual(dt);
  playerFX(dt);
  updateLampLights(dt, sim.s);
  moonLight.target.position.copy(carB.position);
  moonLight.position.copy(carB.position).addScaledVector(moonDir, 300);
  minimapCam.position.x = carB.position.x;
  minimapCam.position.z = carB.position.z;
  hudT -= dt;
  if (hudT <= 0) { hudT = 0.15; updateHUD(); }
  updateAlertDOM();
}
renderer.shadowMap.autoUpdate = false;
function renderFrame() {
  requestAnimationFrame(renderFrame);
  const dt = Math.min(clock.getDelta(), 0.05);
  const et = clock.elapsedTime;
  update(dt, et);
  updateCamera(dt, et);
  renderer.shadowMap.needsUpdate = true;
  renderer.setScissorTest(false);                  // scissor OFF for the full clear
  renderer.setViewport(0, 0, innerWidth, innerHeight);
  renderer.setScissor(0, 0, innerWidth, innerHeight);
  renderer.clear();
  renderer.setScissorTest(true);
  renderer.render(scene, camMode === 'cockpit' ? fpsCam : chaseCam);
  const mapX = innerWidth - MAP_PX - MAP_MARG;
  renderer.setViewport(mapX, MAP_MARG, MAP_PX, MAP_PX);
  renderer.setScissor(mapX, MAP_MARG, MAP_PX, MAP_PX);
  renderer.clearDepth();
  renderer.render(scene, minimapCam);
}

/* ---------- boot ---------- */
initEditor({
  rebuild, toast,
  getSpawnU: () => player.spawnU,
  setSpawnU: u => { player.spawnU = u; respawnAll(); },
  scenario
});
buildTrack(trackDef);
defaultScenario();
setMode('manual', true);
resetRun();
renderVehicles();
syncTrackUI();

window.v2vScene = {
  THREE, scene, renderer,
  camera: fpsCam, minimapCamera: minimapCam, carB,
  get carA() { return scenario[0] ? scenario[0].root : null; },
  vehicles: scenario, sim, player, get track() { return T; },
  setBrakeLights: on => scenario.forEach(a => { if (a.dir === 'fwd') a.forceT = on ? 3 : 0; }),
  setFogDensity, setHeadlights, setMode, resetRun, buildTrack: rebuild
};

setTimeout(() => toast('DRIVE WITH W/A/S/D · TOGGLE ABS / TC / ESC IN THE CONSOLE'), 900);
setTimeout(() => toast('TEST: STOP AND HOLD W — TC OFF SMOKES AND FISHTAILS, TC ON LAUNCHES CLEAN'), 4200);
setTimeout(() => toast('TEST: 70 KM/H + HARD BRAKE — ABS OFF PLOWS AND LAYS RUBBER'), 7600);
setTimeout(() => toast('PRESS V — ACC / FCW / AEB / EVASIVE STEER ON THE BSM LINK'), 11000);
renderFrame();