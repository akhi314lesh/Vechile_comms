import * as THREE from 'three';
import { gaugeTex } from './env.js';

/* shared materials */
export const glassMat = new THREE.MeshStandardMaterial({ color: 0x0a1420, metalness: 0.9, roughness: 0.07, transparent: true, opacity: 0.55 });
export const tireMat = new THREE.MeshStandardMaterial({ color: 0x0b0b0d, roughness: 0.95 });
export const rimMat = new THREE.MeshStandardMaterial({ color: 0xb7bdc6, metalness: 0.9, roughness: 0.25 });
export const trimMat = new THREE.MeshStandardMaterial({ color: 0x12151b, roughness: 0.85 });
export const egoHeadMat = new THREE.MeshStandardMaterial({ color: 0x1c2126, emissive: 0xffeecf, emissiveIntensity: 5 });
export const aiHeadMat = new THREE.MeshStandardMaterial({ color: 0x1c2126, emissive: 0xffeecf, emissiveIntensity: 5 });
export const egoTailMat = new THREE.MeshStandardMaterial({ color: 0x2a0508, emissive: 0xff0d05, emissiveIntensity: 0.9 });
export const tailBase = new THREE.MeshStandardMaterial({ color: 0x2a0508, emissive: 0xff1600, emissiveIntensity: 1.1 });
export const onGlareMat = new THREE.SpriteMaterial({ map: null, color: 0xfff3d9, transparent: true, opacity: 0.9, blending: THREE.AdditiveBlending, depthWrite: false });
export const brakeGlowMat = new THREE.SpriteMaterial({ map: null, color: 0xff3018, transparent: true, opacity: 0.5, blending: THREE.AdditiveBlending, depthWrite: false });
export const beamMat = new THREE.MeshBasicMaterial({ color: 0xbcd0ff, transparent: true, opacity: 0.05, blending: THREE.AdditiveBlending, depthWrite: false, side: THREE.BackSide });
const tireGeo = new THREE.CylinderGeometry(0.32, 0.32, 0.24, 16).rotateZ(Math.PI / 2);
const rimGeo = new THREE.CylinderGeometry(0.19, 0.19, 0.26, 8).rotateZ(Math.PI / 2);
const headLensGeo = new THREE.BoxGeometry(0.3, 0.13, 0.08);
const tailLensGeo = new THREE.BoxGeometry(0.32, 0.14, 0.08);
const markerGeo = (() => {
  const tri = new THREE.Shape();
  tri.moveTo(0, -2.0); tri.lineTo(1.25, 1.4); tri.lineTo(-1.25, 1.4); tri.closePath();
  return new THREE.ShapeGeometry(tri).rotateX(Math.PI / 2);
})();

function sedanBody() {
  const s = new THREE.Shape();
  s.moveTo(-2.28, 0.72); s.lineTo(-2.36, 0.42); s.lineTo(-2.3, 0.22); s.lineTo(-1.88, 0.22);
  s.absarc(-1.46, 0.22, 0.42, Math.PI, 0, true);
  s.lineTo(1.04, 0.22);
  s.absarc(1.46, 0.22, 0.42, Math.PI, 0, true);
  s.lineTo(2.3, 0.22); s.lineTo(2.38, 0.5); s.lineTo(2.3, 0.96); s.lineTo(1.45, 1.0);
  s.lineTo(-0.62, 0.9); s.closePath();
  return s;
}
function sedanCabin() {
  const c = new THREE.Shape();
  c.moveTo(-0.62, 0.88); c.lineTo(-0.15, 1.4); c.lineTo(0.9, 1.44); c.lineTo(1.52, 1.0);
  c.closePath();
  return c;
}
function hatchBody() {
  const s = new THREE.Shape();
  s.moveTo(-2.2, 0.72); s.lineTo(-2.28, 0.42); s.lineTo(-2.22, 0.22); s.lineTo(-1.8, 0.22);
  s.absarc(-1.38, 0.22, 0.42, Math.PI, 0, true);
  s.lineTo(1.12, 0.22);
  s.absarc(1.54, 0.22, 0.42, Math.PI, 0, true);
  s.lineTo(2.26, 0.22); s.lineTo(2.34, 0.5); s.lineTo(2.3, 1.02); s.lineTo(1.35, 1.05);
  s.lineTo(-0.6, 0.9); s.closePath();
  return s;
}
function hatchCabin() {
  const c = new THREE.Shape();
  c.moveTo(-0.6, 0.88); c.lineTo(-0.1, 1.44); c.lineTo(1.45, 1.46); c.lineTo(2.1, 1.06);
  c.closePath();
  return c;
}
function extrudeProfile(shape, width, bevel) {
  const g = new THREE.ExtrudeGeometry(shape, {
    depth: width - 2 * bevel, bevelEnabled: true, bevelThickness: bevel,
    bevelSize: bevel, bevelSegments: 2, curveSegments: 12 });
  g.rotateY(-Math.PI / 2);
  g.computeBoundingBox();
  g.translate(-(g.boundingBox.min.x + g.boundingBox.max.x) / 2, 0, 0);
  return g;
}

/* structure: root (yaw, world) → body (roll/pitch suspension) + wheels (flat).
   Front wheels steer (group.rotation.y); tire+rim spin (rotation.x). */
export function buildCar({ body, cabin, paint, metalness = 0.85, roughness = 0.2, wheelZ = [-1.46, 1.46] }) {
  const root = new THREE.Group();
  const bodyG = new THREE.Group();
  root.add(bodyG);
  const bodyGeo = extrudeProfile(body, 1.84, 0.06);
  const cabinGeo = extrudeProfile(cabin, 1.54, 0.04);
  const paintMat = new THREE.MeshStandardMaterial({ color: paint, metalness, roughness, side: THREE.DoubleSide });
  const bm = new THREE.Mesh(bodyGeo, paintMat);
  bm.castShadow = bm.receiveShadow = true;
  const cm = new THREE.Mesh(cabinGeo, glassMat);
  cm.castShadow = true;
  bodyG.add(bm, cm);
  const wheels = [];
  for (const z of wheelZ) for (const sx of [-1, 1]) {
    const w = new THREE.Group();
    const spin = new THREE.Group();
    const tire = new THREE.Mesh(tireGeo, tireMat); tire.castShadow = true;
    spin.add(tire, new THREE.Mesh(rimGeo, rimMat));
    w.add(spin);
    w.position.set(sx * 0.8, 0.32, z);
    root.add(w);
    wheels.push({ group: w, spin, front: z < 0 });
  }
  return { root, body: bodyG, wheels, geos: [bodyGeo, cabinGeo], paintMat };
}
export function headingMarker(colorHex, scale = 1) {
  const m = new THREE.Mesh(markerGeo, new THREE.MeshBasicMaterial({
    color: new THREE.Color(colorHex).multiplyScalar(2.0), side: THREE.DoubleSide, fog: false }));
  m.layers.set(1);
  m.position.y = 3.4;
  m.scale.setScalar(scale);
  return m;
}

/* ego car with cockpit */
export function buildEgo() {
  const c = buildCar({ body: sedanBody(), cabin: sedanCabin(), paint: 0x151b24, metalness: 0.85, roughness: 0.2 });
  c.root.add(headingMarker(0x45e6ff));
  const headSpots = [], beams = [];
  for (const sx of [-1, 1]) {
    const lens = new THREE.Mesh(headLensGeo, egoHeadMat);
    lens.position.set(sx * 0.58, 0.6, -2.45);
    const spot = new THREE.SpotLight(0xf7ead2, 1200, 150, 0.38, 0.5, 2);
    spot.position.set(sx * 0.58, 0.62, -2.2);
    spot.target.position.set(sx * 0.95, 0, -45);
    spot.castShadow = true;
    spot.shadow.mapSize.set(1024, 1024);
    spot.shadow.camera.near = 0.6;
    spot.shadow.camera.far = 150;
    spot.shadow.bias = -0.001;
    spot.shadow.normalBias = 0.25;
    const beam = new THREE.Mesh(new THREE.ConeGeometry(2.6, 13, 20, 1, true).rotateX(Math.PI / 2), beamMat);
    beam.position.set(sx * 0.58, 0.66, -8.8);
    beam.rotation.set(-0.035, sx * 0.02, 0);
    beam.renderOrder = 3;
    c.body.add(lens, spot, spot.target, beam);
    headSpots.push(spot);
    beams.push(beam);
    const tl = new THREE.Mesh(tailLensGeo, egoTailMat);
    tl.position.set(sx * 0.6, 0.92, 2.42);
    c.body.add(tl);
  }
  const dash = new THREE.Mesh(new THREE.BoxGeometry(1.62, 0.28, 0.55), trimMat);
  dash.position.set(0, 0.83, -0.42); dash.rotation.x = 0.06;
  const cluster = new THREE.Mesh(new THREE.PlaneGeometry(0.34, 0.16),
    new THREE.MeshStandardMaterial({ color: 0x000000, emissive: 0xffffff, emissiveMap: gaugeTex, emissiveIntensity: 1.4, roughness: 0.4 }));
  cluster.position.set(-0.33, 0.97, -0.5); cluster.rotation.x = -0.45;
  const wheel = new THREE.Group();
  wheel.add(new THREE.Mesh(new THREE.TorusGeometry(0.165, 0.021, 10, 24), trimMat));
  wheel.add(new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 0.03, 10).rotateX(Math.PI / 2), trimMat));
  for (const a of [0, Math.PI]) {
    const spoke = new THREE.Mesh(new THREE.BoxGeometry(0.15, 0.02, 0.02), trimMat);
    spoke.position.set(Math.cos(a) * 0.08, 0, 0.012);
    wheel.add(spoke);
  }
  const spoke3 = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.14, 0.02), trimMat);
  spoke3.position.set(0, -0.08, 0.012);
  wheel.add(spoke3);
  wheel.position.set(-0.33, 0.9, -0.1); wheel.rotation.x = -0.45;
  const floor = new THREE.Mesh(new THREE.BoxGeometry(1.6, 0.06, 2.0), trimMat);
  floor.position.set(0, 0.38, 0.5);
  c.body.add(dash, cluster, wheel, floor);
  for (const sx of [-1, 1]) {
    const pil = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.72, 0.1), trimMat);
    pil.position.set(sx * 0.71, 1.12, -0.39);
    pil.rotation.set(0.72, 0, -sx * 0.06);
    c.body.add(pil);
  }
  const mirror = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.08, 0.035),
    new THREE.MeshStandardMaterial({ color: 0x0a0e14, metalness: 0.6, roughness: 0.15 }));
  mirror.position.set(0, 1.31, -0.5); mirror.rotation.x = 0.18;
  c.body.add(mirror);
  const cabinLight = new THREE.PointLight(0x36435e, 2.2, 2.8, 2);
  cabinLight.position.set(0, 1.15, 0.3);
  c.body.add(cabinLight);
  return { ...c, headSpots, beams };
}

/* AI car */
export function buildAI({ hatch, paint, onc, bigLights }) {
  const c = buildCar({
    body: hatch ? hatchBody() : sedanBody(),
    cabin: hatch ? hatchCabin() : sedanCabin(),
    paint, metalness: 0.55, roughness: 0.32,
    wheelZ: hatch ? [-1.38, 1.54] : [-1.46, 1.46]
  });
  c.root.add(headingMarker(onc ? 0xdfe8f0 : 0xffa23c, onc ? 0.8 : 1));
  const tailM = tailBase.clone();
  const glows = [], lights = [];
  for (const sx of [-1, 1]) {
    const head = new THREE.Mesh(headLensGeo, aiHeadMat);
    head.position.set(sx * 0.55, 0.6, -2.42);
    const tail = new THREE.Mesh(tailLensGeo, tailM);
    tail.position.set(sx * 0.6, 0.9, 2.42);
    const glare = new THREE.Sprite(onGlareMat);
    glare.position.set(sx * 0.55, 0.62, -2.5);
    glare.scale.set(1.9, 1.9, 1);
    const glow = new THREE.Sprite(brakeGlowMat);
    glow.position.set(sx * 0.6, 0.9, 2.45);
    glow.scale.setScalar(0.5);
    c.body.add(head, tail, glare, glow);
    glows.push(glow);
    if (bigLights) {
      const pl = new THREE.PointLight(0xff1a08, 3.4, 11, 2);
      pl.position.set(sx * 0.55, 0.8, 2.6);
      c.body.add(pl);
      lights.push(pl);
    }
  }
  return { ...c, tailM, glows, lights };
}
export function disposeAI(built) {
  built.geos.forEach(g => g.dispose());
  built.paintMat.dispose();
  built.tailM.dispose();
}