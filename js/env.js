import * as THREE from 'three';

/* renderer */
export const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.15;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.autoClear = false;
renderer.setClearColor(0x04060c, 1);
document.body.appendChild(renderer.domElement);

export const scene = new THREE.Scene();
export const fog = new THREE.FogExp2(0x0a0f1d, 0.02);
scene.fog = fog;

/* ---------- canvas-baked textures ---------- */
export function makeTexture(w, h, draw) {
  const c = document.createElement('canvas'); c.width = w; c.height = h;
  draw(c.getContext('2d'), w, h);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.anisotropy = renderer.capabilities.getMaxAnisotropy();
  return t;
}
export function speckle(ctx, w, h, n, alpha, spread) {
  for (let i = 0; i < n; i++) {
    const g = (Math.random() * spread) | 0;
    ctx.fillStyle = `rgba(${g + 18},${g + 20},${g + 26},${alpha})`;
    ctx.fillRect(Math.random() * w, Math.random() * h, 1 + Math.random() * 2, 1 + Math.random() * 2);
  }
}
const bermTex = makeTexture(256, 256, (ctx, w, h) => {
  ctx.fillStyle = '#141b10'; ctx.fillRect(0, 0, w, h);
  const cols = ['#1c2715', '#232e18', '#282213', '#101608', '#2a2414'];
  for (let i = 0; i < 260; i++) {
    ctx.fillStyle = cols[(Math.random() * cols.length) | 0];
    ctx.globalAlpha = 0.25 + Math.random() * 0.3;
    ctx.beginPath();
    ctx.ellipse(Math.random() * w, Math.random() * h, 4 + Math.random() * 22, 3 + Math.random() * 14, Math.random() * 3.14, 0, 6.3);
    ctx.fill();
  }
  ctx.globalAlpha = 1; speckle(ctx, w, h, 2000, 0.3, 26);
});
export const concTex = makeTexture(256, 256, (ctx, w, h) => {
  ctx.fillStyle = '#84888f'; ctx.fillRect(0, 0, w, h);
  speckle(ctx, w, h, 2600, 0.5, 42);
  ctx.strokeStyle = 'rgba(40,42,46,0.7)'; ctx.lineWidth = 3;
  ctx.strokeRect(1.5, 1.5, w - 3, h - 3);
  ctx.beginPath(); ctx.moveTo(w / 2, 0); ctx.lineTo(w / 2, h); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();
  ctx.strokeStyle = 'rgba(30,32,30,0.16)'; ctx.lineWidth = 9;
  for (let i = 0; i < 6; i++) {
    const x = Math.random() * w;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x + (Math.random() - .5) * 30, h); ctx.stroke();
  }
});
export function cityTex(warmBias) {
  return makeTexture(128, 256, (ctx, w, h) => {
    ctx.fillStyle = '#000'; ctx.fillRect(0, 0, w, h);
    const cols = 7, rows = 18, cw = w / cols, ch = h / rows;
    for (let x = 0; x < cols; x++) for (let y = 0; y < rows; y++) {
      if (Math.random() > 0.34) continue;
      const a = 0.35 + Math.random() * 0.65;
      ctx.fillStyle = Math.random() < warmBias
        ? `rgba(255,${205 + (Math.random() * 40 | 0)},${150 + (Math.random() * 70 | 0)},${a})`
        : `rgba(${160 + (Math.random() * 50 | 0)},${205 + (Math.random() * 30 | 0)},255,${a})`;
      ctx.fillRect(x * cw + 2.5, y * ch + 3, cw - 6, ch - 7);
    }
  });
}
export const cityTexs = [cityTex(0.75), cityTex(0.45), cityTex(0.9)];
export const glareTex = makeTexture(128, 128, (ctx) => {
  const g = ctx.createRadialGradient(64, 64, 0, 64, 64, 62);
  g.addColorStop(0, 'rgba(255,255,255,1)'); g.addColorStop(0.18, 'rgba(255,244,220,0.9)');
  g.addColorStop(0.45, 'rgba(255,220,160,0.28)'); g.addColorStop(1, 'rgba(255,200,120,0)');
  ctx.fillStyle = g; ctx.fillRect(0, 0, 128, 128);
});
export const poolTex = makeTexture(128, 128, (ctx) => {
  const g = ctx.createRadialGradient(64, 64, 4, 64, 64, 62);
  g.addColorStop(0, 'rgba(255,214,150,0.85)'); g.addColorStop(0.5, 'rgba(255,190,120,0.25)');
  g.addColorStop(1, 'rgba(255,180,110,0)');
  ctx.fillStyle = g; ctx.fillRect(0, 0, 128, 128);
});
export const smokeTex = makeTexture(64, 64, (ctx) => {
  const g = ctx.createRadialGradient(32, 32, 2, 32, 32, 30);
  g.addColorStop(0, 'rgba(255,255,255,0.9)'); g.addColorStop(0.5, 'rgba(255,255,255,0.35)');
  g.addColorStop(1, 'rgba(255,255,255,0)');
  ctx.fillStyle = g; ctx.fillRect(0, 0, 64, 64);
});
export const starTex = makeTexture(64, 64, (ctx) => {
  const g = ctx.createRadialGradient(32, 32, 0, 32, 32, 30);
  g.addColorStop(0, 'rgba(255,255,255,1)'); g.addColorStop(0.25, 'rgba(220,235,255,0.5)');
  g.addColorStop(1, 'rgba(200,220,255,0)');
  ctx.fillStyle = g; ctx.fillRect(0, 0, 64, 64);
});
export const moonTex = makeTexture(256, 256, (ctx) => {
  let g = ctx.createRadialGradient(128, 128, 20, 128, 128, 126);
  g.addColorStop(0, 'rgba(185,200,255,0.5)'); g.addColorStop(0.4, 'rgba(140,160,230,0.12)');
  g.addColorStop(1, 'rgba(120,140,220,0)');
  ctx.fillStyle = g; ctx.fillRect(0, 0, 256, 256);
  g = ctx.createRadialGradient(118, 118, 10, 128, 128, 56);
  g.addColorStop(0, '#fdfeff'); g.addColorStop(0.8, '#e8ecf8'); g.addColorStop(1, '#b9c2dd');
  ctx.beginPath(); ctx.arc(128, 128, 56, 0, 7); ctx.fillStyle = g; ctx.fill();
  ctx.globalAlpha = 0.12; ctx.fillStyle = '#8f9ab8';
  for (let i = 0; i < 9; i++) {
    ctx.beginPath();
    ctx.arc(128 + (Math.random() - .5) * 76, 128 + (Math.random() - .5) * 76, 4 + Math.random() * 11, 0, 7);
    ctx.fill();
  }
});
export const chevTex = makeTexture(128, 80, (ctx, w, h) => {
  ctx.fillStyle = '#0c0c0e'; ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = 'rgba(255,182,60,0.95)'; ctx.lineWidth = 13; ctx.lineCap = 'round';
  for (let i = 0; i < 3; i++) {
    const x = 24 + i * 34;
    ctx.beginPath(); ctx.moveTo(x - 12, 14); ctx.lineTo(x + 10, h / 2); ctx.lineTo(x - 12, h - 14); ctx.stroke();
  }
  ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(255,182,60,0.5)'; ctx.strokeRect(3, 3, w - 6, h - 6);
});
chevTex.wrapS = chevTex.wrapT = THREE.ClampToEdgeWrapping;
export const signTex = makeTexture(512, 192, (ctx, w, h) => {
  ctx.fillStyle = '#0b3a24'; ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = '#e8eeea'; ctx.lineWidth = 5; ctx.strokeRect(10, 10, w - 20, h - 20);
  ctx.fillStyle = '#f2f5ef'; ctx.textAlign = 'center';
  ctx.font = 'bold 52px Arial, sans-serif'; ctx.fillText('SHARP CURVE', w / 2, 78);
  ctx.fillStyle = '#d9a013'; ctx.fillRect(w / 2 - 70, 104, 140, 62);
  ctx.fillStyle = '#101210'; ctx.font = 'bold 46px Arial, sans-serif'; ctx.fillText('40', w / 2, 152);
});
signTex.wrapS = signTex.wrapT = THREE.ClampToEdgeWrapping;
export const gaugeTex = makeTexture(128, 64, (ctx) => {
  ctx.fillStyle = '#020407'; ctx.fillRect(0, 0, 128, 64);
  ctx.strokeStyle = 'rgba(110,255,205,0.7)'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(36, 56, 24, Math.PI * 1.05, Math.PI * 1.75); ctx.stroke();
  ctx.beginPath(); ctx.arc(92, 56, 24, Math.PI * 1.05, Math.PI * 1.75); ctx.stroke();
  ctx.strokeStyle = 'rgba(255,150,90,0.9)'; ctx.lineWidth = 1.6;
  ctx.beginPath(); ctx.moveTo(36, 56); ctx.lineTo(52, 34); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(92, 56); ctx.lineTo(102, 40); ctx.stroke();
  ctx.fillStyle = 'rgba(110,255,205,0.5)'; ctx.fillRect(58, 8, 3, 3); ctx.fillRect(66, 8, 3, 3);
});
gaugeTex.wrapS = gaugeTex.wrapT = THREE.ClampToEdgeWrapping;
export { bermTex };

/* ---------- sky, stars, moon, base lights ---------- */
export const skyGroup = new THREE.Group();
scene.add(skyGroup);
{
  const skyMat = new THREE.ShaderMaterial({
    side: THREE.BackSide, depthWrite: false, fog: false,
    uniforms: {
      top: { value: new THREE.Color(0x060913) },
      horizon: { value: new THREE.Color(0x10192e) },
      dusk: { value: new THREE.Color(0x5a2c14) },
      duskDir: { value: new THREE.Vector2(-0.55, -0.75).normalize() }
    },
    vertexShader: `varying vec3 vDir;
      void main() { vDir = normalize(position);
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
    fragmentShader: `varying vec3 vDir;
      uniform vec3 top; uniform vec3 horizon; uniform vec3 dusk; uniform vec2 duskDir;
      void main() {
        float h = clamp(vDir.y, 0.0, 1.0);
        vec3 col = mix(horizon, top, pow(h, 0.45));
        float az = dot(normalize(vDir.xz), duskDir);
        float duskAmt = pow(clamp(az, 0.0, 1.0), 4.0) * (1.0 - pow(clamp(vDir.y * 1.6, 0.0, 1.0), 0.5));
        col = mix(col, dusk, duskAmt * 0.6);
        gl_FragColor = vec4(col, 1.0);
        #include <tonemapping_fragment>
        #include <colorspace_fragment>
      }`
  });
  const sky = new THREE.Mesh(new THREE.SphereGeometry(820, 32, 18), skyMat);
  sky.frustumCulled = false; sky.renderOrder = -10;
  skyGroup.add(sky);
  const pts = [], cols = [];
  for (let i = 0; i < 900; i++) {
    const v = new THREE.Vector3(Math.random() * 2 - 1, Math.random(), Math.random() * 2 - 1);
    if (v.lengthSq() < 0.01 || v.y < 0.06) continue;
    v.normalize().multiplyScalar(760);
    pts.push(v.x, v.y, v.z);
    const c = 0.5 + Math.random() * 0.5;
    cols.push(c, c, Math.min(1, c * 1.05));
  }
  const sg = new THREE.BufferGeometry();
  sg.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  sg.setAttribute('color', new THREE.Float32BufferAttribute(cols, 3));
  skyGroup.add(new THREE.Points(sg, new THREE.PointsMaterial({
    map: starTex, vertexColors: true, size: 2.4, sizeAttenuation: false,
    transparent: true, opacity: 0.85, depthWrite: false, fog: false, color: 0xdfe8ff })));
  const moon = new THREE.Sprite(new THREE.SpriteMaterial({ map: moonTex, fog: false, transparent: true, depthWrite: false }));
  moon.scale.set(70, 70, 1);
  moon.position.set(150, 285, -516);
  skyGroup.add(moon);
}
scene.add(new THREE.AmbientLight(0x1a2238, 0.6));
scene.add(new THREE.HemisphereLight(0x131c32, 0x04060a, 0.3));
export const moonDir = new THREE.Vector3(150, 285, -516).normalize();
export const moonLight = new THREE.DirectionalLight(0x7f9cf5, 0.8);
moonLight.castShadow = true;
moonLight.shadow.mapSize.set(2048, 2048);
Object.assign(moonLight.shadow.camera, { left: -95, right: 95, top: 95, bottom: -95, near: 20, far: 520 });
moonLight.shadow.camera.updateProjectionMatrix();
moonLight.shadow.bias = -0.0004;
moonLight.shadow.normalBias = 0.6;
scene.add(moonLight, moonLight.target);