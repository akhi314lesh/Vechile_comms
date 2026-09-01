/* procedural audio: engine, skid, wind, impacts — WebAudio, no assets */
export class AudioFX {
  constructor() { this.ready = false; this.muted = false; }
  init() {
    if (this.ready) return;
    try {
      const ctx = new (window.AudioContext || window.webkitAudioContext)();
      this.ctx = ctx;
      this.master = ctx.createGain();
      this.master.gain.value = 0.5;
      this.master.connect(ctx.destination);
      // engine: detuned saw + square through a lowpass
      this.eOsc = ctx.createOscillator(); this.eOsc.type = 'sawtooth';
      this.eOsc2 = ctx.createOscillator(); this.eOsc2.type = 'square';
      this.eFilt = ctx.createBiquadFilter(); this.eFilt.type = 'lowpass'; this.eFilt.frequency.value = 700;
      this.eGain = ctx.createGain(); this.eGain.gain.value = 0;
      this.eOsc.connect(this.eFilt); this.eOsc2.connect(this.eFilt);
      this.eFilt.connect(this.eGain); this.eGain.connect(this.master);
      this.eOsc.start(); this.eOsc2.start();
      // noise source (skid + wind + impacts)
      const len = ctx.sampleRate;
      const buf = ctx.createBuffer(1, len, ctx.sampleRate);
      const d = buf.getChannelData(0);
      for (let i = 0; i < len; i++) d[i] = Math.random() * 2 - 1;
      this.noise = ctx.createBufferSource();
      this.noise.buffer = buf; this.noise.loop = true;
      this.skidF = ctx.createBiquadFilter(); this.skidF.type = 'bandpass'; this.skidF.frequency.value = 850; this.skidF.Q.value = 1.4;
      this.skidG = ctx.createGain(); this.skidG.gain.value = 0;
      this.windF = ctx.createBiquadFilter(); this.windF.type = 'lowpass'; this.windF.frequency.value = 380;
      this.windG = ctx.createGain(); this.windG.gain.value = 0;
      this.noise.connect(this.skidF); this.skidF.connect(this.skidG); this.skidG.connect(this.master);
      this.noise.connect(this.windF); this.windF.connect(this.windG); this.windG.connect(this.master);
      this.noise.start();
      this.ready = true;
    } catch { }
  }
  resume() { if (this.ctx && this.ctx.state === 'suspended') this.ctx.resume(); }
  setMuted(m) { this.muted = m; if (this.master) this.master.gain.value = m ? 0 : 0.5; }
  update(rpm, throttle, speed, skid) {
    if (!this.ready) return;
    const t = this.ctx.currentTime;
    const f = 38 + rpm * 0.021;
    this.eOsc.frequency.setTargetAtTime(f, t, 0.05);
    this.eOsc2.frequency.setTargetAtTime(f * 0.5, t, 0.05);
    this.eGain.gain.setTargetAtTime(0.015 + throttle * 0.05, t, 0.08);
    this.skidG.gain.setTargetAtTime(Math.min(0.4, skid * 0.4), t, 0.05);
    this.windG.gain.setTargetAtTime(Math.min(0.12, speed * speed * 0.00004), t, 0.1);
  }
  impact(strength) {
    if (!this.ready) return;
    const ctx = this.ctx, t = ctx.currentTime;
    const src = ctx.createBufferSource();
    const buf = ctx.createBuffer(1, ctx.sampleRate * 0.3, ctx.sampleRate);
    const d = buf.getChannelData(0);
    for (let i = 0; i < d.length; i++) d[i] = (Math.random() * 2 - 1) * Math.pow(1 - i / d.length, 2.5);
    src.buffer = buf;
    const f = ctx.createBiquadFilter(); f.type = 'lowpass'; f.frequency.value = 500;
    const g = ctx.createGain(); g.gain.value = Math.min(0.7, strength * 0.1);
    src.connect(f); f.connect(g); g.connect(this.master);
    src.start(t);
    const osc = ctx.createOscillator(); osc.type = 'sine';
    osc.frequency.setValueAtTime(90, t);
    osc.frequency.exponentialRampToValueAtTime(35, t + 0.25);
    const og = ctx.createGain();
    og.gain.setValueAtTime(Math.min(0.6, strength * 0.09), t);
    og.gain.exponentialRampToValueAtTime(0.001, t + 0.3);
    osc.connect(og); og.connect(this.master);
    osc.start(t); osc.stop(t + 0.32);
  }
}