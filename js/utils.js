/**
 * js/utils.js — Seeded PRNG, constants, helpers
 * All modules import from here for shared utilities.
 */

/* CONSTANTS */
export const GRAV = 9.81;
export const CFG = { laneW: 3.7, shoulder: 1.1 };
export const PHYS_H = 1 / 120;
export const CAM_W = 80, CAM_H = 45;
export const MAP_PX = 240, MAP_MARG = 20;

/* Sensor ranges */
export const SENSOR_RANGES = {
  ultrasonic: 30,
  camera: 120,
  radar: 160,
  v2v: 250
};

/* Max perceived objects for RL observation */
export const MAX_PERCEIVED_OBJECTS = 12;

/* RL observation feature counts */
export const OBS_EGO_FEATURES = 9;
export const OBS_ROAD_FEATURES = 4;
export const OBS_OBJ_FEATURES = 7;
export const OBS_TOTAL = OBS_EGO_FEATURES + OBS_ROAD_FEATURES + MAX_PERCEIVED_OBJECTS * OBS_OBJ_FEATURES;
// 9 + 4 + 12*7 = 97

/* BSM staleness */
export const MAX_BSM_AGE = 2.0; // seconds

/* SEEDED PRNG — xoshiro128**
   Deterministic when seeded. Call createRNG(seed) for each episode. */
export function createRNG(seed) {
  // Simple seed expansion from a single integer
  let s0 = (seed >>> 0) | 1;
  let s1 = (seed * 2654435761 >>> 0) | 1;
  let s2 = (seed * 2246822519 >>> 0) | 1;
  let s3 = (seed * 3266489917 >>> 0) | 1;

  function rotl(x, k) {
    return ((x << k) | (x >>> (32 - k))) >>> 0;
  }

  function next() {
    const result = (rotl((s1 * 5) >>> 0, 7) * 9) >>> 0;
    const t = (s1 << 9) >>> 0;
    s2 ^= s0;
    s3 ^= s1;
    s1 ^= s2;
    s0 ^= s3;
    s2 ^= t;
    s3 = rotl(s3, 11);
    return result;
  }

  const rng = {
    /** Returns float in [0, 1) */
    random() {
      return (next() >>> 0) / 4294967296;
    },
    /** Returns integer in [min, max] inclusive */
    randInt(min, max) {
      return min + Math.floor(rng.random() * (max - min + 1));
    },
    /** Returns float in [min, max) */
    randFloat(min, max) {
      return min + rng.random() * (max - min);
    },
    /** Returns true with probability p */
    chance(p) {
      return rng.random() < p;
    },
    /** Pick random element from array */
    pick(arr) {
      return arr[Math.floor(rng.random() * arr.length)];
    },
    /** Gaussian via Box-Muller */
    gaussian(mean = 0, std = 1) {
      const u1 = rng.random();
      const u2 = rng.random();
      return mean + std * Math.sqrt(-2 * Math.log(u1 || 1e-10)) * Math.cos(2 * Math.PI * u2);
    }
  };
  return rng;
}

/* MATH HELPERS */
export function clamp(v, lo, hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

export function wrapAngle(a) {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

export function lerp(a, b, t) {
  return a + (b - a) * t;
}

export function $id(id) {
  return document.getElementById(id);
}

/* Normalize a value from [lo, hi] to [0, 1], clamped */
export function normalize01(v, lo, hi) {
  return clamp((v - lo) / (hi - lo || 1), 0, 1);
}

/* Normalize a value from [lo, hi] to [-1, 1], clamped */
export function normalizeSymmetric(v, lo, hi) {
  return clamp(2 * (v - lo) / (hi - lo || 1) - 1, -1, 1);
}
