/**
 * trajectory.js
 * Generic spline trajectory models for position and feedforward editing.
 */

window.SplineTrajectoryModel = class SplineTrajectoryModel {
  static SAMPLE_RATE = 500;
  static MAX_KEYFRAMES_PER_JOINT = 120;

  constructor(durationSec = 0.4, config = {}) {
    this.duration = Math.max(0.05, Number(durationSec) || 0.4);
    this.jointNames = [...(config.jointNames ?? [])];
    this.jointColors = [...(config.jointColors ?? [])];
    this.jointRanges = [...(config.jointRanges ?? [])];
    this.uploadOrder = [...(config.uploadOrder ?? this.jointNames.map((_, i) => i))];
    this.unitLabel = config.unitLabel ?? '';
    this.version = config.version ?? 1;
    this.jointCount = this.jointNames.length;

    this.keyframes = this.jointNames.map(() => [
      { t: 0, q: 0 },
      { t: this.duration, q: 0 },
    ]);
    this._samples = new Array(this.jointCount).fill(null);
    this._rebuildAll();
  }

  _timeEps() {
    return 1 / SplineTrajectoryModel.SAMPLE_RATE;
  }

  _clampQ(j, q) {
    const range = this.jointRanges[j];
    if (!Array.isArray(range) || range.length !== 2) return q;
    return Math.max(range[0], Math.min(range[1], q));
  }

  _normalizeJointKeyframes(j) {
    const eps = this._timeEps();
    const dur = this.duration;
    const src = Array.isArray(this.keyframes[j]) ? this.keyframes[j] : [];

    const cleaned = src
      .map(kf => ({
        t: Number.isFinite(kf?.t) ? kf.t : 0,
        q: this._clampQ(j, Number.isFinite(kf?.q) ? kf.q : 0),
      }))
      .sort((a, b) => a.t - b.t);

    const firstQ = cleaned.length > 0 ? cleaned[0].q : 0;
    const endQ = cleaned.length > 0 ? cleaned[cleaned.length - 1].q : firstQ;
    const interior = [];
    let prev = 0;

    for (const kf of cleaned) {
      if (kf.t <= 0 || kf.t >= dur) continue;
      let t = Math.max(eps, Math.min(dur - eps, kf.t));
      if (t <= prev + eps * 0.25) t = prev + eps;
      if (t >= dur - eps * 0.5) break;
      interior.push({ t, q: kf.q });
      prev = t;
    }

    this.keyframes[j] = [
      { t: 0, q: firstQ },
      ...interior,
      { t: dur, q: endQ },
    ];
  }

  _rebuildAll() {
    for (let j = 0; j < this.jointCount; j++) this._rebuildJoint(j);
  }

  _rebuildJoint(j) {
    this._normalizeJointKeyframes(j);
    const kfs = this.keyframes[j];
    const nSamples = Math.round(this.duration * SplineTrajectoryModel.SAMPLE_RATE) + 1;
    const out = new Float32Array(nSamples);
    this._samples[j] = out;

    if (kfs.length === 0) return;
    if (kfs.length === 1) {
      out.fill(kfs[0].q);
      return;
    }

    const n = kfs.length;
    const a = kfs.map(k => k.q);
    const h = [];
    for (let i = 0; i < n - 1; i++) h.push(kfs[i + 1].t - kfs[i].t);

    const alpha = new Array(n - 1).fill(0);
    for (let i = 1; i < n - 1; i++) {
      alpha[i] = (3 / h[i]) * (a[i + 1] - a[i]) - (3 / h[i - 1]) * (a[i] - a[i - 1]);
    }

    const l = new Array(n).fill(1);
    const mu = new Array(n).fill(0);
    const z = new Array(n).fill(0);
    for (let i = 1; i < n - 1; i++) {
      l[i] = 2 * (kfs[i + 1].t - kfs[i - 1].t) - h[i - 1] * mu[i - 1];
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    const c = new Array(n).fill(0);
    const b = new Array(n - 1).fill(0);
    const d = new Array(n - 1).fill(0);
    for (let i = n - 2; i >= 0; i--) {
      c[i] = z[i] - mu[i] * c[i + 1];
      b[i] = (a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2 * c[i]) / 3;
      d[i] = (c[i + 1] - c[i]) / (3 * h[i]);
    }

    const t0 = kfs[0].t;
    const tn = kfs[n - 1].t;
    let seg = 0;
    for (let i = 0; i < nSamples; i++) {
      const t = i / SplineTrajectoryModel.SAMPLE_RATE;
      if (t <= t0) { out[i] = a[0]; continue; }
      if (t >= tn) { out[i] = a[n - 1]; continue; }
      while (seg < n - 2 && t > kfs[seg + 1].t) seg++;
      const dx = t - kfs[seg].t;
      out[i] = a[seg] + b[seg] * dx + c[seg] * dx * dx + d[seg] * dx * dx * dx;
    }
  }

  getValueAt(j, t) {
    const s = this._samples[j];
    if (!s || s.length === 0) return 0;
    const idx = Math.max(0, Math.min(s.length - 1, Math.round(t * SplineTrajectoryModel.SAMPLE_RATE)));
    return s[idx];
  }

  getPoseAt(t) {
    return Array.from({ length: this.jointCount }, (_, j) => this.getValueAt(j, t));
  }

  getCurvePoints(j, nPoints = 300) {
    const dt = this.duration / Math.max(1, nPoints - 1);
    return Array.from({ length: nPoints }, (_, i) => ({
      x: +(i * dt).toFixed(4),
      y: this.getValueAt(j, i * dt),
    }));
  }

  getKeyframePoints(j) {
    return this.keyframes[j].map((kf, idx) => ({
      x: kf.t,
      y: kf.q,
      idx,
    }));
  }

  setKeyframe(j, idx, t, q) {
    const kfs = this.keyframes[j];
    if (!kfs || idx < 0 || idx >= kfs.length) return idx;
    const last = kfs.length - 1;
    const eps = this._timeEps();
    const qq = this._clampQ(j, q);
    if (idx === last) {
      kfs[last] = { t: this.duration, q: qq };
      this._rebuildJoint(j);
      return idx;
    }

    if (idx === 0) {
      kfs[0] = { t: 0, q: qq };
      this._rebuildJoint(j);
      return 0;
    }

    const prev = kfs[idx - 1].t + eps;
    const next = kfs[idx + 1].t - eps;
    let tt = Math.max(0, Math.min(this.duration, t));
    if (prev > next) {
      tt = (kfs[idx - 1].t + kfs[idx + 1].t) * 0.5;
    } else {
      tt = Math.max(prev, Math.min(next, tt));
    }
    kfs[idx] = { t: tt, q: qq };
    this._rebuildJoint(j);
    return idx;
  }

  addKeyframe(j, t) {
    const eps = this._timeEps();
    t = Math.max(eps, Math.min(this.duration - eps, t));
    const q = this.getValueAt(j, t);
    const kfs = this.keyframes[j];
    if (kfs.length >= SplineTrajectoryModel.MAX_KEYFRAMES_PER_JOINT) return -1;

    for (let i = 1; i < kfs.length - 1; i++) {
      if (Math.abs(kfs[i].t - t) < eps * 0.5) return i;
    }

    let idx = kfs.length - 1;
    while (idx > 0 && kfs[idx - 1].t > t) idx--;
    const prevT = kfs[idx - 1]?.t ?? 0;
    const nextT = kfs[idx]?.t ?? this.duration;
    if (nextT - prevT <= eps * 1.2) return -1;
    t = Math.max(prevT + eps, Math.min(nextT - eps, t));

    kfs.splice(idx, 0, { t, q: this._clampQ(j, q) });
    this._rebuildJoint(j);
    return idx;
  }

  removeKeyframe(j, idx) {
    const kfs = this.keyframes[j];
    if (!kfs || kfs.length <= 2) return;
    const last = kfs.length - 1;
    if (idx <= 0 || idx >= last) return;
    kfs.splice(idx, 1);
    this._rebuildJoint(j);
  }

  setJointKeyframes(j, keyframes) {
    this.keyframes[j] = (keyframes ?? []).map(kf => ({
      t: Number.isFinite(kf?.t) ? kf.t : 0,
      q: Number.isFinite(kf?.q) ? kf.q : 0,
    }));
    this._rebuildJoint(j);
  }

  setDuration(newDur) {
    newDur = Math.max(0.05, Number(newDur) || this.duration);
    const oldDur = this.duration;
    const scale = newDur / oldDur;
    for (let j = 0; j < this.jointCount; j++) {
      const kfs = this.keyframes[j];
      const firstQ = this._clampQ(j, kfs[0]?.q ?? 0);
      const lastQ = this._clampQ(j, kfs[kfs.length - 1]?.q ?? firstQ);
      const interior = kfs.slice(1, -1).map(kf => ({
        t: kf.t * scale,
        q: this._clampQ(j, kf.q),
      }));
      this.keyframes[j] = [
        { t: 0, q: firstQ },
        ...interior,
        { t: newDur, q: lastQ },
      ];
    }
    this.duration = newDur;
    this._rebuildAll();
  }

  canPadTime() {
    return this.keyframes.every(kfs => Array.isArray(kfs) && kfs.length < SplineTrajectoryModel.MAX_KEYFRAMES_PER_JOINT);
  }

  trimStart(removeSec) {
    const cut = Math.min(Math.max(0, Number(removeSec) || 0), this.duration - 0.05);
    if (!(cut > 0)) return false;
    const oldDuration = this.duration;
    const newDuration = oldDuration - cut;
    for (let j = 0; j < this.jointCount; j++) {
      const kfs = this.keyframes[j];
      const firstQ = this._clampQ(j, this.getValueAt(j, cut));
      const lastQ = this._clampQ(j, kfs[kfs.length - 1]?.q ?? firstQ);
      const interior = kfs
        .filter(kf => kf.t > cut && kf.t < oldDuration)
        .map(kf => ({
          t: kf.t - cut,
          q: this._clampQ(j, kf.q),
        }));
      this.keyframes[j] = [
        { t: 0, q: firstQ },
        ...interior,
        { t: newDuration, q: lastQ },
      ];
    }
    this.duration = newDuration;
    this._rebuildAll();
    return true;
  }

  trimEnd(removeSec) {
    const cut = Math.min(Math.max(0, Number(removeSec) || 0), this.duration - 0.05);
    if (!(cut > 0)) return false;
    const newDuration = this.duration - cut;
    for (let j = 0; j < this.jointCount; j++) {
      const kfs = this.keyframes[j];
      const firstQ = this._clampQ(j, kfs[0]?.q ?? 0);
      const lastQ = this._clampQ(j, this.getValueAt(j, newDuration));
      const interior = kfs
        .filter(kf => kf.t > 0 && kf.t < newDuration)
        .map(kf => ({
          t: kf.t,
          q: this._clampQ(j, kf.q),
        }));
      this.keyframes[j] = [
        { t: 0, q: firstQ },
        ...interior,
        { t: newDuration, q: lastQ },
      ];
    }
    this.duration = newDuration;
    this._rebuildAll();
    return true;
  }

  prependTime(extraSec) {
    const dt = Number(extraSec);
    if (!(dt > 0) || !this.canPadTime()) return false;
    const newDuration = this.duration + dt;
    for (let j = 0; j < this.jointCount; j++) {
      const kfs = this.keyframes[j];
      const firstQ = this._clampQ(j, kfs[0]?.q ?? 0);
      this.keyframes[j] = [
        { t: 0, q: firstQ },
        ...kfs.map(kf => ({
          t: kf.t + dt,
          q: this._clampQ(j, kf.q),
        })),
      ];
    }
    this.duration = newDuration;
    this._rebuildAll();
    return true;
  }

  appendTime(extraSec) {
    const dt = Number(extraSec);
    if (!(dt > 0) || !this.canPadTime()) return false;
    const oldDuration = this.duration;
    const newDuration = oldDuration + dt;
    for (let j = 0; j < this.jointCount; j++) {
      const kfs = this.keyframes[j];
      const lastQ = this._clampQ(j, kfs[kfs.length - 1]?.q ?? kfs[0]?.q ?? 0);
      this.keyframes[j] = [
        ...kfs.map(kf => ({
          t: kf.t,
          q: this._clampQ(j, kf.q),
        })),
        { t: newDuration, q: lastQ },
      ];
    }
    this.duration = newDuration;
    this._rebuildAll();
    return true;
  }

  scaleAll(factor) {
    for (let j = 0; j < this.jointCount; j++) {
      const kfs = this.keyframes[j];
      for (let i = 0; i < kfs.length; i++) {
        const q = this._clampQ(j, kfs[i].q * factor);
        kfs[i] = { t: kfs[i].t, q };
      }
      this._rebuildJoint(j);
    }
  }

  getFrames(order = this.uploadOrder) {
    const n = this._samples[0]?.length ?? 0;
    const out = [];
    for (let i = 0; i < n; i++) {
      out.push(order.map(j => this._samples[j][i]));
    }
    return out;
  }

  get numSamples() {
    return this._samples[0]?.length ?? 0;
  }

  rebuildAll() {
    this._rebuildAll();
  }

  toJSON() {
    return {
      version: this.version,
      duration: this.duration,
      keyframes: this.keyframes,
    };
  }

  loadFromJSON(obj) {
    if (!obj || !Array.isArray(obj.keyframes)) return;
    this.duration = Math.max(0.05, Number(obj.duration) || this.duration || 0.4);
    this.keyframes = this.jointNames.map((_, j) =>
      (obj.keyframes[j] ?? []).map(kf => ({ t: kf.t, q: kf.q }))
    );
    this._rebuildAll();
  }
};

window.TrajectoryModel = class TrajectoryModel extends window.SplineTrajectoryModel {
  static JOINT_NAMES = ['Shoulder', 'Spine', 'Hip', 'Arm L', 'Arm R'];
  static JOINT_COLORS = ['#2196F3', '#4CAF50', '#F44336', '#FF9800', '#9C27B0'];
  static JOINT_RANGES = [
    [-1.6, 1.3],
    [-130 * Math.PI / 180, 0],
    [-40 * Math.PI / 180, 90 * Math.PI / 180],
    [-Math.PI / 2, Math.PI / 2],
    [-Math.PI / 2, Math.PI / 2],
  ];

  constructor(durationSec = 0.4) {
    super(durationSec, {
      jointNames: window.TrajectoryModel.JOINT_NAMES,
      jointColors: window.TrajectoryModel.JOINT_COLORS,
      jointRanges: window.TrajectoryModel.JOINT_RANGES,
      uploadOrder: [0, 3, 4, 1, 2],
      unitLabel: 'rad',
      version: 2,
    });
  }

  static fromJSON(obj) {
    const m = new window.TrajectoryModel(obj?.duration ?? 0.4);
    m.loadFromJSON(obj);
    return m;
  }
};

window.FeedforwardTrajectoryModel = class FeedforwardTrajectoryModel extends window.SplineTrajectoryModel {
  static JOINT_NAMES = ['Shoulder', 'Spine', 'Hip'];
  static JOINT_COLORS = ['#2196F3', '#4CAF50', '#F44336'];
  static JOINT_RANGES = [
    [-12, 12],
    [-12, 12],
    [-12, 12],
  ];

  constructor(durationSec = 0.4) {
    super(durationSec, {
      jointNames: window.FeedforwardTrajectoryModel.JOINT_NAMES,
      jointColors: window.FeedforwardTrajectoryModel.JOINT_COLORS,
      jointRanges: window.FeedforwardTrajectoryModel.JOINT_RANGES,
      uploadOrder: [0, 1, 2],
      unitLabel: 'Volts',
      version: 1,
    });
  }

  static fromJSON(obj) {
    const m = new window.FeedforwardTrajectoryModel(obj?.duration ?? 0.4);
    m.loadFromJSON(obj);
    return m;
  }
};
