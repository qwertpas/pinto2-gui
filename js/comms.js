/* Direct USB ESP-NOW bridge connection (Web Serial). */
window.BridgeProtocol = {
  crc(bytes) {
    let crc = 0xffffffff;
    for (const byte of bytes) {
      crc ^= byte;
      for (let i = 0; i < 8; i++) crc = (crc >>> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
    }
    return (crc ^ 0xffffffff) >>> 0;
  },
  encode(type, seq, payload = new Uint8Array()) {
    const raw = new Uint8Array(payload.length + 10), view = new DataView(raw.buffer);
    raw[0] = 1; raw[1] = type;
    view.setUint16(2, seq, true); view.setUint16(4, payload.length, true);
    raw.set(payload, 6); view.setUint32(raw.length - 4, this.crc(raw.subarray(0, -4)), true);
    const out = [0];
    let index = 0, code = 1;
    for (const byte of raw) {
      if (byte === 0) { out[index] = code; index = out.length; out.push(0); code = 1; }
      else {
        out.push(byte);
        if (++code === 255) { out[index] = code; index = out.length; out.push(0); code = 1; }
      }
    }
    out[index] = code; out.push(0);
    return Uint8Array.from(out);
  },
  decode(bytes) {
    const out = [];
    for (let i = 0; i < bytes.length;) {
      const code = bytes[i++];
      if (!code || i + code - 1 > bytes.length) throw new Error('Invalid USB frame');
      for (let j = 1; j < code; j++) out.push(bytes[i++]);
      if (code < 255 && i < bytes.length) out.push(0);
    }
    const raw = Uint8Array.from(out), view = new DataView(raw.buffer);
    if (raw.length < 10 || raw[0] !== 1 || view.getUint16(4, true) + 10 !== raw.length ||
        view.getUint32(raw.length - 4, true) !== this.crc(raw.subarray(0, -4))) throw new Error('Invalid USB frame CRC or length');
    return { type: raw[1], payload: raw.slice(6, -4) };
  },
};

window.Comms = class Comms {
  constructor() {
    this._callbacks = {};
    this._connected = false;
    this._seq = 1;
    this._req = 1;
    this._queue = [];
    this._busy = false;
    this._pending = null;
    this._pose = null;
    this._runSamples = null;
    this._runMeta = null;
    this._lastPongMs = 0;
    this._rttMs = null;
    this._lastRx = 0;
    this._generation = 0;
  }
  on(event, fn) { (this._callbacks[event] ??= []).push(fn); return this; }
  off(event, fn) { this._callbacks[event] = (this._callbacks[event] ?? []).filter(f => f !== fn); }
  _emit(event, data) {
    for (const fn of this._callbacks[event] ?? []) { try { fn(data); } catch (e) { console.error(e); } }
  }
  _error(error) { this._emit('reply', { text: `ERR ${error.message}` }); }
  get isConnected() { return this._connected; }
  get ageSinceLastPongMs() { return this._lastPongMs ? Date.now() - this._lastPongMs : Infinity; }
  get lastRttMs() { return this._rttMs; }

  async connect() {
    if (this._port || this._connecting) return;
    if (!navigator.serial) throw new Error('USB control requires desktop Chrome or Edge over HTTPS.');
    this._connecting = true;
    try {
      const port = await navigator.serial.requestPort();
      await port.open({ baudRate: 3000000, bufferSize: 65536 });
      this._port = port;
      await port.setSignals({ dataTerminalReady: true, requestToSend: true });
      this._writer = port.writable.getWriter();
      this._reader = port.readable.getReader();
      this._readTask = this._read();
      const hello = await this._exchange(1, new Uint8Array(), 0x81);
      if (hello.length !== 36) throw new Error('Unsupported bridge firmware');
      if (!hello[2]) await this._exchange(2, new Uint8Array(), 0x82, 5000);
      if (hello[3] !== 0) {
        await this._exchange(3, Uint8Array.of(0, 0, 0, 0), 0x87);
        await new Promise(resolve => setTimeout(resolve, 150));
      }
      await this._telemetry(100);
      await this._ping();
      await this._status();
      this._connected = true;
      this._emit('connected');
      this._emit('robot_status', { connected: true, rtt_ms: this._rttMs });
      this._timer = setInterval(() => {
        if (!this._busy && !this._pose && this._queue.length === 0) {
          this._enqueue(async () => {
            try {
              await this._ping();
              if (++this._poll % 2 === 0) await this._status();
            } catch (e) {
              this._emit('robot_status', { connected: Date.now() - this._lastRx < 2500, rtt_ms: null });
              this._error(e);
            }
          });
        }
      }, 1000);
      this._poll = 0;
    } catch (e) {
      await this.disconnect();
      throw e;
    } finally { this._connecting = false; }
  }

  async disconnect(stop = false) {
    if (this._closing) return;
    if (stop && this._connected) {
      this.sendRaw('STOP');
      await this._work;
    }
    this._closing = true;
    clearInterval(this._timer);
    this._connected = false;
    this._generation++;
    this._queue = []; this._pose = null;
    this._pending?.reject(new Error('USB disconnected'));
    // Finish the current writer operation before releasing its lock.
    if (this._work) await this._work;
    if (this._reader) { await this._reader.cancel().catch(() => {}); await this._readTask; }
    this._writer?.releaseLock();
    if (this._port) await this._port.close().catch(e => this._error(e));
    this._reader = null; this._writer = null; this._port = null;
    this._runSamples = null; this._runMeta = null;
    this._lastPongMs = 0; this._rttMs = null;
    this._emit('disconnected');
    this._closing = false;
  }
  async _read() {
    let packet = [];
    try {
      while (true) {
        const { value, done } = await this._reader.read();
        if (done) break;
        for (const byte of value) {
          if (byte) { packet.push(byte); if (packet.length > 8192) packet = []; }
          else if (packet.length) {
            try { this._receive(BridgeProtocol.decode(packet)); }
            catch (e) { this._error(e); }
            packet = [];
          }
        }
      }
    } catch (e) { if (!this._closing) this._error(e); }
    finally {
      this._reader.releaseLock();
      this._pending?.reject(new Error('USB connection ended'));
      if (!this._closing) queueMicrotask(() => this.disconnect());
    }
  }
  _receive({ type, payload }) {
    if (type === 0x88) {
      const error = new Error(new TextDecoder().decode(payload.subarray(4)));
      if (this._pending) this._pending.reject(error); else this._error(error);
      return;
    }
    if (type === 0x84) { this._samples(payload); return; }
    const pending = this._pending;
    if (pending && type === pending.type && (pending.id == null ||
        (payload.length >= 20 && new DataView(payload.buffer).getUint32(0, true) === pending.id))) pending.resolve(payload);
  }
  _write(type, payload) { return this._writer.write(BridgeProtocol.encode(type, this._seq++, payload)); }
  _exchange(type, payload, replyType, timeout = 2000, id = null) {
    if (this._pending) return Promise.reject(new Error('USB request already active'));
    return new Promise((resolve, reject) => {
      let done = false;
      const finish = (fn, value) => {
        if (done) return;
        done = true;
        clearTimeout(timer); this._pending = null; fn(value);
      };
      const timer = setTimeout(() => finish(reject, new Error('Robot reply timed out')), timeout);
      this._pending = { type: replyType, id, resolve: v => finish(resolve, v), reject: e => finish(reject, e) };
      this._write(type, payload).catch(e => finish(reject, e));
    });
  }
  async _app(op, { slot = 0, value = 0, payload = new Uint8Array() } = {}, reply = true) {
    const id = this._req++, bytes = new Uint8Array(20 + payload.length), view = new DataView(bytes.buffer);
    view.setUint32(0, id, true); bytes[4] = op; bytes[5] = slot;
    view.setUint32(8, value, true); bytes.set(payload, 20);
    if (!reply) return this._write(12, bytes);
    const result = await this._exchange(12, bytes, 0x89, 1000, id);
    if (result[5]) throw new Error(`Robot command ${op} failed (${result[5]})`);
    this._lastRx = Date.now();
    return result;
  }
  _telemetry(hz) {
    const bytes = new Uint8Array(4), view = new DataView(bytes.buffer);
    view.setUint16(0, hz, true); view.setUint16(2, 364, true);
    return this._write(5, bytes);
  }
  async _ping() {
    const start = performance.now();
    const reply = await this._app(1);
    this._lastPongMs = Date.now(); this._rttMs = performance.now() - start;
    this._emit('pong', { rtt_ms: this._rttMs, robot_time_ms: new DataView(reply.buffer).getUint32(8, true) });
    this._emit('latency', this._rttMs);
    if (this._connected) this._emit('robot_status', { connected: true, rtt_ms: this._rttMs });
    return new DataView(reply.buffer).getUint32(8, true);
  }
  async _status(print = false) {
    const result = await this._app(2), view = new DataView(result.buffer, result.byteOffset + 20);
    const floats = offset => Array.from({ length: 3 }, (_, i) => view.getFloat32(offset + 4 * i, true));
    this._emit('kp_update', { motor_kp: floats(16) });
    this._emit('kd_update', { motor_kd: floats(28) });
    if (print) {
      const text = `STATUS mode=${['OFF', 'HOLD', 'REPLAY'][view.getUint8(0)]} pos=${view.getUint8(1) ? 1 : 0} ` +
        `frame=${view.getFloat32(4, true).toFixed(1)} loop=${view.getUint8(2) ? 1 : 0} ` +
        `scale=${view.getFloat32(8, true).toFixed(3)} speed=${view.getFloat32(12, true).toFixed(3)} ` +
        `kp=${floats(16).map(v=>v.toFixed(2)).join('/')} kd=${floats(28).map(v=>v.toFixed(2)).join('/')} ` +
        `traj_frames=${view.getUint32(44, true)} active_slot=${view.getUint8(48)} ` +
        `cache_mask=0x${view.getUint8(49).toString(16).padStart(2,'0')} upload_mask=0x${view.getUint8(50).toString(16).padStart(2,'0')} ` +
        `slot_frames=${Array.from({length:8},(_,i)=>view.getUint32(52+i*4,true)).join(',')}`;
      this._emit('reply', { text });
    }
  }
  _enqueue(fn, first = false) {
    if (!this._connected) { this._error(new Error('Connect the USB bridge first')); return; }
    if (first) this._queue.unshift(fn); else this._queue.push(fn);
    this._drain();
  }
  _drain() {
    if (this._busy || this._closing) return;
    this._busy = true;
    this._work = (async () => {
      while (this._connected && !this._closing && (this._queue.length || this._pose)) {
        const command = this._queue.shift();
        try {
          if (command) await command();
          else { const pose = this._pose; this._pose = null; await this._app(14, { payload: pose }, false); }
        } catch (e) { this._error(e); }
      }
      this._busy = false;
    })();
  }
  sendPose(pose, enabled = 1) {
    if (!this._connected) return;
    const values = pose.slice(); values[3] += Math.PI / 2; values[4] += Math.PI / 2;
    this._pose = this._poseBytes(values, enabled);
    this._drain();
  }
  _poseBytes(values, enabled) {
    if (values.length !== 5 || !values.every(Number.isFinite)) throw new Error('Invalid pose');
    const bytes = new Uint8Array(24), view = new DataView(bytes.buffer);
    values.forEach((v, i) => view.setFloat32(i * 4, v, true)); view.setUint32(20, enabled ? 1 : 0, true);
    return bytes;
  }
  uploadTrajectory(slot, frames) {
    const generation = this._generation;
    this._enqueue(async () => {
      try {
        if (!Number.isInteger(slot) || slot < 0 || slot > 7 || frames.length < 1 || frames.length > 2000 ||
            frames.some(f => f.length !== 8 || !f.every(Number.isFinite))) throw new Error('Invalid trajectory (maximum 2000 frames)');
        const bytes = new Uint8Array(frames.length * 32), view = new DataView(bytes.buffer);
        frames.forEach((f, i) => f.forEach((v, j) => view.setFloat32(i * 32 + j * 4, v + (j === 1 || j === 2 ? Math.PI / 2 : 0), true)));
        await this._write(6, new Uint8Array());
        try {
          await this._app(15, { slot, value: frames.length });
          for (let i = 0; i < frames.length; i += 16) {
            if (generation !== this._generation) throw new Error('Upload cancelled');
            await this._app(16, { slot, value: i, payload: bytes.slice(i * 32, (i + 16) * 32) });
          }
          if (generation !== this._generation) throw new Error('Upload cancelled');
          await this._app(17, { slot });
        } finally { if (!this._closing) await this._telemetry(100); }
        this._emit('traj_ack', { ok: true, slot_index: slot, frames: frames.length });
      } catch (e) { this._emit('traj_ack', { ok: false, slot_index: slot, msg: e.message }); throw e; }
    });
  }
  sendRun(slot, meta = null, loop = false) {
    this._pose = null;
    this._enqueue(async () => {
      if (!Number.isInteger(slot) || slot < 0 || slot > 7) throw new Error('RUN slot must be 0..7');
      this._runMeta = meta;
      try {
        await this._telemetry(500);
        await this._app(loop ? 9 : 8, { slot });
        this._emit('reply', { text: `OK ${loop ? 'RUN_LOOP' : 'RUN'} ${slot}` });
      } catch (e) { this._runMeta = null; throw e; }
    });
  }
  sendRaw(text) {
    const parts = text.trim().toUpperCase().split(/\s+/), command = parts[0];
    const stop = command === 'STOP' || command === 'IDLE';
    if (stop) { this._pose = null; this._queue = []; this._generation++; }
    if (command === 'RUN' || command === 'RUN_LOOP') { this.sendRun(Number(parts[1] ?? 0), null, command === 'RUN_LOOP'); return; }
    this._enqueue(async () => {
      if (command === 'PING') { const time = await this._ping(); this._emit('reply', { text: `PONG ${time}` }); return; }
      if (command === 'STATUS') { await this._status(true); return; }
      if (command === 'P' && parts.length === 7) {
        await this._app(14, { payload: this._poseBytes(parts.slice(1, 6).map(Number), Number(parts[6])) }, false); return;
      }
      if (command === 'KP' || command === 'KD') {
        const joint = ['SHOULDER', 'SPINE', 'HIP'].indexOf(parts[1]), value = Number(parts[2]);
        const max = command === 'KP' ? 10 : 0.1;
        if (parts.length !== 3 || joint < 0 || !Number.isFinite(value) || value < 0 || value > max) throw new Error(`${command} requires SHOULDER/SPINE/HIP and 0..${max}`);
        await this._app(command === 'KP' ? 13 : 20, { slot: joint, value: Math.round(value * 100) });
        await this._status();
      } else if (stop) { await this._app(3); await this._telemetry(100); }
      else throw new Error(`Unsupported command: ${text}`);
      this._emit('reply', { text: `OK ${text}` });
    }, stop);
  }
  _samples(payload) {
    const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength), start = 20;
    if (payload.length < 24 || view.getUint16(start, true) !== 0x5354 || payload[22] !== 2 || payload.length !== 24 + payload[23] * 72) throw new Error('Unsupported telemetry format');
    this._lastRx = Date.now();
    const samples = [];
    for (let i = 0; i < payload[23]; i++) {
      const offset = 24 + i * 72, f = j => view.getFloat32(offset + j * 4, true);
      const rawRpy = [f(12), f(13), f(14)], [r, p, y] = rawRpy.map(v => v * Math.PI / 360);
      const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
      const imu = [cy*cp*cr+sy*sp*sr, cy*cp*sr-sy*sp*cr, cy*sp*cr+sy*cp*sr, sy*cp*cr-cy*sp*sr];
      const [qw,qx,qy,qz] = imu;
      const rpy = [Math.atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy)), Math.asin(Math.max(-1,Math.min(1,2*(qw*qy-qz*qx)))), Math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))].map(v=>v*180/Math.PI);
      const modeId = payload[offset + 68];
      const sample = { shoulder: f(0), spine: f(1), hip: f(2), arm_l: f(3), arm_r: f(4), bus: [f(5), f(6), f(7)],
        imu, imu_raw_rpy: rawRpy, imu_rpy: rpy, traj_t: f(15), time_ms: view.getUint32(offset + 64, true), mode: ['idle', 'live', 'hold', 'run'][modeId], mode_id: modeId };
      samples.push(sample);
      if (sample.mode === 'run' || (sample.mode === 'hold' && this._runSamples)) {
        this._runSamples ??= [];
        this._runSamples.push({ t_s: sample.time_ms / 1000, meas_shoulder: f(0), meas_spine: f(1), meas_hip: f(2), meas_arm_l: f(3), meas_arm_r: f(4),
          bus_v: sample.bus, imu_quat: imu, imu_rpy_deg: rpy, traj_t: sample.traj_t, mode: sample.mode });
      }
      if (sample.mode !== 'run' && this._runSamples) {
        this._emit('log_data', { samples: this._runSamples, count: this._runSamples.length, run_meta: this._runMeta });
        this._runSamples = null; this._runMeta = null;
      }
    }
    if (samples.length) this._emit('telemetry', { ...samples.at(-1), ts_ms: samples.at(-1).time_ms, robot_time_us: Number(view.getBigUint64(4, true)), samples });
  }
};
