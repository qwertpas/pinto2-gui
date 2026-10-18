/* The same CSV formats as server.py, stored and parsed in the browser. */
window.TrajectoryFiles = {
  normalize(q) { const n = Math.hypot(...q); return n > 1e-9 ? q.map(v => v / n) : [1, 0, 0, 0]; },
  multiply(a, b) {
    const [w,x,y,z] = a, [v,i,j,k] = b;
    return [w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v];
  },
  pitch(angle) { return [Math.cos(angle / 2), 0, Math.sin(angle / 2), 0]; },
  rows(text) {
    // CSV quoting, including embedded commas and newlines.
    const rows = []; let row = [], cell = '', quoted = false;
    for (let i = 0; i < text.length; i++) {
      const c = text[i];
      if (c === '"') { if (quoted && text[i + 1] === '"') { cell += '"'; i++; } else quoted = !quoted; }
      else if (!quoted && (c === ',' || c === '\n')) {
        row.push(cell.replace(/\r$/, '')); cell = '';
        if (c === '\n') { if (row.some(v => v.trim())) rows.push(row); row = []; }
      } else cell += c;
    }
    if (quoted) throw new Error('Unclosed CSV quote');
    row.push(cell.replace(/\r$/, '')); if (row.some(v => v.trim())) rows.push(row);
    return rows;
  },
  parse(text, count = 20) {
    const rows = this.rows(text.replace(/^\uFEFF/, ''));
    if (!rows.length) throw new Error('CSV is empty');
    const doc = { version: 4, duration: 0.4, positionKeyframes: Array.from({length:5}, () => []), feedforwardKeyframesVolts: Array.from({length:3}, () => []) };
    const refs = [];
    const number = v => v == null || v.trim() === '' ? null : (Number.isFinite(Number(v)) ? Number(v) : null);
    if (rows[0][0].trim().toLowerCase() === 'format') {
      const format = rows[0][1].trim().toLowerCase();
      if (!['keyframe_v1','keyframe_v2','keyframe_v3'].includes(format)) throw new Error('Unsupported keyframe format');
      doc.duration = number(rows[1]?.[1]) ?? 0.4;
      const header = rows[2].map(v => v.trim().toLowerCase());
      for (const values of rows.slice(3)) {
        const row = Object.fromEntries(header.map((k,i) => [k,values[i]]));
        const kind = format === 'keyframe_v1' ? 'position' : row.kind?.trim().toLowerCase();
        const t = number(row.t), q = number(row.q), joint = number(row.joint);
        if (t == null || q == null) continue;
        if (kind === 'reference_imu_quat') {
          const quat = [q, number(row.qx), number(row.qy), number(row.qz)];
          if (quat.every(v => v != null)) refs.push({u:t,quat});
        } else {
          const target = kind === 'position' ? doc.positionKeyframes : kind === 'feedforward' ? doc.feedforwardKeyframesVolts : [];
          if (Number.isInteger(joint) && joint >= 0 && joint < target.length) target[joint].push({t,q});
        }
      }
      if (!doc.positionKeyframes.some(k => k.length)) throw new Error('CSV has no valid position rows');
    } else {
      const header = rows[0].map(v => v.trim().toLowerCase()), has = keys => keys.every(k => header.includes(k));
      const raw = has(['theta2','theta3','theta4','phi5','tau2','tau3','tau4']);
      const mappings = [
        ['shoulder','spine','hip','arm_l','arm_r'],
        ['cmd_shoulder_to_spine','cmd_spine_to_hip','cmd_hip_to_ankles','cmd_shoulder_to_wrist_l','cmd_shoulder_to_wrist_r'],
        ['meas_shoulder','meas_spine','meas_hip','meas_arm_l','meas_arm_r'],
        ['tgt_shoulder','tgt_spine','tgt_hip','tgt_arm_l','tgt_arm_r'],
      ];
      const joints = mappings.find(has);
      if (!raw && !joints) throw new Error('CSV does not contain supported trajectory columns');
      const time = ['t_local_s','t_s','time_s','t','time_us','replay_frame'].find(k => header.includes(k));
      if (!time) throw new Error('CSV has no supported time column');
      const ffV = ['ff_shoulder_v','ff_spine_v','ff_hip_v'], ffTau = ['tau_ff_shoulder_to_spine','tau_ff_spine_to_hip','tau_ff_hip_to_ankles'];
      const quatKeys = ['ref_imu_quat','imu_quat','root_quat'].map(prefix => ['w','x','y','z'].map(k => `${prefix}_${k}`)).find(has);
      const cycle = ['cycle_idx','cycle'].find(k => header.includes(k));
      let firstCycle = null;
      const samples = [];
      for (const values of rows.slice(1)) {
        const row = Object.fromEntries(header.map((k,i) => [k, values[i]]));
        if (cycle && row[cycle]?.trim()) { firstCycle ??= row[cycle].trim(); if (row[cycle].trim() !== firstCycle) continue; }
        let t = number(row[time]); if (t == null) continue;
        if (time === 'time_us') t /= 1e6; else if (time === 'replay_frame') t /= 500;
        const get = keys => keys.map(k => number(row[k]));
        const pose = raw ? [number(row.theta2),number(row.theta3),number(row.theta4),0,0] : get(joints);
        if (pose.some(v => v == null)) continue;
        let ff, quat = null;
        if (raw) {
          pose[0] += Math.PI / 2; pose[2] += Math.PI / 2;
          ff = get(['tau2','tau3','tau4']);
          if (ff.some(v => v == null) || number(row.phi5) == null) continue;
          ff = ff.map(v => v / 0.0871);
          const pitch = 1.5 * Math.PI - Number(row.phi5) - Number(row.theta4) - Number(row.theta3);
          quat = this.pitch(Math.atan2(Math.sin(pitch),Math.cos(pitch)));
        } else {
          ff = has(ffV) ? get(ffV).map(v => v ?? 0) : has(ffTau) ? get(ffTau).map(v => (v ?? 0) / 0.0871) : [0,0,0];
          if (quatKeys) {
            quat = get(quatKeys);
            if (quat.some(v => v == null)) quat = null;
            else if (quatKeys[0] === 'root_quat_w') quat = this.multiply(quat,this.pitch(pose[0]));
          }
        }
        samples.push({t,pose,ff,quat});
      }
      samples.sort((a,b) => a.t-b.t);
      if (!samples.length) throw new Error('CSV has no valid sample rows');
      const t0 = samples[0].t;
      doc.duration = Math.max(0.05, samples.at(-1).t-t0);
      let index = 1;
      count = Math.max(2,count);
      for (let i=0;i<count;i++) {
        const t = doc.duration*i/(count-1), query = t+t0;
        while (index < samples.length-1 && samples[index].t < query) index++;
        const a = samples[Math.max(0,index-1)], b = samples[Math.min(index,samples.length-1)];
        const u = b.t > a.t ? Math.max(0,Math.min(1,(query-a.t)/(b.t-a.t))) : 0;
        for (const [target,key] of [[doc.positionKeyframes,'pose'],[doc.feedforwardKeyframesVolts,'ff']]) {
          target.forEach((frames,j) => frames.push({t,q:a[key][j]*(1-u)+b[key][j]*u}));
        }
      }
      for (const s of samples) if (s.quat) refs.push({u:(s.t-t0)/doc.duration,quat:s.quat});
    }
    if (refs.length) {
      refs.sort((a,b) => a.u-b.u);
      refs.forEach(s => {s.u=Math.max(0,Math.min(1,s.u));s.quat=this.normalize(s.quat);});
      refs[0].u=0; refs.at(-1).u=1;
      doc.referenceImu={samples:refs};
    }
    return doc;
  },
  serialize(doc) {
    const rows = ['format,keyframe_v3',`duration,${doc.duration.toFixed(9)}`,'kind,joint,t,q,qx,qy,qz'];
    for (const [kind,key] of [['position','positionKeyframes'],['feedforward','feedforwardKeyframesVolts']]) {
      doc[key].forEach((frames,j) => frames.forEach(({t,q}) => rows.push(`${kind},${j},${t.toFixed(9)},${q.toFixed(9)},,,`)));
    }
    for (const s of doc.referenceImu?.samples ?? []) rows.push(`reference_imu_quat,,${s.u.toFixed(9)},${this.normalize(s.quat).map(v=>v.toFixed(9)).join(',')}`);
    return rows.join('\n')+'\n';
  },
  stored() { return JSON.parse(localStorage.getItem('pinto-trajectories') ?? '{}'); },
  async list() {
    const res = await fetch('saved_traj/index.json');
    if (!res.ok) throw new Error(`Trajectory list: HTTP ${res.status}`);
    const bundled = await res.json(), saved = this.stored();
    const names = Object.keys(saved).sort((a,b)=>saved[b].mtime-saved[a].mtime);
    const files = [...new Set([...names,...bundled.files])];
    return {files,latest:files[0] ?? null};
  },
  async load(name) {
    const saved = this.stored()[name];
    if (saved) return this.parse(saved.csv);
    const res = await fetch(`saved_traj/${name.split('/').map(encodeURIComponent).join('/')}`);
    if (!res.ok) throw new Error(`Trajectory: HTTP ${res.status}`);
    return this.parse(await res.text());
  },
  save(name, doc) {
    name = name.split(/[\\/]/).at(-1).trim().replace(/[^A-Za-z0-9._-]+/g,'_') || `traj_${Date.now()}.csv`;
    if (!name.toLowerCase().endsWith('.csv')) name += '.csv';
    const csv = this.serialize(doc), saved = this.stored();
    saved[name] = {csv,mtime:Date.now()};
    localStorage.setItem('pinto-trajectories',JSON.stringify(saved));
    const url=URL.createObjectURL(new Blob([csv],{type:'text/csv'})), a=document.createElement('a');
    a.href=url; a.download=name; a.click(); setTimeout(()=>URL.revokeObjectURL(url),1000);
    return name;
  },
};
