/**
 * chart_editor.js
 * Generic Chart.js spline editor used for position and feedforward curves.
 */

window.ChartEditor = class ChartEditor {
  constructor(canvasId, traj, opts = {}) {
    this.traj = traj;
    this.opts = opts;
    this.playhead = 0;
    this.selectedJoint = 0;
    this.visible = new Array(this.traj.jointCount).fill(true);
    this._logData = null;
    this._dragging = null;
    this._hoverSnap = null;
    this._canvas = document.getElementById(canvasId);
    this._jointCount = this.traj.jointCount;
    this._measuredKeys = [...(opts.measuredKeys ?? [])];
    this._yAxisLabel = opts.yAxisLabel ?? this.traj.unitLabel ?? '';

    this._scrubberPlugin = {
      id: 'scrubber',
      afterDraw: (chart) => {
        const { ctx, chartArea, scales } = chart;
        if (!chartArea) return;
        const x = scales.x.getPixelForValue(this.playhead);
        ctx.save();
        ctx.strokeStyle = '#ffb300';
        ctx.lineWidth = 2;
        ctx.shadowColor = '#ffb300';
        ctx.shadowBlur = 6;
        ctx.setLineDash([]);
        ctx.beginPath();
        ctx.moveTo(x, chartArea.top);
        ctx.lineTo(x, chartArea.bottom);
        ctx.stroke();
        ctx.shadowBlur = 0;
        const label = this.playhead.toFixed(3) + 's';
        const pad = 4;
        const tw = ctx.measureText(label).width;
        const lx = Math.min(x + 4, scales.x.right - tw - pad * 2 - 2);
        const ly = chartArea.top + 2;
        ctx.fillStyle = 'rgba(255, 179, 0, 0.90)';
        ctx.beginPath();
        if (ctx.roundRect) ctx.roundRect(lx - pad, ly, tw + pad * 2, 16, 3);
        else ctx.rect(lx - pad, ly, tw + pad * 2, 16);
        ctx.fill();
        ctx.fillStyle = '#000';
        ctx.font = '10px monospace';
        ctx.textAlign = 'left';
        ctx.fillText(label, lx, ly + 11);
        if (this._hoverSnap && this.visible[this.selectedJoint]) {
          const point = this._dataToPixel(this._hoverSnap.t, this._hoverSnap.q);
          const valueLabel = `${this._hoverSnap.t.toFixed(3)}s, ${this._hoverSnap.q.toFixed(3)}`;
          ctx.font = '10px monospace';
          const boxPadX = 5;
          const boxPadY = 4;
          const boxW = ctx.measureText(valueLabel).width + boxPadX * 2;
          const boxH = 18;
          const rawBoxX = point.x + 10;
          const rawBoxY = point.y + 10;
          const boxX = Math.max(chartArea.left, Math.min(rawBoxX, chartArea.right - boxW));
          const boxY = Math.max(chartArea.top, Math.min(rawBoxY, chartArea.bottom - boxH));
          ctx.fillStyle = 'rgba(13, 17, 23, 0.92)';
          ctx.strokeStyle = this.traj.jointColors[this.selectedJoint];
          ctx.lineWidth = 1;
          ctx.beginPath();
          if (ctx.roundRect) ctx.roundRect(boxX, boxY, boxW, boxH, 4);
          else ctx.rect(boxX, boxY, boxW, boxH);
          ctx.fill();
          ctx.stroke();
          ctx.fillStyle = '#e6edf3';
          ctx.fillText(valueLabel, boxX + boxPadX, boxY + 12);
        }
        ctx.restore();
      },
    };

    this._chart = this._buildChart();
    this._attachEvents(this._canvas);
  }

  _lineDatasetIndex(j) { return j; }
  _keyframeDatasetIndex(j) { return this._jointCount + j; }
  _measuredDatasetIndex(j) { return this._jointCount * 2 + j; }

  _buildChart() {
    const colors = this.traj.jointColors;
    const names = this.traj.jointNames;
    const lineDatasets = Array.from({ length: this._jointCount }, (_, j) => ({
      type: 'line',
      label: names[j],
      data: this.traj.getCurvePoints(j),
      borderColor: colors[j],
      backgroundColor: 'transparent',
      borderWidth: j === this.selectedJoint ? 2.5 : 1.5,
      pointRadius: 0,
      tension: 0,
      parsing: false,
      hidden: !this.visible[j],
    }));
    const kfDatasets = Array.from({ length: this._jointCount }, (_, j) => ({
      type: 'scatter',
      label: names[j] + ' kf',
      data: this.traj.getKeyframePoints(j),
      borderColor: colors[j],
      backgroundColor: j === this.selectedJoint ? colors[j] : 'transparent',
      pointRadius: ctx => (ctx.raw?.locked ? 5 : (j === this.selectedJoint ? 6 : 3)),
      pointBorderColor: colors[j],
      pointBorderWidth: ctx => (ctx.raw?.locked ? 2 : 1),
      pointHitRadius: 12,
      pointHoverRadius: 8,
      showLine: false,
      parsing: false,
      hidden: !this.visible[j],
    }));
    const measDatasets = Array.from({ length: this._jointCount }, (_, j) => ({
      type: 'line',
      label: names[j] + ' meas',
      data: [],
      borderColor: colors[j],
      backgroundColor: 'transparent',
      borderWidth: 1.5,
      borderDash: [5, 4],
      borderCapStyle: 'round',
      pointRadius: 0,
      tension: 0,
      parsing: false,
      hidden: true,
      order: 1,
    }));

    const darkGrid = 'rgba(255,255,255,0.07)';
    const darkTick = 'rgba(255,255,255,0.45)';
    const darkLabel = 'rgba(255,255,255,0.6)';

    const bgPlugin = {
      id: 'chartBg',
      beforeDraw(chart) {
        const { ctx } = chart;
        ctx.save();
        ctx.fillStyle = '#0d1117';
        ctx.fillRect(0, 0, chart.width, chart.height);
        ctx.restore();
      },
    };

    return new Chart(this._canvas, {
      type: 'scatter',
      data: { datasets: [...lineDatasets, ...kfDatasets, ...measDatasets] },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        interaction: { mode: 'nearest', intersect: false },
        plugins: {
          legend: { display: false },
          tooltip: { enabled: false },
        },
        scales: {
          x: {
            type: 'linear',
            min: 0,
            max: this.traj.duration,
            title: { display: true, text: 'Time (s)', color: darkLabel, font: { size: 11 } },
            ticks: { maxTicksLimit: 12, color: darkTick },
            grid: { color: darkGrid },
            border: { color: darkGrid },
          },
          y: {
            title: { display: true, text: this._yAxisLabel, color: darkLabel, font: { size: 11 } },
            ticks: { color: darkTick, maxTicksLimit: 8 },
            grid: { color: darkGrid },
            border: { color: darkGrid },
          },
        },
      },
      plugins: [bgPlugin, this._scrubberPlugin],
    });
  }

  _attachEvents(canvas) {
    canvas.addEventListener('mousedown', e => this._onMouseDown(e));
    canvas.addEventListener('mousemove', e => this._onMouseMove(e));
    canvas.addEventListener('mouseup', e => this._onMouseUp(e));
    canvas.addEventListener('mouseleave', e => {
      this._hoverSnap = null;
      this._onMouseUp(e);
      this._chart.update('none');
    });
    canvas.addEventListener('dblclick', e => this._onDblClick(e));
    canvas.addEventListener('contextmenu', e => { e.preventDefault(); this._onRightClick(e); });
  }

  _canvasPos(e) {
    const rect = this._canvas.getBoundingClientRect();
    return { x: e.clientX - rect.left, y: e.clientY - rect.top };
  }

  _pixelToData(px, py) {
    const scales = this._chart.scales;
    return {
      t: scales.x.getValueForPixel(px),
      q: scales.y.getValueForPixel(py),
    };
  }

  _dataToPixel(t, q) {
    const scales = this._chart.scales;
    return {
      x: scales.x.getPixelForValue(t),
      y: scales.y.getPixelForValue(q),
    };
  }

  _findNearestKeyframe(px, py, thresh = 12) {
    const j = this.selectedJoint;
    if (!this.visible[j]) return null;
    let nearest = null;
    let minD = thresh;
    const kfs = this.traj.keyframes[j];
    kfs.forEach((kf, idx) => {
      const p = this._dataToPixel(kf.t, kf.q);
      const d = Math.hypot(px - p.x, py - p.y);
      if (d < minD) {
        nearest = { j, idx, locked: false };
        minD = d;
      }
    });
    return nearest;
  }

  _findNearestCurve(px, py, thresh = 12) {
    const { t } = this._pixelToData(px, py);
    let nearest = null;
    let minD = thresh;
    for (let j = 0; j < this._jointCount; j++) {
      if (!this.visible[j]) continue;
      const q = this.traj.getValueAt(j, t);
      const p = this._dataToPixel(t, q);
      const d = Math.abs(py - p.y);
      if (d < minD) {
        nearest = j;
        minD = d;
      }
    }
    return nearest;
  }

  _isScrubberHit(px, thresh = 8) {
    const sx = this._chart.scales.x.getPixelForValue(this.playhead);
    return Math.abs(px - sx) < thresh;
  }

  _onMouseDown(e) {
    const { x, y } = this._canvasPos(e);
    const ca = this._chart.chartArea;
    if (!ca || x < ca.left || x > ca.right || y < ca.top || y > ca.bottom) return;

    const kfHit = this._findNearestKeyframe(x, y);
    if (kfHit && !kfHit.locked) {
      this._dragging = { type: 'keyframe', ...kfHit };
      this.opts.onEditStart?.();
      return;
    }

    if (this._isScrubberHit(x)) {
      this._dragging = { type: 'scrubber' };
      return;
    }

    const jHit = this._findNearestCurve(x, y);
    if (jHit !== null && jHit !== this.selectedJoint) {
      this.setSelectedJoint(jHit);
      this.opts.onJointSelect?.(jHit);
      return;
    }

    this._dragging = { type: 'scrubber' };
    const { t } = this._pixelToData(x, y);
    this._updateScrubber(Math.max(0, Math.min(this.traj.duration, t)));
  }

  _onMouseMove(e) {
    const { x, y } = this._canvasPos(e);
    const ca = this._chart.chartArea;
    if (ca && x >= ca.left && x <= ca.right && y >= ca.top && y <= ca.bottom && this.visible[this.selectedJoint]) {
      const { t } = this._pixelToData(x, y);
      const clampedT = Math.max(0, Math.min(this.traj.duration, t));
      this._hoverSnap = { t: clampedT, q: this.traj.getValueAt(this.selectedJoint, clampedT) };
    } else {
      this._hoverSnap = null;
    }

    if (!this._dragging) {
      this._chart.update('none');
      return;
    }

    const { t, q } = this._pixelToData(x, y);

    if (this._dragging.type === 'scrubber') {
      this._updateScrubber(Math.max(0, Math.min(this.traj.duration, t)));
      return;
    }

    if (this._dragging.type === 'keyframe') {
      const { j, idx } = this._dragging;
      this.traj.setKeyframe(j, idx, t, q);
      this.refresh();
      this.opts.onKeyframeEdit?.();
    }
  }

  _onMouseUp() {
    if (!this._dragging) return;
    const drag = this._dragging;
    this._dragging = null;
    if (drag.type === 'keyframe') this.opts.onEditCommit?.();
  }

  _onDblClick(e) {
    e.preventDefault();
    this._dragging = null;
    const { x, y } = this._canvasPos(e);
    const ca = this._chart.chartArea;
    if (!ca || x < ca.left || x > ca.right || y < ca.top || y > ca.bottom) return;
    if (this._findNearestKeyframe(x, y, 14)) return;
    const { t } = this._pixelToData(x, y);
    this.opts.onEditStart?.();
    const idx = this.traj.addKeyframe(this.selectedJoint, t);
    if (idx < 0) {
      this.opts.onEditCommit?.();
      return;
    }
    this.refresh();
    this.opts.onKeyframeEdit?.();
    this.opts.onEditCommit?.();
  }

  _onRightClick(e) {
    const { x, y } = this._canvasPos(e);
    const kfHit = this._findNearestKeyframe(x, y, 16);
    if (kfHit && !kfHit.locked) {
      this.opts.onEditStart?.();
      this.traj.removeKeyframe(kfHit.j, kfHit.idx);
      this.refresh();
      this.opts.onKeyframeEdit?.();
      this.opts.onEditCommit?.();
    }
  }

  _updateScrubber(t) {
    this.playhead = t;
    this._chart.update('none');
    this.opts.onScrub?.(t);
  }

  _syncVisibility() {
    const ds = this._chart.data.datasets;
    for (let j = 0; j < this._jointCount; j++) {
      const visible = !!this.visible[j];
      ds[this._lineDatasetIndex(j)].hidden = !visible;
      ds[this._keyframeDatasetIndex(j)].hidden = !visible;
      const key = this._measuredKeys[j];
      ds[this._measuredDatasetIndex(j)].hidden = !visible || !this._logData || !key;
    }
  }

  setPlayhead(t) {
    this.playhead = Math.max(0, Math.min(this.traj.duration, t));
    this._chart.update('none');
  }

  setSelectedJoint(j) {
    this.selectedJoint = Math.max(0, Math.min(this._jointCount - 1, j));
    const ds = this._chart.data.datasets;
    for (let i = 0; i < this._jointCount; i++) {
      ds[this._lineDatasetIndex(i)].borderWidth = i === this.selectedJoint ? 2.5 : 1.5;
      ds[this._keyframeDatasetIndex(i)].backgroundColor = i === this.selectedJoint ? this.traj.jointColors[i] : 'transparent';
    }
    this._syncVisibility();
    this._chart.update('none');
  }

  setVisible(j, visible) {
    this.visible[j] = visible;
    this._syncVisibility();
    this._chart.update('none');
  }

  setLogData(logData) {
    this._logData = logData;
    const ds = this._chart.data.datasets;
    for (let j = 0; j < this._jointCount; j++) {
      const key = this._measuredKeys[j];
      if (!logData || logData.length === 0 || !key) {
        ds[this._measuredDatasetIndex(j)].data = [];
      } else {
        ds[this._measuredDatasetIndex(j)].data = logData.map(s => ({ x: s.plot_t ?? s.t_s, y: s[key] }));
      }
    }
    this._syncVisibility();
    this._chart.update('none');
  }

  setTrajectory(traj) {
    this.traj = traj;
    this._jointCount = traj.jointCount;
    if (!Array.isArray(this.visible) || this.visible.length !== this._jointCount) {
      this.visible = new Array(this._jointCount).fill(true);
    } else {
      this.visible = this.visible.map(Boolean);
    }
    this.selectedJoint = Math.max(0, Math.min(this._jointCount - 1, this.selectedJoint));
    this.playhead = Math.max(0, Math.min(this.traj.duration, this.playhead));
    this.refresh();
    this.setSelectedJoint(this.selectedJoint);
    this.setLogData(this._logData);
    this.setPlayhead(this.playhead);
  }

  snapshotViewState() {
    return {
      playhead: this.playhead,
      selectedJoint: this.selectedJoint,
      visible: [...this.visible],
    };
  }

  restoreViewState(viewState = null) {
    if (Array.isArray(viewState?.visible) && viewState.visible.length === this._jointCount) {
      this.visible = viewState.visible.map(Boolean);
    } else {
      this.visible = new Array(this._jointCount).fill(true);
    }
    this.selectedJoint = Number.isInteger(viewState?.selectedJoint) ? viewState.selectedJoint : 0;
    this.playhead = Number.isFinite(viewState?.playhead) ? viewState.playhead : 0;
    this.refresh();
    this.setSelectedJoint(this.selectedJoint);
    this.setLogData(this._logData);
    this.setPlayhead(this.playhead);
  }

  refresh() {
    const ds = this._chart.data.datasets;
    for (let j = 0; j < this._jointCount; j++) {
      ds[this._lineDatasetIndex(j)].data = this.traj.getCurvePoints(j);
      ds[this._keyframeDatasetIndex(j)].data = this.traj.getKeyframePoints(j);
    }
    this._chart.options.scales.x.max = this.traj.duration;

    let yMin = Infinity;
    let yMax = -Infinity;
    for (let j = 0; j < this._jointCount; j++) {
      if (!this.visible[j]) continue;
      for (const kf of this.traj.keyframes[j]) {
        yMin = Math.min(yMin, kf.q);
        yMax = Math.max(yMax, kf.q);
      }
    }
    if (this._logData && this._measuredKeys.length === this._jointCount) {
      for (const s of this._logData) {
        for (let j = 0; j < this._jointCount; j++) {
          const key = this._measuredKeys[j];
          if (!key || !this.visible[j]) continue;
          const val = s[key];
          if (val == null) continue;
          yMin = Math.min(yMin, val);
          yMax = Math.max(yMax, val);
        }
      }
    }
    if (isFinite(yMin) && isFinite(yMax)) {
      const pad = Math.max(0.2, (yMax - yMin) * 0.12);
      this._chart.options.scales.y.min = yMin - pad;
      this._chart.options.scales.y.max = yMax + pad;
    }

    this._syncVisibility();
    this._chart.update('none');
  }

  setDuration(dur) {
    this._chart.options.scales.x.max = dur;
    this.refresh();
  }
};
