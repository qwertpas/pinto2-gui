/**
 * viewer3d.js  –  Viewer3D
 * Three.js 3D robot viewer with three URDF-matching robot instances:
 *   - Reference: translucent blue  (driven by target trajectory + reference IMU)
 *   - Logged:    solid orange      (driven by recorded run telemetry)
 *   - Current:   translucent green (driven by live telemetry)
 *
 * Robot geometry built from squirrel2.urdf parameters (all primitives):
 *   shoulder → spine → hip → ankles → feet (chain, all revolute around Y)
 *   shoulder → wrist_L  (revolute around X)
 *   shoulder → wrist_R  (revolute around X)
 *
 * Joint order in setPose([shoulder, spine, hip, arm_l, arm_r]):
 *   idx 0 = shoulder_to_spine  (shoulder motor)
 *   idx 1 = spine_to_hip       (spine motor)
 *   idx 2 = hip_to_ankles      (hip motor)
 *   idx 3 = shoulder_to_wrist_L (arm_l servo)
 *   idx 4 = shoulder_to_wrist_R (arm_r servo)
 *
 * IMU orientation: each robot body uses only its own quaternion stream.
 *
 * Exposes: window.Viewer3D
 * Requires: Three.js global `THREE`
 */

window.Viewer3D = class Viewer3D {
  constructor(containerId) {
    const container = document.getElementById(containerId);
    this._container = container;

    // ── Renderer ─────────────────────────────────────────────────────────────
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setClearColor(0x0d1117, 1);
    renderer.shadowMap.enabled = true;
    container.appendChild(renderer.domElement);
    this._renderer = renderer;

    // ── Scene ────────────────────────────────────────────────────────────────
    const scene = new THREE.Scene();
    scene.add(new THREE.AmbientLight(0x8090c0, 0.9));
    const dirLight = new THREE.DirectionalLight(0xe0eeff, 1.2);
    dirLight.position.set(0.4, 1.2, 0.6);
    dirLight.castShadow = true;
    scene.add(dirLight);
    const rimLight = new THREE.DirectionalLight(0xffa040, 0.4);
    rimLight.position.set(-0.5, -0.2, -0.4);
    scene.add(rimLight);

    // Ground grid (subtle on dark bg)
    const gridHelper = new THREE.GridHelper(0.6, 12, 0x1e3a5f, 0x152a45);
    gridHelper.position.y = -0.02;
    scene.add(gridHelper);

    this._scene = scene;

    // ── Camera ───────────────────────────────────────────────────────────────
    const w = container.clientWidth || 400;
    const h = container.clientHeight || 300;
    const camera = new THREE.PerspectiveCamera(42, w / h, 0.001, 10);
    this._camera = camera;

    // Orbit: look at center of robot body, slight elevation + 45° from front
    this._orbit = { theta: Math.PI / 4, phi: 0.30, radius: 0.42 };
    this._attachOrbit(renderer.domElement);

    // ── Robot groups (under scene) ──────────────────────────────────────────
    this._referenceRoot = new THREE.Group();
    this._referenceRoot.position.set(0, 0.05, 0);
    scene.add(this._referenceRoot);

    this._loggedRoot = new THREE.Group();
    this._loggedRoot.position.set(0, 0.05, 0);
    scene.add(this._loggedRoot);

    this._currentRoot = new THREE.Group();
    this._currentRoot.position.set(0, 0.05, 0);
    scene.add(this._currentRoot);

    // Display convention: map the robot yaw axis onto Three.js world-up.
    this._displayQuat = new THREE.Quaternion()
      .setFromAxisAngle(new THREE.Vector3(1, 0, 0), -Math.PI * 0.5);
    this._worldUp = new THREE.Vector3(0, 1, 0);

    // Materials — reference: translucent blue; logged: solid orange; current: translucent green
    this._matReference = {
      body:  new THREE.MeshLambertMaterial({ color: 0x90caf9, transparent: true, opacity: 0.40, depthWrite: false }),
      bone:  new THREE.MeshLambertMaterial({ color: 0x90caf9, transparent: true, opacity: 0.28, depthWrite: false }),
      tip:   new THREE.MeshLambertMaterial({ color: 0x64b5f6, transparent: true, opacity: 0.40, depthWrite: false }),
    };
    this._matLogged = {
      body:  new THREE.MeshLambertMaterial({ color: 0xffa726 }),
      bone:  new THREE.MeshLambertMaterial({ color: 0xd4881a }),
      tip:   new THREE.MeshLambertMaterial({ color: 0xffd54f }),
    };
    this._matCurrent = {
      body:  new THREE.MeshLambertMaterial({ color: 0x81c784, transparent: true, opacity: 0.32, depthWrite: false }),
      bone:  new THREE.MeshLambertMaterial({ color: 0x66bb6a, transparent: true, opacity: 0.24, depthWrite: false }),
      tip:   new THREE.MeshLambertMaterial({ color: 0xa5d6a7, transparent: true, opacity: 0.36, depthWrite: false }),
    };

    this._reference = this._buildRobot(this._referenceRoot, this._matReference);
    this._logged = this._buildRobot(this._loggedRoot, this._matLogged);
    this._current = this._buildRobot(this._currentRoot, this._matCurrent);

    this._loggedRoot.visible = false;
    this._currentRoot.visible = false;

    this._referenceImuQuat = new THREE.Quaternion();
    this._loggedImuQuat = new THREE.Quaternion();
    this._currentImuQuat = new THREE.Quaternion();

    // Resize observer
    const ro = new ResizeObserver(() => this._resize());
    ro.observe(container);
    this._resize();

    // Start render loop
    this._animate();
  }

  // ── Robot builder ──────────────────────────────────────────────────────────

  /**
   * Builds a robot Three.js hierarchy.
   * Returns { joints: [j0..j4], root } for later pose updates.
   *
   * URDF structure (from squirrel2.urdf):
   *   shoulder link (root)
   *     shoulder_to_spine joint (origin=0,0,0 axis=Y) → spine link (offset=0,0,0)
   *       spine_to_hip joint (origin=-0.12,0,0 axis=Y) → hip link (offset=-0.12,0,0)
   *         hip_to_ankles joint (origin=-0.066,0,0 axis=Y) → ankles link
   *           (fixed) foot_L at (0,-0.03,-0.041)
   *           (fixed) foot_R at (0, 0.03,-0.041)
   *     shoulder_to_wrist_L joint (origin=0,0.04,0 axis=X) → wrist_L
   *     shoulder_to_wrist_R joint (origin=0,-0.04,0 axis=X) → wrist_R
   */
  /**
   * Builds a robot Three.js hierarchy exactly matching squirrel2.urdf visuals.
   *
   * URDF visual origins (link-local frame, all-joints-zero world coords):
   *   shoulder sphere    : (0,     0,  0)      ← shoulder link origin
   *   spine   sphere     : (−0.12, 0,  0)      ← spine link visual origin
   *   hip     sphere     : (−0.066,0,  0) in hip frame → (−0.186, 0, 0) world
   *   foot_L             : (0, −0.03, −0.041)  relative to ankles joint
   *   foot_R             : (0, +0.03, −0.041)
   *   wrist_L/R tip      : (0,   0,  −0.041)   relative to wrist joint
   *
   * Joint hierarchy:
   *   shoulder → [j0] spine → [j1] hip → [j2] ankles → feet (fixed)
   *   shoulder → [j3] wrist_L
   *   shoulder → [j4] wrist_R
   */
  _buildRobot(parent, mat) {
    const joints = new Array(5);

    // ── Shoulder link (root) ─────────────────────────────────────────────────
    const shoulderLink = new THREE.Group();
    parent.add(shoulderLink);
    this._addSphere(shoulderLink, 0, 0, 0, 0.018, mat.body);
    // Arm stubs: shoulder (0,0,0) → wrist pivots (0, ±0.04, 0), box along Y
    this._addBox(shoulderLink, 0,  0.020, 0, 0.012, 0.040, 0.012, mat.bone);
    this._addBox(shoulderLink, 0, -0.020, 0, 0.012, 0.040, 0.012, mat.bone);

    // ── j0: shoulder_to_spine  (axis Y, origin 0,0,0) ───────────────────────
    const j0pivot = new THREE.Group();
    shoulderLink.add(j0pivot);
    joints[0] = j0pivot;

    const spineLink = new THREE.Group();
    j0pivot.add(spineLink);
    // Spine sphere is at (−0.12, 0, 0) in spine link frame (= j0pivot frame)
    this._addSphere(spineLink, -0.12, 0, 0, 0.018, mat.body);
    // Spine body bar connecting shoulder to spine sphere
    this._addBox(spineLink, -0.06, 0, 0, 0.120, 0.012, 0.012, mat.bone);

    // ── j1: spine_to_hip  (axis Y, origin −0.12,0,0 in spine frame) ─────────
    const j1pivot = new THREE.Group();
    j1pivot.position.set(-0.12, 0, 0);
    spineLink.add(j1pivot);
    joints[1] = j1pivot;

    const hipLink = new THREE.Group();
    j1pivot.add(hipLink);
    // Hip sphere at (−0.066, 0, 0) in hip link frame (= j1pivot frame)
    this._addSphere(hipLink, -0.066, 0, 0, 0.018, mat.body);
    // Hip body bar
    this._addBox(hipLink, -0.033, 0, 0, 0.066, 0.012, 0.012, mat.bone);

    // ── j2: hip_to_ankles  (axis Y, origin −0.066,0,0 in hip frame) ─────────
    const j2pivot = new THREE.Group();
    j2pivot.position.set(-0.066, 0, 0);
    hipLink.add(j2pivot);
    joints[2] = j2pivot;

    const anklesLink = new THREE.Group();
    j2pivot.add(anklesLink);
    // Leg bones: ankle (0,0,0) → foot (0, ±0.030, −0.041)  midpoint=±0.015, −0.0205
    this._addBox(anklesLink, 0, -0.015, -0.0205, 0.008, 0.030, 0.041, mat.bone);
    this._addBox(anklesLink, 0,  0.015, -0.0205, 0.008, 0.030, 0.041, mat.bone);
    // Foot tips (fixed joints: ankles_to_foot_L/R origin 0,∓0.03,−0.041)
    this._addSphere(anklesLink, 0, -0.030, -0.041, 0.012, mat.tip);
    this._addSphere(anklesLink, 0,  0.030, -0.041, 0.012, mat.tip);

    // ── j3: shoulder_to_wrist_L  (axis X, origin 0,+0.04,0) ─────────────────
    const j3pivot = new THREE.Group();
    j3pivot.position.set(0, 0.040, 0);
    shoulderLink.add(j3pivot);
    joints[3] = j3pivot;

    const wristL = new THREE.Group();
    j3pivot.add(wristL);
    this._addBox(wristL, 0, 0, -0.0205, 0.008, 0.008, 0.041, mat.bone);
    this._addSphere(wristL, 0, 0, -0.041, 0.012, mat.tip);

    // ── j4: shoulder_to_wrist_R  (axis X, origin 0,−0.04,0) ─────────────────
    const j4pivot = new THREE.Group();
    j4pivot.position.set(0, -0.040, 0);
    shoulderLink.add(j4pivot);
    joints[4] = j4pivot;

    const wristR = new THREE.Group();
    j4pivot.add(wristR);
    this._addBox(wristR, 0, 0, -0.0205, 0.008, 0.008, 0.041, mat.bone);
    this._addSphere(wristR, 0, 0, -0.041, 0.012, mat.tip);

    return { joints, root: parent, shoulderLink };
  }

  _addSphere(parent, x, y, z, r, mat) {
    const mesh = new THREE.Mesh(new THREE.SphereGeometry(r, 16, 10), mat);
    mesh.position.set(x, y, z);
    parent.add(mesh);
    return mesh;
  }

  _addBox(parent, x, y, z, w, h, d, mat, euler) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
    mesh.position.set(x, y, z);
    if (euler) mesh.rotation.copy(euler);
    parent.add(mesh);
    return mesh;
  }

  // ── Pose application ───────────────────────────────────────────────────────

  /**
   * Set joint angles for one robot.
   * @param {{joints:THREE.Group[]}} robot
   * @param {number[]} pose  [shoulder, spine, hip, arm_l, arm_r]  (radians)
   */
  _applyPose(robot, pose) {
    const shoulder = pose[0] ?? 0;
    const armOffset = 0.5 * Math.PI;

    // URDF root is shoulder; compensate root yaw so spine appears fixed in world.
    robot.shoulderLink.rotation.y = -shoulder;

    // joints 0,1,2 rotate around Y axis; joints 3,4 rotate around X axis
    robot.joints[0].rotation.y = shoulder;
    robot.joints[1].rotation.y = pose[1] ?? 0;
    robot.joints[2].rotation.y = pose[2] ?? 0;
    robot.joints[3].rotation.x = (pose[3] ?? 0) + armOffset;
    robot.joints[4].rotation.x = (pose[4] ?? 0) - armOffset;
  }

  _applyRootQuat(root, imuQuat) {
    root.quaternion.copy(this._displayQuat).multiply(imuQuat);
  }

  _worldQuatFromImu(imuQuat) {
    return this._displayQuat.clone().multiply(imuQuat);
  }

  _twistAroundAxis(quat, axis) {
    const dot = quat.x * axis.x + quat.y * axis.y + quat.z * axis.z;
    const twist = new THREE.Quaternion(axis.x * dot, axis.y * dot, axis.z * dot, quat.w);
    if (twist.lengthSq() <= 1e-12) return new THREE.Quaternion();
    return twist.normalize();
  }

  _alignWorldYaw(worldQuat, referenceWorldQuat) {
    const delta = referenceWorldQuat.clone().multiply(worldQuat.clone().invert());
    const twist = this._twistAroundAxis(delta, this._worldUp);
    return twist.multiply(worldQuat).normalize();
  }

  _composeRootQuat() {
    const referenceWorldQuat = this._worldQuatFromImu(this._referenceImuQuat);
    this._referenceRoot.quaternion.copy(referenceWorldQuat);
    this._loggedRoot.quaternion.copy(this._alignWorldYaw(
      this._worldQuatFromImu(this._loggedImuQuat),
      referenceWorldQuat,
    ));
    this._applyRootQuat(this._currentRoot, this._currentImuQuat);
  }

  _setQuat(target, quat) {
    if (!quat || quat.length !== 4) {
      target.identity();
      return;
    }
    target.set(quat[1], quat[2], quat[3], quat[0]).normalize();
  }

  _setQuatFromRpyDeg(target, rpyDeg) {
    if (!rpyDeg || rpyDeg.length !== 3) {
      target.identity();
      return;
    }
    const toRad = Math.PI / 180;
    const euler = new THREE.Euler(rpyDeg[0] * toRad, rpyDeg[1] * toRad, rpyDeg[2] * toRad, 'XYZ');
    target.setFromEuler(euler).normalize();
  }

  // ── Public API ─────────────────────────────────────────────────────────────

  setReferencePose(pose) {
    this._applyPose(this._reference, pose);
  }

  setReferenceImuQuat(quat) {
    this._setQuat(this._referenceImuQuat, quat);
    this._composeRootQuat();
  }

  setLoggedPose(pose) {
    this._applyPose(this._logged, pose);
    this._loggedRoot.visible = true;
  }

  setLoggedImuQuat(quat) {
    this._setQuat(this._loggedImuQuat, quat);
    this._composeRootQuat();
  }

  hideLogged() {
    this._loggedRoot.visible = false;
  }

  setCurrentPose(pose) {
    this._applyPose(this._current, pose);
  }

  setCurrentImuQuat(quat) {
    this._setQuat(this._currentImuQuat, quat);
    this._composeRootQuat();
  }

  setCurrentVisible(visible) {
    this._currentRoot.visible = !!visible;
  }

  // ── Orbit ──────────────────────────────────────────────────────────────────

  _attachOrbit(el) {
    let down = false, lastX = 0, lastY = 0;
    el.addEventListener('mousedown', e => { down = true; lastX = e.clientX; lastY = e.clientY; });
    el.addEventListener('mousemove', e => {
      if (!down) return;
      const dx = e.clientX - lastX, dy = e.clientY - lastY;
      lastX = e.clientX; lastY = e.clientY;
      this._orbit.theta -= dx * 0.01;
      this._orbit.phi    = Math.max(0.05, Math.min(Math.PI / 2, this._orbit.phi + dy * 0.01));
    });
    el.addEventListener('mouseup',   () => { down = false; });
    el.addEventListener('mouseleave',() => { down = false; });
    el.addEventListener('wheel', e => {
      e.preventDefault();
      this._orbit.radius = Math.max(0.15, Math.min(1.5, this._orbit.radius + e.deltaY * 0.0005));
    }, { passive: false });
  }

  _updateCamera() {
    const { theta, phi, radius } = this._orbit;
    this._camera.position.set(
      radius * Math.sin(theta) * Math.cos(phi),
      radius * Math.sin(phi),
      radius * Math.cos(theta) * Math.cos(phi),
    );
    // Orbit around world origin (model is shifted so body center sits near origin).
    this._camera.lookAt(new THREE.Vector3(0, 0, 0));
  }

  // ── Render loop ────────────────────────────────────────────────────────────

  _animate() {
    requestAnimationFrame(() => this._animate());
    this._updateCamera();
    this._composeRootQuat();
    this._renderer.render(this._scene, this._camera);
  }

  _resize() {
    const w = this._container.clientWidth;
    const h = this._container.clientHeight;
    if (!w || !h) return;
    this._renderer.setSize(w, h);
    this._camera.aspect = w / h;
    this._camera.updateProjectionMatrix();
  }
};
