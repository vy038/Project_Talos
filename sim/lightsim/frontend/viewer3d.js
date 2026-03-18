/**
 * @file viewer3d.js
 * @brief Mode 1 - 3D view using Three.js (procedural geometry)
 *
 * Loads Three.js from CDN. Creates an articulated robot model with
 * box/cylinder geometry. Animates from WebSocket servo data.
 *
 * Features: client-side gait generator, full robot-ball collision,
 * click-to-place ball, IK cursor for arm control.
 */

import * as THREE from 'https://cdn.jsdelivr.net/npm/three@0.170.0/build/three.module.js';
import { OrbitControls } from 'https://cdn.jsdelivr.net/npm/three@0.170.0/examples/jsm/controls/OrbitControls.js';
import {
    Vec3, PhysicsWorld, RigidBody,
    SphereShape, PlaneShape, BoxShape,
    BODY_STATIC, BODY_DYNAMIC, BODY_KINEMATIC,
} from './physics.js';

const container = document.getElementById('three-container');

// Scene setup
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0e14);

const camera = new THREE.PerspectiveCamera(60, 1, 0.1, 1000);
camera.position.set(0, 200, 250);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
container.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 30, 0);
controls.enableDamping = true;
controls.maxPolarAngle = Math.PI - 0.05;
controls.minPolarAngle = 0.05;

// Lighting
scene.add(new THREE.AmbientLight(0x404060, 1.5));
const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
dirLight.position.set(100, 200, 100);
scene.add(dirLight);

// Ground grid
const grid = new THREE.GridHelper(400, 20, 0x1e2738, 0x151a24);
scene.add(grid);

// Ground plane (invisible, for raycasting)
const groundPlane = new THREE.Mesh(
    new THREE.PlaneGeometry(2000, 2000),
    new THREE.MeshBasicMaterial({ visible: false })
);
groundPlane.rotation.x = -Math.PI / 2;
groundPlane.position.y = 0;
scene.add(groundPlane);

// Materials
const bodyMat = new THREE.MeshPhongMaterial({ color: 0x16213e, emissive: 0x0a0a1a });
const legMat = new THREE.MeshPhongMaterial({ color: 0x4ade80 });
const legSwingMat = new THREE.MeshPhongMaterial({ color: 0xf87171 });
const armMat = new THREE.MeshPhongMaterial({ color: 0xf59e0b });
const gripperMat = new THREE.MeshPhongMaterial({ color: 0xa78bfa });
const jointMat = new THREE.MeshPhongMaterial({ color: 0x888888 });

// Robot body
const bodyGeom = new THREE.BoxGeometry(40, 15, 80);
const body = new THREE.Mesh(bodyGeom, bodyMat);
scene.add(body);

// Physics & Odometry State
const robotPos = { x: 0, y: 30, z: 0, yaw: 0 };
let lastLegs = null;

// Leg configuration
const legAttach = [
    { x: 20, z: -30, angle: 30 },
    { x: 20, z: 0, angle: 0 },
    { x: 20, z: 30, angle: -30 },
    { x: -20, z: 30, angle: 210 },
    { x: -20, z: 0, angle: 180 },
    { x: -20, z: -30, angle: 150 },
];

const HIP_LEN_FLAT = 25;
const HIP_LEN_DIAG = 35;
const UPPER_SHIN_LEN = 30;
const LOWER_SHIN_LEN = 30;

// Create leg groups
const legs = [];
// Robot mesh references tracked by physics engine (robotColliders in physics section)



for (let i = 0; i < 6; i++) {
    const attach = legAttach[i];
    const hipGroup = new THREE.Group();
    hipGroup.position.set(attach.x, 0, attach.z);
    hipGroup.rotation.y = attach.angle * Math.PI / 180;
    body.add(hipGroup);

    const hipGeom1 = new THREE.BoxGeometry(HIP_LEN_FLAT, 6, 6);
    const hipMesh1 = new THREE.Mesh(hipGeom1, legMat);
    hipMesh1.position.x = HIP_LEN_FLAT / 2;
    hipGroup.add(hipMesh1);

    const hipGeom2 = new THREE.BoxGeometry(HIP_LEN_DIAG, 6, 6);
    const hipMesh2 = new THREE.Mesh(hipGeom2, legMat);
    hipMesh2.position.set(HIP_LEN_FLAT + (HIP_LEN_DIAG / 2) * Math.cos(0.8), (HIP_LEN_DIAG / 2) * Math.sin(0.8), 0);
    hipMesh2.rotation.z = 0.8;
    hipGroup.add(hipMesh2);

    const kneeGroup = new THREE.Group();
    kneeGroup.position.set(HIP_LEN_FLAT + HIP_LEN_DIAG * Math.cos(0.8), HIP_LEN_DIAG * Math.sin(0.8), 0);
    kneeGroup.rotation.z = -0.15;
    hipGroup.add(kneeGroup);

    const jointGeom = new THREE.SphereGeometry(4, 8, 8);
    const jointMesh = new THREE.Mesh(jointGeom, jointMat);
    kneeGroup.add(jointMesh);

    const upperShinGroup = new THREE.Group();
    upperShinGroup.rotation.z = 1.25;
    kneeGroup.add(upperShinGroup);

    const upperGeom = new THREE.BoxGeometry(4, UPPER_SHIN_LEN, 4);
    const upperMesh = new THREE.Mesh(upperGeom, legMat);
    upperMesh.position.y = -UPPER_SHIN_LEN / 2;
    upperShinGroup.add(upperMesh);

    const lowerShinGroup = new THREE.Group();
    lowerShinGroup.position.y = -UPPER_SHIN_LEN;
    upperShinGroup.add(lowerShinGroup);

    const lowerGeom = new THREE.BoxGeometry(4, LOWER_SHIN_LEN, 4);
    const lowerMesh = new THREE.Mesh(lowerGeom, legMat);
    lowerMesh.position.y = -LOWER_SHIN_LEN / 2;
    lowerShinGroup.add(lowerMesh);

    const footGeom = new THREE.SphereGeometry(3, 6, 6);
    const footMesh = new THREE.Mesh(footGeom, legMat);
    footMesh.position.y = -LOWER_SHIN_LEN;
    lowerShinGroup.add(footMesh);


    legs.push({
        hipGroup, hipMesh1, hipMesh2, kneeGroup,
        upperShinGroup, upperMesh, lowerShinGroup, lowerMesh, footMesh,
        baseAngle: attach.angle * Math.PI / 180,
    });
}

// Arm
const armBaseGroup = new THREE.Group();
armBaseGroup.position.set(0, 8, 25);
armBaseGroup.rotation.x = -Math.PI / 4;
body.add(armBaseGroup);

const armBaseMesh = new THREE.Mesh(new THREE.CylinderGeometry(5, 5, 8, 8), armMat);
armBaseGroup.add(armBaseMesh);

const shoulderGroup = new THREE.Group();
shoulderGroup.position.y = 4;
armBaseGroup.add(shoulderGroup);

const shoulderMesh = new THREE.Mesh(new THREE.BoxGeometry(6, 90, 6), armMat);
shoulderMesh.position.y = 45;
shoulderGroup.add(shoulderMesh);

const elbowGroup = new THREE.Group();
elbowGroup.position.y = 90;
shoulderGroup.add(elbowGroup);

const elbowJoint = new THREE.Mesh(new THREE.SphereGeometry(4, 8, 8), jointMat);
elbowGroup.add(elbowJoint);

const forearmMesh = new THREE.Mesh(new THREE.BoxGeometry(5, 75, 5), armMat);
forearmMesh.position.y = 37.5;
elbowGroup.add(forearmMesh);

const gripperGroup = new THREE.Group();
gripperGroup.position.y = 75;
elbowGroup.add(gripperGroup);

const gripL = new THREE.Mesh(new THREE.BoxGeometry(2, 10, 2), gripperMat);
gripL.position.set(-4, 5, 0);
gripperGroup.add(gripL);

const gripR = new THREE.Mesh(new THREE.BoxGeometry(2, 10, 2), gripperMat);
gripR.position.set(4, 5, 0);
gripperGroup.add(gripR);


// ============================================================================
// ESP-CAM Camera Module (mounted front-bottom of body)
// ============================================================================

// Camera constants matching espcam firmware
const CAM_W = 320, CAM_H = 240;
const CAM_FOV_DEG = 60.0;
const CAM_HFOV_RAD = CAM_FOV_DEG * Math.PI / 180;
const CAM_VFOV_RAD = 2 * Math.atan(Math.tan(CAM_HFOV_RAD / 2) * (CAM_H / CAM_W));
const CAM_FOCAL_PX = (CAM_W / 2) / Math.tan(CAM_HFOV_RAD / 2);
const SIM_MM_PER_PX = 2.5; // scale: 8px ball radius = 20mm real
const BALL_REAL_RADIUS_MM = 20.0;

// Camera group — attached to body, front-bottom (-Z face, opposite arm at +Z)
// ~4cm below base (16px at 2.5mm/px scale), at front face
const camGroup = new THREE.Group();
camGroup.position.set(0, -22, -38);
camGroup.rotation.y = Math.PI; // face outward (local +Z = world -Z = robot front)
body.add(camGroup);

// Camera body mesh (small dark box representing ESP32-CAM)
const camBoxMat = new THREE.MeshPhongMaterial({ color: 0x222222, emissive: 0x050505 });
const camBoxMesh = new THREE.Mesh(new THREE.BoxGeometry(10, 8, 6), camBoxMat);
camGroup.add(camBoxMesh);

// Lens (small cylinder on front face)
const lensMat = new THREE.MeshPhongMaterial({ color: 0x111111, emissive: 0x001133 });
const lensMesh = new THREE.Mesh(new THREE.CylinderGeometry(2.5, 2.5, 2, 8), lensMat);
lensMesh.rotation.x = Math.PI / 2;
lensMesh.position.z = 4;
camGroup.add(lensMesh);

// LED indicator (small red sphere)
const ledMat = new THREE.MeshPhongMaterial({ color: 0xff0000, emissive: 0x330000 });
const ledMesh = new THREE.Mesh(new THREE.SphereGeometry(0.8, 6, 6), ledMat);
ledMesh.position.set(4, 3, 3);
camGroup.add(ledMesh);

// FOV frustum wireframe — shows camera field of view
const frustumLen = 120;
const frustumHalfW = Math.tan(CAM_HFOV_RAD / 2) * frustumLen;
const frustumHalfH = Math.tan(CAM_VFOV_RAD / 2) * frustumLen;

const frustumVerts = new Float32Array([
    // 4 lines from apex to corners
    0, 0, 0,   frustumHalfW,  frustumHalfH, frustumLen,
    0, 0, 0,  -frustumHalfW,  frustumHalfH, frustumLen,
    0, 0, 0,   frustumHalfW, -frustumHalfH, frustumLen,
    0, 0, 0,  -frustumHalfW, -frustumHalfH, frustumLen,
    // Rectangle at far end
     frustumHalfW,  frustumHalfH, frustumLen,  -frustumHalfW,  frustumHalfH, frustumLen,
    -frustumHalfW,  frustumHalfH, frustumLen,  -frustumHalfW, -frustumHalfH, frustumLen,
    -frustumHalfW, -frustumHalfH, frustumLen,   frustumHalfW, -frustumHalfH, frustumLen,
     frustumHalfW, -frustumHalfH, frustumLen,   frustumHalfW,  frustumHalfH, frustumLen,
]);
const frustumGeom = new THREE.BufferGeometry();
frustumGeom.setAttribute('position', new THREE.BufferAttribute(frustumVerts, 3));
const frustumLines = new THREE.LineSegments(frustumGeom,
    new THREE.LineBasicMaterial({ color: 0x44ff44, transparent: true, opacity: 0.3 })
);
camGroup.add(frustumLines);

// Semi-transparent FOV fill (4-sided pyramid)
const fillVerts = new Float32Array([
    // Triangle fan from apex to each edge of far rectangle
    0,0,0,  frustumHalfW, frustumHalfH, frustumLen,  -frustumHalfW, frustumHalfH, frustumLen,
    0,0,0, -frustumHalfW, frustumHalfH, frustumLen,  -frustumHalfW,-frustumHalfH, frustumLen,
    0,0,0, -frustumHalfW,-frustumHalfH, frustumLen,   frustumHalfW,-frustumHalfH, frustumLen,
    0,0,0,  frustumHalfW,-frustumHalfH, frustumLen,   frustumHalfW, frustumHalfH, frustumLen,
]);
const fillGeom = new THREE.BufferGeometry();
fillGeom.setAttribute('position', new THREE.BufferAttribute(fillVerts, 3));
const fillMat = new THREE.MeshBasicMaterial({
    color: 0x44ff44, transparent: true, opacity: 0.04,
    side: THREE.DoubleSide, depthWrite: false,
});
camGroup.add(new THREE.Mesh(fillGeom, fillMat));

// ============================================================================
// Simulated Camera Detection (pinhole projection of ball into camera view)
// ============================================================================

let camDetectionFrame = 0;

function simulateCameraDetection(ballPos) {
    // Get camera world position and orientation
    const camWorldPos = new THREE.Vector3();
    const camWorldQuat = new THREE.Quaternion();
    camGroup.getWorldPosition(camWorldPos);
    camGroup.getWorldQuaternion(camWorldQuat);

    // Ball position relative to camera
    const ballWorld = new THREE.Vector3(ballPos.x, ballPos.y, ballPos.z);
    const ballRel = ballWorld.clone().sub(camWorldPos);

    // Transform to camera local space (camera looks along +Z)
    const invQuat = camWorldQuat.clone().invert();
    const ballLocal = ballRel.clone().applyQuaternion(invQuat);

    // Ball behind camera
    if (ballLocal.z <= 0) return null;

    // Check horizontal and vertical FOV
    const angleH = Math.atan2(ballLocal.x, ballLocal.z);
    const angleV = Math.atan2(-ballLocal.y, ballLocal.z);
    if (Math.abs(angleH) > CAM_HFOV_RAD / 2 || Math.abs(angleV) > CAM_VFOV_RAD / 2) return null;

    // Pinhole projection to camera image pixels
    const cx = CAM_W / 2 + (ballLocal.x / ballLocal.z) * CAM_FOCAL_PX;
    const cy = CAM_H / 2 - (ballLocal.y / ballLocal.z) * CAM_FOCAL_PX;

    // Distance from camera to ball (sim units)
    const distSim = ballRel.length();
    const distMm = distSim * SIM_MM_PER_PX;

    // Pixel radius on camera image
    const ballSimRadius = ballRadius; // from physics ball
    const pixelRad = (ballSimRadius / ballLocal.z) * CAM_FOCAL_PX;

    // Blob area (circular approximation)
    const blobPixels = Math.round(Math.PI * pixelRad * pixelRad);

    // Offsets normalized to [-1, 1]
    const ox = (cx - CAM_W / 2) / (CAM_W / 2);
    const oy = (cy - CAM_H / 2) / (CAM_H / 2);

    // Bearing angle (degrees)
    const brgDeg = angleH * 180 / Math.PI;

    // Build UART packet (11 bytes)
    const det = 1;
    const pktX = Math.max(0, Math.min(65535, Math.round(cx)));
    const pktY = Math.max(0, Math.min(65535, Math.round(cy)));
    const pktR = Math.max(0, Math.min(65535, Math.round(distMm)));

    const pktBytes = [
        0xAA, 0x55, 0x01, det,
        (pktX >> 8) & 0xFF, pktX & 0xFF,
        (pktY >> 8) & 0xFF, pktY & 0xFF,
        (pktR >> 8) & 0xFF, pktR & 0xFF,
        0 // checksum placeholder
    ];
    let chk = 0;
    for (let i = 2; i < 10; i++) chk ^= pktBytes[i];
    pktBytes[10] = chk;

    const pktHex = pktBytes.map(b => b.toString(16).toUpperCase().padStart(2, '0')).join('');

    camDetectionFrame++;
    return {
        type: 'camera_detection',
        frame: camDetectionFrame,
        det: 1,
        cx: Math.round(cx),
        cy: Math.round(cy),
        blob: blobPixels,
        px_r: pixelRad,
        ox: parseFloat(ox.toFixed(3)),
        oy: parseFloat(oy.toFixed(3)),
        brg_deg: parseFloat(brgDeg.toFixed(2)),
        dist_mm: parseFloat(distMm.toFixed(1)),
        tof: 0,
        pkt: pktHex,
        pkt_decoded: {
            x: pktX,
            y: pktY,
            r: pktR,
            checksum: '0x' + chk.toString(16).toUpperCase().padStart(2, '0'),
        },
    };
}

// Send "not detected" frame
function sendNoDetection() {
    camDetectionFrame++;
    const pktBytes = [0xAA, 0x55, 0x01, 0, 0, 0, 0, 0, 0, 0, 0];
    let chk = 0;
    for (let i = 2; i < 10; i++) chk ^= pktBytes[i];
    pktBytes[10] = chk;
    const pktHex = pktBytes.map(b => b.toString(16).toUpperCase().padStart(2, '0')).join('');

    return {
        type: 'camera_detection',
        frame: camDetectionFrame,
        det: 0,
        cx: 0, cy: 0, blob: 0, px_r: 0,
        ox: 0, oy: 0, brg_deg: 0,
        dist_mm: 0, tof: 0,
        pkt: pktHex,
        pkt_decoded: { x: 0, y: 0, r: 0, checksum: '0x' + chk.toString(16).toUpperCase().padStart(2, '0') },
    };
}

// Throttle camera detection to ~20 FPS (every 3 render frames at 60fps)
let camTickCounter = 0;

// ============================================================================
// IK Cursor Gizmo
// ============================================================================

const SIM_LINK1 = 90;
const SIM_LINK2 = 75;

const ikCursorMat = new THREE.MeshPhongMaterial({ color: 0x00ffff, emissive: 0x003333, transparent: true, opacity: 0.9 });
const ikCursorMesh = new THREE.Mesh(new THREE.SphereGeometry(3, 10, 10), ikCursorMat);
ikCursorMesh.visible = false;
scene.add(ikCursorMesh);

// Axis handles: draggable cylinders with cone tips along X (red), Y (green), Z (blue)
const HANDLE_LEN = 28;
const HANDLE_RADIUS = 1.5;
const CONE_RADIUS = 3.5;
const CONE_LEN = 7;

function createAxisHandle(color, axis) {
    const group = new THREE.Group();
    const mat = new THREE.MeshPhongMaterial({ color, emissive: color, emissiveIntensity: 0.3 });
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(HANDLE_RADIUS, HANDLE_RADIUS, HANDLE_LEN, 8), mat);
    shaft.position.y = HANDLE_LEN / 2;
    group.add(shaft);
    const cone = new THREE.Mesh(new THREE.ConeGeometry(CONE_RADIUS, CONE_LEN, 8), mat);
    cone.position.y = HANDLE_LEN + CONE_LEN / 2;
    group.add(cone);
    // Orient: default is Y-up; rotate for X and Z
    if (axis === 'x') group.rotation.z = -Math.PI / 2;
    if (axis === 'z') group.rotation.x = Math.PI / 2;
    group.userData = { axis, mat, baseColor: color };
    return group;
}

const ikHandleX = createAxisHandle(0xff3333, 'x');
const ikHandleY = createAxisHandle(0x33ff33, 'y');
const ikHandleZ = createAxisHandle(0x3388ff, 'z');
ikCursorMesh.add(ikHandleX);
ikCursorMesh.add(ikHandleY);
ikCursorMesh.add(ikHandleZ);

// Collect handle meshes for raycasting (need the actual meshes, not groups)
const ikHandleMeshes = [];
[ikHandleX, ikHandleY, ikHandleZ].forEach(h => {
    h.children.forEach(child => {
        child.userData.axis = h.userData.axis;
        ikHandleMeshes.push(child);
    });
});

// Line from arm base to IK cursor
const ikLineMat = new THREE.LineDashedMaterial({ color: 0x00ffff, dashSize: 5, gapSize: 3, transparent: true, opacity: 0.5 });
const ikLineGeom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
const ikLine = new THREE.Line(ikLineGeom, ikLineMat);
ikLine.computeLineDistances();
ikLine.visible = false;
scene.add(ikLine);

// Workspace sphere (wireframe showing max reach)
const workspaceSphere = new THREE.Mesh(
    new THREE.SphereGeometry(SIM_LINK1 + SIM_LINK2, 16, 12),
    new THREE.MeshBasicMaterial({ color: 0x00ffff, wireframe: true, transparent: true, opacity: 0.08 })
);
workspaceSphere.visible = false;
scene.add(workspaceSphere);

// IK Solver — uses Three.js Matrix4 to properly account for arm mount transform
// (body position/yaw, arm offset, 45° tilt, +PI base offset, +60° elbow bias)
function ikSolveFromWorld(cursorWorld) {
    // Build static arm mount matrix: body transform * arm offset * tilt
    // This does NOT include any servo rotations — just the fixed geometry
    const bodyMatrix = new THREE.Matrix4();
    bodyMatrix.makeRotationY(robotPos.yaw);
    bodyMatrix.setPosition(body.position.x, body.position.y, body.position.z);

    const armOffsetMatrix = new THREE.Matrix4();
    const tiltMatrix = new THREE.Matrix4().makeRotationX(-Math.PI / 4);
    const transMatrix = new THREE.Matrix4().makeTranslation(0, 8, 25);
    armOffsetMatrix.multiplyMatrices(transMatrix, tiltMatrix);

    const mountMatrix = new THREE.Matrix4();
    mountMatrix.multiplyMatrices(bodyMatrix, armOffsetMatrix);

    // Invert to transform world → arm-local (pre-servo) space
    const invMount = mountMatrix.clone().invert();
    const local = cursorWorld.clone().applyMatrix4(invMount);

    // Subtract shoulder pivot offset (shoulderGroup is at y=4 in armBaseGroup)
    local.y -= 4;

    // Base servo angle from XZ plane in arm-local space
    // rotation.y = (base - 90) * PI/180 + PI, so pointing at (lx, lz):
    // rotation.y = atan2(lx, lz), therefore base = (atan2(lx,lz) - PI) * 180/PI + 90
    let baseDeg = (Math.atan2(local.x, local.z) - Math.PI) * 180 / Math.PI + 90;
    if (baseDeg < 0) baseDeg += 360;
    baseDeg = Math.max(0, Math.min(180, baseDeg));

    // Radial distance from Y axis and height along Y
    const r = Math.sqrt(local.x * local.x + local.z * local.z);
    const h = local.y;
    const dist = Math.sqrt(r * r + h * h);

    if (dist > SIM_LINK1 + SIM_LINK2 || dist < Math.abs(SIM_LINK1 - SIM_LINK2) || dist < 1) {
        return null; // unreachable
    }

    // 2-link planar IK in the (radial, height) plane
    // Interior elbow angle γ via law of cosines
    const cosGamma = (SIM_LINK1 * SIM_LINK1 + SIM_LINK2 * SIM_LINK2 - dist * dist) / (2 * SIM_LINK1 * SIM_LINK2);
    const gamma = Math.acos(Math.max(-1, Math.min(1, cosGamma)));

    // Shoulder triangle offset β
    const cosBeta = (SIM_LINK1 * SIM_LINK1 + dist * dist - SIM_LINK2 * SIM_LINK2) / (2 * SIM_LINK1 * dist);
    const beta = Math.acos(Math.max(-1, Math.min(1, cosBeta)));

    // Shoulder geometric angle from +Y axis toward radial (elbow-up solution)
    const shoulderRad = Math.atan2(r, h) - beta;
    // Map to servo: shoulderRad = (servo - 90) * PI/180
    const shoulderDeg = shoulderRad * 180 / Math.PI + 90;

    // Elbow bend = PI - γ
    // Map to servo: (servo - 90 + 60) * PI/180 = PI - γ → servo = (PI - γ) * 180/PI + 30
    const elbowDeg = (Math.PI - gamma) * 180 / Math.PI + 30;

    return {
        base: Math.max(0, Math.min(180, baseDeg)),
        shoulder: Math.max(0, Math.min(180, shoulderDeg)),
        elbow: Math.max(0, Math.min(180, elbowDeg)),
    };
}

window.IKCursor = {
    active: false,
    dragging: false,
    worldPos: new THREE.Vector3(0, 100, 0),
};

// ============================================================================
// Resize handler
// ============================================================================

function onResize() {
    const w = container.clientWidth;
    const h = container.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
}
window.addEventListener('resize', onResize);
setTimeout(onResize, 100);

// ============================================================================
// State management
// ============================================================================

let latestState = null;
SimState.onUpdate((state) => { latestState = state; });

const smoothed = {
    hip: [90, 90, 90, 90, 90, 90],
    knee: [90, 90, 90, 90, 90, 90],
    arm: { base: 90, shoulder: 90, elbow: 90, gripper: 0 }
};

function lerp(start, end, amt) {
    return (1 - amt) * start + amt * end;
}

// ============================================================================
// Manual Arm Override
// ============================================================================

window.ArmControl = {
    override: false,
    base: 90, shoulder: 90, elbow: 90, gripper: 0
};

// ============================================================================
// Client-Side Gait Control
// ============================================================================

window.GaitControl = {
    active: false,
    command: 'stop',
    speed: 0.5,
    gait: 0,   // 0=tripod, 1=wave, 2=ripple
    phase: 0,
};

function generateGaitAngles() {
    const gc = window.GaitControl;
    if (!gc.active || gc.command === 'stop') return null;

    const speed = gc.speed;
    const timeScale = window.TimeScale || 1.0;
    gc.phase = (gc.phase + speed * 0.025 * timeScale) % 1.0;

    const hip = new Array(6);
    const knee = new Array(6);
    const sweepRange = 20 + speed * 15;
    const phase = gc.phase;
    const gaitType = gc.gait;

    for (let i = 0; i < 6; i++) {
        let legPhase;

        if (gaitType === 0) {
            // Tripod: FR(0)+RR(2)+ML(4) vs MR(1)+RL(3)+FL(5)
            const isGroupA = (i === 0 || i === 2 || i === 4);
            legPhase = isGroupA ? phase : (phase + 0.5) % 1.0;
        } else if (gaitType === 1) {
            // Wave: each leg offset by 1/6
            const order = [0, 5, 1, 4, 2, 3];
            legPhase = (phase + order[i] / 6) % 1.0;
        } else {
            // Ripple: pairs offset by 1/3
            const order = [0, 3, 1, 4, 2, 5];
            legPhase = (phase + Math.floor(order[i] / 2) / 3) % 1.0;
        }

        const swingDuty = gaitType === 1 ? 0.167 : 0.5;
        const isSwing = legPhase < swingDuty;
        const subPhase = isSwing ? legPhase / swingDuty : (legPhase - swingDuty) / (1.0 - swingDuty);

        // Firmware servo conventions:
        //   hip_direction  = {-1,-1,-1, 1, 1, 1}  (left/right servos mounted mirrored)
        //   knee_direction = { 1, 1, 1,-1,-1,-1}
        //   forward:    all legs direction=+1, combined with hip_dir gives opposite hip for L/R
        //   turn_right: right dir=-1, left dir=+1, combined with hip_dir gives SAME hip for all
        const hipDir = i < 3 ? -1 : 1;   // firmware hip_direction table

        if (gc.command === 'forward' || gc.command === 'backward') {
            const dir = gc.command === 'forward' ? 1 : -1;
            const netDir = dir * hipDir;
            if (isSwing) {
                hip[i] = 90 + netDir * sweepRange * (subPhase * 2 - 1);
                knee[i] = 65;  // lifted (uniform, matches firmware)
            } else {
                hip[i] = 90 + netDir * sweepRange * (1 - subPhase * 2);
                knee[i] = 90;  // stance (uniform, matches firmware)
            }
        } else if (gc.command === 'turn_left' || gc.command === 'turn_right') {
            // For turning, direction per side: turn_right = right:-1, left:+1
            const turnBase = gc.command === 'turn_right' ? -1 : 1;
            const legDir = i < 3 ? turnBase : -turnBase;
            const netDir = legDir * hipDir;
            if (isSwing) {
                hip[i] = 90 + netDir * sweepRange * (subPhase * 2 - 1);
                knee[i] = 65;
            } else {
                hip[i] = 90 + netDir * sweepRange * (1 - subPhase * 2);
                knee[i] = 90;
            }
        } else {
            hip[i] = 90;
            knee[i] = 90;
        }
    }

    return { hip, knee };
}

// ============================================================================
// Update Robot
// ============================================================================

function updateRobot(state) {
    if (!state) return;

    let sum_dx_r = 0, sum_dx_l = 0;
    let count_r = 0, count_l = 0;

    // Check for client-side gait override
    const gaitOverride = generateGaitAngles();

    // Effective leg data: override if gait active, else firmware
    const legData = gaitOverride || (state.legs ? { hip: state.legs.hip, knee: state.legs.knee } : null);

    if (legData) {
        for (let i = 0; i < 6; i++) {
            const targetHip = legData.hip[i] !== undefined ? legData.hip[i] : 90;
            const targetKnee = legData.knee[i] !== undefined ? legData.knee[i] : 90;

            smoothed.hip[i] = lerp(smoothed.hip[i], targetHip, 0.1);
            smoothed.knee[i] = lerp(smoothed.knee[i], targetKnee, 0.1);

            const leg = legs[i];
            const hipAngle = smoothed.hip[i];
            const kneeAngle = smoothed.knee[i];

            // Odometry — firmware knee values are uniform (swing ≈ 65, stance ≈ 90)
            const isSwing = kneeAngle < 85;
            if (!isSwing && lastLegs) {
                const prevHipAngle = lastLegs.hip[i];
                let dAngleDeg = hipAngle - prevHipAngle;
                const isRight = i < 3;

                if (Math.abs(dAngleDeg) < 10) {
                    let forwardSweep = isRight ? dAngleDeg : -dAngleDeg;
                    const ROUGH_RADIUS = HIP_LEN_FLAT + HIP_LEN_DIAG * Math.cos(0.5);
                    const FRICTION_SCALAR = 0.5;
                    let dx = (forwardSweep * Math.PI / 180) * ROUGH_RADIUS * FRICTION_SCALAR;

                    if (isRight) { sum_dx_r += dx; count_r++; }
                    else { sum_dx_l += dx; count_l++; }
                }
            }

            // Hip: negate offset to match physical servo mounting direction
            const hipOffset = (hipAngle - 90) * Math.PI / 180;
            leg.hipGroup.rotation.y = leg.baseAngle - hipOffset;

            // Knee: firmware sends uniform values; hipGroup PI rotation handles L/R mirroring
            const kneeRad = -(kneeAngle - 90 + 35) * Math.PI / 180;
            leg.kneeGroup.rotation.z = kneeRad;

            const staticFlare = 0.65;
            const hipDiagTilt = 0.7;
            leg.lowerShinGroup.rotation.z = -(kneeRad - 0.15 + hipDiagTilt + staticFlare);

            const mat = isSwing ? legSwingMat : legMat;
            leg.hipMesh1.material = mat;
            leg.hipMesh2.material = mat;
            leg.upperMesh.material = mat;
            leg.lowerMesh.material = mat;
            leg.footMesh.material = mat;
        }

        const dx_r = count_r > 0 ? sum_dx_r / count_r : 0;
        const dx_l = count_l > 0 ? sum_dx_l / count_l : 0;
        const dz = (dx_r + dx_l) / 2;
        const dyaw = (dx_r - dx_l) / 40;

        robotPos.yaw += dyaw;
        robotPos.x -= dz * Math.sin(robotPos.yaw);
        robotPos.z -= dz * Math.cos(robotPos.yaw);

        lastLegs = { hip: [...smoothed.hip], knee: [...smoothed.knee] };
    }

    body.position.set(robotPos.x, robotPos.y, robotPos.z);
    body.rotation.y = robotPos.yaw;

    // Dynamic Gravity
    body.updateMatrixWorld(true);
    let lowestFootY = Infinity;
    for (let i = 0; i < 6; i++) {
        const worldPos = new THREE.Vector3();
        legs[i].footMesh.getWorldPosition(worldPos);
        if (worldPos.y < lowestFootY) lowestFootY = worldPos.y;
    }
    if (lowestFootY !== Infinity) {
        const targetBodyY = robotPos.y + (3 - lowestFootY);
        robotPos.y = lerp(robotPos.y, targetBodyY, 0.4);
        body.position.y = robotPos.y;
    }

    controls.target.set(robotPos.x, robotPos.y, robotPos.z);

    // Arm
    const armSource = window.ArmControl.override
        ? { base: window.ArmControl.base, shoulder: window.ArmControl.shoulder, elbow: window.ArmControl.elbow, gripper: window.ArmControl.gripper }
        : (window.IKCursor.active ? null : state.arm);

    // IK cursor: continuously track gripper when active and not dragging
    if (window.IKCursor.active && !window.IKCursor.dragging) {
        const gripWorld = new THREE.Vector3();
        gripperGroup.getWorldPosition(gripWorld);
        ikCursorMesh.position.copy(gripWorld);
        window.IKCursor.worldPos.copy(gripWorld);
    }

    if (armSource) {
        smoothed.arm.base = lerp(smoothed.arm.base, armSource.base !== undefined ? armSource.base : 90, 0.1);
        smoothed.arm.shoulder = lerp(smoothed.arm.shoulder, armSource.shoulder !== undefined ? armSource.shoulder : 90, 0.1);
        smoothed.arm.elbow = lerp(smoothed.arm.elbow, armSource.elbow !== undefined ? armSource.elbow : 90, 0.1);
        smoothed.arm.gripper = lerp(smoothed.arm.gripper, armSource.gripper !== undefined ? armSource.gripper : 0, 0.1);
    }

    const baseRad = (smoothed.arm.base - 90) * Math.PI / 180 + Math.PI;
    armBaseGroup.rotation.y = baseRad;

    const shoulderRad = (smoothed.arm.shoulder - 90) * Math.PI / 180;
    shoulderGroup.rotation.x = shoulderRad;

    const elbowRad = (smoothed.arm.elbow - 90 + 60) * Math.PI / 180;
    elbowGroup.rotation.x = elbowRad;

    const gripAngle = smoothed.arm.gripper / 180;
    const spread = 2 + gripAngle * 6;
    gripL.position.x = -spread;
    gripR.position.x = spread;

    window.ArmControl.currentBase = smoothed.arm.base;
    window.ArmControl.currentShoulder = smoothed.arm.shoulder;
    window.ArmControl.currentElbow = smoothed.arm.elbow;
    window.ArmControl.currentGripper = smoothed.arm.gripper;

    // Export effective smoothed state for debug view
    window.SmoothedState = {
        legs: { hip: [...smoothed.hip], knee: [...smoothed.knee] },
        arm: { base: smoothed.arm.base, shoulder: smoothed.arm.shoulder, elbow: smoothed.arm.elbow, gripper: smoothed.arm.gripper }
    };

    // Update IK cursor visuals
    if (window.IKCursor.active) {
        ikCursorMesh.visible = true;
        ikLine.visible = true;
        workspaceSphere.visible = true;

        // Position workspace sphere at arm base world position
        const armBaseWorld = new THREE.Vector3();
        armBaseGroup.getWorldPosition(armBaseWorld);
        workspaceSphere.position.copy(armBaseWorld);

        // Update line from arm base to cursor
        const positions = ikLine.geometry.attributes.position;
        positions.setXYZ(0, armBaseWorld.x, armBaseWorld.y, armBaseWorld.z);
        positions.setXYZ(1, ikCursorMesh.position.x, ikCursorMesh.position.y, ikCursorMesh.position.z);
        positions.needsUpdate = true;
        ikLine.computeLineDistances();

        // Check reachability - color the cursor
        const distToBase = ikCursorMesh.position.distanceTo(armBaseWorld);
        const reachable = distToBase <= (SIM_LINK1 + SIM_LINK2) && distToBase >= Math.abs(SIM_LINK1 - SIM_LINK2);
        ikCursorMat.color.setHex(reachable ? 0x00ffcc : 0xff3333);
        ikCursorMat.emissive.setHex(reachable ? 0x003322 : 0x330000);

        // Update readouts
        const relPos = ikCursorMesh.position.clone().sub(armBaseWorld);
        updateIKReadout(relPos, distToBase, reachable);
    } else {
        ikCursorMesh.visible = false;
        ikLine.visible = false;
        workspaceSphere.visible = false;
    }
}

function updateIKReadout(relPos, dist, reachable) {
    const el = document.getElementById('ik-readout');
    if (!el) return;
    el.innerHTML = `<div style="color:${reachable ? '#4ade80' : '#f87171'}">` +
        `X: ${relPos.x.toFixed(1)} Y: ${relPos.y.toFixed(1)} Z: ${relPos.z.toFixed(1)}</div>` +
        `<div>Dist: ${dist.toFixed(1)} / ${(SIM_LINK1 + SIM_LINK2).toFixed(0)}</div>` +
        `<div>B:${smoothed.arm.base.toFixed(0)}° S:${smoothed.arm.shoulder.toFixed(0)}° E:${smoothed.arm.elbow.toFixed(0)}°</div>`;
}

// ============================================================================
// Rigid Body Physics Engine
// ============================================================================

// Physics world: gravity in pixel-space (~9.81 m/s² * scale factor)
const physicsWorld = new PhysicsWorld({
    gravity: new Vec3(0, -9.81, 0),
    pixelScale: 50,  // 50 px ≈ 1 meter
    fixedTimeStep: 1 / 120,
    maxSubSteps: 4,
});

// Ground plane (static)
const groundBody = physicsWorld.addBody(new RigidBody({
    type: BODY_STATIC,
    shape: new PlaneShape(new Vec3(0, 1, 0), 0),
    friction: 0.6,
    restitution: 0.4,
}));

// Ball (dynamic rigid body)
const ballRadius = 8;
const ballGeom = new THREE.SphereGeometry(ballRadius, 16, 16);
const ballMat_color = new THREE.MeshPhongMaterial({ color: 0xe94560 });
const ballMesh = new THREE.Mesh(ballGeom, ballMat_color);
scene.add(ballMesh);

const ballBody = physicsWorld.addBody(new RigidBody({
    mass: 0.5,
    shape: new SphereShape(ballRadius),
    position: new Vec3(0, 150, -50),
    restitution: 0.15,   // low bounce — not a bouncy ball
    friction: 0.6,
    linearDamping: 0.04,
}));

// Ghost ball: shown on hover when ball-place-mode is active
const ghostMat = new THREE.MeshBasicMaterial({ color: 0xe94560, wireframe: true, transparent: true, opacity: 0.5 });
const ghostMesh = new THREE.Mesh(new THREE.SphereGeometry(ballRadius, 12, 12), ghostMat);
ghostMesh.visible = false;
scene.add(ghostMesh);

// Robot collider bodies (kinematic — updated from Three.js world positions each frame)
const robotColliders = [];

function createRobotCollider(mesh, radiusOverride) {
    if (!mesh.geometry.boundingSphere) mesh.geometry.computeBoundingSphere();
    const r = radiusOverride || mesh.geometry.boundingSphere.radius * 1.1;
    const rb = physicsWorld.addBody(new RigidBody({
        type: BODY_KINEMATIC,
        shape: new SphereShape(r),
        restitution: 0.3,
        friction: 0.5,
        userData: mesh,
    }));
    robotColliders.push(rb);
    return rb;
}

// Body collider
createRobotCollider(body, 30);

// Leg segment colliders
for (const leg of legs) {
    createRobotCollider(leg.hipMesh1);
    createRobotCollider(leg.hipMesh2);
    createRobotCollider(leg.upperMesh);
    createRobotCollider(leg.lowerMesh);
    createRobotCollider(leg.footMesh);
}

// Arm colliders
createRobotCollider(armBaseMesh);
createRobotCollider(shoulderMesh);
createRobotCollider(forearmMesh);
createRobotCollider(gripL);
createRobotCollider(gripR);

// Camera collider
createRobotCollider(camBoxMesh, 6);

// Sync kinematic colliders to their Three.js mesh world positions
function syncRobotColliders() {
    const wp = new THREE.Vector3();
    for (const rb of robotColliders) {
        rb.userData.getWorldPosition(wp);
        rb.position.set(wp.x, wp.y, wp.z);
    }
}

// Ball placement mode
window.BallPlaceMode = false;
window.HideBallGhost = () => { ghostMesh.visible = false; };

window.PhysicsBall = {
    get x() { return ballBody.position.x; },
    set x(v) { ballBody.position.x = v; },
    get y() { return ballBody.position.y; },
    set y(v) { ballBody.position.y = v; },
    get z() { return ballBody.position.z; },
    set z(v) { ballBody.position.z = v; },
    get vx() { return ballBody.velocity.x; },
    set vx(v) { ballBody.velocity.x = v; },
    get vy() { return ballBody.velocity.y; },
    set vy(v) { ballBody.velocity.y = v; },
    get vz() { return ballBody.velocity.z; },
    set vz(v) { ballBody.velocity.z = v; },
    active: false,
    dragging: false,
    drop: function (worldX, worldZ) {
        if (worldX !== undefined && worldZ !== undefined) {
            ballBody.position.set(worldX, 150, worldZ);
        } else {
            ballBody.position.set(
                robotPos.x + Math.sin(robotPos.yaw) * 50,
                150,
                robotPos.z + Math.cos(robotPos.yaw) * -50
            );
        }
        ballBody.velocity.set((Math.random() - 0.5) * 1, 0, (Math.random() - 0.5) * 1);
        ballBody.angularVelocity = new Vec3();
        this.active = true;
        this.dragging = false;
    },
    reset: function () {
        ballBody.position.set(0, ballRadius, 0);
        ballBody.velocity.set(0, 0, 0);
        ballBody.angularVelocity = new Vec3();
        this.active = true;
        this.dragging = false;
    },
    setPosition: function (x, y, z) {
        ballBody.position.set(x, y, z);
        ballBody.velocity.set(0, 0, 0);
        ballBody.angularVelocity = new Vec3();
        this.active = true;
        this.dragging = false;
    }
};

// Step physics and sync Three.js visuals
function updatePhysics() {
    if (!window.PhysicsBall.active) return;

    const ts = window.TimeScale || 1.0;

    if (window.PhysicsBall.dragging) {
        // While dragging, freeze physics — just sync mesh
        ballBody.velocity.set(0, 0, 0);
        ballMesh.position.set(ballBody.position.x, ballBody.position.y, ballBody.position.z);
        return;
    }

    // Update kinematic robot colliders from Three.js world transforms
    syncRobotColliders();

    // Step physics world (dt scaled by time scale)
    physicsWorld.step((1 / 60) * ts);

    // Sync ball mesh from physics body
    ballMesh.position.set(ballBody.position.x, ballBody.position.y, ballBody.position.z);

    // Spin ball from angular velocity
    const av = ballBody.angularVelocity;
    ballMesh.rotation.x += av.x * (1 / 60);
    ballMesh.rotation.y += av.y * (1 / 60);
    ballMesh.rotation.z += av.z * (1 / 60);
}

// ============================================================================
// Pointer Events (IK cursor, ball drag, ball placement)
// ============================================================================

const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();
let dragPlaneY = 0;
let lastDragPos = { x: 0, z: 0 };
let dragVel = { x: 0, z: 0 };
let activeDrag = null; // 'ball', 'ik-x', 'ik-y', 'ik-z', 'ik', or null
let ikDragAxis = null; // THREE.Vector3 for axis-constrained drag direction
let ikDragOrigin = null; // cursor position at drag start
let ikLastMouse = { x: 0, y: 0 }; // previous mouse NDC for delta-based axis drag

// Helper: find closest point on a line to a ray (for axis-constrained dragging)

renderer.domElement.addEventListener('pointerdown', (e) => {
    const rect = renderer.domElement.getBoundingClientRect();
    mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(mouse, camera);

    // Ball placement mode: click ground to place ball
    if (window.BallPlaceMode) {
        const groundHits = raycaster.intersectObject(groundPlane);
        if (groundHits.length > 0) {
            const pt = groundHits[0].point;
            ghostMesh.visible = false;
            window.PhysicsBall.drop(pt.x, pt.z);
            e.preventDefault();
            return;
        }
    }

    // IK cursor: check axis handles first (more specific), then center sphere
    if (window.IKCursor.active && ikCursorMesh.visible) {
        // Check axis handle meshes
        const handleHits = raycaster.intersectObjects(ikHandleMeshes, false);
        if (handleHits.length > 0) {
            const axis = handleHits[0].object.userData.axis;
            activeDrag = 'ik-' + axis;
            ikDragAxis = axis === 'x' ? new THREE.Vector3(1, 0, 0) :
                         axis === 'y' ? new THREE.Vector3(0, 1, 0) :
                                        new THREE.Vector3(0, 0, 1);
            ikDragOrigin = ikCursorMesh.position.clone();
            ikLastMouse = { x: mouse.x, y: mouse.y };
            window.IKCursor.dragging = true;
            controls.enabled = false;
            e.preventDefault();
            return;
        }
        // Check center sphere (free drag on XZ plane)
        const ikHits = raycaster.intersectObject(ikCursorMesh, false);
        if (ikHits.length > 0) {
            activeDrag = 'ik';
            window.IKCursor.dragging = true;
            dragPlaneY = ikCursorMesh.position.y;
            controls.enabled = false;
            e.preventDefault();
            return;
        }
    }

    // Ball drag
    if (window.PhysicsBall.active) {
        const ballHits = raycaster.intersectObject(ballMesh);
        if (ballHits.length > 0) {
            activeDrag = 'ball';
            window.PhysicsBall.dragging = true;
            window.PhysicsBall.vy = 0;
            window.PhysicsBall.vx = 0;
            window.PhysicsBall.vz = 0;
            dragPlaneY = window.PhysicsBall.y;
            lastDragPos = { x: window.PhysicsBall.x, z: window.PhysicsBall.z };
            dragVel = { x: 0, z: 0 };
            controls.enabled = false;
            e.preventDefault();
            return;
        }
    }
});

renderer.domElement.addEventListener('pointermove', (e) => {
    const rect = renderer.domElement.getBoundingClientRect();

    // Ghost ball placement preview (runs even when not dragging)
    if (window.BallPlaceMode && !activeDrag) {
        const mx = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        const my = -((e.clientY - rect.top) / rect.height) * 2 + 1;
        raycaster.setFromCamera(new THREE.Vector2(mx, my), camera);
        const groundHits = raycaster.intersectObject(groundPlane);
        if (groundHits.length > 0) {
            const pt = groundHits[0].point;
            ghostMesh.position.set(pt.x, ballRadius, pt.z);
            ghostMesh.visible = true;
        }
        return;
    }
    ghostMesh.visible = false;

    if (!activeDrag) return;
    mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(mouse, camera);

    if (activeDrag === 'ik-x' || activeDrag === 'ik-y' || activeDrag === 'ik-z') {
        // Delta-based axis drag: project the world axis onto screen space,
        // measure mouse delta along that screen direction, convert to world movement.
        // This works correctly at any camera angle without degenerate geometry.
        const ndcA = ikCursorMesh.position.clone().project(camera);
        const ndcB = ikCursorMesh.position.clone().addScaledVector(ikDragAxis, 10).project(camera);
        const screenDx = ndcB.x - ndcA.x;
        const screenDy = ndcB.y - ndcA.y;
        const screenLen = Math.sqrt(screenDx * screenDx + screenDy * screenDy);
        if (screenLen > 0.01) {
            const dMouseX = mouse.x - ikLastMouse.x;
            const dMouseY = mouse.y - ikLastMouse.y;
            // Component of mouse delta along the projected axis direction
            const t = (dMouseX * screenDx + dMouseY * screenDy) / screenLen;
            // Scale: 10 world units maps to screenLen NDC units
            const worldMove = t * (10 / screenLen);
            ikCursorMesh.position.addScaledVector(ikDragAxis, worldMove);
            if (activeDrag === 'ik-y') ikCursorMesh.position.y = Math.max(0, ikCursorMesh.position.y);
            ikDragOrigin.copy(ikCursorMesh.position);
            solveAndApplyIK();
        }
        ikLastMouse = { x: mouse.x, y: mouse.y };
        return;
    }

    // Project onto horizontal plane at drag height
    const planeNormal = new THREE.Vector3(0, 1, 0);
    const planePoint = new THREE.Vector3(0, dragPlaneY, 0);
    const plane = new THREE.Plane();
    plane.setFromNormalAndCoplanarPoint(planeNormal, planePoint);
    const intersect = new THREE.Vector3();
    raycaster.ray.intersectPlane(plane, intersect);
    if (!intersect) return;

    if (activeDrag === 'ball') {
        dragVel.x = intersect.x - lastDragPos.x;
        dragVel.z = intersect.z - lastDragPos.z;
        lastDragPos.x = intersect.x;
        lastDragPos.z = intersect.z;
        window.PhysicsBall.x = intersect.x;
        window.PhysicsBall.z = intersect.z;

        // Shift+drag to change height
        if (e.shiftKey) {
            window.PhysicsBall.y = Math.max(ballRadius, dragPlaneY + (mouse.y * 50));
        }
    } else if (activeDrag === 'ik') {
        // Free drag: move IK cursor on horizontal XZ plane
        ikCursorMesh.position.x = intersect.x;
        ikCursorMesh.position.z = intersect.z;
        solveAndApplyIK();
    }
});

function solveAndApplyIK() {
    window.IKCursor.worldPos.copy(ikCursorMesh.position);
    const solution = ikSolveFromWorld(ikCursorMesh.position.clone());
    if (solution) {
        window.ArmControl.override = true;
        window.ArmControl.base = solution.base;
        window.ArmControl.shoulder = solution.shoulder;
        window.ArmControl.elbow = solution.elbow;

        const baseSlider = document.getElementById('arm-base-slider');
        const shoulderSlider = document.getElementById('arm-shoulder-slider');
        const elbowSlider = document.getElementById('arm-elbow-slider');
        if (baseSlider) { baseSlider.value = solution.base; document.getElementById('arm-base-val').textContent = solution.base.toFixed(0); }
        if (shoulderSlider) { shoulderSlider.value = solution.shoulder; document.getElementById('arm-shoulder-val').textContent = solution.shoulder.toFixed(0); }
        if (elbowSlider) { elbowSlider.value = solution.elbow; document.getElementById('arm-elbow-val').textContent = solution.elbow.toFixed(0); }

        const toggle = document.getElementById('arm-override-toggle');
        if (toggle && !toggle.checked) toggle.checked = true;
    }
}

renderer.domElement.addEventListener('pointerup', () => {
    if (activeDrag === 'ball') {
        window.PhysicsBall.dragging = false;
        window.PhysicsBall.vx = dragVel.x * 3;
        window.PhysicsBall.vz = dragVel.z * 3;
        window.PhysicsBall.vy = 2;
    } else if (activeDrag && activeDrag.startsWith('ik')) {
        window.IKCursor.dragging = false;
    }
    activeDrag = null;
    ikDragAxis = null;
    ikDragOrigin = null;
    controls.enabled = true;
});

// ============================================================================
// Reset function (exposed globally for reset button)
// ============================================================================

window.resetRobotPosition = function() {
    robotPos.x = 0;
    robotPos.y = 30;
    robotPos.z = 0;
    robotPos.yaw = 0;
    lastLegs = null;
    for (let i = 0; i < 6; i++) {
        smoothed.hip[i] = 90;
        smoothed.knee[i] = 90;
    }
    smoothed.arm.base = 90;
    smoothed.arm.shoulder = 90;
    smoothed.arm.elbow = 90;
    smoothed.arm.gripper = 0;
    body.position.set(0, 30, 0);
    body.rotation.y = 0;
    controls.target.set(0, 30, 0);
    if (window.PhysicsBall) {
        window.PhysicsBall.active = false;
    }
};

// ============================================================================
// Render loop
// ============================================================================

function animate() {
    requestAnimationFrame(animate);

    const pane = document.getElementById('pane-view3d');
    if (pane && pane.classList.contains('active')) {
        updateRobot(latestState);
        updatePhysics();
        controls.update();
        renderer.render(scene, camera);

        // Export ball position for UI readout
        window.BallPosition = {
            x: ballBody.position.x,
            y: ballBody.position.y,
            z: ballBody.position.z,
        };

        // Simulated camera detection (~20 FPS = every 3 frames at 60fps)
        camTickCounter++;
        if (camTickCounter >= 3 && window.PhysicsBall.active) {
            camTickCounter = 0;
            const det = simulateCameraDetection(ballBody.position);
            if (det) {
                window.SimWS.send(det);
                window.LatestCamDetection = det;
            } else {
                const noDet = sendNoDetection();
                window.SimWS.send(noDet);
                window.LatestCamDetection = noDet;
            }
        } else if (!window.PhysicsBall.active && camTickCounter >= 3) {
            camTickCounter = 0;
            const noDet = sendNoDetection();
            window.SimWS.send(noDet);
            window.LatestCamDetection = noDet;
        }
    }
}
animate();
