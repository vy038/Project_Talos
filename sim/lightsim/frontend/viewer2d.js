/**
 * @file viewer2d.js
 * @brief Mode 2 - Top-down 2D canvas view of the hexapod
 *
 * Draws the robot body as a hexagon, legs as lines from body to computed
 * foot positions, and the arm as a chain of segments.
 */

(function() {
    const canvas = document.getElementById('canvas-2d');
    const ctx = canvas.getContext('2d');

    // Resize canvas to fill container
    function resize() {
        const parent = canvas.parentElement;
        canvas.width = parent.clientWidth;
        canvas.height = parent.clientHeight;
    }
    window.addEventListener('resize', resize);
    resize();

    // Robot geometry (approximate, in canvas pixels)
    const BODY_WIDTH = 40;
    const BODY_LENGTH = 80;
    const HIP_LENGTH = 40;     // hip segment length
    const UPPER_SHIN_LEN = 30; // upper knee angle length
    const LOWER_SHIN_LEN = 30; // lower leg straight down length
    const ARM_BASE_LEN = 30;
    const ARM_LINK1 = 120;
    const ARM_LINK2 = 105;

    // Leg attachment configuration (matches 3D view)
    // Legs: 0=FR, 1=MR, 2=RR, 3=RL, 4=ML, 5=FL
    const legAttach = [
        { x:  20, y: -30, angle:  -30 },  // 0: FR (30 deg forward)
        { x:  20, y:   0, angle:    0 },  // 1: MR (neutral)
        { x:  20, y:  30, angle:   30 },  // 2: RR (30 deg back)
        { x: -20, y:  30, angle:  150 },  // 3: RL (30 deg back)
        { x: -20, y:   0, angle:  180 },  // 4: ML (neutral)
        { x: -20, y: -30, angle:  210 },  // 5: FL (30 deg forward)
    ];

    // Colors
    const COLOR_BODY = '#16213e';
    const COLOR_BODY_STROKE = '#e94560';
    const COLOR_LEG_STANCE = '#4ade80';
    const COLOR_LEG_SWING = '#f87171';
    const COLOR_ARM = '#f59e0b';
    const COLOR_GRIPPER = '#a78bfa';
    const COLOR_TEXT = '#888';
    const COLOR_GROUND = '#0a0a1a';

    let latestState = null;

    SimState.onUpdate((state) => {
        latestState = state;
    });

    function drawRobot(state) {
        const cx = canvas.width / 2;
        const cy = canvas.height / 2;

        // Clear
        ctx.fillStyle = COLOR_GROUND;
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        // Grid
        ctx.strokeStyle = '#1a1a2e';
        ctx.lineWidth = 1;
        for (let x = 0; x < canvas.width; x += 40) {
            ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height); ctx.stroke();
        }
        for (let y = 0; y < canvas.height; y += 40) {
            ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(canvas.width, y); ctx.stroke();
        }

        // Draw body rectangle
        ctx.fillStyle = COLOR_BODY;
        ctx.fillRect(cx - BODY_WIDTH / 2, cy - BODY_LENGTH / 2, BODY_WIDTH, BODY_LENGTH);
        ctx.strokeStyle = COLOR_BODY_STROKE;
        ctx.lineWidth = 2;
        ctx.strokeRect(cx - BODY_WIDTH / 2, cy - BODY_LENGTH / 2, BODY_WIDTH, BODY_LENGTH);

        // Front indicator
        ctx.beginPath();
        ctx.moveTo(cx, cy - BODY_LENGTH / 2 + 5);
        ctx.lineTo(cx - 8, cy - BODY_LENGTH / 2 + 18);
        ctx.lineTo(cx + 8, cy - BODY_LENGTH / 2 + 18);
        ctx.closePath();
        ctx.fillStyle = COLOR_BODY_STROKE;
        ctx.fill();

        // Draw legs — prefer SmoothedState from viewer3d (includes client-side gait overrides)
        const ss = window.SmoothedState;
        const legSource = (ss && ss.legs) ? ss.legs : (state && state.legs ? state.legs : null);
        if (legSource) {
            for (let i = 0; i < 6; i++) {
                const attach = legAttach[i];

                const hipAngle = legSource.hip[i] !== undefined ? legSource.hip[i] : 90;
                const kneeAngle = legSource.knee[i] !== undefined ? legSource.knee[i] : 90;

                // Hip attachment point on body
                const hx = cx + attach.x;
                const hy = cy + attach.y;

                // Hip angle: 90=straight out, <90=forward, >90=backward
                // Convert to canvas angle relative to attachment direction
                const hipOffsetRad = (hipAngle - 90) * Math.PI / 180;
                const hipDir = attach.angle * Math.PI / 180 - hipOffsetRad;

                // End of hip segment (knee joint)
                const kx = hx + HIP_LENGTH * Math.cos(hipDir);
                const ky = hy + HIP_LENGTH * Math.sin(hipDir);

                const kneeRadOuter = (kneeAngle - 90 + 35) * Math.PI / 180;

                // Knee angle affects foot position (simplified 2D top-down projection)
                // Lower knee angle = foot lifted (swing phase)
                const isSwing = kneeAngle < 85;
                
                // The upper shin flares DIRECTLY OUTWARD from body (matching the 3D -0.75 rad Z flare).
                // In 2D top-down the upper shin terminates at a fixed outward offset from the knee endpoint.
                // Right side (0,1,2): flare goes further away from body center (subtract). Left side (3,4,5): add.
                const flareDir = hipDir + (i < 3 ? -0.75 : 0.75);
                const footExtension = UPPER_SHIN_LEN * (isSwing ? 1.0 : 0.65); // extended when swing, compact at neutral

                const fx = kx + footExtension * Math.cos(flareDir);
                const fy = ky + footExtension * Math.sin(flareDir);

                // Draw hip segment
                ctx.beginPath();
                ctx.moveTo(hx, hy);
                ctx.lineTo(kx, ky);
                ctx.strokeStyle = isSwing ? COLOR_LEG_SWING : COLOR_LEG_STANCE;
                ctx.lineWidth = 3;
                ctx.stroke();

                // Draw upper shin (angles outward)
                ctx.beginPath();
                ctx.moveTo(kx, ky);
                ctx.lineTo(fx, fy);
                ctx.strokeStyle = isSwing ? COLOR_LEG_SWING : COLOR_LEG_STANCE;
                ctx.lineWidth = 2;
                ctx.stroke();

                // The lower shin points straight down into the floor, so top-down it practically occupies the exact same 2D pixel as `fx, fy`!
                // We'll draw an Ankle dot and a Foot dot on top of each other, or slightly offset the Foot dot to hint at motion.
                const ankleX = fx;
                const ankleY = fy;

                // Foot dot
                ctx.beginPath();
                ctx.arc(ankleX, ankleY, 3, 0, Math.PI * 2);
                ctx.fillStyle = isSwing ? COLOR_LEG_SWING : COLOR_LEG_STANCE;
                ctx.fill();

                // Knee joint dot
                ctx.beginPath();
                ctx.arc(kx, ky, 2, 0, Math.PI * 2);
                ctx.fillStyle = '#666';
                ctx.fill();

                // Leg label
                ctx.fillStyle = COLOR_TEXT;
                ctx.font = '9px monospace';
                ctx.fillText(`L${i}`, hx - 6, hy - 8);
            }
        }

        // Draw arm — prefer SmoothedState (includes arm override and IK)
        const armSource = (ss && ss.arm) ? ss.arm : (state && state.arm ? state.arm : null);
        if (armSource) {
            const armBase = armSource.base !== undefined ? armSource.base : 90;
            const armShoulder = armSource.shoulder !== undefined ? armSource.shoulder : 90;
            const armElbow = armSource.elbow !== undefined ? armSource.elbow : 90;
            const armGripper = armSource.gripper !== undefined ? armSource.gripper : 0;

            // Arm base at back of body
            const abx = cx;
            const aby = cy + BODY_LENGTH / 2 - 10;

            // Base rotation (top-down): 90=forward, 0=right, 180=left
            // Reaches forward over the body, so neutral points UP (-Y).
            const baseRad = -(armBase - 90) * Math.PI / 180 - Math.PI / 2;

            // Shoulder segment
            const shoulderLen = ARM_LINK1 * (armShoulder / 180);
            const sx = abx + shoulderLen * Math.cos(baseRad);
            const sy = aby + shoulderLen * Math.sin(baseRad);

            // Elbow segment
            const elbowLen = ARM_LINK2 * (armElbow / 180);
            const ex = sx + elbowLen * Math.cos(baseRad);
            const ey = sy + elbowLen * Math.sin(baseRad);

            // Draw arm segments
            ctx.beginPath();
            ctx.moveTo(abx, aby);
            ctx.lineTo(sx, sy);
            ctx.strokeStyle = COLOR_ARM;
            ctx.lineWidth = 4;
            ctx.stroke();

            ctx.beginPath();
            ctx.moveTo(sx, sy);
            ctx.lineTo(ex, ey);
            ctx.strokeStyle = COLOR_ARM;
            ctx.lineWidth = 3;
            ctx.stroke();

            // Gripper
            const gripOpen = armGripper / 180;
            const gripSize = 6 + gripOpen * 8;
            ctx.beginPath();
            ctx.arc(ex, ey, gripSize, baseRad - 0.5, baseRad + 0.5);
            ctx.strokeStyle = COLOR_GRIPPER;
            ctx.lineWidth = 3;
            ctx.stroke();

            // Joint dots
            ctx.beginPath(); ctx.arc(abx, aby, 4, 0, Math.PI * 2);
            ctx.fillStyle = COLOR_ARM; ctx.fill();
            ctx.beginPath(); ctx.arc(sx, sy, 3, 0, Math.PI * 2);
            ctx.fillStyle = COLOR_ARM; ctx.fill();
        }

        // Draw Interactive Physics Ball if active
        if (window.PhysicsBall && window.PhysicsBall.active) {
            // Note: 3D coordinates (X, Z) map to 2D coordinates (X, Y) roughly based on robot center,
            // but the ball is world-space. Since the 2D view anchors the robot at the screen center,
            // we must render the ball perfectly relative to the robot's current position difference.
            // But since odometry physically translates robotPos in 3D, and the 2D view doesn't translate...
            // It's easiest to just draw the ball if it exists near the robot, ignoring absolute translation for simple 2D preview.
            
            // Simplified: only show ball in 3D for perfect fidelity.
        }

        // Info text
        ctx.fillStyle = COLOR_TEXT;
        ctx.font = '11px monospace';
        ctx.fillText('Top-Down View', 10, 20);

        if (state && state.legs) {
            const hip = state.legs.hip || [];
            const knee = state.legs.knee || [];
            for (let i = 0; i < 6; i++) {
                ctx.fillText(
                    `L${i}: hip=${(hip[i]||0).toFixed(0)}° knee=${(knee[i]||0).toFixed(0)}°`,
                    10, 40 + i * 14
                );
            }
        }

        if (state && state.arm) {
            ctx.fillText(
                `Arm: B=${state.arm.base.toFixed(0)}° S=${state.arm.shoulder.toFixed(0)}° E=${state.arm.elbow.toFixed(0)}° G=${state.arm.gripper.toFixed(0)}°`,
                10, 130
            );
        }
    }

    // Render loop
    function animate() {
        // Only render when 2D tab is active
        if (document.getElementById('pane-view2d') && document.getElementById('pane-view2d').classList.contains('active')) {
            drawRobot(latestState);
        }
        requestAnimationFrame(animate);
    }
    animate();
})();
