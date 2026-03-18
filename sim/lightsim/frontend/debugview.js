/**
 * @file debugview.js
 * @brief Debug telemetry dashboard
 *
 * Panels: Servo bars, I2C bus monitor, IMU data, Leg angles, Arm angles, Terminal logs
 * Uses window.SmoothedState (from viewer3d.js) when available so servo bars
 * reflect client-side overrides (manual gait, arm override, IK).
 */

(function() {
    let container = document.getElementById('debug-main-content');
    if (!container) {
        container = document.createElement('div');
        container.id = 'debug-main-content';
        document.getElementById('pane-debug').querySelector('.pane-body').appendChild(container);
    }

    container.innerHTML = `
        <div class="debug-panel" style="grid-column: 1/2; grid-row: 1/2; overflow-y:auto;">
            <h3>All Servos</h3>
            <div id="all-servos-merged"></div>
        </div>
        <div class="debug-panel" style="grid-column: 2/3; grid-row: 1/2;">
            <h3>IMU / Sensors</h3>
            <div id="imu-data"></div>
        </div>
        <div class="debug-panel" style="grid-column: 1/2; grid-row: 2/3;">
            <h3>I2C Bus Monitor</h3>
            <div id="i2c-monitor" class="i2c-log"><span class="i2c-placeholder">Waiting for I2C traffic...</span></div>
        </div>
        <div class="debug-panel" style="grid-column: 2/3; grid-row: 2/3;">
            <h3>Serial Terminal</h3>
            <div id="serial-terminal" class="serial-log"></div>
        </div>
    `;

    const legNames = [
        'L0 FR hip', 'L0 FR knee',
        'L1 MR hip', 'L1 MR knee',
        'L2 RR hip', 'L2 RR knee',
        'L3 RL hip', 'L3 RL knee',
        'L4 ML hip', 'L4 ML knee',
        'L5 FL hip', 'L5 FL knee',
    ];

    const armNames = ['Base', 'Shoulder', 'Elbow', 'Gripper'];

    function createServoBar(label) {
        return `<div class="servo-bar-container">
            <span class="servo-bar-label">${label}</span>
            <div class="servo-bar"><div class="servo-bar-fill" style="width:50%"></div></div>
            <span class="servo-bar-value">90</span>
        </div>`;
    }

    // Build merged servo panel: leg + arm + raw channels all in one
    const mergedEl = document.getElementById('all-servos-merged');
    mergedEl.innerHTML =
        `<div class="servo-section-label">Legs</div><div id="leg-servos"></div>` +
        `<div class="servo-section-label" style="margin-top:4px;">Arm</div><div id="arm-servos"></div>` +
        `<div class="servo-section-label" style="margin-top:4px;">Raw Channels (0-15)</div><div id="all-servos"></div>`;

    const legEl = document.getElementById('leg-servos');
    legEl.innerHTML = legNames.map(n => createServoBar(n)).join('');
    const legBars = legEl.querySelectorAll('.servo-bar-fill');
    const legValues = legEl.querySelectorAll('.servo-bar-value');

    const armEl = document.getElementById('arm-servos');
    armEl.innerHTML = armNames.map(n => createServoBar(n)).join('');
    const armBars = armEl.querySelectorAll('.servo-bar-fill');
    const armValues = armEl.querySelectorAll('.servo-bar-value');

    const allEl = document.getElementById('all-servos');
    allEl.innerHTML = Array.from({length: 16}, (_, i) =>
        createServoBar(`ch${i}`)
    ).join('');
    const allBars = allEl.querySelectorAll('.servo-bar-fill');
    const allValues = allEl.querySelectorAll('.servo-bar-value');

    const i2cMonitor = document.getElementById('i2c-monitor');
    const i2cHistory = [];
    const MAX_I2C_ENTRIES = 100;
    let i2cHasData = false;

    const serialTerminal = document.getElementById('serial-terminal');
    const MAX_SERIAL_LINES = 200;

    function updateServoBar(bar, valueEl, angle) {
        const pct = (angle / 180) * 100;
        bar.style.width = pct + '%';
        valueEl.textContent = angle.toFixed(0);

        // Color: blue near neutral, warm at extremes
        const dist = Math.abs(angle - 90) / 90;
        const r = Math.round(59 * (1 - dist) + 239 * dist);
        const g = Math.round(130 * (1 - dist) + 68 * dist);
        const b = Math.round(246 * (1 - dist) + 68 * dist);
        bar.style.background = `rgb(${r},${g},${b})`;
    }

    SimState.onUpdate((state) => {
        if (!state) return;
        if (state.type === 'log') return;

        const ss = window.SmoothedState;

        // Leg servos
        const hip = (ss && ss.legs) ? ss.legs.hip : (state.legs ? state.legs.hip : []);
        const knee = (ss && ss.legs) ? ss.legs.knee : (state.legs ? state.legs.knee : []);
        for (let i = 0; i < 6; i++) {
            if (hip[i] !== undefined) updateServoBar(legBars[i*2], legValues[i*2], hip[i]);
            if (knee[i] !== undefined) updateServoBar(legBars[i*2+1], legValues[i*2+1], knee[i]);
        }

        // Arm servos
        const arm = (ss && ss.arm) ? ss.arm : state.arm;
        if (arm) {
            const vals = [arm.base, arm.shoulder, arm.elbow, arm.gripper];
            vals.forEach((v, i) => {
                if (v !== undefined) updateServoBar(armBars[i], armValues[i], v);
            });
        }

        // All body servos (raw channels 0-15)
        if (state.body_servos) {
            state.body_servos.forEach((v, i) => {
                updateServoBar(allBars[i], allValues[i], v);
            });
        }

        // IMU data
        if (state.imu) {
            const imuEl = document.getElementById('imu-data');
            const a = state.imu.accel || [0,0,0];
            const g = state.imu.gyro || [0,0,0];
            imuEl.innerHTML = `
                <div class="sensor-row"><span class="sensor-label">Accel X</span><span class="sensor-value">${a[0].toFixed(2)}</span></div>
                <div class="sensor-row"><span class="sensor-label">Accel Y</span><span class="sensor-value">${a[1].toFixed(2)}</span></div>
                <div class="sensor-row"><span class="sensor-label">Accel Z</span><span class="sensor-value">${a[2].toFixed(2)}</span></div>
                <div class="sensor-row" style="margin-top:4px;padding-top:4px;border-top:1px solid var(--border)"><span class="sensor-label">Gyro X</span><span class="sensor-value">${g[0].toFixed(2)}</span></div>
                <div class="sensor-row"><span class="sensor-label">Gyro Y</span><span class="sensor-value">${g[1].toFixed(2)}</span></div>
                <div class="sensor-row"><span class="sensor-label">Gyro Z</span><span class="sensor-value">${g[2].toFixed(2)}</span></div>
            `;
        }

        // I2C log
        if (state.i2c_log && state.i2c_log.length > 0) {
            if (!i2cHasData) {
                i2cMonitor.innerHTML = '';
                i2cHasData = true;
            }
            for (const entry of state.i2c_log) {
                const opClass = entry.op === 'W' ? 'op-w' : 'op-r';
                const html = `<div class="i2c-entry">
                    <span class="${opClass}">${entry.op}</span>
                    <span class="addr">${entry.addr}</span>
                    reg=${entry.reg}
                    ${entry.data ? entry.data : ''}
                </div>`;
                i2cHistory.push(html);
            }

            while (i2cHistory.length > MAX_I2C_ENTRIES) i2cHistory.shift();
            i2cMonitor.innerHTML = i2cHistory.join('');
            i2cMonitor.scrollTop = i2cMonitor.scrollHeight;
        }
    });

    // Serial terminal: listen for log messages
    window.addEventListener('sim-log', (e) => {
        const msg = e.detail;
        const line = document.createElement('div');
        line.className = 'serial-line';

        if (msg.includes('[E]') || msg.includes('ERROR')) {
            line.style.color = '#ef4444';
        } else if (msg.includes('[W]') || msg.includes('WARN')) {
            line.style.color = '#f59e0b';
        } else if (msg.includes('[I]') || msg.includes('INFO')) {
            line.style.color = '#22c55e';
        } else if (msg.includes('[D]') || msg.includes('DEBUG')) {
            line.style.color = '#5a6578';
        } else {
            line.style.color = '#c8cdd5';
        }

        line.textContent = msg;
        serialTerminal.appendChild(line);

        while (serialTerminal.children.length > MAX_SERIAL_LINES) {
            serialTerminal.removeChild(serialTerminal.firstChild);
        }
        serialTerminal.scrollTop = serialTerminal.scrollHeight;
    });
})();
