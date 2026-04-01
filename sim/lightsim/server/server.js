/**
 * @file server.js
 * @brief Bridge server for Lightsim
 *
 * - Spawns the C simulator as a child process
 * - Reads JSON lines from simulator's stdout
 * - Forwards state updates to connected browsers via WebSocket
 * - Serves the frontend static files
 * - Provides REST API for control inputs (sensor injection, recompile)
 */

const express = require('express');
const { WebSocketServer } = require('ws');
const { spawn, execSync } = require('child_process');
const path = require('path');
const http = require('http');
const readline = require('readline');
const net = require('net');

const INJECTION_PORT = 9998;

const PORT = process.env.PORT || 3000;
const CAM_WS_PORT = 8765;
const SIM_PATH = path.join(__dirname, '..', 'build', 'talos_sim');
const FRONTEND_PATH = path.join(__dirname, '..', 'frontend');
const ESPCAM_TESTS_PATH = path.join(__dirname, '..', '..', '..', 'firmware', 'espcam', 'tests');

// ============================================================================
// Express app + HTTP server
// ============================================================================

const app = express();
app.use(express.json());
app.use(express.static(FRONTEND_PATH));
app.use('/espcam', express.static(ESPCAM_TESTS_PATH));

const server = http.createServer(app);

// ============================================================================
// WebSocket server
// ============================================================================

const wss = new WebSocketServer({ server });
let latestState = null;

// Camera WebSocket server (port 8765) — feeds test_red_ball.html
const camWss = new WebSocketServer({ port: CAM_WS_PORT });
let camFrameCount = 0;

function broadcastCamera(data) {
    const json = JSON.stringify(data);
    for (const client of camWss.clients) {
        if (client.readyState === 1) client.send(json);
    }
}

wss.on('connection', (ws) => {
    console.log(`[WS] Client connected (total: ${wss.clients.size})`);

    // Send latest state immediately so client doesn't start blank
    if (latestState) {
        ws.send(latestState);
    }

    // Handle messages from browser (camera detection data, etc.)
    ws.on('message', (raw) => {
        try {
            const msg = JSON.parse(raw);
            if (msg.type && msg.type !== 'tof_update') {
                console.log(`[WS←Browser] type=${msg.type} pkt=${msg.pkt ? 'yes' : 'NO'}`);
            }
            if (msg.type === 'tof_update') {
                // Forward ToF distance to C sim so vl53l0x_stub returns correct value
                injWrite(JSON.stringify({ type: 'tof', mm: msg.tof_mm }) + '\n');
            } else if (msg.type === 'camera_detection') {
                // Forward simulated camera detection to camera WS (port 8765)
                camFrameCount++;
                msg.frame = camFrameCount;
                delete msg.type;
                broadcastCamera(msg);

                // Inject the pre-built UART packet directly into the C simulator.
                // viewer3d.js already built and checksummed msg.pkt (hex string).
                if (msg.pkt) {
                    const bytes = Buffer.from(msg.pkt, 'hex');
                    injWrite(bytes);
                    if (msg.det) {
                        console.log(`[CAM→SIM] Injected ${bytes.length}B: ${msg.pkt.substring(0,22)}... det=${msg.det} cx=${msg.cx} cy=${msg.cy}`);
                    }
                }
            }
        } catch (e) { /* ignore malformed */ }
    });

    ws.on('close', () => {
        console.log(`[WS] Client disconnected (total: ${wss.clients.size})`);
    });
});

function broadcast(jsonLine) {
    const data = jsonLine;
    for (const client of wss.clients) {
        if (client.readyState === 1) { // OPEN
            client.send(data);
        }
    }
}

// ============================================================================
// Simulator process management
// ============================================================================

let simProcess = null;

// ============================================================================
// TCP injection client — connects to the C sim's injection server (port 9998)
// ============================================================================

let injSocket = null;
let injReconnectTimer = null;

function injWrite(data) {
    if (injSocket && !injSocket.destroyed) {
        injSocket.write(data);
    }
}

function connectInjection() {
    if (injReconnectTimer) { clearTimeout(injReconnectTimer); injReconnectTimer = null; }
    if (injSocket && !injSocket.destroyed) { injSocket.removeAllListeners(); injSocket.destroy(); }

    injSocket = new net.Socket();
    injSocket.connect(INJECTION_PORT, '127.0.0.1', () => {
        console.log('[INJ] Connected to sim injection server');
    });
    injSocket.on('error', () => { /* sim not ready yet, will retry */ });
    injSocket.on('close', () => {
        injSocket = null;
        injReconnectTimer = setTimeout(connectInjection, 500);
    });
}

function killSimulator() {
    if (!simProcess) return;
    console.log('[SIM] Killing simulator...');
    if (injReconnectTimer) { clearTimeout(injReconnectTimer); injReconnectTimer = null; }
    if (injSocket) { injSocket.removeAllListeners(); injSocket.destroy(); injSocket = null; }
    simProcess.kill('SIGTERM');
    // Force kill after 500ms if SIGTERM didn't work
    const proc = simProcess;
    setTimeout(() => {
        try { proc.kill('SIGKILL'); } catch (e) { /* already dead */ }
    }, 500);
    simProcess = null;
}

function startSimulator() {
    if (simProcess) {
        killSimulator();
    }

    console.log(`[SIM] Starting: ${SIM_PATH} --json`);
    simProcess = spawn(SIM_PATH, ['--json'], {
        stdio: ['ignore', 'pipe', 'pipe'],
    });

    // Connect injection client after a short delay to let the C server socket bind
    setTimeout(connectInjection, 300);

    // Read JSON lines from stdout
    const rl = readline.createInterface({ input: simProcess.stdout });
    rl.on('line', (line) => {
        if (line.startsWith('{')) {
            latestState = line;
            broadcast(line);
        }
    });

    // Forward simulator stderr to our console AND to browser via WebSocket
    let stderrBuf = '';
    simProcess.stderr.on('data', (data) => {
        const text = data.toString();
        process.stderr.write(`[FW] ${text}`);

        // Buffer and split into lines (stderr can arrive in chunks)
        stderrBuf += text;
        const lines = stderrBuf.split('\n');
        stderrBuf = lines.pop(); // keep incomplete last line in buffer
        for (const line of lines) {
            if (line.trim()) {
                broadcast(JSON.stringify({ type: 'log', message: line }));
            }
        }
    });

    simProcess.on('exit', (code, signal) => {
        console.log(`[SIM] Process exited (code=${code}, signal=${signal})`);
        simProcess = null;
    });

    simProcess.on('error', (err) => {
        console.error(`[SIM] Failed to start: ${err.message}`);
        simProcess = null;
    });
}

// ============================================================================
// REST API
// ============================================================================

// Recompile and restart ("flash")
app.post('/api/compile', (req, res) => {
    console.log('[API] Recompile requested...');
    try {
        // Kill current sim
        killSimulator();

        // Rebuild
        const buildDir = path.join(__dirname, '..', 'build');
        execSync('cmake --build . -- -j$(nproc)', { cwd: buildDir, stdio: 'pipe', timeout: 120000 });
        console.log('[API] Build succeeded');

        // Restart
        startSimulator();
        res.json({ status: 'ok', message: 'Recompiled and restarted' });
    } catch (err) {
        console.error(`[API] Build failed: ${err.message}`);
        res.status(500).json({ status: 'error', message: err.stderr?.toString() || err.message });
    }
});

// Build and inject an 11-byte detection UART packet into the C simulator stdin.
// The C sim's stdin reader calls sim_uart_inject() which fills the virtual UART
// buffer, making it available to uart_cam_task's xUARTReadTimeout().
function injectDetectionPacket({ detected, ball_x, ball_y, ball_radius }) {
    const packet = Buffer.alloc(11);
    packet[0] = 0xAA;
    packet[1] = 0x55;
    packet[2] = 0x01;
    packet[3] = detected ? 1 : 0;
    packet[4] = (ball_x >> 8) & 0xFF;
    packet[5] =  ball_x       & 0xFF;
    packet[6] = (ball_y >> 8) & 0xFF;
    packet[7] =  ball_y       & 0xFF;
    packet[8] = (ball_radius >> 8) & 0xFF;
    packet[9] =  ball_radius       & 0xFF;
    let checksum = 0;
    for (let i = 2; i < 10; i++) checksum ^= packet[i];
    packet[10] = checksum;

    injWrite(packet);
    return true;
}

// Inject ball detection (simulated camera UART packet)
app.post('/api/inject/detection', (req, res) => {
    const { detected, ball_x, ball_y, ball_radius } = req.body;
    if (injectDetectionPacket({ detected, ball_x, ball_y, ball_radius })) {
        res.json({ status: 'ok' });
    } else {
        res.status(503).json({ status: 'error', message: 'Simulator not running' });
    }
});

// Movement command (from D-pad / keyboard)
let latestCommand = { type: 'move', command: 'stop', speed: 0, gait: 0 };
app.post('/api/command', (req, res) => {
    latestCommand = req.body;
    injWrite(JSON.stringify(req.body) + '\n');
    res.json({ status: 'ok' });
});

// Run test sequence
app.post('/api/test', (req, res) => {
    const testName = req.body.test || 'gait';
    injWrite(JSON.stringify({ type: 'test', test: testName }) + '\n');
    res.json({ status: 'ok', test: testName });
});

// Force state machine transition
app.post('/api/control/state', (req, res) => {
    // TODO: implement when stdin command protocol is added to simulator
    res.json({ status: 'ok', message: 'Not yet implemented' });
});

// Set simulated sensor values
app.post('/api/control/sensor', (req, res) => {
    // TODO: implement when stdin command protocol is added to simulator
    res.json({ status: 'ok', message: 'Not yet implemented' });
});

// Get current status
app.get('/api/status', (req, res) => {
    res.json({
        simulator: simProcess ? 'running' : 'stopped',
        clients: wss.clients.size,
        lastUpdate: latestState ? JSON.parse(latestState).timestamp_us : null,
    });
});

// ============================================================================
// Start
// ============================================================================

server.listen(PORT, () => {
    console.log(`\n=== Lightsim Server ===`);
    console.log(`Frontend: http://localhost:${PORT}`);
    console.log(`WebSocket: ws://localhost:${PORT}`);
    console.log(`Camera WS: ws://localhost:${CAM_WS_PORT}`);
    console.log(`API: http://localhost:${PORT}/api/status\n`);
    startSimulator();
});

// Cleanup on exit
process.on('SIGINT', () => {
    console.log('\nShutting down...');
    killSimulator();
    server.close();
    process.exit(0);
});

process.on('SIGTERM', () => {
    killSimulator();
    server.close();
    process.exit(0);
});
