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

const PORT = process.env.PORT || 3000;
const SIM_PATH = path.join(__dirname, '..', 'build', 'talos_sim');
const FRONTEND_PATH = path.join(__dirname, '..', 'frontend');

// ============================================================================
// Express app + HTTP server
// ============================================================================

const app = express();
app.use(express.json());
app.use(express.static(FRONTEND_PATH));

const server = http.createServer(app);

// ============================================================================
// WebSocket server
// ============================================================================

const wss = new WebSocketServer({ server });
let latestState = null;

wss.on('connection', (ws) => {
    console.log(`[WS] Client connected (total: ${wss.clients.size})`);

    // Send latest state immediately so client doesn't start blank
    if (latestState) {
        ws.send(latestState);
    }

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

function killSimulator() {
    if (!simProcess) return;
    console.log('[SIM] Killing simulator...');
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
        stdio: ['pipe', 'pipe', 'pipe'],
    });

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
        execSync('cmake --build .', { cwd: buildDir, stdio: 'pipe', timeout: 30000 });
        console.log('[API] Build succeeded');

        // Restart
        startSimulator();
        res.json({ status: 'ok', message: 'Recompiled and restarted' });
    } catch (err) {
        console.error(`[API] Build failed: ${err.message}`);
        res.status(500).json({ status: 'error', message: err.stderr?.toString() || err.message });
    }
});

// Inject ball detection (simulated camera UART packet)
app.post('/api/inject/detection', (req, res) => {
    const { detected, ball_x, ball_y, ball_radius } = req.body;

    // Build the 11-byte detection packet per state_machine.h protocol
    const packet = Buffer.alloc(11);
    packet[0] = 0xAA;                          // start byte 0
    packet[1] = 0x55;                          // start byte 1
    packet[2] = 0x01;                          // type = detection
    packet[3] = detected ? 1 : 0;             // detected flag
    packet[4] = (ball_x >> 8) & 0xFF;         // x high
    packet[5] = ball_x & 0xFF;                // x low
    packet[6] = (ball_y >> 8) & 0xFF;         // y high
    packet[7] = ball_y & 0xFF;                // y low
    packet[8] = (ball_radius >> 8) & 0xFF;    // radius high
    packet[9] = ball_radius & 0xFF;           // radius low

    // Checksum: XOR of bytes 2-9
    let checksum = 0;
    for (let i = 2; i < 10; i++) checksum ^= packet[i];
    packet[10] = checksum;

    // Send to simulator's stdin
    if (simProcess && simProcess.stdin.writable) {
        simProcess.stdin.write(packet);
        res.json({ status: 'ok' });
    } else {
        res.status(503).json({ status: 'error', message: 'Simulator not running' });
    }
});

// Movement command (from D-pad / keyboard)
let latestCommand = { type: 'move', command: 'stop', speed: 0, gait: 0 };
app.post('/api/command', (req, res) => {
    latestCommand = req.body;
    // Write command to sim stdin if running
    if (simProcess && simProcess.stdin.writable) {
        try {
            simProcess.stdin.write(JSON.stringify(req.body) + '\n');
        } catch (e) { /* ignore */ }
    }
    res.json({ status: 'ok' });
});

// Run test sequence
app.post('/api/test', (req, res) => {
    const testName = req.body.test || 'gait';
    // Write test command to sim stdin if running
    if (simProcess && simProcess.stdin.writable) {
        try {
            simProcess.stdin.write(JSON.stringify({ type: 'test', test: testName }) + '\n');
        } catch (e) { /* ignore */ }
    }
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
