/**
 * @file server.js
 * @brief Bridge server for Lightsim
 *
 * - Spawns the C simulator as a child process per WebSocket session
 * - Reads JSON lines from simulator's stdout
 * - Forwards state updates to the owning browser client via WebSocket
 * - Serves the frontend static files
 * - Provides REST API for control inputs (sensor injection, recompile)
 *
 * Multi-user isolation: each WebSocket connection gets its own simulator
 * process on a unique TCP injection port. The browser receives a session ID
 * on connect and passes it as ?sid= in every REST API call.
 */

const express = require('express');
const { WebSocketServer } = require('ws');
const { spawn, execSync } = require('child_process');
const path = require('path');
const http = require('http');
const readline = require('readline');
const net = require('net');
const crypto = require('crypto');

const PORT = process.env.PORT || 3000;
const SIM_PATH = path.join(__dirname, '..', 'build', 'talos_sim');
const FRONTEND_PATH = path.join(__dirname, '..', 'frontend');
const ESPCAM_TESTS_PATH = path.join(__dirname, '..', '..', '..', 'firmware', 'espcam', 'tests');

// ============================================================================
// Session management
// ============================================================================

// Each entry: { sid, ws, simProcess, injSocket, injReconnectTimer, injPort, latestState }
const sessions = new Map();
let nextInjPort = 10000;

function allocPort() {
    return nextInjPort++;
}

function createSession(ws) {
    const sid = crypto.randomUUID();
    const session = {
        sid,
        ws,
        simProcess: null,
        injSocket: null,
        injReconnectTimer: null,
        injPort: allocPort(),
        latestState: null,
    };
    sessions.set(sid, session);
    ws.send(JSON.stringify({ type: 'session', sid }));
    console.log(`[SESSION] Created ${sid.slice(0, 8)} on inj-port ${session.injPort}`);
    return session;
}

function destroySession(sid) {
    const session = sessions.get(sid);
    if (!session) return;
    killSimulator(session);
    sessions.delete(sid);
    console.log(`[SESSION] Destroyed ${sid.slice(0, 8)}`);
}

function getSession(req) {
    const sid = req.query.sid || (req.body && req.body.sid);
    return sid ? sessions.get(sid) : null;
}

// ============================================================================
// Express app + HTTP server
// ============================================================================

const app = express();
app.use(express.json());
app.use(express.static(FRONTEND_PATH));
app.use('/espcam', express.static(ESPCAM_TESTS_PATH));

const server = http.createServer(app);

// ============================================================================
// WebSocket servers
// ============================================================================

const wss = new WebSocketServer({ noServer: true });

// Camera WebSocket server (path /camera) — feeds test_red_ball.html
const camWss = new WebSocketServer({ noServer: true });
let camFrameCount = 0;

server.on('upgrade', (request, socket, head) => {
    if (request.url === '/camera') {
        camWss.handleUpgrade(request, socket, head, (ws) => {
            camWss.emit('connection', ws, request);
        });
    } else {
        wss.handleUpgrade(request, socket, head, (ws) => {
            wss.emit('connection', ws, request);
        });
    }
});

function broadcastCamera(data) {
    const json = JSON.stringify(data);
    for (const client of camWss.clients) {
        if (client.readyState === 1) client.send(json);
    }
}

wss.on('connection', (ws) => {
    const session = createSession(ws);

    if (session.latestState) ws.send(session.latestState);

    ws.on('message', (raw) => {
        try {
            const msg = JSON.parse(raw);
            if (msg.type === 'tof_update') {
                injWrite(session, JSON.stringify({ type: 'tof', mm: msg.tof_mm }) + '\n');
            } else if (msg.type === 'camera_detection') {
                camFrameCount++;
                msg.frame = camFrameCount;
                delete msg.type;
                broadcastCamera(msg);
                if (msg.pkt) {
                    const bytes = Buffer.from(msg.pkt, 'hex');
                    injWrite(session, bytes);
                    if (msg.det) {
                        console.log(`[CAM→SIM] ${session.sid.slice(0,8)} injected ${bytes.length}B det=${msg.det}`);
                    }
                }
            }
        } catch (e) { /* ignore malformed */ }
    });

    ws.on('close', () => {
        console.log(`[WS] Disconnected ${session.sid.slice(0, 8)}`);
        destroySession(session.sid);
    });
});

// ============================================================================
// Per-session simulator management
// ============================================================================

function injWrite(session, data) {
    if (session.injSocket && !session.injSocket.destroyed) {
        session.injSocket.write(data);
    }
}

function connectInjection(session) {
    if (session.injReconnectTimer) { clearTimeout(session.injReconnectTimer); session.injReconnectTimer = null; }
    if (session.injSocket && !session.injSocket.destroyed) {
        session.injSocket.removeAllListeners();
        session.injSocket.destroy();
    }

    session.injSocket = new net.Socket();
    session.injSocket.connect(session.injPort, '127.0.0.1', () => {
        console.log(`[${session.sid.slice(0,8)}][INJ] Connected on port ${session.injPort}`);
    });
    session.injSocket.on('error', () => { /* sim not ready yet, will retry */ });
    session.injSocket.on('close', () => {
        session.injSocket = null;
        // Only retry if session still exists
        if (sessions.has(session.sid)) {
            session.injReconnectTimer = setTimeout(() => connectInjection(session), 500);
        }
    });
}

function killSimulator(session) {
    if (session.injReconnectTimer) { clearTimeout(session.injReconnectTimer); session.injReconnectTimer = null; }
    if (session.injSocket) { session.injSocket.removeAllListeners(); session.injSocket.destroy(); session.injSocket = null; }
    if (!session.simProcess) return;
    console.log(`[${session.sid.slice(0,8)}][SIM] Killing simulator`);
    session.simProcess.kill('SIGTERM');
    const proc = session.simProcess;
    setTimeout(() => { try { proc.kill('SIGKILL'); } catch (e) {} }, 500);
    session.simProcess = null;
}

function startSimulator(session) {
    if (session.simProcess) killSimulator(session);

    console.log(`[${session.sid.slice(0,8)}][SIM] Starting with inj-port ${session.injPort}`);
    session.simProcess = spawn(SIM_PATH, ['--json', '--inj-port', String(session.injPort)], {
        stdio: ['ignore', 'pipe', 'pipe'],
    });

    setTimeout(() => connectInjection(session), 300);

    const rl = readline.createInterface({ input: session.simProcess.stdout });
    rl.on('line', (line) => {
        if (line.startsWith('{')) {
            session.latestState = line;
            if (session.ws.readyState === 1) session.ws.send(line);
        }
    });

    let stderrBuf = '';
    session.simProcess.stderr.on('data', (data) => {
        stderrBuf += data.toString();
        const lines = stderrBuf.split('\n');
        stderrBuf = lines.pop();
        for (const line of lines) {
            if (line.trim() && session.ws.readyState === 1) {
                session.ws.send(JSON.stringify({ type: 'log', message: line }));
            }
        }
    });

    session.simProcess.on('exit', (code, signal) => {
        console.log(`[${session.sid.slice(0,8)}][SIM] Exited (code=${code}, signal=${signal})`);
        session.simProcess = null;
    });

    session.simProcess.on('error', (err) => {
        console.error(`[${session.sid.slice(0,8)}][SIM] Failed to start: ${err.message}`);
        session.simProcess = null;
    });
}

// ============================================================================
// REST API
// ============================================================================

// Shared compile lock — only one build at a time
let compiling = false;

app.post('/api/compile', (req, res) => {
    const session = getSession(req);
    if (!session) return res.status(400).json({ status: 'error', message: 'Missing or invalid ?sid=' });

    if (compiling) return res.status(409).json({ status: 'error', message: 'Build already in progress' });
    compiling = true;
    console.log(`[${session.sid.slice(0,8)}][API] Recompile requested`);

    // Kill only this session's simulator before rebuilding
    if (session.simProcess) {
        try { session.simProcess.kill('SIGKILL'); } catch (e) {}
        session.simProcess = null;
    }
    killSimulator(session);

    const projectDir = path.join(__dirname, '..');
    const buildDir   = path.join(projectDir, 'build');
    const { exec }   = require('child_process');
    const buildCmd   = `mkdir -p "${buildDir}" && cd "${buildDir}" && cmake .. && cmake --build . -j$(nproc)`;

    exec(buildCmd, { timeout: 120000 }, (error, stdout, stderr) => {
        compiling = false;
        if (error) {
            console.error(`[API] Build failed: ${error.message}`);
            return res.status(500).json({ status: 'error', message: stderr || error.message });
        }
        console.log('[API] Build succeeded');
        startSimulator(session);
        res.json({ status: 'ok', message: 'Compiled and started' });
    });
});

function injectDetectionPacket(session, { detected, ball_x, ball_y, ball_radius }) {
    const packet = Buffer.alloc(11);
    packet[0]  = 0xAA;
    packet[1]  = 0x55;
    packet[2]  = 0x01;
    packet[3]  = detected ? 1 : 0;
    packet[4]  = (ball_x >> 8) & 0xFF;
    packet[5]  =  ball_x       & 0xFF;
    packet[6]  = (ball_y >> 8) & 0xFF;
    packet[7]  =  ball_y       & 0xFF;
    packet[8]  = (ball_radius >> 8) & 0xFF;
    packet[9]  =  ball_radius       & 0xFF;
    let chk = 0;
    for (let i = 2; i < 10; i++) chk ^= packet[i];
    packet[10] = chk;
    injWrite(session, packet);
    return true;
}

app.post('/api/inject/detection', (req, res) => {
    const session = getSession(req);
    if (!session) return res.status(400).json({ status: 'error', message: 'Missing or invalid ?sid=' });
    const { detected, ball_x, ball_y, ball_radius } = req.body;
    injectDetectionPacket(session, { detected, ball_x, ball_y, ball_radius });
    res.json({ status: 'ok' });
});

app.post('/api/kill', (req, res) => {
    const session = getSession(req);
    if (!session) return res.status(400).json({ status: 'error', message: 'Missing or invalid ?sid=' });
    killSimulator(session);
    res.json({ status: 'ok', message: 'Simulator killed' });
});

app.post('/api/start', (req, res) => {
    const session = getSession(req);
    if (!session) return res.status(400).json({ status: 'error', message: 'Missing or invalid ?sid=' });
    startSimulator(session);
    res.json({ status: 'ok', message: 'Simulator started' });
});

app.post('/api/command', (req, res) => {
    const session = getSession(req);
    if (!session) return res.status(400).json({ status: 'error', message: 'Missing or invalid ?sid=' });
    injWrite(session, JSON.stringify(req.body) + '\n');
    res.json({ status: 'ok' });
});

app.post('/api/test', (req, res) => {
    const session = getSession(req);
    if (!session) return res.status(400).json({ status: 'error', message: 'Missing or invalid ?sid=' });
    const testName = req.body.test || 'gait';

    if (session.simProcess) {
        try { session.simProcess.kill('SIGKILL'); } catch (e) {}
        session.simProcess = null;
    }
    killSimulator(session);
    startSimulator(session);

    setTimeout(() => {
        injWrite(session, JSON.stringify({ type: 'test', test: testName }) + '\n');
    }, 1000);

    res.json({ status: 'ok', test: testName, message: 'Simulator wiped and test scheduled' });
});

app.post('/api/control/state', (req, res) => {
    res.json({ status: 'ok', message: 'Not yet implemented' });
});

app.post('/api/control/sensor', (req, res) => {
    res.json({ status: 'ok', message: 'Not yet implemented' });
});

app.get('/api/status', (req, res) => {
    const session = getSession(req);
    res.json({
        simulator: session && session.simProcess ? 'running' : 'stopped',
        sessions: sessions.size,
        lastUpdate: session && session.latestState ? JSON.parse(session.latestState).timestamp_us : null,
    });
});

// ============================================================================
// Start
// ============================================================================

server.listen(PORT, '0.0.0.0', () => {
    console.log(`\n=== Lightsim Server ===`);
    console.log(`Frontend: http://localhost:${PORT}`);
    console.log(`WebSocket: ws://localhost:${PORT} (Main), ws://localhost:${PORT}/camera (Camera)`);
    console.log(`API: http://localhost:${PORT}/api/status\n`);
    console.log('[SIM] Waiting for launch command from UI...');
});

process.on('SIGINT', () => {
    console.log('\nShutting down...');
    for (const [sid] of sessions) destroySession(sid);
    server.close();
    process.exit(0);
});

process.on('SIGTERM', () => {
    for (const [sid] of sessions) destroySession(sid);
    server.close();
    process.exit(0);
});
