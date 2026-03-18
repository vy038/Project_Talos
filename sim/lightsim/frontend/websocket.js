/**
 * @file websocket.js
 * @brief WebSocket client - connects to bridge server and dispatches state updates
 */

const SimState = {
    data: null,
    listeners: [],
    frameCount: 0,
    lastFpsTime: Date.now(),

    onUpdate(callback) {
        this.listeners.push(callback);
    },

    dispatch(state) {
        this.data = state;
        this.frameCount++;
        for (const cb of this.listeners) {
            cb(state);
        }
    }
};

(function connectWebSocket() {
    const wsUrl = `ws://${window.location.host}`;
    const statusEl = document.getElementById('ws-status');
    const fpsEl = document.getElementById('fps-counter');

    let ws = null;

    function connect() {
        ws = new WebSocket(wsUrl);

        ws.onopen = () => {
            statusEl.textContent = 'ONLINE';
            statusEl.className = 'connected';
            const dot = document.getElementById('ws-dot');
            if (dot) dot.className = 'status-dot connected';
        };

        ws.onmessage = (event) => {
            try {
                const msg = JSON.parse(event.data);

                if (msg.type === 'log') {
                    // Dispatch serial log as a DOM event for debugview
                    window.dispatchEvent(new CustomEvent('sim-log', { detail: msg.message }));
                } else {
                    // Normal state update
                    SimState.dispatch(msg);
                }
            } catch (e) {
                // Ignore malformed messages
            }
        };

        ws.onclose = () => {
            statusEl.textContent = 'OFFLINE';
            statusEl.className = 'disconnected';
            const dot = document.getElementById('ws-dot');
            if (dot) dot.className = 'status-dot';
            setTimeout(connect, 2000);
        };

        ws.onerror = () => {
            ws.close();
        };
    }

    connect();

    // FPS counter
    setInterval(() => {
        const now = Date.now();
        const elapsed = (now - SimState.lastFpsTime) / 1000;
        const fps = Math.round(SimState.frameCount / elapsed);
        fpsEl.textContent = `${fps} Hz`;
        SimState.frameCount = 0;
        SimState.lastFpsTime = now;
    }, 1000);
})();
