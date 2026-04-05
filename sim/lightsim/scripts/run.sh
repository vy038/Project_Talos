#!/bin/bash
# Start the lightsim bridge server (compile via the UI button)
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "=== Starting Lightsim ==="

if [ -f "$PROJECT_DIR/server/server.js" ]; then
    # Run bridge server (which spawns the simulator)
    cd "$PROJECT_DIR/server"
    node server.js
else
    # Run simulator directly
    echo "Running simulator directly (no bridge server yet)..."
    "$PROJECT_DIR/build/talos_sim"
fi
