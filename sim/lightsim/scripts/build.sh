#!/bin/bash
# Build the lightsim simulator
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "=== Building Lightsim ==="

# Build C simulator
mkdir -p "$PROJECT_DIR/build"
cd "$PROJECT_DIR/build"
cmake ..
make -j$(nproc)

echo ""
echo "Build complete: $PROJECT_DIR/build/talos_sim"

# Install Node.js dependencies if server exists
if [ -f "$PROJECT_DIR/server/package.json" ]; then
    echo ""
    echo "Installing Node.js dependencies..."
    cd "$PROJECT_DIR/server"
    npm install
fi

echo ""
echo "=== Build Done ==="
