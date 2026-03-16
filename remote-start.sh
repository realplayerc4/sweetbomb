#!/bin/bash

# Configuration
PROJECT_DIR="/home/jetson/sweetbomb"
NODE_BIN="/home/jetson/node-v18.20.7-linux-arm64/bin/node"
PYTHON_BIN="/usr/bin/python3"
BACKEND_PORT=8000
FRONTEND_PORT=5173

echo "🚀 Starting Sweetbomb Services Offline..."

# Kill old processes
pkill -f "uvicorn"
pkill -f "node server.cjs"

# Start Backend
echo "📡 Starting Backend on port $BACKEND_PORT..."
cd $PROJECT_DIR
nohup $PYTHON_BIN -m uvicorn main:combined_app --host 0.0.0.0 --port $BACKEND_PORT > backend.log 2>&1 &
BACKEND_PID=$!

# Start Frontend
echo "🎨 Starting Frontend on port $FRONTEND_PORT..."
cd $PROJECT_DIR/ui/frontend
nohup $NODE_BIN server.cjs > frontend.log 2>&1 &
FRONTEND_PID=$!

echo "✅ Services started!"
echo "Backend PID: $BACKEND_PID"
echo "Frontend PID: $FRONTEND_PID"
echo "Monitor logs with: tail -f backend.log $PROJECT_DIR/ui/frontend/frontend.log"
