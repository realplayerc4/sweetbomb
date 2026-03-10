#!/bin/bash

# 简单启动脚本 - 前后端一起启动

echo "🚀 启动 Sweetbomb 项目..."

# 清理旧进程
echo "🧹 清理旧进程..."
pkill -f "uvicorn" 2>/dev/null
pkill -f "vite" 2>/dev/null
sleep 1

# 获取脚本所在目录作为基础路径
BASE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$BASE_DIR"

# 启动后端 (在后台)
echo "📡 启动后端 (http://0.0.0.0:8000)..."
python3 -m uvicorn main:combined_app --host 0.0.0.0 --port 8000 --reload > "$BASE_DIR/logs/sweetbomb.log" 2>&1 &
BACKEND_PID=$!

# 等待后端启动
sleep 2

# 启动前端 (在后台)
echo "🎨 启动前端 (http://0.0.0.0:5173)..."
cd "$BASE_DIR/ui/frontend"
if [ ! -d "node_modules" ]; then
    echo "⚠️ node_modules 不存在，正在安装依赖..."
    npm install
fi
npm run dev -- --host 0.0.0.0 &
FRONTEND_PID=$!

echo ""
echo "✅ 服务已启动!"
echo "  - 前端: http://localhost:5173"
echo "  - 后端: http://localhost:8000"
echo ""
echo "按 Ctrl+C 停止所有服务"

# 等待用户按 Ctrl+C
trap "echo ''; echo '🛑 停止服务...'; kill $BACKEND_PID $FRONTEND_PID 2>/dev/null; exit 0" INT
wait
