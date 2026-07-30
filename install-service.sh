#!/bin/bash

# Sweetbomb systemd 服务安装脚本
# 将服务文件复制到系统目录并启用开机自启动

echo "🚀 安装 Sweetbomb systemd 服务..."

# 检查是否有 root 权限
if [ "$EUID" -ne 0 ]; then
    echo "❌ 请使用 sudo 运行此脚本"
    exit 1
fi

# 复制服务文件到系统目录
echo "📦 复制服务文件..."
cp sweetbomb-backend.service /etc/systemd/system/
cp sweetbomb-frontend.service /etc/systemd/system/
cp sweetbomb-admin-frontend.service /etc/systemd/system/

# 重新加载 systemd 配置
echo "🔄 重新加载 systemd 配置..."
systemctl daemon-reload

# 启用服务（开机自启动）
echo "✅ 启用开机自启动..."
systemctl enable sweetbomb-backend.service
systemctl enable sweetbomb-frontend.service
systemctl enable sweetbomb-admin-frontend.service

# 启动服务
echo "🎯 启动服务..."
systemctl start sweetbomb-backend.service
systemctl start sweetbomb-frontend.service
systemctl start sweetbomb-admin-frontend.service

# 检查状态
echo ""
echo "📊 服务状态:"
systemctl status sweetbomb-backend.service --no-pager
echo ""
systemctl status sweetbomb-frontend.service --no-pager
echo ""
systemctl status sweetbomb-admin-frontend.service --no-pager

echo ""
echo "✅ 安装完成！"
echo ""
echo "常用命令:"
echo "  启动:   systemctl start sweetbomb-backend sweetbomb-frontend sweetbomb-admin-frontend"
echo "  停止:   systemctl stop sweetbomb-backend sweetbomb-frontend sweetbomb-admin-frontend"
echo "  重启:   systemctl restart sweetbomb-backend sweetbomb-frontend sweetbomb-admin-frontend"
echo "  状态:   systemctl status sweetbomb-backend sweetbomb-frontend sweetbomb-admin-frontend"
echo "  日志:   journalctl -u sweetbomb-backend -u sweetbomb-frontend -u sweetbomb-admin-frontend -f"
echo "  禁用自启: systemctl disable sweetbomb-backend sweetbomb-frontend sweetbomb-admin-frontend"
echo ""
echo "端口说明:"
echo "  后端 (Backend):    http://localhost:8000"
echo "  前端 (Frontend):   http://localhost:5173 - 机器人操控 (SW)"
echo "  管理前端 (Admin):  http://localhost:5174 - 后台管理 (SWNFP)"