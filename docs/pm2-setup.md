# PM2 服务管理配置指南

## 服务配置

项目使用 PM2 管理两个服务：

| 服务名 | 端口 | 类型 | 说明 |
|--------|------|------|------|
| sweetbomb-8000 | 8000 | Python/FastAPI | 后端 API 服务 |
| sweetbomb-5173 | 5173 | Vite/React | 前端开发服务器 |

## 常用命令

```bash
# 启动所有服务
pm2 start ecosystem.config.cjs

# 查看状态
pm2 status

# 查看日志
pm2 logs

# 重启服务
pm2 restart all
pm2 restart sweetbomb-8000

# 停止服务
pm2 stop all
pm2 delete all

# 监控面板
pm2 monit
```

## 开机自启配置

### 1. 保存当前进程列表

```bash
pm2 save
```

### 2. 生成启动脚本

```bash
# 生成 systemd 启动脚本
sudo env PATH=$PATH:/home/jetson/.nvm/versions/node/v20.20.1/bin /home/jetson/.nvm/versions/node/v20.20.1/lib/node_modules/pm2/bin/pm2 startup systemd -u jetson --hp /home/jetson
```

### 3. 启用服务

```bash
sudo systemctl enable pm2-jetson
```

### 4. 手动启动/停止

```bash
# 启动 PM2 服务
sudo systemctl start pm2-jetson

# 停止 PM2 服务
sudo systemctl stop pm2-jetson

# 查看状态
sudo systemctl status pm2-jetson
```

## 日志轮转配置

已创建日志轮转脚本：`scripts/log-rotate.sh`

### 添加定时任务

```bash
# 编辑 crontab
crontab -e

# 添加以下行（每天凌晨2点执行日志轮转）
0 2 * * * /home/jetson/sweetbomb/scripts/log-rotate.sh
```

### 日志路径

```
/home/jetson/.pm2/logs/
├── sweetbomb-8000-out.log
├── sweetbomb-8000-error.log
├── sweetbomb-5173-out.log
└── sweetbomb-5173-error.log
```

## 故障排查

### 服务无法启动

```bash
# 查看详细日志
pm2 logs --lines 100

# 检查配置文件语法
node -e "console.log(require('./ecosystem.config.cjs'))"
```

### 端口被占用

```bash
# 查看端口占用
sudo lsof -i :8000
sudo lsof -i :5173

# 释放端口
kill -9 <PID>
```

### 内存不足

```bash
# 查看内存使用
pm2 monit

# 限制内存使用（在配置中设置）
max_memory_restart: '1G'
```

## 配置文件位置

- PM2 配置：`/home/jetson/sweetbomb/ecosystem.config.cjs`
- PM2 进程列表：`/home/jetson/.pm2/dump.pm2`
- 日志目录：`/home/jetson/.pm2/logs/`
- 启动脚本：`/etc/systemd/system/pm2-jetson.service`
