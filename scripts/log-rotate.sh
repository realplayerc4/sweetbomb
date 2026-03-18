#!/bin/bash
# PM2 日志轮转脚本
# 添加到 crontab: 0 2 * * * /home/jetson/sweetbomb/scripts/log-rotate.sh

LOG_DIR="/home/jetson/.pm2/logs"
MAX_SIZE="100M"  # 单个日志文件最大大小
MAX_DAYS=7       # 保留天数

# 创建日志目录（如果不存在）
mkdir -p "$LOG_DIR"

# 清理过大的日志文件（超过 MAX_SIZE 的日志会被清空）
find "$LOG_DIR" -name "*.log" -size +$MAX_SIZE -exec sh -c 'echo "[$(date)] Rotating large log: {}" >> /var/log/pm2-log-rotate.log; cat /dev/null > {}' \;

# 删除超过 MAX_DAYS 天的旧日志文件
find "$LOG_DIR" -name "*.log.*" -mtime +$MAX_DAYS -delete
find "$LOG_DIR" -name "*.log.old" -mtime +$MAX_DAYS -delete

# 记录执行日志
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Log rotation completed" >> /var/log/pm2-log-rotate.log
