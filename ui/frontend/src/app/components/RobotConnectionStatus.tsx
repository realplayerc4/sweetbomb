import { useEffect, useState } from 'react';

interface RobotConnectionStatusProps {
  className?: string;
}

interface ConnectionStatus {
  connected: boolean;
  missed_heartbeats: number;
}

export function RobotConnectionStatus({ className }: RobotConnectionStatusProps) {
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const API_BASE_URL = import.meta.env.VITE_API_BASE_URL || 'http://localhost:8000';

    const fetchConnectionStatus = async () => {
      try {
        const response = await fetch(`${API_BASE_URL}/api/robot/connection`);
        if (response.ok) {
          const data: ConnectionStatus = await response.json();
          setConnected(data.connected);
        } else {
          setConnected(false);
        }
      } catch (error) {
        console.error('获取机器人连接状态失败:', error);
        setConnected(false);
      }
    };

    // 立即获取一次
    fetchConnectionStatus();

    // 每1秒轮询一次
    const intervalId = setInterval(fetchConnectionStatus, 1000);

    return () => {
      clearInterval(intervalId);
    };
  }, []);

  return (
    <div
      className={`flex items-center gap-2 ${className}`}
      title={connected ? '机器人已连接' : '机器人未连接'}
    >
      {/* 4像素小圆点 */}
      <div
        className={`w-1 h-1 rounded-full ${connected ? 'bg-green-500' : 'bg-[#FD802E]'
          }`}
        style={{ width: '4px', height: '4px' }}
      />
      <span className={connected ? "text-xs text-muted-foreground" : "text-xs text-[#FD802E]"}>
        {connected ? '已连接' : '未连接'}
      </span>
    </div>
  );
}
