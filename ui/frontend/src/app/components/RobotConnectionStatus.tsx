import { useEffect, useState } from 'react';

interface RobotConnectionStatusProps {
  className?: string;
}

export function RobotConnectionStatus({ className }: RobotConnectionStatusProps) {
  const [connected, setConnected] = useState(false);
  const [ws, setWs] = useState<WebSocket | null>(null);

  useEffect(() => {
    // 连接WebSocket服务器（由后端TCP服务器转发）
    const connectWebSocket = () => {
      const socket = new WebSocket('ws://localhost:8000/ws/robot');

      socket.onopen = () => {
        console.log('机器人连接状态WebSocket已连接');
        setConnected(true);
      };

      socket.onclose = () => {
        console.log('机器人连接状态WebSocket已断开');
        setConnected(false);
        // 3秒后重连
        setTimeout(connectWebSocket, 3000);
      };

      socket.onerror = (error) => {
        console.error('机器人连接状态WebSocket错误:', error);
        setConnected(false);
      };

      // 监听机器人连接状态消息
      socket.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (data.type === 'robot_connection') {
            setConnected(data.connected);
          }
        } catch (e) {
          console.error('解析机器人状态消息失败:', e);
        }
      };

      setWs(socket);
    };

    connectWebSocket();

    return () => {
      if (ws) {
        ws.close();
      }
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
