"""
铲糖机器人TCP通讯服务器

实现与下位机的TCP/IP通讯协议，本机作为服务端（端口9090），
机器人作为客户端主动连接。

协议特点：
- 报文以UTF-8编码字符串传输
- "{"为报文头，"}"为报文尾
- 重要数据1s内收不到回复需要重发
- 远程控制模式下500ms无报文机器人自动停止
"""

import asyncio
import json
import logging
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Callable, Dict, Optional, Set

logger = logging.getLogger(__name__)


class RobotMode(Enum):
    """机器人模式"""
    AUTO = "auto"           # 自动任务模式
    REMOTE = "remote"       # 远程控制模式
    HANDLE = "handle"       # 手柄控制模式


class RobotStatus(Enum):
    """机器人状态"""
    IDLE = "idle"           # 空闲
    RUNNING = "running"     # 运行中


class MessageType(Enum):
    """报文类型"""
    STATUS = "status"           # 状态查询
    TASK = "task"               # 任务下发
    TASK_FINISH = "taskFinish"  # 任务完成
    CANCEL_TASK = "cancelTask"  # 取消任务
    COMMAND = "command"         # 命令
    REMOTE_CONTROL = "remoteControl"  # 远程控制


@dataclass
class RobotState:
    """机器人状态数据"""
    mode: RobotMode = RobotMode.AUTO
    status: RobotStatus = RobotStatus.IDLE
    charge: float = 0.0                 # 电量 0-100
    speed: float = 0.0                  # 速度 m/s
    fault: str = ""                     # 故障信息
    fault_level: str = ""               # 故障等级
    task_id: str = ""                   # 当前任务号
    station: str = ""                   # 当前站点
    map_name: str = ""                  # 当前地图名
    x: float = 0.0                      # X坐标 mm
    y: float = 0.0                      # Y坐标 mm
    z: float = 0.0                      # Z坐标 mm
    a: float = 0.0                      # 车体角度 °
    boom: float = 0.0                   # 臂位置 mm
    bucket: float = 0.0                  # 铲斗角度 °
    last_update: datetime = field(default_factory=datetime.now)

    def to_dict(self) -> Dict:
        """转换为字典"""
        return {
            "Mode": self.mode.value,
            "Status": self.status.value,
            "Charge": f"{self.charge:.1f}",
            "Speed": f"{self.speed:.2f}",
            "Fault": self.fault,
            "FaultLevel": self.fault_level,
            "TaskId": self.task_id,
            "Station": self.station,
            "Map": self.map_name,
            "X": str(int(self.x)),
            "Y": str(int(self.y)),
            "Z": str(int(self.z)),
            "A": f"{self.a:.2f}",
            "Boom": f"{self.boom:.2f}",
            "Bucket": f"{self.bucket:.2f}",
        }


class RobotTCPClient:
    """机器人TCP客户端连接"""

    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter, server: 'RobotTCPServer'):
        self.reader = reader
        self.writer = writer
        self.server = server
        self.addr = writer.get_extra_info('peername')
        self.robot_id: Optional[str] = None
        self.connected = True
        self.last_heartbeat = datetime.now()
        self.state = RobotState()

        # 远程控制超时检测
        self.remote_control_active = False
        self.last_remote_control_time: Optional[datetime] = None

    async def send_message(self, message: str):
        """发送报文"""
        try:
            self.writer.write(message.encode('utf-8'))
            await self.writer.drain()
            logger.debug(f"发送给 {self.addr}: {message[:100]}...")
        except Exception as e:
            logger.error(f"发送失败 {self.addr}: {e}")
            self.connected = False

    def update_heartbeat(self):
        """更新心跳时间"""
        self.last_heartbeat = datetime.now()

    def is_timeout(self, timeout_seconds: float = 10.0) -> bool:
        """检查是否超时"""
        return (datetime.now() - self.last_heartbeat).total_seconds() > timeout_seconds

    async def close(self):
        """关闭连接"""
        self.connected = False
        self.writer.close()
        await self.writer.wait_closed()


class RobotTCPServer:
    """铲糖机器人TCP服务器"""

    def __init__(self, host: str = "0.0.0.0", port: int = 9090):
        self.host = host
        self.port = port
        self.server: Optional[asyncio.Server] = None
        self.clients: Dict[str, RobotTCPClient] = {}  # robot_id -> client
        self.running = False

        # 消息处理器
        self.message_handlers: Dict[MessageType, Callable] = {
            MessageType.STATUS: self._handle_status,
            MessageType.TASK: self._handle_task,
            MessageType.TASK_FINISH: self._handle_task_finish,
            MessageType.CANCEL_TASK: self._handle_cancel_task,
            MessageType.COMMAND: self._handle_command,
            MessageType.REMOTE_CONTROL: self._handle_remote_control,
        }

        # 回调函数
        self.on_robot_connected: Optional[Callable[[str], None]] = None
        self.on_robot_disconnected: Optional[Callable[[str], None]] = None
        self.on_state_update: Optional[Callable[[str, RobotState], None]] = None
        self.on_task_complete: Optional[Callable[[str, str], None]] = None

    async def start(self):
        """启动服务器"""
        self.server = await asyncio.start_server(
            self._handle_client,
            self.host,
            self.port
        )
        self.running = True
        addr = self.server.sockets[0].getsockname()
        logger.info(f"机器人TCP服务器启动: {addr}")

        # 启动后台任务
        asyncio.create_task(self._heartbeat_checker())
        asyncio.create_task(self._remote_control_monitor())

    async def stop(self):
        """停止服务器"""
        self.running = False
        # 关闭所有客户端连接
        for client in list(self.clients.values()):
            await client.close()
        self.clients.clear()

        if self.server:
            self.server.close()
            await self.server.wait_closed()
        logger.info("机器人TCP服务器已停止")

    async def send_camera_distance(self, distance_mm: int, robot_id: Optional[str] = None) -> bool:
        """
        发送相机检测距离到机器人

        Args:
            distance_mm: 距离值，单位毫米
            robot_id: 指定机器人ID，为None则发送到所有连接的机器人

        Returns:
            bool: 是否发送成功

        协议格式: {MessageType=cameraCheckDistance=xxxx}
        """
        message = f"{{MessageType=cameraCheckDistance={distance_mm}}}"

        if robot_id:
            # 发送到指定机器人
            client = self.clients.get(robot_id)
            if not client:
                logger.warning(f"机器人 {robot_id} 未连接，无法发送相机距离")
                return False
            await client.send_message(message)
            logger.debug(f"发送相机距离到 {robot_id}: {distance_mm}mm")
        else:
            # 发送到所有连接的机器人
            if not self.clients:
                logger.debug("没有连接的机器人，跳过发送相机距离")
                return False

            for rid, client in list(self.clients.items()):
                try:
                    await client.send_message(message)
                    logger.debug(f"发送相机距离到 {rid}: {distance_mm}mm")
                except Exception as e:
                    logger.error(f"发送相机距离到 {rid} 失败: {e}")

        return True

    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        """处理客户端连接"""
        client = RobotTCPClient(reader, writer, self)
        logger.info(f"机器人连接: {client.addr}")

        try:
            while client.connected and self.running:
                # 读取数据
                data = await reader.read(4096)
                if not data:
                    break

                # 处理报文
                message = data.decode('utf-8')
                await self._process_message(client, message)
                client.update_heartbeat()

        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error(f"处理客户端错误 {client.addr}: {e}")
        finally:
            await self._remove_client(client)

    async def _process_message(self, client: RobotTCPClient, message: str):
        """解析并处理报文"""
        # 解析报文
        lines = message.strip().split('\n')
        data = {}

        for line in lines:
            line = line.strip()
            if not line or line in ['{', '}']:
                continue
            if '=' in line:
                key, value = line.split('=', 1)
                data[key.strip()] = value.strip()

        # 获取消息类型
        msg_type_str = data.get('MessageType', '')
        try:
            msg_type = MessageType(msg_type_str)
        except ValueError:
            logger.warning(f"未知的消息类型: {msg_type_str}")
            return

        # 调用对应的处理器
        handler = self.message_handlers.get(msg_type)
        if handler:
            await handler(client, data)
        else:
            logger.warning(f"未实现的消息处理器: {msg_type}")

    async def _handle_status(self, client: RobotTCPClient, data: Dict):
        """处理状态查询"""
        # 更新机器人状态
        client.state.mode = RobotMode(data.get('Mode', 'auto'))
        client.state.status = RobotStatus(data.get('Status', 'idle'))
        client.state.charge = float(data.get('Charge', 0))
        client.state.speed = float(data.get('Speed', 0))
        client.state.fault = data.get('Fault', '')
        client.state.fault_level = data.get('FaultLevel', '')
        client.state.task_id = data.get('TaskId', '')
        client.state.station = data.get('Station', '')
        client.state.map_name = data.get('Map', '')
        client.state.x = float(data.get('X', 0))
        client.state.y = float(data.get('Y', 0))
        client.state.z = float(data.get('Z', 0))
        client.state.a = float(data.get('A', 0))
        client.state.boom = float(data.get('Boom', 0))
        client.state.bucket = float(data.get('Bucket', 0))
        client.state.last_update = datetime.now()

        # 发送状态回复 (包含完整状态数据)
        response = f"{{\nMessageType=status\nMode={client.state.mode.value}\nStatus={client.state.status.value}\nCharge={client.state.charge:.1f}\nSpeed={client.state.speed:.2f}\nFault={client.state.fault}\nFaultLevel={client.state.fault_level}\nTaskId={client.state.task_id}\nStation={client.state.station}\nMap={client.state.map_name}\nX={int(client.state.x)}\nY={int(client.state.y)}\nZ={int(client.state.z)}\nA={client.state.a:.2f}\nBoom={client.state.boom:.2f}\nBucket={client.state.bucket:.2f}\n}}"
        await client.send_message(response)

        # 触发回调
        if self.on_state_update and client.robot_id:
            await self._trigger_callback(self.on_state_update, client.robot_id, client.state)

        logger.debug(f"状态更新: {client.robot_id}, 电量: {client.state.charge}%")

    async def _handle_task(self, client: RobotTCPClient, data: Dict):
        """处理任务下发"""
        task_id = data.get('TaskId', '')
        pick_station = data.get('PickStationId', '')
        drop_station = data.get('DropStationId', '')

        logger.info(f"收到任务: {task_id}, 取货: {pick_station}, 放货: {drop_station}")

        # 发送确认回复
        response = f"{{\nMessageType=task\nTaskId={task_id}\nResult=1\n}}"
        await client.send_message(response)

    async def _handle_task_finish(self, client: RobotTCPClient, data: Dict):
        """处理任务完成"""
        task_id = data.get('TaskId', '')
        logger.info(f"任务完成: {task_id}")

        # 发送确认回复
        response = f"{{\nMessageType=taskFinish\nTaskId={task_id}\n}}"
        await client.send_message(response)

        # 触发回调
        if self.on_task_complete and client.robot_id:
            await self._trigger_callback(self.on_task_complete, client.robot_id, task_id)

    async def _handle_cancel_task(self, client: RobotTCPClient, data: Dict):
        """处理取消任务"""
        logger.info(f"取消任务: {client.robot_id}")

        # 发送确认回复
        response = "{\nMessageType=cancelTask\n}"
        await client.send_message(response)

    async def _handle_command(self, client: RobotTCPClient, data: Dict):
        """处理命令"""
        operate = data.get('Operate', '')
        logger.info(f"收到命令: {operate}, 机器人: {client.robot_id}")

        # 发送确认回复
        response = f"{{\nMessageType=command\nOperate={operate}\n}}"
        await client.send_message(response)

    async def _handle_remote_control(self, client: RobotTCPClient, data: Dict):
        """处理远程控制"""
        # 解析控制数据
        robot_control = data.get('Robot', '')
        actuator_control = data.get('Actuator', '')

        # 更新远程控制状态
        client.remote_control_active = True
        client.last_remote_control_time = datetime.now()

        logger.debug(f"远程控制: Robot={robot_control}, Actuator={actuator_control}")

        # 远程控制无回复报文

    async def _heartbeat_checker(self):
        """心跳检测器"""
        while self.running:
            try:
                await asyncio.sleep(5)  # 每5秒检查一次

                now = datetime.now()
                timeout_clients = []

                for robot_id, client in self.clients.items():
                    if client.is_timeout(timeout_seconds=10):
                        logger.warning(f"机器人 {robot_id} 心跳超时")
                        timeout_clients.append(client)

                # 关闭超时连接
                for client in timeout_clients:
                    await self._remove_client(client)

            except Exception as e:
                logger.error(f"心跳检测错误: {e}")

    async def _remote_control_monitor(self):
        """远程控制超时监控"""
        while self.running:
            try:
                await asyncio.sleep(0.1)  # 每100ms检查一次

                now = datetime.now()

                for client in self.clients.values():
                    if client.remote_control_active:
                        # 检查是否超过500ms未收到远程控制报文
                        if client.last_remote_control_time:
                            elapsed = (now - client.last_remote_control_time).total_seconds()
                            if elapsed > 0.5:
                                logger.warning(f"机器人 {client.robot_id} 远程控制超时，自动停止")
                                client.remote_control_active = False
                                # 这里可以触发自动停止逻辑

            except Exception as e:
                logger.error(f"远程控制监控错误: {e}")

    async def _remove_client(self, client: RobotTCPClient):
        """移除客户端"""
        if client.robot_id and client.robot_id in self.clients:
            del self.clients[client.robot_id]

        await client.close()

        if self.on_robot_disconnected and client.robot_id:
            await self._trigger_callback(self.on_robot_disconnected, client.robot_id)

        logger.info(f"机器人断开连接: {client.robot_id}")

    async def _trigger_callback(self, callback: Callable, *args):
        """触发回调函数"""
        try:
            if asyncio.iscoroutinefunction(callback):
                await callback(*args)
            else:
                callback(*args)
        except Exception as e:
            logger.error(f"回调执行错误: {e}")

    # 公共API

    async def send_command(self, robot_id: str, operate: str) -> bool:
        """发送命令到指定机器人"""
        client = self.clients.get(robot_id)
        if not client:
            logger.warning(f"机器人 {robot_id} 未连接")
            return False

        message = f"{{\nMessageType=command\nOperate={operate}\n}}"
        await client.send_message(message)
        return True

    async def send_task(self, robot_id: str, task_id: str, pick_station: str, drop_station: str) -> bool:
        """发送任务到指定机器人"""
        client = self.clients.get(robot_id)
        if not client:
            logger.warning(f"机器人 {robot_id} 未连接")
            return False

        message = f"{{\nMessageType=task\nTaskId={task_id}\nPickStationId={pick_station}\nDropStationId={drop_station}\n}}"
        await client.send_message(message)
        return True

    async def cancel_task(self, robot_id: str) -> bool:
        """取消指定机器人的任务"""
        client = self.clients.get(robot_id)
        if not client:
            logger.warning(f"机器人 {robot_id} 未连接")
            return False

        message = "{\nMessageType=cancelTask\n}"
        await client.send_message(message)
        return True

    def get_robot_ids(self) -> list:
        """获取所有连接的机器人ID"""
        return list(self.clients.keys())

    def get_robot_state(self, robot_id: str) -> Optional[RobotState]:
        """获取指定机器人的状态"""
        client = self.clients.get(robot_id)
        return client.state if client else None


# 使用示例
async def main():
    """示例用法"""
    server = RobotTCPServer(host="0.0.0.0", port=9090)

    # 设置回调
    async def on_connect(robot_id):
        print(f"机器人 {robot_id} 已连接")

    async def on_disconnect(robot_id):
        print(f"机器人 {robot_id} 已断开")

    async def on_state_update(robot_id, state):
        print(f"机器人 {robot_id} 状态更新: 电量{state.charge}%, 位置({state.x}, {state.y})")

    async def on_task_complete(robot_id, task_id):
        print(f"机器人 {robot_id} 完成任务: {task_id}")

    server.on_robot_connected = on_connect
    server.on_robot_disconnected = on_disconnect
    server.on_state_update = on_state_update
    server.on_task_complete = on_task_complete

    # 启动服务器
    await server.start()

    # 保持运行
    try:
        while True:
            await asyncio.sleep(1)

            # 示例：每10秒查询一次所有机器人的状态
            for robot_id in server.get_robot_ids():
                state = server.get_robot_state(robot_id)
                if state:
                    logger.info(f"机器人 {robot_id}: 电量{state.charge}%, 状态{state.status.value}")

    except KeyboardInterrupt:
        print("正在停止服务器...")
    finally:
        await server.stop()


if __name__ == "__main__":
    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # 运行
    asyncio.run(main())
