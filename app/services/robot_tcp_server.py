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
import threading
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Callable, Dict, Optional, Set

logger = logging.getLogger(__name__)

# TaskId生成器（线程安全）
_task_id_lock = threading.Lock()
_task_id_counter = 0
_task_id_last_timestamp = ""


def generate_task_id() -> str:
    """生成任务ID: YYYYMMDDHHMMSS + 3位序号（同一秒内序号递增）"""
    global _task_id_counter, _task_id_last_timestamp
    with _task_id_lock:
        ts = datetime.now().strftime('%Y%m%d%H%M%S')
        if ts == _task_id_last_timestamp:
            _task_id_counter += 1
        else:
            _task_id_counter = 1
            _task_id_last_timestamp = ts
        return f"{ts}{_task_id_counter:03d}"


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
    PAUSE_TASK = "pauseTask"    # 暂停任务
    PAUSE_CANCEL = "pauseCancel"  # 取消暂停


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
    boom: float = 0.0                   # 臂角度 °
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
        self.connected = True
        self.last_heartbeat = datetime.now()
        self.state = RobotState()

        # 远程控制超时检测
        self.remote_control_active = False
        self.last_remote_control_time: Optional[datetime] = None

        # 心跳计数 - 用于检测掉线
        self.missed_heartbeats = 0
        self.max_missed_heartbeats = 20  # 20次未回复判定为掉线

    async def send_message(self, message: str):
        """发送报文"""
        try:
            # 确保报文以UTF-8编码，包含正确的换行符
            encoded = message.encode('utf-8')
            self.writer.write(encoded)
            await self.writer.drain()
            # 记录十六进制前50字节用于调试
            hex_str = ' '.join(f'{b:02x}' for b in encoded[:50])
            logger.debug(f"发送给 {self.addr}: {hex_str}")
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
        self.client: Optional[RobotTCPClient] = None
        self.running = False

        # 消息处理器
        self.message_handlers: Dict[MessageType, Callable] = {
            MessageType.STATUS: self._handle_status,
            MessageType.TASK: self._handle_task,
            MessageType.TASK_FINISH: self._handle_task_finish,
            MessageType.CANCEL_TASK: self._handle_cancel_task,
            MessageType.COMMAND: self._handle_command,
            MessageType.REMOTE_CONTROL: self._handle_remote_control,
            MessageType.PAUSE_TASK: self._handle_pause_task,
            MessageType.PAUSE_CANCEL: self._handle_pause_cancel,
        }

        # 回调函数
        self.on_robot_connected: Optional[Callable[[], None]] = None
        self.on_robot_disconnected: Optional[Callable[[], None]] = None
        self.on_state_update: Optional[Callable[[RobotState], None]] = None
        self.on_task_complete: Optional[Callable[[str], None]] = None

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
        # 关闭客户端连接
        if self.client:
            await self.client.close()
            self.client = None

        if self.server:
            self.server.close()
            await self.server.wait_closed()
        logger.info("机器人TCP服务器已停止")

    async def send_camera_distance(self, distance_mm: int) -> bool:
        """
        发送相机检测距离到机器人

        Args:
            distance_mm: 距离值，单位毫米

        Returns:
            bool: 是否发送成功

        协议格式: {MessageType=cameraCheckDistance=xxxx}
        """
        message = f"{{MessageType=cameraCheckDistance={distance_mm}}}"

        if not self.client or not self.client.connected:
            logger.warning("机器人未连接，无法发送相机距离")
            return False

        try:
            await self.client.send_message(message)
            logger.debug(f"发送相机距离: {distance_mm}mm")
        except Exception as e:
            logger.error(f"发送相机距离失败: {e}")
            return False

        return True

    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        """处理客户端连接"""
        # 如果已有连接，先断开旧连接
        if self.client and self.client.connected:
            logger.info(f"新连接到来，断开旧连接: {self.client.addr}")
            await self._remove_client(self.client)

        client = RobotTCPClient(reader, writer, self)
        self.client = client
        logger.info(f"机器人连接: {client.addr}")

        if self.on_robot_connected:
            await self._trigger_callback(self.on_robot_connected)

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
        """处理状态查询回复"""
        # 收到回复，重置未回复计数
        client.missed_heartbeats = 0

        # 更新机器人状态
        # 模式下位机返回中文：手柄控制/自动任务/远程控制
        mode_map = {
            '手柄控制': RobotMode.HANDLE,
            '自动任务': RobotMode.AUTO,
            '远程控制': RobotMode.REMOTE,
            'auto': RobotMode.AUTO,
            'handle': RobotMode.HANDLE,
            'remote': RobotMode.REMOTE,
        }
        try:
            mode_str = data.get('Mode', 'auto')
            if mode_str in mode_map:
                client.state.mode = mode_map[mode_str]
            elif mode_str in [m.value for m in RobotMode]:
                client.state.mode = RobotMode(mode_str)
        except (ValueError, KeyError):
            pass

        # 状态下位机返回中文：空闲/运行中
        status_map = {
            '空闲': RobotStatus.IDLE,
            '运行中': RobotStatus.RUNNING,
            'idle': RobotStatus.IDLE,
            'running': RobotStatus.RUNNING,
        }
        try:
            status_str = data.get('Status', 'idle')
            if status_str in status_map:
                client.state.status = status_map[status_str]
            elif status_str in [s.value for s in RobotStatus]:
                client.state.status = RobotStatus(status_str)
        except (ValueError, KeyError):
            pass

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

        # 状态查询不需要回复（因为是上位机主动查询，下位机回复）

        # 触发回调
        if self.on_state_update:
            await self._trigger_callback(self.on_state_update, client.state)

        logger.debug(f"状态更新: 电量: {client.state.charge}%")

    async def _handle_task(self, client: RobotTCPClient, data: Dict):
        """处理任务下发"""
        task_id = data.get('TaskId', '')
        pick_station = data.get('PickStationId', '')
        drop_station = data.get('DropStationId', '')

        logger.info(f"收到任务: {task_id}, 取货: {pick_station}, 放货: {drop_station}")
        # 不需要回复下位机

    async def _handle_task_finish(self, client: RobotTCPClient, data: Dict):
        """处理任务完成"""
        task_id = data.get('TaskId', '')
        logger.info(f"任务完成: {task_id}")

        # 发送确认回复（只有taskFinish需要回复）
        response = f"{{\nMessageType=taskFinish\nTaskId={task_id}\n}}"
        await client.send_message(response)

        # 触发回调
        if self.on_task_complete:
            await self._trigger_callback(self.on_task_complete, task_id)

    async def _handle_cancel_task(self, client: RobotTCPClient, data: Dict):
        """处理取消任务"""
        logger.info("取消任务")
        # 不需要回复下位机

    async def _handle_pause_task(self, client: RobotTCPClient, data: Dict):
        """处理暂停任务"""
        logger.info("暂停任务")
        # 不需要回复下位机

    async def _handle_pause_cancel(self, client: RobotTCPClient, data: Dict):
        """处理取消暂停"""
        logger.info("取消暂停")
        # 不需要回复下位机

    async def _handle_command(self, client: RobotTCPClient, data: Dict):
        """处理命令"""
        operate = data.get('Operate', '')
        logger.info(f"收到命令: {operate}")
        # 不需要回复下位机

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
        """心跳检测器 - 每250ms发送status查询，20次未回复判定掉线"""
        while self.running:
            try:
                await asyncio.sleep(0.25)  # 每250ms发送一次

                if not self.client or not self.client.connected:
                    continue

                # 发送status查询报文
                status_query = "{\nMessageType=status\n}"
                try:
                    await self.client.send_message(status_query)
                    self.client.missed_heartbeats += 1

                    # 检查是否超过20次未回复（5秒）
                    if self.client.missed_heartbeats >= 20:
                        logger.warning("机器人超过20次未回复状态查询，判定掉线")
                        await self._remove_client(self.client)
                except Exception as e:
                    logger.error(f"发送状态查询失败: {e}")
                    await self._remove_client(self.client)

            except Exception as e:
                logger.error(f"心跳检测错误: {e}")

    async def _remote_control_monitor(self):
        """远程控制超时监控"""
        while self.running:
            try:
                await asyncio.sleep(0.1)  # 每100ms检查一次

                if not self.client:
                    continue

                if self.client.remote_control_active:
                    # 检查是否超过500ms未收到远程控制报文
                    if self.client.last_remote_control_time:
                        elapsed = (datetime.now() - self.client.last_remote_control_time).total_seconds()
                        if elapsed > 0.5:
                            logger.warning("机器人远程控制超时，自动停止")
                            self.client.remote_control_active = False
                            # 这里可以触发自动停止逻辑

            except Exception as e:
                logger.error(f"远程控制监控错误: {e}")

    async def _remove_client(self, client: RobotTCPClient):
        """移除客户端"""
        if self.client is client:
            self.client = None

        await client.close()

        if self.on_robot_disconnected:
            await self._trigger_callback(self.on_robot_disconnected)

        logger.info(f"机器人断开连接: {client.addr}")

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

    async def send_command(self, operate: str) -> bool:
        """发送命令到机器人"""
        if not self.client or not self.client.connected:
            logger.warning("机器人未连接")
            return False

        message = f"{{\nMessageType=command\nOperate={operate}\n}}"
        await self.client.send_message(message)
        return True

    async def send_task(self, task_id: str, pick_station: str, drop_station: str) -> bool:
        """发送任务到机器人"""
        if not self.client or not self.client.connected:
            logger.warning("机器人未连接")
            return False

        message = f"{{\nMessageType=task\nTaskId={task_id}\nPickStationId={pick_station}\nDropStationId={drop_station}\n}}"
        await self.client.send_message(message)
        return True

    async def send_simple_task(self, task_id: str, task_type: str) -> bool:
        """发送简单任务（新协议格式）"""
        valid_types = {"pick", "drop", "charge", "allPick", "allDrop"}
        if task_type not in valid_types:
            logger.error(f"无效的任务类型: {task_type}")
            return False

        if not self.client or not self.client.connected:
            logger.warning("机器人未连接")
            return False

        message = f"{{\nMessageType=task\nTaskId={task_id}\nType={task_type}\n}}"
        await self.client.send_message(message)
        return True

    async def cancel_task(self) -> bool:
        """取消机器人的任务"""
        if not self.client or not self.client.connected:
            logger.warning("机器人未连接")
            return False

        message = "{\nMessageType=cancelTask\n}"
        await self.client.send_message(message)
        return True

    async def pause_task(self) -> bool:
        """发送暂停任务命令"""
        if not self.client or not self.client.connected:
            logger.warning("机器人未连接")
            return False

        message = "{MessageType=pauseTask}"
        await self.client.send_message(message)
        return True

    async def pause_cancel(self) -> bool:
        """发送取消暂停命令"""
        if not self.client or not self.client.connected:
            logger.warning("机器人未连接")
            return False

        message = "{MessageType=pauseCancel}"
        await self.client.send_message(message)
        return True

    def is_connected(self) -> bool:
        """检查机器人是否已连接"""
        return self.client is not None and self.client.connected

    def get_state(self) -> Optional[RobotState]:
        """获取机器人状态"""
        if self.client:
            return self.client.state
        return None


# 使用示例
async def main():
    """示例用法"""
    server = RobotTCPServer(host="0.0.0.0", port=9090)

    # 设置回调
    async def on_connect():
        print("机器人已连接")

    async def on_disconnect():
        print("机器人已断开")

    async def on_state_update(state):
        print(f"机器人状态更新: 电量{state.charge}%, 位置({state.x}, {state.y})")

    async def on_task_complete(task_id):
        print(f"机器人完成任务: {task_id}")

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

            # 示例：每10秒查询一次机器人状态
            if server.is_connected():
                state = server.get_state()
                if state:
                    logger.info(f"机器人: 电量{state.charge}%, 状态{state.status.value}")

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
