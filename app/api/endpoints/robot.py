"""机器人状态 API 端点"""

from fastapi import APIRouter, HTTPException
from typing import Optional
from pydantic import BaseModel
from datetime import datetime

from app.services.robot_tcp_server import RobotTCPServer

router = APIRouter()

# 全局 TCP Server 实例引用
_robot_server: Optional[RobotTCPServer] = None


def set_robot_server(server: RobotTCPServer):
    """设置全局 TCP Server 实例"""
    global _robot_server
    _robot_server = server


def get_robot_server() -> Optional[RobotTCPServer]:
    """获取全局 TCP Server 实例"""
    return _robot_server


class RobotStatusResponse(BaseModel):
    """机器人状态响应"""
    connected: bool
    mode: str
    status: str
    charge: float
    speed: float
    fault: str
    fault_level: str
    task_id: str
    station: str
    map_name: str
    x: float
    y: float
    z: float
    a: float
    boom: float
    bucket: float
    last_update: Optional[datetime] = None


class ConnectionStatusResponse(BaseModel):
    """连接状态响应"""
    connected: bool
    missed_heartbeats: int = 0


@router.get("/status", response_model=RobotStatusResponse)
async def get_robot_status():
    """获取机器人状态"""
    server = get_robot_server()
    if not server:
        raise HTTPException(status_code=503, detail="机器人服务未启动")

    client = server.client
    if not client:
        # 返回未连接状态
        return RobotStatusResponse(
            connected=False,
            mode="auto",
            status="idle",
            charge=0,
            speed=0,
            fault="",
            fault_level="",
            task_id="",
            station="",
            map_name="",
            x=0,
            y=0,
            z=0,
            a=0,
            boom=0,
            bucket=0,
        )

    state = client.state
    return RobotStatusResponse(
        connected=client.connected,
        mode=state.mode.value,
        status=state.status.value,
        charge=state.charge,
        speed=state.speed,
        fault=state.fault,
        fault_level=state.fault_level,
        task_id=state.task_id,
        station=state.station,
        map_name=state.map_name,
        x=state.x,
        y=state.y,
        z=state.z,
        a=state.a,
        boom=state.boom,
        bucket=state.bucket,
        last_update=state.last_update
    )


@router.get("/connection", response_model=ConnectionStatusResponse)
async def get_connection_status():
    """获取机器人连接状态（用于前端检测）"""
    server = get_robot_server()
    if not server:
        return ConnectionStatusResponse(
            connected=False,
            missed_heartbeats=0
        )

    client = server.client
    if not client:
        return ConnectionStatusResponse(
            connected=False,
            missed_heartbeats=0
        )

    return ConnectionStatusResponse(
        connected=client.connected,
        missed_heartbeats=client.missed_heartbeats
    )