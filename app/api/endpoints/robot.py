"""机器人状态 API 端点"""

from fastapi import APIRouter, HTTPException
from typing import Dict, List, Optional
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
    robot_id: str
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
    robot_id: Optional[str] = None
    missed_heartbeats: int = 0
    client_count: int = 0


@router.get("/status", response_model=List[RobotStatusResponse])
async def get_all_robot_status():
    """获取所有连接的机器人状态"""
    server = get_robot_server()
    if not server:
        raise HTTPException(status_code=503, detail="机器人服务未启动")

    results = []
    for robot_id, client in server.clients.items():
        state = client.state
        results.append(RobotStatusResponse(
            robot_id=robot_id,
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
        ))

    return results


@router.get("/status/{robot_id}", response_model=RobotStatusResponse)
async def get_robot_status(robot_id: str):
    """获取指定机器人的状态"""
    server = get_robot_server()
    if not server:
        raise HTTPException(status_code=503, detail="机器人服务未启动")

    client = server.clients.get(robot_id)
    if not client:
        raise HTTPException(status_code=404, detail=f"机器人 {robot_id} 未连接")

    state = client.state
    return RobotStatusResponse(
        robot_id=robot_id,
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
            client_count=0
        )

    # 获取第一个连接的客户端
    connected_client = None
    for client in server.clients.values():
        if client.connected:
            connected_client = client
            break

    return ConnectionStatusResponse(
        connected=connected_client is not None,
        robot_id=connected_client.robot_id if connected_client else None,
        missed_heartbeats=connected_client.missed_heartbeats if connected_client else 0,
        client_count=len(server.clients)
    )
