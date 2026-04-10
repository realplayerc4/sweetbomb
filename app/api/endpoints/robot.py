"""机器人状态 API 端点"""

from fastapi import APIRouter, HTTPException
from typing import Optional
from pydantic import BaseModel
from datetime import datetime

from app.services.robot_tcp_server import RobotTCPServer, generate_task_id

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


class TaskRequest(BaseModel):
    """任务请求"""
    pick_station: str
    drop_station: str


class TaskResponse(BaseModel):
    """任务响应"""
    success: bool
    task_id: str
    message: str


@router.post("/task", response_model=TaskResponse)
async def send_task(request: TaskRequest):
    """发送任务到机器人"""
    import datetime

    server = get_robot_server()
    if not server:
        raise HTTPException(status_code=503, detail="机器人服务未启动")

    if not server.is_connected():
        raise HTTPException(status_code=503, detail="机器人未连接")

    # 生成任务ID（时间戳格式）
    task_id = datetime.datetime.now().strftime('%Y%m%d%H%M%S%f')[:17]

    # 发送任务
    result = await server.send_task(task_id, request.pick_station, request.drop_station)

    if result:
        return TaskResponse(
            success=True,
            task_id=task_id,
            message="任务已发送"
        )
    else:
        raise HTTPException(status_code=500, detail="发送任务失败")


class SimpleTaskResponse(BaseModel):
    """简单任务响应"""
    success: bool
    task_id: str
    message: str


class StopResponse(BaseModel):
    """停止响应"""
    success: bool
    message: str


@router.post("/scoop", response_model=SimpleTaskResponse)
async def send_scoop():
    """发送铲取任务 (Type=pick)"""
    server = get_robot_server()
    if not server or not server.is_connected():
        raise HTTPException(status_code=503, detail="机器人未连接")

    task_id = generate_task_id()
    result = await server.send_simple_task(task_id, "pick")

    if result:
        return SimpleTaskResponse(
            success=True,
            task_id=task_id,
            message="铲取任务已发送"
        )
    else:
        raise HTTPException(status_code=500, detail="发送失败")


@router.post("/dump", response_model=SimpleTaskResponse)
async def send_dump():
    """发送倾倒任务 (Type=drop)"""
    server = get_robot_server()
    if not server or not server.is_connected():
        raise HTTPException(status_code=503, detail="机器人未连接")

    task_id = generate_task_id()
    result = await server.send_simple_task(task_id, "drop")

    if result:
        return SimpleTaskResponse(
            success=True,
            task_id=task_id,
            message="倾倒任务已发送"
        )
    else:
        raise HTTPException(status_code=500, detail="发送失败")


@router.post("/dock", response_model=SimpleTaskResponse)
async def send_dock():
    """发送回桩任务 (Type=charge)"""
    server = get_robot_server()
    if not server or not server.is_connected():
        raise HTTPException(status_code=503, detail="机器人未连接")

    task_id = generate_task_id()
    result = await server.send_simple_task(task_id, "charge")

    if result:
        return SimpleTaskResponse(
            success=True,
            task_id=task_id,
            message="回桩任务已发送"
        )
    else:
        raise HTTPException(status_code=500, detail="发送失败")


@router.post("/stop", response_model=StopResponse)
async def stop_robot():
    """发送停止命令"""
    server = get_robot_server()
    if not server or not server.is_connected():
        raise HTTPException(status_code=503, detail="机器人未连接")

    result = await server.cancel_task()

    if result:
        return StopResponse(
            success=True,
            message="停止命令已发送"
        )
    else:
        raise HTTPException(status_code=500, detail="发送失败")


@router.post("/pause", response_model=StopResponse)
async def pause_robot():
    """发送暂停任务命令 (pauseTask)"""
    server = get_robot_server()
    if not server or not server.is_connected():
        raise HTTPException(status_code=503, detail="机器人未连接")

    result = await server.pause_task()

    if result:
        return StopResponse(
            success=True,
            message="暂停命令已发送"
        )
    else:
        raise HTTPException(status_code=500, detail="发送失败")


@router.post("/resume", response_model=StopResponse)
async def resume_robot():
    """发送取消暂停命令 (pauseCancel)"""
    server = get_robot_server()
    if not server or not server.is_connected():
        raise HTTPException(status_code=503, detail="机器人未连接")

    result = await server.pause_cancel()

    if result:
        return StopResponse(
            success=True,
            message="取消暂停命令已发送"
        )
    else:
        raise HTTPException(status_code=500, detail="发送失败")


@router.post("/nav-pick", response_model=SimpleTaskResponse)
async def nav_to_pick():
    """导航到取货点 (Type=allPick)"""
    server = get_robot_server()
    if not server or not server.is_connected():
        raise HTTPException(status_code=503, detail="机器人未连接")

    task_id = generate_task_id()
    result = await server.send_simple_task(task_id, "allPick")

    if result:
        return SimpleTaskResponse(
            success=True,
            task_id=task_id,
            message="导航到取货点任务已发送"
        )
    else:
        raise HTTPException(status_code=500, detail="发送失败")


@router.post("/nav-drop", response_model=SimpleTaskResponse)
async def nav_to_drop():
    """导航到卸货点 (Type=allDrop)"""
    server = get_robot_server()
    if not server or not server.is_connected():
        raise HTTPException(status_code=503, detail="机器人未连接")

    task_id = generate_task_id()
    result = await server.send_simple_task(task_id, "allDrop")

    if result:
        return SimpleTaskResponse(
            success=True,
            task_id=task_id,
            message="导航到卸货点任务已发送"
        )
    else:
        raise HTTPException(status_code=500, detail="发送失败")