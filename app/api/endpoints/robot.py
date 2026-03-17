"""机器人 API 端点。

提供铲糖机器人的控制接口：
- 运动控制（前后左右）
- 紧急停止
- 伺服控制
- 状态查询
- 距离分析
- 铲糖自主循环控制
"""

import asyncio
import logging
from typing import Dict, Any, Optional, Tuple
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, Query, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, Field

from app.services.robot_controller import (
    RobotController,
    get_robot_controller,
    RobotState,
    MoveDirection,
)
from app.services.distance_analyzer import (
    DistanceAnalyzer,
    get_distance_analyzer,
    DistanceAnalysisResult,
)
from app.services.mock_navigation import get_mock_navigation
from app.services.behavior_tree_engine import (
    BehaviorTreeEngine,
    create_sugar_harvest_engine,
)
from app.services.robot_tcp_server import RobotTCPServer
from app.api.dependencies import get_robot_tcp_server

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/robot", tags=["robot"])


# ==================== 请求/响应模型 ====================


class MoveRequest(BaseModel):
    """运动控制请求。"""
    direction: MoveDirection
    speed: float = Field(default=0.5, ge=0.0, le=1.0, description="速度 (0.0-1.0)")
    duration: float = Field(default=1.0, gt=0, description="持续时间（秒）")


class ServoRequest(BaseModel):
    """伺服控制请求。"""
    servo_id: str = Field(description="伺服 ID ('lift' 或 'dump')")
    angle: float = Field(ge=0.0, le=180.0, description="目标角度（度）")


class DistanceAnalysisRequest(BaseModel):
    """距离分析请求。"""
    region_of_interest: Optional[Dict[str, Any]] = Field(
        default=None, description="感兴趣区域（可选）"
    )


class SugarHarvestConfig(BaseModel):
    """铲糖循环配置。"""
    # 导航点配置
    navigation_point: Tuple[float, float] = Field(
        description="取糖导航点 (x, y)"
    )
    dump_point: Tuple[float, float] = Field(
        description="卸载点 (x, y)"
    )

    # 设备参数
    bucket_width_m: float = Field(default=0.6, description="铲斗宽度（米）")
    approach_offset_m: float = Field(default=0.05, description="到达距离前的偏移量")

    # 伺服角度
    scoop_position: float = Field(default=90.0, description="铲取位置（度）")
    dump_position: float = Field(default=135.0, description="倾倒位置（度）")

    # 循环控制
    max_cycles: int = Field(default=10, ge=1, le=100, description="最大循环次数")
    height_threshold_m: float = Field(default=0.20, description="切换推垛模式高度阈值（米）")


class SugarHarvestStartRequest(BaseModel):
    """启动铲糖循环请求。"""
    config: SugarHarvestConfig


class RobotStatusResponse(BaseModel):
    """机器人状态响应。"""
    state: RobotState
    battery_level: float
    current_position: Tuple[float, float, float]
    orientation: Tuple[float, float, float]
    left_track_speed: float
    right_track_speed: float
    servos: Dict[str, Dict[str, Any]]
    timestamp: datetime


# ==================== 全局状态 ====================

# 铲糖循环引擎实例
_sugar_harvest_engine: Optional[BehaviorTreeEngine] = None
_harvest_task: Optional[asyncio.Task] = None


# ==================== 依赖注入 ====================


def get_robot_ctrl() -> RobotController:
    """获取机器人控制器实例。"""
    return get_robot_controller()


def get_distance_analyzer_service() -> DistanceAnalyzer:
    """获取距离分析器实例。"""
    return get_distance_analyzer()


# ==================== 运动控制端点 ====================


# 注：原有的运动控制、伺服控制等API端点已删除，改用TCP协议直接与机器人通信






# ==================== 伺服控制端点 ====================


@router.post("/servo", response_model=Dict[str, Any])
async def control_servo(
    request: ServoRequest,
    robot: RobotController = Depends(get_robot_ctrl)
):
    """控制伺服电机。

    可用伺服：
    - lift: 铲斗举升
    - dump: 翻斗
    """
    try:
        success = await robot.set_servo_angle(
            servo_id=request.servo_id,
            angle=request.angle,
        )

        status = robot.get_status()
        servo_status = status.servos.get(request.servo_id)

        return {
            "success": success,
            "message": f"伺服 {request.servo_id} 角度已设置为 {request.angle}°",
            "servo_status": {
                "current_angle": servo_status.current_angle if servo_status else 0,
                "is_moving": servo_status.is_moving if servo_status else False,
            } if servo_status else None,
        }

    except Exception as e:
        logger.error(f"伺服控制失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/scoop", response_model=Dict[str, Any])
async def scoop_action(robot: RobotController = Depends(get_robot_ctrl)):
    """执行铲取动作。"""
    try:
        success = await robot.scoop()
        return {
            "success": success,
            "message": "铲取动作已完成",
            "status": robot.get_status().state,
        }

    except Exception as e:
        logger.error(f"铲取动作失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/dump", response_model=Dict[str, Any])
async def dump_action(robot: RobotController = Depends(get_robot_ctrl)):
    """执行倾倒动作。"""
    try:
        success = await robot.dump()
        return {
            "success": success,
            "message": "倾倒动作已完成",
            "status": robot.get_status().state,
        }

    except Exception as e:
        logger.error(f"倾倒动作失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/dock", response_model=Dict[str, Any])
async def dock_robot(robot: RobotController = Depends(get_robot_ctrl)):
    """控制机器人回桩。"""
    try:
        success = await robot.dock()
        return {
            "success": success,
            "message": "回桩指令已执行",
            "status": robot.get_status().state,
        }

    except Exception as e:
        logger.error(f"回桩失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))



# ==================== 状态查询端点 ====================


@router.get("/status", response_model=RobotStatusResponse)
async def get_robot_status(robot: RobotController = Depends(get_robot_ctrl)):
    """获取机器人当前状态。"""
    try:
        status = robot.get_status()

        return RobotStatusResponse(
            state=status.state,
            battery_level=status.battery_level,
            current_position=status.current_position,
            orientation=status.orientation,
            left_track_speed=status.left_track_speed,
            right_track_speed=status.right_track_speed,
            servos={
                k: {
                    "name": v.name,
                    "current_angle": v.current_angle,
                    "target_angle": v.target_angle,
                    "is_moving": v.is_moving,
                }
                for k, v in status.servos.items()
            },
            timestamp=status.timestamp,
        )

    except Exception as e:
        logger.error(f"获取状态失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# ==================== 距离分析端点 ====================


@router.post("/distance_analyze", response_model=DistanceAnalysisResult)
async def analyze_distance(
    request: DistanceAnalysisRequest,
    analyzer: DistanceAnalyzer = Depends(get_distance_analyzer_service),
):
    """分析糖堆距离和高度。

    使用 RealSense 点云数据计算：
    - 最深入点的距离（用于前进）
    - 糖堆高度（用于判断是否切换推垛模式）

    注意：当前使用模拟点云数据，真实点云数据需要从设备获取。
    """
    try:
        # TODO: 从 RealSense 设备获取真实点云数据
        # 当前使用模拟数据进行演示
        import numpy as np
        np.random.seed(42)
        n_points = 100
        point_cloud = np.random.rand(n_points, 3) * 2
        point_cloud[:, 0] += 1.0  # X 轴偏移，模拟距离 1m
        point_cloud[:, 2] = np.random.rand(n_points) * 0.5  # Z 轴高度变化

        result = analyzer.analyze(point_cloud)

        if result is None:
            raise HTTPException(
                status_code=400,
                detail="距离分析失败：点云数据不足或无效"
            )

        return result

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"距离分析失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# ==================== 铲糖自主循环端点 ====================


@router.post("/auto_cycle/start", response_model=Dict[str, Any])
async def start_sugar_harvest_cycle(
    request: SugarHarvestStartRequest,
    robot: RobotController = Depends(get_robot_ctrl),
):
    """启动铲糖自主循环。

    循环流程：
    1. 导航到取糖点
    2. RealSense 分析糖堆距离和高度
    3. 检查糖堆高度（< 20cm 时切换推垛模式）
    4. 前进铲糖
    5. 原路倒退
    6. 导航到卸载点
    7. 翻斗卸载
    8. 重复直到达到最大循环次数

    循环终止条件：
    - 达到最大循环次数
    - 糖堆高度 < 20cm（切换推垛模式）
    - 手动停止
    """
    global _sugar_harvest_engine, _harvest_task

    try:
        # 检查是否已有循环在运行
        if _sugar_harvest_engine and _sugar_harvest_engine.is_running:
            raise HTTPException(
                status_code=400,
                detail="铲糖循环已在运行中"
            )

        # 获取服务实例
        navigation_service = get_mock_navigation()
        distance_analyzer = get_distance_analyzer()

        # 创建行为树引擎
        _sugar_harvest_engine = create_sugar_harvest_engine(
            robot_controller=robot,
            navigation_service=navigation_service,
            distance_analyzer=distance_analyzer,
            task_params=request.config.dict(),
        )

        # 启动循环任务
        engine_ref = _sugar_harvest_engine

        async def run_harvest():
            try:
                await engine_ref.start()
            except Exception as e:
                logger.error(f"铲糖循环执行失败: {e}", exc_info=True)

        _harvest_task = asyncio.create_task(run_harvest())

        # 任务完成后清理
        def cleanup_task(task):
            global _sugar_harvest_engine
            _sugar_harvest_engine = None

        _harvest_task.add_done_callback(cleanup_task)

        return {
            "success": True,
            "message": "铲糖自主循环已启动",
            "config": request.config.dict(),
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"启动铲糖循环失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/auto_cycle/stop", response_model=Dict[str, Any])
async def stop_sugar_harvest_cycle():
    """停止铲糖自主循环。"""
    global _sugar_harvest_engine, _harvest_task

    try:
        if _sugar_harvest_engine is None:
            raise HTTPException(
                status_code=400,
                detail="没有正在运行的铲糖循环"
            )

        # 请求停止行为树
        await _sugar_harvest_engine.stop()

        # 取消任务
        if _harvest_task and not _harvest_task.done():
            _harvest_task.cancel()
            try:
                await _harvest_task
            except asyncio.CancelledError:
                pass

        _sugar_harvest_engine = None
        _harvest_task = None

        return {
            "success": True,
            "message": "铲糖自主循环已停止",
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"停止铲糖循环失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/auto_cycle/status", response_model=Dict[str, Any])
async def get_sugar_harvest_status():
    """获取铲糖循环状态。"""
    global _sugar_harvest_engine

    is_running = _sugar_harvest_engine is not None and _sugar_harvest_engine.is_running

    status_info = {
        "is_running": is_running,
    }

    if _sugar_harvest_engine:
        status_info.update({
            "current_cycle": _sugar_harvest_engine.context.current_cycle,
            "max_cycles": _sugar_harvest_engine.context.max_cycles,
            "sugar_height": _sugar_harvest_engine.context.sugar_height,
            "height_threshold": _sugar_harvest_engine.context.height_threshold,
            "blackboard": _sugar_harvest_engine.context.blackboard,
        })

    return status_info


@router.get("/tcp/status", response_model=Dict[str, Any])
async def get_tcp_robot_status(
    tcp_server: Optional[RobotTCPServer] = Depends(get_robot_tcp_server)
):
    """获取通过 TCP 连接的机器人状态。

    返回:
    - 连接状态: 是否有机器人连接
    - 连接数: 当前连接的机器人数量
    - 每个机器人的详细状态: 电量、位置、模式等
    """
    if not tcp_server:
        return {
            "connected": False,
            "count": 0,
            "message": "TCP 服务器未启动",
            "robots": []
        }

    robot_ids = tcp_server.get_robot_ids()
    robots = []

    for rid in robot_ids:
        state = tcp_server.get_robot_state(rid)
        if state:
            robots.append({
                "id": rid,
                "mode": state.mode.value,
                "status": state.status.value,
                "charge": state.charge,
                "speed": state.speed,
                "fault": state.fault,
                "fault_level": state.fault_level,
                "task_id": state.task_id,
                "station": state.station,
                "map_name": state.map_name,
                "position": {"x": state.x, "y": state.y, "z": state.z, "a": state.a},
                "boom": state.boom,
                "bucket": state.bucket,
                "last_update": state.last_update.isoformat() if state.last_update else None
            })

    return {
        "connected": len(robots) > 0,
        "count": len(robots),
        "message": f"{len(robots)} 个机器人已连接" if robots else "无机器人连接",
        "robots": robots
    }


# ==================== WebSocket 端点 ====================


@router.websocket("/ws")
async def robot_websocket(websocket: WebSocket):
    """WebSocket 端点，用于实时推送机器人连接状态。

    连接建立后，服务器会每 1 秒检查一次机器人 TCP 连接状态，
    并在状态变化时推送给客户端。

    消息格式:
    - 连接状态: {"type": "robot_connection", "connected": true/false}
    """
    await websocket.accept()
    logger.info("机器人状态 WebSocket 连接已建立")

    # 获取 TCP Server 实例
    tcp_server = get_robot_tcp_server()

    # 上一次的状态，用于检测变化
    last_connected = None

    try:
        while True:
            # 检查当前连接状态
            current_connected = False
            if tcp_server:
                robot_ids = tcp_server.get_robot_ids()
                current_connected = len(robot_ids) > 0

            # 状态变化时推送
            if current_connected != last_connected:
                message = {
                    "type": "robot_connection",
                    "connected": current_connected
                }
                await websocket.send_json(message)
                logger.debug(f"机器人连接状态推送: {current_connected}")
                last_connected = current_connected

            # 每 1 秒检查一次
            await asyncio.sleep(1)

    except WebSocketDisconnect:
        logger.info("机器人状态 WebSocket 连接已断开")
    except Exception as e:
        logger.error(f"WebSocket 错误: {e}")
        try:
            await websocket.close()
        except:
            pass
