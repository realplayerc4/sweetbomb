"""PathMap 数据模型 - 用于解析机器人路径系统 pathMap.xml"""

from typing import List, Optional
from pydantic import BaseModel, Field


class Node(BaseModel):
    """路径节点"""
    id: int
    x: float  # 坐标 mm
    y: float  # 坐标 mm


class Segment(BaseModel):
    """路径线段"""
    id: int
    type: str  # line 或 arcm
    begin_node: int
    end_node: int
    nav_type: str  # laser, rtk, rfid
    max_speed: float
    move_direction: str  # forward 或 backward
    # 弧线参数（仅 arcm 类型有效）
    ox: Optional[float] = None  # 圆心 X
    oy: Optional[float] = None  # 圆心 Y
    direction: Optional[str] = None  # cw 或 ccw


class PickStationPosition(BaseModel):
    """取货位坐标"""
    station_id: int  # 站号 (如 101, 102...)
    x: float  # 坐标 mm
    y: float  # 坐标 mm


class PickStation(BaseModel):
    """取货站"""
    id: int
    connect_node: int  # 连接的节点 ID
    start_id: int  # 起始站号（如 101）
    ox: float  # 圆心 X (mm)
    oy: float  # 圆心 Y (mm)
    r: float = Field(alias="R")  # 半径 (mm)
    station_num: int  # 站位数量
    max_speed: float  # 对接最大速度
    # 预计算的所有取货位坐标
    positions: List[PickStationPosition] = []

    class Config:
        populate_by_name = True  # 允许使用别名 R


class DropStation(BaseModel):
    """放货站"""
    id: int
    connect_node: int  # 连接的节点 ID
    node: int  # 放货节点 ID
    boom_pos: float  # 大臂位置
    bucket_pos: float  # 小臂位置
    bucket_out_pos: float  # 翻斗卸料位置
    # 预计算的坐标
    x: Optional[float] = None  # 坐标 mm
    y: Optional[float] = None  # 坐标 mm


class ChargeStation(BaseModel):
    """充电站"""
    id: int
    node: int  # 充电节点 ID
    vehicle_th: float  # 车体角度
    ip: str  # 充电桩 IP
    port: int  # 充电桩端口


class Actuator(BaseModel):
    """执行器参数"""
    pick_boom_pos: float
    pick_bucket_pos: float
    walk_run_boom_pos: float
    walk_run_bucket_pos: float


class PathMapData(BaseModel):
    """完整路径地图数据"""
    nodes: List[Node]
    segments: List[Segment]
    pick_stations: List[PickStation]
    drop_stations: List[DropStation]
    charge_stations: List[ChargeStation] = []
    actuator: Optional[Actuator] = None