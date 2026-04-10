from fastapi import APIRouter, Depends, HTTPException
from typing import Optional
from datetime import datetime
from pydantic import BaseModel

from app.models.stream import PointCloudStatus
from app.services.rs_manager import RealSenseManager
from app.api.dependencies import get_realsense_manager

router = APIRouter()


# Pydantic models for API responses
class PointCloudSettings(BaseModel):
    """点云分析参数设置。"""
    teethHeight: float      # Z1: 铲齿放平时的高度 (米)
    cameraToTeeth: float    # 相机到铲齿前沿距离 (米)
    bucketDepth: float      # 铲斗深度 (米)
    bucketVolume: float     # 铲斗目标体积 (升)


class PointCloudSettingsResponse(BaseModel):
    """点云设置响应。"""
    device_id: str
    settings: PointCloudSettings
    message: str = "Settings updated successfully"


class PointCloudAnalysisResponse(BaseModel):
    """点云分析结果响应模型。"""
    device_id: str
    volume: float  # 实际体积 (m³)
    target_volume: float  # 目标体积 (m³)
    volume_reached: bool  # 是否达到目标体积
    pile_height: float  # 堆体高度 (m)
    material_distance: Optional[float]  # 铲齿到物料距离 (m)
    nearest_point: Optional[tuple]  # 最近点坐标 (x, y, z)
    has_material: bool  # 是否检测到物料
    timestamp: Optional[datetime]  # 分析时间戳

    class Config:
        arbitrary_types_allowed = True


class MoveDistanceResponse(BaseModel):
    """前进距离响应模型。"""
    device_id: str
    move_distance: float  # 计算后的前进距离 (m)
    material_distance: Optional[float]  # 原始物料距离 (m)
    timestamp: Optional[datetime]  # 计算时间戳

    class Config:
        arbitrary_types_allowed = True

@router.post("/activate", response_model=PointCloudStatus)
async def activate_point_cloud(
    device_id: str,
    rs_manager: RealSenseManager = Depends(get_realsense_manager),
):
    try:
        return rs_manager.activate_point_cloud(device_id, True)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.post("/deactivate", response_model=PointCloudStatus)
async def deactivate_point_cloud(
    device_id: str,
    rs_manager: RealSenseManager = Depends(get_realsense_manager),
):
    try:
        return rs_manager.activate_point_cloud(device_id, False)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.get("/status", response_model=PointCloudStatus)
async def get_stream_status(
    device_id: str,
    rs_manager: RealSenseManager = Depends(get_realsense_manager)
):
    try:
        return rs_manager.get_point_cloud_status(device_id)
    except Exception as e:
        raise HTTPException(status_code=404, detail=str(e))


@router.get("/analysis", response_model=PointCloudAnalysisResponse)
async def get_point_cloud_analysis(
    device_id: str,
    rs_manager: RealSenseManager = Depends(get_realsense_manager)
):
    """获取最新的点云分析结果。

    此端点返回后端计算的点云分析结果，包括：
    - 体积 (m³)
    - 堆体高度 (m)
    - 铲齿到物料距离 (m)
    - 最近点坐标

    结果由后端独立计算，不依赖前端连接。
    """
    try:
        result = rs_manager.get_analysis_result(device_id)
        if result is None:
            raise HTTPException(
                status_code=404,
                detail=f"No analysis result available for device {device_id}. "
                       f"Make sure point cloud is activated and streaming."
            )

        return PointCloudAnalysisResponse(
            device_id=device_id,
            volume=result.actual_volume,
            target_volume=result.target_volume,
            volume_reached=result.volume_reached,
            pile_height=result.pile_height,
            material_distance=result.material_distance,
            nearest_point=(result.nearest_x, result.nearest_y, result.pile_max_z)
            if result.nearest_x is not None and result.nearest_y is not None
            else None,
            has_material=result.has_material,
            timestamp=rs_manager.get_analysis_timestamp(device_id),
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/move_distance", response_model=MoveDistanceResponse)
async def get_move_distance(
    device_id: str,
    rs_manager: RealSenseManager = Depends(get_realsense_manager)
):
    """获取计算后的前进距离（move_distance）。

    此端点返回后端计算的 move_distance。
    结果由后端独立计算，可直接用于机器人控制。
    """
    try:
        move_distance = rs_manager.get_move_distance(device_id)
        if move_distance is None:
            raise HTTPException(
                status_code=404,
                detail=f"No move distance available for device {device_id}. "
                       f"Make sure point cloud is activated and streaming."
            )

        # 获取原始 material_distance
        analysis_result = rs_manager.get_analysis_result(device_id)
        material_distance = analysis_result.material_distance if analysis_result else None

        return MoveDistanceResponse(
            device_id=device_id,
            move_distance=move_distance,
            material_distance=material_distance,
            timestamp=rs_manager.get_analysis_timestamp(device_id),
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/{device_id}/settings", response_model=PointCloudSettingsResponse)
async def update_pointcloud_settings(
    device_id: str,
    settings: PointCloudSettings,
    rs_manager: RealSenseManager = Depends(get_realsense_manager),
):
    """更新点云分析参数。

    前端修改相机高度、铲齿高度等参数时调用，后端实时生效。
    """
    try:
        updated = rs_manager.update_pointcloud_settings(device_id, {
            "teeth_height": settings.teethHeight,
            "camera_to_teeth": settings.cameraToTeeth,
            "bucket_depth": settings.bucketDepth,
            "bucket_volume": settings.bucketVolume,
        })
        return PointCloudSettingsResponse(
            device_id=device_id,
            settings=PointCloudSettings(
                teethHeight=updated["teeth_height"],
                cameraToTeeth=updated["camera_to_teeth"],
                bucketDepth=updated["bucket_depth"],
                bucketVolume=updated["bucket_volume"],
            ),
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))