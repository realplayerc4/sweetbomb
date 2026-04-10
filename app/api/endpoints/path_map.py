"""PathMap API 端点"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional

from app.services.path_map_manager import PathMapManager

router = APIRouter()

# 全局 PathMapManager 实例
_manager: Optional[PathMapManager] = None


def get_manager() -> PathMapManager:
    """获取 PathMapManager 实例"""
    global _manager
    if _manager is None:
        _manager = PathMapManager()
    return _manager


class PickStationUpdate(BaseModel):
    """取货站更新请求"""
    ox: Optional[float] = None
    oy: Optional[float] = None
    r: Optional[float] = None
    station_num: Optional[int] = None
    max_speed: Optional[float] = None


class DropStationUpdate(BaseModel):
    """放货站更新请求"""
    node: Optional[int] = None
    boom_pos: Optional[float] = None
    bucket_pos: Optional[float] = None
    bucket_out_pos: Optional[float] = None


@router.get("/")
async def get_path_map():
    """获取完整 pathMap 数据"""
    manager = get_manager()
    try:
        data = manager.parse_xml()
        return data
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"解析 pathMap.xml 失败: {str(e)}")


@router.get("/stations")
async def get_stations():
    """获取站点数据（简化视图）"""
    manager = get_manager()
    try:
        return manager.get_stations()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"获取站点数据失败: {str(e)}")


@router.put("/pick_station/{id}")
async def update_pick_station(id: int, update: PickStationUpdate):
    """更新取货站参数"""
    manager = get_manager()
    try:
        result = manager.update_pick_station(id, **update.model_dump(exclude_none=True))
        return {"success": True, "data": result}
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"更新取货站失败: {str(e)}")


@router.put("/drop_station/{id}")
async def update_drop_station(id: int, update: DropStationUpdate):
    """更新放货站参数"""
    manager = get_manager()
    try:
        result = manager.update_drop_station(id, **update.model_dump(exclude_none=True))
        return {"success": True, "data": result}
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"更新放货站失败: {str(e)}")