"""
地图 API 端点

提供地图图片的获取和管理功能。
地图从 txt 格式转换为 PNG/SVG 图片后提供。
"""

import os
from pathlib import Path
from typing import List, Optional

from fastapi import APIRouter, HTTPException, Query, Response
from fastapi.responses import FileResponse, StreamingResponse

from app.services.map_converter import MapConverter, convert_map

router = APIRouter()

# 地图数据目录
MAP_DIR = Path(__file__).parent.parent.parent.parent / "data" / "map"
CACHE_DIR = MAP_DIR / "cache"
CACHE_DIR.mkdir(parents=True, exist_ok=True)


def get_map_files() -> List[Path]:
    """获取所有地图 txt 文件列表"""
    if not MAP_DIR.exists():
        return []
    return sorted(MAP_DIR.glob("*.txt"))


def get_cached_png(txt_path: Path) -> Optional[Path]:
    """获取缓存的 PNG 文件路径，如果不存在则返回 None"""
    cache_path = CACHE_DIR / f"{txt_path.stem}.png"
    if cache_path.exists():
        # 检查 txt 文件是否比缓存新
        if txt_path.stat().st_mtime <= cache_path.stat().st_mtime:
            return cache_path
    return None


def generate_png(txt_path: Path, point_size: float = 1.0, dpi: int = 150) -> Path:
    """生成 PNG 图片并缓存"""
    cache_path = CACHE_DIR / f"{txt_path.stem}.png"

    # 使用 map_converter 服务生成 PNG
    convert_map(
        input_path=str(txt_path),
        output_path=str(cache_path),
        format="png",
        point_size=point_size,
        dpi=dpi,
    )

    return cache_path


@router.get("/")
async def list_maps():
    """
    获取所有可用地图列表

    返回地图文件列表，包含文件名、大小、生成时间等信息
    """
    map_files = get_map_files()

    maps = []
    for map_file in map_files:
        stat = map_file.stat()

        # 检查是否有缓存的 PNG
        cached_png = get_cached_png(map_file)

        maps.append({
            "filename": map_file.name,
            "name": map_file.stem,
            "size_bytes": stat.st_size,
            "modified_time": stat.st_mtime,
            "has_cached_png": cached_png is not None,
            "png_url": f"/api/map/{map_file.stem}.png" if cached_png else None,
        })

    return {
        "total": len(maps),
        "maps": maps,
    }


@router.get("/{map_name}.png")
async def get_map_png(
    map_name: str,
    refresh: bool = Query(False, description="强制重新生成缓存"),
    point_size: float = Query(1.0, description="点大小倍数", ge=0.1, le=5.0),
    dpi: int = Query(150, description="图片分辨率", ge=72, le=300),
):
    """
    获取地图 PNG 图片

    自动将 txt 地图转换为 PNG 图片格式。支持缓存机制，避免重复转换。
    """
    # 查找 txt 文件
    txt_path = MAP_DIR / f"{map_name}.txt"
    if not txt_path.exists():
        # 尝试查找其他可能的文件名
        for ext in [".txt", "_gridmap.txt", ".gridmap"]:
            candidate = MAP_DIR / f"{map_name}{ext}"
            if candidate.exists():
                txt_path = candidate
                break

    if not txt_path.exists():
        raise HTTPException(
            status_code=404,
            detail=f"地图文件不存在: {map_name}"
        )

    # 检查缓存
    if not refresh:
        cached = get_cached_png(txt_path)
        if cached:
            return FileResponse(
                cached,
                media_type="image/png",
                headers={
                    "Cache-Control": "public, max-age=86400",
                    "X-Cache": "HIT",
                }
            )

    # 生成新的 PNG
    try:
        png_path = generate_png(txt_path, point_size, dpi)
        return FileResponse(
            png_path,
            media_type="image/png",
            headers={
                "Cache-Control": "public, max-age=86400",
                "X-Cache": "MISS",
            }
        )
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"地图转换失败: {str(e)}"
        )


@router.get("/{map_name}/info")
async def get_map_info(map_name: str):
    """
    获取地图详细信息

    返回地图的元数据，包括分辨率、边界框、点数等
    """
    # 查找 txt 文件
    txt_path = MAP_DIR / f"{map_name}.txt"
    if not txt_path.exists():
        # 尝试其他可能的文件名
        for ext in [".txt", "_gridmap.txt", ".gridmap"]:
            candidate = MAP_DIR / f"{map_name}{ext}"
            if candidate.exists():
                txt_path = candidate
                break

    if not txt_path.exists():
        raise HTTPException(
            status_code=404,
            detail=f"地图文件不存在: {map_name}"
        )

    # 解析地图
    try:
        converter = MapConverter()
        map_data = converter.parse_grid_map(str(txt_path))

        return {
            "filename": txt_path.name,
            "name": map_name,
            "resolution": map_data.resolution,
            "origin": {
                "x": map_data.origin_x,
                "y": map_data.origin_y,
            },
            "grid_size": {
                "width": map_data.grid_width,
                "height": map_data.grid_height,
            },
            "bounds": {
                "min_x": map_data.bounds[0],
                "min_y": map_data.bounds[1],
                "max_x": map_data.bounds[2],
                "max_y": map_data.bounds[3],
            },
            "point_count": len(map_data.points),
            "png_url": f"/api/map/{map_name}.png",
        }

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"解析地图失败: {str(e)}"
        )


# 清理缓存的 API（管理员功能）
@router.post("/cache/clear")
async def clear_cache():
    """清理所有地图缓存"""
    import shutil

    if CACHE_DIR.exists():
        # 删除所有缓存文件
        for file in CACHE_DIR.glob("*.png"):
            file.unlink()

    return {
        "message": "地图缓存已清理",
        "cache_dir": str(CACHE_DIR),
    }
