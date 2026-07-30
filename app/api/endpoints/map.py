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

from app.services.map_converter import MapConverter, convert_map, coordinate_rotate, coordinate_rotate

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


def get_cached_png(txt_path: Path, theta: float = 0.0) -> Optional[Path]:
    """获取缓存的 PNG 文件路径，如果不存在则返回 None"""
    suffix = "" if theta == 0.0 else f"_theta{theta:.4f}"
    cache_path = CACHE_DIR / f"{txt_path.stem}{suffix}.png"
    if cache_path.exists():
        # 检查 txt 文件是否比缓存新
        if txt_path.stat().st_mtime <= cache_path.stat().st_mtime:
            return cache_path
    return None


def generate_png(txt_path: Path, point_size: float = 1.0, dpi: int = 150, theta: float = 0.0) -> tuple:
    """生成 PNG 图片并缓存，返回路径和偏移量信息"""
    suffix = "" if theta == 0.0 else f"_theta{theta:.4f}"
    cache_path = CACHE_DIR / f"{txt_path.stem}{suffix}.png"

    # 使用 map_converter 服务生成 PNG
    converter = MapConverter()
    map_data = converter.parse_grid_map(str(txt_path))
    result = converter.to_png(map_data, str(cache_path), point_size, dpi, theta=theta)

    # result 是 (path, offset_info) 的元组
    if isinstance(result, tuple):
        return result[0], result[1]
    return cache_path, {}


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
    theta: float = Query(0.0, description="旋转角度（度）"),
):
    """
    获取地图 PNG 图片

    自动将 txt 地图转换为 PNG 图片格式。支持缓存机制，避免重复转换。

    参数:
        theta: 旋转角度（度），逆时针 > 0，顺时针 < 0
    """
    # 将角度转换为弧度
    theta_rad = theta * 3.141592653589793 / 180.0
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

    # 检查缓存（使用弧度值）
    if not refresh:
        cached = get_cached_png(txt_path, theta_rad)
        if cached:
            return FileResponse(
                cached,
                media_type="image/png",
                headers={
                    "Cache-Control": "public, max-age=86400",
                    "X-Cache": "HIT",
                }
            )

    # 生成新的 PNG（传递弧度值），同时获取偏移量信息
    try:
        png_path, offset_info = generate_png(txt_path, point_size, dpi, theta_rad)
        return FileResponse(
            png_path,
            media_type="image/png",
            headers={
                "Cache-Control": "public, max-age=86400",
                "X-Cache": "MISS",
                "X-Offset-Left": str(offset_info.get('offset_x_left', 0)),
                "X-Offset-Top": str(offset_info.get('offset_y_top', 0)),
                "X-Offset-Bottom": str(offset_info.get('offset_y_bottom', 0)),
                "X-Data-Width": str(offset_info.get('data_width_px', 0)),
                "X-Data-Height": str(offset_info.get('data_height_px', 0)),
            }
        )
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"地图转换失败: {str(e)}"
        )


@router.get("/{map_name}/info")
async def get_map_info(
    map_name: str,
    theta: float = Query(0.0, description="旋转角度（度）"),
):
    """
    获取地图详细信息

    返回地图的元数据，包括分辨率、边界框、点数等。
    当提供theta参数时，返回旋转后的边界信息。
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
        import math
        converter = MapConverter()
        map_data = converter.parse_grid_map(str(txt_path))

        theta_rad = theta * math.pi / 180.0

        # 生成PNG获取偏移量信息
        png_path, offset_info = generate_png(txt_path, 1.0, 150, theta_rad)

        # 计算旋转后的边界
        corners = [
            (map_data.bounds[0], map_data.bounds[1]),  # min_x, min_y
            (map_data.bounds[2], map_data.bounds[1]),  # max_x, min_y
            (map_data.bounds[2], map_data.bounds[3]),  # max_x, max_y
            (map_data.bounds[0], map_data.bounds[3]),  # min_x, max_y
        ]
        rotated_corners = [coordinate_rotate(x, y, theta_rad) for x, y in corners]
        rx = [p[0] for p in rotated_corners]
        ry = [p[1] for p in rotated_corners]

        # 旋转后的数据范围（包含margin）
        data_width = max(rx) - min(rx)
        data_height = max(ry) - min(ry)
        margin = max(data_width, data_height) * 0.05
        png_bounds = {
            "min_x": min(rx) - margin,
            "min_y": min(ry) - margin,
            "max_x": max(rx) + margin,
            "max_y": max(ry) + margin,
        }

        # 从offset_info获取实际PNG的偏移量和尺寸
        offset_x_left = offset_info.get('offset_x_left', 0)
        offset_y_top = offset_info.get('offset_y_top', 0)
        offset_y_bottom = offset_info.get('offset_y_bottom', 0)
        data_width_px = offset_info.get('data_width_px', 0)
        data_height_px = offset_info.get('data_height_px', 0)
        actual_width_px = offset_info.get('actual_width_px', 0)
        actual_height_px = offset_info.get('actual_height_px', 0)

        # axes实际显示的数据范围（精确匹配PNG中的显示区域）
        axes_min_x = offset_info.get('axes_min_x', png_bounds['min_x'])
        axes_max_x = offset_info.get('axes_max_x', png_bounds['max_x'])
        axes_min_y = offset_info.get('axes_min_y', png_bounds['min_y'])
        axes_max_y = offset_info.get('axes_max_y', png_bounds['max_y'])

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
            "rotated_bounds": {
                "min_x": min(rx),
                "min_y": min(ry),
                "max_x": max(rx),
                "max_y": max(ry),
            },
            "png_bounds": png_bounds,
            "img_width_px": round(actual_width_px, 1),
            "img_height_px": round(actual_height_px, 1),
            "point_count": len(map_data.points),
            "png_url": f"/api/map/{map_name}.png",
            # 像素偏移量（用于前端车辆定位）
            "offset": {
                "x_left": round(offset_x_left, 1),
                "y_top": round(offset_y_top, 1),
                "y_bottom": round(offset_y_bottom, 1),
            },
            "data_area_px": {
                "width": round(data_width_px, 1),
                "height": round(data_height_px, 1),
            },
            # axes实际显示的数据范围（精确匹配PNG显示区域）
            "axes_range": {
                "min_x": round(axes_min_x, 6),
                "max_x": round(axes_max_x, 6),
                "min_y": round(axes_min_y, 6),
                "max_y": round(axes_max_y, 6),
            },
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
