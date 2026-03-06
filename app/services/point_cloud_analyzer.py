"""点云分析器

负责在后端进行点云数据分析和计算，包括：
- 体积计算（在指定宽度范围内积分）
- 最近物料点检测
- 达到目标体积的深度位置计算
"""

import numpy as np
from typing import Tuple, Optional, List, Dict, Any
from dataclasses import dataclass


@dataclass
class PointCloudAnalysisResult:
    """点云分析结果"""
    # 体积相关
    actual_volume: float          # 实际计算到的体积 (m³)
    target_volume: float        # 目标体积 (m³)
    volume_reached: bool        # 是否达到目标体积
    target_depth_x: Optional[float] = None  # 达到目标体积时的X深度 (从相机起算)

    # 最近物料点
    nearest_x: Optional[float] = None     # 最近物料的X坐标
    nearest_y: Optional[float] = None     # 最近物料的Y坐标
    material_distance: Optional[float] = None  # 铲齿到物料的距离

    # 堆体高度
    pile_height: float = 0.0  # 堆体高度 (m)，相机高度 - 最高点Z坐标
    pile_max_z: Optional[float] = None  # 最高点Z坐标
    camera_height: float = 1.0  # 相机高度 (m)，默认1m

    # 状态
    has_material: bool = False  # 是否在范围内检测到物料



class PointCloudAnalyzer:
    """点云分析器"""

    def __init__(
        self,
        cell_size: float = 0.02,           # 格子大小 (m)
        bucket_width: float = 0.6,         # 铲斗宽度 (m)
        working_range_x: float = 2.0,      # X方向工作范围 (m)
    ):
        self.cell_size = cell_size
        self.bucket_width = bucket_width
        self.working_range_x = working_range_x
        self.bucket_half_width = bucket_width / 2

    def analyze(
        self,
        point_cloud: np.ndarray,
        target_volume: float,
        camera_to_teeth: float,
        z1: float,
        z2: float,
    ) -> PointCloudAnalysisResult:
        """
        分析点云数据

        Args:
            point_cloud: Float32Array，格式为 [x0, y0, z0, x1, y1, z1, ...]
            target_volume: 目标体积 (L)
            camera_to_teeth: 相机到铲齿距离 (m)
            z1: 铲齿高度 (m)
            z2: 铲齿+斗高 (m)

        Returns:
            PointCloudAnalysisResult: 分析结果
        """
        if point_cloud is None or len(point_cloud) == 0:
            return PointCloudAnalysisResult(
                actual_volume=0.0,
                target_volume=target_volume / 1000,  # L to m³
                volume_reached=False,
                has_material=False,
            )

        # 转换为 numpy 数组并 reshape 为 [N, 3]
        points = point_cloud.reshape(-1, 3)

        # 工作范围定义
        min_x = camera_to_teeth
        max_x = camera_to_teeth + self.working_range_x
        min_y = -self.bucket_half_width
        max_y = self.bucket_half_width

        # 过滤在工作范围内的点
        mask = (
            (points[:, 0] >= min_x) & (points[:, 0] <= max_x) &  # X范围
            (points[:, 1] >= min_y) & (points[:, 1] <= max_y) &  # Y范围（铲斗宽度）
            (points[:, 2] >= z1) & (points[:, 2] <= z2)          # Z范围
        )

        valid_points = points[mask]

        if len(valid_points) == 0:
            return PointCloudAnalysisResult(
                actual_volume=0.0,
                target_volume=target_volume / 1000,
                volume_reached=False,
                has_material=False,
            )

        # 找到最近物料点（X坐标最小的点）
        nearest_idx = np.argmin(valid_points[:, 0])
        nearest_point = valid_points[nearest_idx]
        nearest_x = nearest_point[0]
        nearest_y = nearest_point[1]

        # 计算铲齿到物料的距离
        material_distance = nearest_x - camera_to_teeth

        # 计算体积（简化方法：平均高度 × 底面积）
        avg_height = np.mean(valid_points[:, 2]) - z1  # 相对于Z1的高度
        base_area = self.working_range_x * self.bucket_width
        actual_volume = max(0.0, avg_height * base_area)

        # 计算堆体高度（在X≤3m, Y∈[-0.3,0.3]范围内找最高点）
        # 相机高度0.6m（从地面起算）
        camera_height = 0.6
        pile_search_range_x = 3.0  # X≤3m

        # 在valid_points中进一步筛选X≤3m的点
        pile_mask = valid_points[:, 0] <= pile_search_range_x
        pile_points = valid_points[pile_mask]

        if len(pile_points) > 0:
            # 找到最高点的Z坐标（Z坐标是相机直接输出的值）
            pile_max_z = np.max(pile_points[:, 2])
            # 堆体高度 = 相机高度 + 最高点Z坐标
            # 例如：相机高度0.6m，最高点Z=0.2m
            # 堆体高度 = 0.6 + 0.2 = 0.8m
            pile_height = camera_height + pile_max_z
        else:
            pile_max_z = None
            pile_height = 0.0

        # 计算达到目标体积的深度
        target_volume_m3 = target_volume / 1000  # L to m³
        volume_reached = actual_volume >= target_volume_m3

        # 估算达到目标体积的深度（假设高度均匀分布）
        target_depth_x = None
        if volume_reached:
            # 已经达到，找到实际达到目标体积的深度
            # 按X坐标排序，找到累计体积达到目标的点
            sorted_indices = np.argsort(valid_points[:, 0])
            sorted_points = valid_points[sorted_indices]

            cumulative_volume = 0.0
            cell_area = self.cell_size * self.cell_size

            for i, point in enumerate(sorted_points):
                height = max(0.0, point[2] - z1)
                cell_volume = height * cell_area
                cumulative_volume += cell_volume

                if cumulative_volume >= target_volume_m3:
                    target_depth_x = point[0]
                    break
        else:
            # 未达到目标体积，估算需要的深度
            if actual_volume > 0:
                # 线性外推
                avg_height_current = actual_volume / base_area
                needed_height = target_volume_m3 / base_area
                depth_factor = needed_height / avg_height_current if avg_height_current > 0 else 1.0
                target_depth_x = min_x + (max_x - min_x) * depth_factor

        return PointCloudAnalysisResult(
            actual_volume=actual_volume,
            target_volume=target_volume_m3,
            volume_reached=volume_reached,
            target_depth_x=target_depth_x,
            nearest_x=nearest_x,
            nearest_y=nearest_y,
            material_distance=material_distance,
            pile_height=pile_height if 'pile_height' in locals() else 0.0,
            pile_max_z=pile_max_z if 'pile_max_z' in locals() else None,
            camera_height=camera_height if 'camera_height' in locals() else 1.0,
            has_material=True,
        )


def analyze_point_cloud(
    point_cloud: np.ndarray,
    target_volume: float,
    camera_to_teeth: float,
    z1: float,
    z2: float,
) -> PointCloudAnalysisResult:
    """
    便捷的独立函数接口

    Args:
        point_cloud: 点云数据，格式为 [x0, y0, z0, x1, y1, z1, ...]
        target_volume: 目标体积 (L)
        camera_to_teeth: 相机到铲齿距离 (m)
        z1: 铲齿高度 (m)
        z2: 铲齿+斗高 (m)

    Returns:
        PointCloudAnalysisResult: 分析结果
    """
    analyzer = PointCloudAnalyzer()
    return analyzer.analyze(point_cloud, target_volume, camera_to_teeth, z1, z2)
