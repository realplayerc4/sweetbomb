"""距离分析器。

基于点云数据计算糖堆距离和高度，采用"最深入点"算法。
铲斗宽度 600mm，在宽度范围内筛选点云并计算距离和高度。
"""

import logging
from typing import Optional, Tuple
from dataclasses import dataclass
from datetime import datetime
import numpy as np
from pydantic import BaseModel

logger = logging.getLogger(__name__)


class DistanceAnalysisResult(BaseModel):
    """距离分析结果。

    包含糖堆距离、高度、有效点数和最深入点坐标。
    """
    distance_m: float                    # 最深入点的距离（米）
    sugar_height_m: float                # 糖堆高度（米）
    point_count: int                     # 有效点云点数
    deepest_point: Tuple[float, float, float]  # 最深入点坐标 (x, y, z)
    bucket_width_m: float = 0.6          # 铲斗宽度 600mm
    should_switch_to_push: bool = False  # 是否需要切换到推垛模式
    timestamp: datetime = None

    class Config:
        # 允许 datetime 类型
        arbitrary_types_allowed = True


class DistanceAnalyzer:
    """距离分析器。

    算法：
    1. 在铲斗宽度范围内 (600mm) 筛选点云
    2. 找最深入的点（用于前进距离）
    3. 计算糖堆高度（最高点 - 最低点）
    4. 判断是否需要切换推垛模式（高度 < 20cm）

    精度要求不高，适合实时控制。
    """

    def __init__(
        self,
        bucket_width_m: float = 0.6,
        height_threshold_m: float = 0.20,
        min_points: int = 10,
    ):
        """初始化距离分析器。

        Args:
            bucket_width_m: 铲斗宽度（米），默认 600mm
            height_threshold_m: 切换推垛模式的高度阈值（米），默认 20cm
            min_points: 最小有效点数
        """
        self.bucket_width_m = bucket_width_m
        self.bucket_half_width = bucket_width_m / 2.0
        self.height_threshold_m = height_threshold_m
        self.min_points = min_points

    def analyze(
        self, point_cloud: np.ndarray
    ) -> Optional[DistanceAnalysisResult]:
        """分析点云数据，计算距离和高度。

        Args:
            point_cloud: Nx3 或 Nx4 点云数组，格式 [x, y, z, ...]
                        坐标系：X 轴为前进方向，Y 轴为左右方向，Z 轴为高度

        Returns:
            DistanceAnalysisResult: 分析结果，如果没有有效点则返回 None
        """
        if point_cloud is None or len(point_cloud) == 0:
            logger.warning("点云数据为空")
            return None

        # 提取 XYZ 坐标
        if point_cloud.shape[1] >= 3:
            xyz = point_cloud[:, :3]
        else:
            logger.error(f"无效的点云格式: {point_cloud.shape}")
            return None

        # 筛选铲斗宽度范围内的点
        # Y 轴（左右方向）在 [-bucket_half_width, bucket_half_width] 范围内
        y_mask = np.abs(xyz[:, 1]) <= self.bucket_half_width
        filtered_xyz = xyz[y_mask]

        if len(filtered_xyz) < self.min_points:
            logger.warning(
                f"铲斗宽度内有效点数不足: {len(filtered_xyz)} < {self.min_points}"
            )
            return None

        # 剔除无效点（NaN 或 Inf）
        valid_mask = np.isfinite(filtered_xyz).all(axis=1)
        filtered_xyz = filtered_xyz[valid_mask]

        if len(filtered_xyz) < self.min_points:
            logger.warning(
                f"剔除无效点后剩余点数不足: {len(filtered_xyz)} < {self.min_points}"
            )
            return None

        # 找最深的点（X 轴最小值 = 最近距离）
        deepest_idx = np.argmin(filtered_xyz[:, 0])
        deepest_point = filtered_xyz[deepest_idx]
        distance_m = deepest_point[0]

        # 计算糖堆高度（Z 轴最高点 - 最低点）
        z_values = filtered_xyz[:, 2]
        sugar_height_m = z_values.max() - z_values.min()

        # 判断是否需要切换推垛模式
        should_switch = sugar_height_m < self.height_threshold_m

        result = DistanceAnalysisResult(
            distance_m=float(distance_m),
            sugar_height_m=float(sugar_height_m),
            point_count=len(filtered_xyz),
            deepest_point=(
                float(deepest_point[0]),
                float(deepest_point[1]),
                float(deepest_point[2]),
            ),
            bucket_width_m=self.bucket_width_m,
            should_switch_to_push=should_switch,
            timestamp=datetime.now(),
        )

        logger.info(
            f"距离分析完成: 距离={distance_m:.3f}m, "
            f"高度={sugar_height_m:.3f}m, "
            f"有效点数={len(filtered_xyz)}, "
            f"切换推垛模式={should_switch}"
        )

        return result

    async def analyze_async(
        self, point_cloud: np.ndarray
    ) -> Optional[DistanceAnalysisResult]:
        """异步分析点云数据。

        Args:
            point_cloud: Nx3 或 Nx4 点云数组

        Returns:
            DistanceAnalysisResult: 分析结果
        """
        # 在实际应用中，这里可以异步处理大量点云数据
        return self.analyze(point_cloud)

    def set_bucket_width(self, width_m: float) -> None:
        """设置铲斗宽度。

        Args:
            width_m: 铲斗宽度（米）
        """
        self.bucket_width_m = width_m
        self.bucket_half_width = width_m / 2.0
        logger.info(f"铲斗宽度已更新: {width_m:.3f}m")

    def set_height_threshold(self, threshold_m: float) -> None:
        """设置推垛模式切换阈值。

        Args:
            threshold_m: 高度阈值（米）
        """
        self.height_threshold_m = threshold_m
        logger.info(f"推垛模式切换阈值已更新: {threshold_m:.3f}m")


# 全局配置
_analyzer_config = {
    "bucket_width_m": 0.6,
    "height_threshold_m": 0.20,
    "min_points": 10,
}


def get_distance_analyzer() -> DistanceAnalyzer:
    """获取配置好的距离分析器实例。"""
    return DistanceAnalyzer(
        bucket_width_m=_analyzer_config["bucket_width_m"],
        height_threshold_m=_analyzer_config["height_threshold_m"],
        min_points=_analyzer_config["min_points"],
    )


def update_analyzer_config(**kwargs) -> None:
    """更新距离分析器全局配置。

    Args:
        **kwargs: 配置参数
    """
    _analyzer_config.update(kwargs)
    logger.info(f"距离分析器配置已更新: {_analyzer_config}")
