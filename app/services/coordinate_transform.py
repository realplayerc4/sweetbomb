"""坐标转换模块

统一管理 RealSense 到 ROS 标准坐标系的转换逻辑。
基于 REP 103 (ROS Enhancement Proposals) 标准定义。
"""

import numpy as np


class CoordinateTransform:
    """坐标系转换工具类"""

    @staticmethod
    def realsense_to_ros(vertices: np.ndarray) -> np.ndarray:
        """
        将 RealSense 坐标转换为 ROS 标准坐标系

        RealSense SDK 原生坐标系 (Optical Frame):
            - X 轴：指向相机右侧
            - Y 轴：指向相机下方
            - Z 轴：指向相机正前方（深度值）

        ROS 标准坐标系 (REP 103 - Body Frame / World Frame):
            - X 轴：指向正前方（前进方向）
            - Y 轴：指向左侧
            - Z 轴：指向上方（垂直方向）

        变换公式:
            ros_x = rs_z      (前)
            ros_y = -rs_x     (左 = -右)
            ros_z = -rs_y     (上 = -下)

        Args:
            vertices: N x 3 的点云数组，形状为 (N, 3)

        Returns:
            转换后的点云数组，形状为 (N, 3)
        """
        if len(vertices) == 0:
            return vertices

        # 过滤无效点：只保留 Z (rs_z) > 0 的有效深度点
        valid_mask = vertices[:, 2] > 0
        if not np.any(valid_mask):
            return np.empty((0, 3), dtype=np.float32)

        filtered_vertices = vertices[valid_mask]

        return np.stack([
            filtered_vertices[:, 2],     # X = Z (前)
            -filtered_vertices[:, 0],    # Y = -X (左 = -右)
            -filtered_vertices[:, 1],    # Z = -Y (上)
        ], axis=1)


# 便捷函数
def transform_realsense_to_robot(vertices: np.ndarray) -> np.ndarray:
    """
    便捷函数：将 RealSense 点云转换为 ROS 标准坐标系

    Args:
        vertices: N x 3 的点云数组

    Returns:
        转换后的点云数组
    """
    return CoordinateTransform.realsense_to_ros(vertices)
