#!/usr/bin/env python3
"""
轻量级点云体素化节点 (优化版)
替代 octomap_server2，直接将点云转换为 MarkerArray 用于可视化
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import struct
import time
import numpy as np
from typing import List, Dict, Tuple, Optional

class PointCloudVoxelizer(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_voxelizer')
        
        # 参数声明
        self.declare_parameter('input_cloud_topic', '/camera/depth/color/points')
        self.declare_parameter('output_marker_topic', '/occupied_cells_vis_array')
        self.declare_parameter('voxel_size', 0.15)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('max_range', 3.0)
        self.declare_parameter('min_z', -1.0)
        self.declare_parameter('max_z', 3.0)
        self.declare_parameter('publish_rate', 10.0)  # 最大发布频率 (Hz)
        self.declare_parameter('point_skip', 4)  # 每 N 个点取一个 (下采样)
        
        # 获取参数
        self.input_topic: str = self.get_parameter('input_cloud_topic').value
        self.output_topic: str = self.get_parameter('output_marker_topic').value
        self.voxel_size: float = self.get_parameter('voxel_size').value
        self.frame_id: str = self.get_parameter('frame_id').value
        self.max_range: float = self.get_parameter('max_range').value
        self.min_z: float = self.get_parameter('min_z').value
        self.max_z: float = self.get_parameter('max_z').value
        self.publish_rate: float = self.get_parameter('publish_rate').value
        self.point_skip: int = self.get_parameter('point_skip').value
        
        # 节流控制
        self.min_interval: float = 1.0 / self.publish_rate
        self.last_publish_time: float = 0.0
        
        # 性能统计
        self.frame_count: int = 0
        self.total_time: float = 0.0
        
        # QoS 配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅与发布
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            sensor_qos
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.output_topic,
            10
        )
        
        self.get_logger().info(f'🚀 PointCloud Voxelizer 启动 (Numpy Accelerated)')
        self.get_logger().info(f'   输入: {self.input_topic}')
        self.get_logger().info(f'   输出: {self.output_topic}')
        self.get_logger().info(f'   体素大小: {self.voxel_size}m')
        self.get_logger().info(f'   发布频率: {self.publish_rate} Hz')
        self.get_logger().info(f'   点下采样: 1/{self.point_skip}')
    
    def cloud_callback(self, msg: PointCloud2) -> None:
        """处理点云消息"""
        # 节流：限制发布频率
        current_time = time.time()
        if current_time - self.last_publish_time < self.min_interval:
            return
        
        start_time = time.time()
        
        # 解析点云 (使用 Numpy 加速)
        points = self.parse_pointcloud2_numpy(msg)
        if points is None or len(points) == 0:
            return
        
        # 体素化
        voxels = self.voxelize_numpy(points)
        
        # 发布 MarkerArray
        marker_array = self.create_marker_array(voxels, msg.header.stamp)
        self.marker_pub.publish(marker_array)
        
        self.last_publish_time = current_time
        
        # 性能统计
        elapsed = time.time() - start_time
        self.frame_count += 1
        self.total_time += elapsed
        
        if self.frame_count % 30 == 0:
            avg_time = self.total_time / self.frame_count * 1000
            self.get_logger().info(
                f'📊 {len(points)} 点 -> {len(voxels)} 体素 | '
                f'平均处理时间: {avg_time:.1f}ms'
            )
            # 重置统计以避免长期漂移
            self.frame_count = 0
            self.total_time = 0.0
            
    def parse_pointcloud2_numpy(self, msg: PointCloud2) -> Optional[np.ndarray]:
        """解析 PointCloud2 消息，提取 XYZ 坐标 (Numpy 加速版)"""
        # 简单检查字段
        field_names = [f.name for f in msg.fields]
        if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
            self.get_logger().warn('点云缺少 x/y/z 字段')
            return None
            
        try:
            # 构造 structured dtype
            dtype_struct = []
            current_offset = 0
            # 按 offset 排序字段
            sorted_fields = sorted(msg.fields, key=lambda f: f.offset)
            
            for f in sorted_fields:
                if f.offset > current_offset:
                    # 填充 padding
                    dtype_struct.append(('pad_%d' % current_offset, np.uint8, f.offset - current_offset))
                    current_offset = f.offset
                
                if f.datatype == PointField.FLOAT32:
                    base_type = 'f4'
                    size = 4
                elif f.datatype == PointField.FLOAT64:
                    base_type = 'f8'
                    size = 8
                elif f.datatype == PointField.UINT8:
                    base_type = 'u1'
                    size = 1
                elif f.datatype == PointField.UINT16:
                    base_type = 'u2'
                    size = 2
                elif f.datatype == PointField.UINT32:
                    base_type = 'u4'
                    size = 4
                elif f.datatype == PointField.INT8:
                    base_type = 'i1'
                    size = 1
                elif f.datatype == PointField.INT16:
                    base_type = 'i2'
                    size = 2
                elif f.datatype == PointField.INT32:
                    base_type = 'i4'
                    size = 4
                else: 
                     base_type = 'f4' # Default fallback
                     size = 4
                     
                dtype_struct.append((f.name, base_type))
                current_offset += size
                
            if current_offset < msg.point_step:
                dtype_struct.append(('pad_end', np.uint8, msg.point_step - current_offset))
                
            # 从 buffer 读取
            cloud_arr = np.frombuffer(msg.data, dtype=dtype_struct)
            
            # 提取 x, y, z
            # 结合 stack: (N, 3)
            # 使用 float32 视图
            points_xyz = np.stack([cloud_arr['x'], cloud_arr['y'], cloud_arr['z']], axis=-1).astype(np.float32)
            
            # 下采样
            if self.point_skip > 1:
                points_xyz = points_xyz[::self.point_skip]
                
            # 过滤无效点 (NaN)
            # np.isnan 对于 (N, 3) 返回 boolean mask，any(axis=1) 找到任何包含 NaN 的行
            mask = ~np.isnan(points_xyz).any(axis=1)
            points_xyz = points_xyz[mask]
            
            # 过滤 (0,0,0)
            mask_zero = np.any(points_xyz != 0, axis=1)
            points_xyz = points_xyz[mask_zero]
            
            # 范围过滤
            # 计算距离平方
            dist_sq = np.sum(points_xyz**2, axis=1)
            mask_range = dist_sq <= (self.max_range * self.max_range)
            points_xyz = points_xyz[mask_range]
            
            # 坐标转换
            # optical: X-右, Y-下, Z-前
            # ROS:     X-前, Y-左, Z-上
            
            # 创建新数组避免修改原视图
            ros_points = np.zeros_like(points_xyz)
            ros_points[:, 0] = points_xyz[:, 2]  # ros_x = opt_z
            ros_points[:, 1] = -points_xyz[:, 0] # ros_y = -opt_x
            ros_points[:, 2] = -points_xyz[:, 1] # ros_z = -opt_y
            
            # 再次过滤 Z 轴高度 (Min Z / Max Z)
            # 注意: 这里的 min_z / max_z 是相对于 ROS 坐标系的
            mask_z = (ros_points[:, 2] >= self.min_z) & (ros_points[:, 2] <= self.max_z)
            ros_points = ros_points[mask_z]
            
            return ros_points
            
        except Exception as e:
            self.get_logger().error(f'Numpy 解析错误: {e}')
            return None

    def voxelize_numpy(self, points: np.ndarray) -> Dict[Tuple[int, int, int], Tuple[float, float, float]]:
        """使用 Numpy 进行体素化"""
        # 计算体素索引: floor(coord / voxel_size)
        inv_size = 1.0 / self.voxel_size
        voxel_indices = np.floor(points * inv_size).astype(np.int32)
        
        # 寻找唯一体素
        unique_indices = np.unique(voxel_indices, axis=0)
        
        # 计算体素中心 (用于显示)
        # center = (index + 0.5) * size
        voxel_centers = (unique_indices.astype(np.float32) + 0.5) * self.voxel_size
        
        # 转换为字典格式
        voxels = {}
        for i in range(len(unique_indices)):
            idx = tuple(unique_indices[i])
            center = tuple(voxel_centers[i])
            voxels[idx] = center
            
        return voxels
    
    def create_marker_array(self, voxels: Dict[Tuple[int, int, int], Tuple[float, float, float]], stamp) -> MarkerArray:
        """创建 MarkerArray 消息"""
        marker_array = MarkerArray()
        
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'voxels'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        
        marker.scale.x = self.voxel_size * 0.95
        marker.scale.y = self.voxel_size * 0.95
        marker.scale.z = self.voxel_size * 0.95
        
        # 如果没有体素
        if not voxels:
            return marker_array
            
        # 准备数据
        centers = list(voxels.values())
        centers_np = np.array(centers, dtype=np.float32)
        
        if len(centers_np) == 0:
            return marker_array
            
        # Z 值用于着色
        z_values = centers_np[:, 2]
        z_min = np.min(z_values)
        z_max = np.max(z_values)
        z_range = max(z_max - z_min, 1e-6) # 避免除零
        
        # 批量创建 Point
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in centers_np]
        
        # 批量计算颜色
        # h = (z - z_min) / z_range
        # 预计算归一化高度
        h_vals = (z_values - z_min) / z_range
        # 限制范围
        h_vals = np.clip(h_vals, 0.0, 1.0) * 0.8
        
        # 向量化颜色计算
        # r, g, b
        colors = np.zeros((len(h_vals), 4), dtype=np.float32)
        colors[:, 3] = 1.0 # Alpha
        
        # 逻辑：
        # h < 0.2: r=0, g=h*5, b=1
        mask1 = h_vals < 0.2
        colors[mask1, 1] = h_vals[mask1] * 5
        colors[mask1, 2] = 1.0
        
        # 0.2 <= h < 0.4: r=0, g=1, b=1-(h-0.2)*5
        mask2 = (h_vals >= 0.2) & (h_vals < 0.4)
        colors[mask2, 1] = 1.0
        colors[mask2, 2] = 1.0 - (h_vals[mask2] - 0.2) * 5
        
        # 0.4 <= h < 0.6: r=(h-0.4)*5, g=1, b=0
        mask3 = (h_vals >= 0.4) & (h_vals < 0.6)
        colors[mask3, 0] = (h_vals[mask3] - 0.4) * 5
        colors[mask3, 1] = 1.0
        
        # 0.6 <= h < 0.8: r=1, g=1-(h-0.6)*5, b=0
        mask4 = (h_vals >= 0.6) & (h_vals < 0.8)
        colors[mask4, 0] = 1.0
        colors[mask4, 1] = 1.0 - (h_vals[mask4] - 0.6) * 5
        
        # h >= 0.8: r=1, g=0, b=0
        mask5 = h_vals >= 0.8
        colors[mask5, 0] = 1.0
        
        # 转换为 StdMsgs ColorRGBA
        marker.colors = [ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=float(c[3])) for c in colors]
        
        marker_array.markers.append(marker)
        return marker_array

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudVoxelizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
