/**
 * OctoMap Web Viewer - Configuration
 * 集中管理可配置项
 */

const CONFIG = {
    // ROS Bridge 配置
    // 使用当前页面的主机名，支持局域网访问
    ROSBRIDGE_PORT: 9090,
    get ROSBRIDGE_URL() {
        return `ws://${window.location.hostname}:${this.ROSBRIDGE_PORT}`;
    },

    // ROS Topics
    MARKER_TOPIC: '/occupied_cells_vis_array',
    MARKER_MESSAGE_TYPE: 'visualization_msgs/MarkerArray',
    THROTTLE_RATE: 100, // ms, 限制消息频率 (10 Hz)

    // 渲染配置
    VOXEL_COLOR: 0xFFA500,       // 体素颜色 (橙色)
    EDGE_COLOR: 0x000000,        // 边缘颜色 (黑色)
    VOXEL_SCALE: 0.95,           // 体素缩放比例 (留出间隙)
    RENDER_EDGES: true,          // 是否渲染边缘线框
    EDGE_SKIP_FACTOR: 1,         // 每 N 个体素绘制一次边缘 (1 = 全部, 2 = 一半)

    // 相机默认位置
    CAMERA_INITIAL: { x: 3, y: 2, z: 3 },
    CAMERA_RESET: { x: -3, y: 0, z: 2 },
    CAMERA_RESET_TARGET: { x: 3, y: 0, z: 0 },
    CAMERA_RESET_LOOKAT: { x: 5, y: 0, z: 0 }
};
