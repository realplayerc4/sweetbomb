"""Point cloud analysis task with real hardware grounding."""

import asyncio
import logging
import time
from typing import Any, Dict, Optional

import numpy as np

from app.models.task import TaskResult
from app.services.tasks.base_task import BaseTask
from app.services.tasks.registry import register_task


logger = logging.getLogger(__name__)


@register_task
class PointCloudAnalysisTask(BaseTask):
    """
    Task for analyzing real point cloud data from the RealSense depth sensor.
    
    1. Grabs `point_cloud['vertices']` (N x 3 NumPy array) directly from rs_manager memory.
    2. Calculates Bounding Box (Min/Max XYZ bounds and dimensions).
    3. Calculates Point Cloud Centroid.
    4. Evaluates valid vertex count.
    """

    task_type = "point_cloud_analysis"
    name = "点云计算"
    description = "利用内部矩阵实时解析深度点云，计算空间包围盒(Bounding Box)限界与物理质心"
    category = "vision"
    requires_device = True

    params_schema = {
        "type": "object",
        "properties": {
            "duration_seconds": {
                "type": "integer",
                "minimum": 3,
                "maximum": 300,
                "default": 10,
                "description": "持续测算追踪时间（秒）"
            },
            "eval_frequency_hz": {
                "type": "integer",
                "minimum": 1,
                "maximum": 10,
                "default": 2,
                "description": "每秒测算频率 (Hz)"
            }
        },
        "additionalProperties": False
    }

    def __init__(
        self,
        task_id: str,
        config,
        params: Dict[str, Any],
        device_id: Optional[str] = None
    ):
        super().__init__(task_id, config, params, device_id)
        
        # 挂载真实的 rs_manager
        self.rs_manager = None
        
        self.duration = self.params.get("duration_seconds", 10)
        self.eval_hz = self.params.get("eval_frequency_hz", 2)
        
    def validate(self) -> bool:
        if not self.device_id:
            raise ValueError("device_id is required for point cloud analysis")
        return True

    def setup(self):
        logger.info(f"Setting up real point cloud analysis for task {self.task_id}")

    async def run(self) -> TaskResult:
        if not self.rs_manager:
            return TaskResult(
                success=False,
                message="感知引擎未注入，无法获取点云原数据（Numpy空间转换失败）。",
                error="Manager not injected"
            )

        status = self.rs_manager.get_stream_status(self.device_id)
        if not status.is_streaming:
            return TaskResult(
                success=False, message="流未启动，无法获取深度帧", error="Not streaming"
            )

        # 尝试让系统开启点云投射
        try:
            self.rs_manager.activate_point_cloud(self.device_id, True)
        except Exception as e:
            logger.warning(f"Could not actively enable point cloud flag (may already be on): {e}")

        interval_sec = 1.0 / self.eval_hz
        total_evals = self.duration * self.eval_hz
        current_eval = 0
        
        # 性能追踪统计
        aggr_bounds = []
        aggr_centroids = []
        
        self.update_progress(
            current_step=0,
            total_steps=total_evals,
            message=f"启动持续空间探测追踪（{self.duration}秒）..."
        )

        start_time = time.time()

        while current_eval < total_evals:
            await self.async_check_paused()
            loop_start = time.time()
            
            if loop_start - start_time > self.duration:
                break

            # 去 fetch 点云，我们要找深度流的 metadata (我们在 StreamController 里它被挂在 raw_frames['depth-*'] 下)
            # 不过最稳定的途径是借由 get_latest_metadata 读取 Socket 广播前的预处理队列。
            try:
                # 为了防错，扫描所有流类型，寻找深度类型
                pc_data = None
                all_metadata = self.rs_manager.metadata_queues.get(self.device_id, {})
                for stream_key, meta in all_metadata.items():
                    if "depth" in stream_key.lower():
                        if "point_cloud" in meta and "vertices" in meta["point_cloud"]:
                            pc_data = meta["point_cloud"]["vertices"]
                            break
                            
                if pc_data is not None and len(pc_data) > 0:
                    # 此时 pc_data 是已经从 Base64 还原，或者根本没有经过 base64（如果使用的是内部引用的话）。
                    # 根据代码，metadata_socket_server_emit 前有一个对 vertices base64编解码的操作，
                    # 如果拿到的并不是 numpy 而是 base64 string，需要我们转。
                    if isinstance(pc_data, str):
                        # Decode base64 场景
                        import base64
                        raw_bytes = base64.b64decode(pc_data)
                        verts = np.frombuffer(raw_bytes, dtype=np.float32).reshape(-1, 3)
                    else:
                        verts = np.asarray(pc_data, dtype=np.float32)

                    # 1. 有效顶点数目
                    valid_pts_count = len(verts)

                    # 2. 空间三轴限界包围盒 (Bounding Box)
                    min_bounds = np.min(verts, axis=0)
                    max_bounds = np.max(verts, axis=0)
                    dimensions = max_bounds - min_bounds

                    # 3. 几何质心 (Geometric Centroid)
                    centroid = np.mean(verts, axis=0)
                    
                    aggr_bounds.append(dimensions)
                    aggr_centroids.append(centroid)

                    msg = (f"有效云图: {valid_pts_count} 节点. "
                           f"当前限界 W:{dimensions[0]:.2f}xH:{dimensions[1]:.2f}xD:{dimensions[2]:.2f}m")
                else:
                    msg = "当前帧未探测到点阵或环境太近导致盲区剔除"
                
                self.update_progress(
                    current_step=current_eval + 1,
                    message=msg
                )

            except Exception as e:
                logger.error(f"Error computing point cloud metrics: {e}")
                self.update_progress(
                    current_step=current_eval + 1,
                    message="矩阵解码计算失败，跳出当前周期..."
                )

            current_eval += 1
            
            if self._stop_requested:
                break
                
            elapsed = time.time() - loop_start
            sleep_time = max(0.01, interval_sec - elapsed)
            await asyncio.sleep(sleep_time)
            
            
        # 测算末期：归纳整理计算结论
        final_summary = {
            "evaluations_done": current_eval,
            "duration": self.duration
        }
        
        if len(aggr_bounds) > 0:
            avg_dims = np.mean(aggr_bounds, axis=0)
            avg_centroid = np.mean(aggr_centroids, axis=0)
            final_summary["average_bounding_box"] = {
                "width_x_meters": float(avg_dims[0]),
                "height_y_meters": float(avg_dims[1]),
                "depth_z_meters": float(avg_dims[2])
            }
            final_summary["average_centroid"] = {
                "x": float(avg_centroid[0]),
                "y": float(avg_centroid[1]),
                "z": float(avg_centroid[2])
            }
            res_message = f"探测{'完成' if not self._stop_requested else '强停'}，平均视场重心距镜头 {avg_centroid[2]:.2f} m"
        else:
            res_message = "探测完毕，但未捕捉到任何有效点阵用于空间测算 (可能在遮挡盲区或特征流关闭)。"

        # 尝试复原原有的点云状态
        try:
            self.rs_manager.activate_point_cloud(self.device_id, False)
        except:
            pass

        return TaskResult(
            success=not self._stop_requested and len(aggr_bounds) > 0,
            message=res_message,
            data=final_summary,
            metrics={
                "hz_performance": current_eval / (time.time() - start_time)
            }
        )

    def teardown(self):
        logger.info(f"Point cloud task {self.task_id} torn down.")
