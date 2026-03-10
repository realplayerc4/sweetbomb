import pyrealsense2 as rs
import time

def test_minimal():
    print("--- 极简 RealSense 测试 ---")
    pipeline = rs.pipeline()
    print("尝试 pipeline.start()...")
    try:
        pipeline.start()
        print("Pipeline 启动成功！正在等待 1 秒...")
        time.sleep(1)
        frames = pipeline.wait_for_frames()
        print(f"成功获取到帧: {frames}")
        pipeline.stop()
        print("测试通过！")
    except Exception as e:
        print(f"Pipeline 启动或获取帧失败: {e}")

if __name__ == "__main__":
    test_minimal()
