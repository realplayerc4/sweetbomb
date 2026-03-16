import pyrealsense2 as rs
import sys
import time

def diagnose_realsense():
    print("--- RealSense 诊断程序 ---")
    print(f"Python 版本: {sys.version}")
    
    try:
        print("\n1. 尝试创建 rs.context()...")
        ctx = rs.context()
        devices = ctx.query_devices()
        print(f"发现设备数量: {len(devices)}")
        if len(devices) == 0:
            print("错误: 未检测到任何 RealSense 设备！请检查 USB 连接。")
        else:
            for i in range(len(devices)):
                try:
                    dev = devices[i]
                    print(f"检测到设备 [{i}]")
                    # 尝试只访问名称，看是否报错
                    name = dev.get_info(rs.camera_info.name)
                    print(f"设备名称: {name}")
                except Exception as e:
                    print(f"访问设备 [{i}] 信息失败: {e}")

        print("\n2. 尝试不查询设备直接启动 Pipeline...")
        try:
            p2 = rs.pipeline()
            p2.start()
            print("成功: 直接启动 Pipeline 成功!")
            p2.stop()
        except Exception as e:
            print(f"失败: 直接启动 Pipeline 出错: {e}")

        print("\n3. 尝试启动 Pipeline (指定基础配置)...")
        pipeline = rs.pipeline(ctx)
        config = rs.config()
        
        # 尝试最基础的配置
        try:
            profile = pipeline.start(config)
            print("成功: Pipeline 已启动!")
            pipeline.stop()
            print("Pipeline 已正常停止。")
        except Exception as e:
            print(f"失败: 启动 Pipeline 时出错: {e}")
            if "bad optional access" in str(e).lower():
                print(">>> 确认错误类型: 'bad optional access'。这通常意味着内核驱动(uvcvideo)或权限问题。")

    except Exception as e:
        print(f"诊断过程中发生严重错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    diagnose_realsense()
