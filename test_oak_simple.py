#!/usr/bin/env python3
"""OAK 相机简单测试脚本。"""

import sys
sys.path.insert(0, '/home/yq/sweetbomb')

import depthai as dai
import numpy as np
import cv2
import time


def test_device_discovery():
    """测试设备发现。"""
    print("=" * 60)
    print("OAK 设备发现测试")
    print("=" * 60)

    devices = dai.Device.getAllAvailableDevices()
    print(f"\n找到 {len(devices)} 个设备:")

    for i, dev in enumerate(devices):
        print(f"\n  [{i}] {dev.name}")
        print(f"       ID: {dev.getDeviceId()}")
        print(f"       协议: {dev.protocol}")
        print(f"       平台: {dev.platform}")

    return devices


def test_streaming(duration=5):
    """测试视频流。"""
    print("\n" + "=" * 60)
    print("OAK 视频流测试")
    print("=" * 60)

    # 获取第一个可用设备
    devices = dai.Device.getAllAvailableDevices()
    if not devices:
        print("❌ 未找到设备")
        return False

    device_info = devices[0]
    print(f"\n连接设备: {device_info.name}")

    try:
        # 创建设备
        device = dai.Device(device_info)
        print("✅ 设备已连接")

        # 创建管道
        pipeline = dai.Pipeline()

        # 彩色相机
        cam_rgb = pipeline.createColorCamera()
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setFps(30)

        # 输出
        xout_rgb = pipeline.createXLinkOut()
        xout_rgb.setStreamName("color")
        cam_rgb.video.link(xout_rgb.input)

        # 启动管道
        device.startPipeline(pipeline)
        print("✅ 管道已启动")

        # 获取输出队列
        queue = device.getOutputQueue(name="color", maxSize=4, blocking=False)

        print(f"\n开始捕获视频流 ({duration} 秒)...")
        print("按 Ctrl+C 提前结束\n")

        frame_count = 0
        start_time = time.time()
        fps_counter = 0
        fps_time = start_time

        while time.time() - start_time < duration:
            try:
                img_frame = queue.tryGet()
                if img_frame is not None:
                    frame = img_frame.getCvFrame()
                    frame_count += 1
                    fps_counter += 1

                    # 计算 FPS
                    now = time.time()
                    if now - fps_time >= 1.0:
                        fps = fps_counter / (now - fps_time)
                        print(f"  FPS: {fps:.1f}, 总帧数: {frame_count}")
                        fps_counter = 0
                        fps_time = now

                    # 显示图像（可选）
                    # cv2.imshow("OAK Stream", frame)
                    # if cv2.waitKey(1) == ord('q'):
                    #     break

            except KeyboardInterrupt:
                print("\n用户中断")
                break

        # cv2.destroyAllWindows()
        device.close()

        print(f"\n✅ 流测试完成")
        print(f"   总帧数: {frame_count}")
        print(f"   运行时间: {time.time() - start_time:.1f} 秒")
        return True

    except Exception as e:
        print(f"\n❌ 流测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """主函数。"""
    print("\n" + "=" * 60)
    print("OAK 相机测试程序")
    print("=" * 60)
    print(f"\nDepthAI 版本: {dai.__version__}")

    # 测试 1: 设备发现
    devices = test_device_discovery()

    if not devices:
        print("\n❌ 未找到 OAK 设备，测试结束")
        return 1

    # 测试 2: 视频流
    print("\n是否测试视频流? (y/n): ", end="")
    try:
        response = input().strip().lower()
        if response == 'y':
            test_streaming(duration=10)
    except EOFError:
        # 非交互式环境，自动测试
        print("y")
        test_streaming(duration=5)

    print("\n" + "=" * 60)
    print("OAK 测试完成")
    print("=" * 60)

    return 0


if __name__ == "__main__":
    sys.exit(main())
