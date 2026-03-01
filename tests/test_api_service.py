"""
API 服务单元测试

本模块测试 Sweetbomb 后端 API 端点的功能，包括：
- RealSense 设备管理 API
- 传感器配置 API
- 视频流控制 API
- WebRTC 视频传输 API
- 集成测试（需要真实硬件）
"""
import pytest
import numpy as np
from unittest.mock import AsyncMock, MagicMock
from fastapi.testclient import TestClient
from .setup_fake_devices import setup_fake_devices
from .mock_dependencies import patch_dependencies, DummyOfferStat
from .pyrealsense_mock import camera_info
from main import app

# 创建测试客户端
client = TestClient(app)


class TestRealSenseAPI:
    """RealSense API 测试套件（使用 mock 设备）"""

    @classmethod
    def setup_class(cls):
        """测试类设置：创建假设备"""
        mock_setup = setup_fake_devices()
        cls.fake_devices = mock_setup["devices"]
        cls.depth_sensors = mock_setup["depth_sensors"]
        cls.color_sensors = mock_setup["color_sensors"]

    @pytest.fixture
    def setup_mock_managers(self, patch_dependencies):
        """
        设置 mock 管理器 fixture
        配置 RealSenseManager 和 WebRTCManager 的 mock 实现
        """
        rs_manager = patch_dependencies["rs_manager"]
        webrtc_manager = patch_dependencies["webrtc_manager"]

        # 配置 mock RealSenseManager
        def mock_start_stream(device_id, configs, align_to=None):
            # 设置 mock 帧队列
            rs_manager.active_streams[device_id] = set(
                [config.stream_type for config in configs]
            )
            rs_manager.frame_queues[device_id] = {}

            for config in configs:
                stream_type = config.stream_type
                rs_manager.frame_queues[device_id][stream_type] = []

                # 添加假帧到队列

                if stream_type.lower() == "depth":
                    # 创建假深度帧（灰度）
                    frame = np.random.randint(
                        0,
                        255,
                        (config.resolution.height, config.resolution.width),
                        dtype=np.uint8,
                    )
                    # 颜色化以模拟深度颜色化效果
                    frame_rgb = np.zeros(
                        (config.resolution.height, config.resolution.width, 3),
                        dtype=np.uint8,
                    )
                    frame_rgb[:, :, 2] = (
                        frame  # 添加到蓝色通道以实现颜色化效果
                    )
                    frame_data = frame_rgb
                elif stream_type.lower() == "color":
                    # 创建假颜色帧
                    frame_data = np.random.randint(
                        0,
                        255,
                        (config.resolution.height, config.resolution.width, 3),
                        dtype=np.uint8,
                    )
                else:
                    # 其他流类型的通用帧
                    frame_data = np.random.randint(
                        0,
                        255,
                        (config.resolution.height, config.resolution.width, 3),
                        dtype=np.uint8,
                    )

                metadata = {
                    "timestamp": 12345678,
                    "frame_number": 42,
                    "width": config.resolution.width,
                    "height": config.resolution.height,
                }

                rs_manager.frame_queues[device_id][stream_type].append(
                    (frame_data, metadata)
)
                )

            # 更新管道以指示正在流式传输
            rs_manager.pipelines[device_id] = MagicMock()

            # 返回状态
            return rs_manager.get_stream_status(device_id)

        def mock_refresh_devices():
            """刷新设备列表的 mock 实现"""
            # 使用我们的 mock 设备填充设备字典
            rs_manager.devices.clear()
            rs_manager.device_infos.clear()
            for dev in self.fake_devices:
                device_id = dev.get_info(camera_info.serial_number)
                rs_manager.devices[device_id] = dev

                # 创建设备信息
                name = dev.get_info(camera_info.name)
                firmware_version = "1.0.0"  # Mock 固件版本

                # 获取传感器
                sensors = []
                for sensor in dev.sensors:
                    try:
                        sensor_name = sensor.get_info(camera_info.name)
                        sensors.append(sensor_name)
                    except RuntimeError:
                        pass

                # 创建设备信息对象
                device_info = {
                    "device_id": device_id,
                    "name": name,
                    "serial_number": device_id,
                    "firmware_version": firmware_version,
                    "physical_port": "USB",
                    "usb_type": "3.0",
                    "product_id": "001",
                    "sensors": sensors,
                    "is_streaming": device_id in rs_manager.pipelines,
                }

                # 转换为 DeviceInfo 模型
                from app.models.device import DeviceInfo

                rs_manager.device_infos[device_id] = DeviceInfo(**device_info)

            return list(rs_manager.device_infos.values())

        # 替换实际方法
        rs_manager.start_stream = mock_start_stream
        rs_manager.refresh_devices = mock_refresh_devices

        # 配置 mock WebRTCManager
        async def mock_create_offer(device_id, stream_types):
            """创建 WebRTC offer 的 mock 实现"""
            session_id = f"test-session-{device_id}"

            mock_stats_dict = {
                "stat1": DummyOfferStat(type="candidate", id="1234", value=42),
                "stat2": DummyOfferStat(type="track", id="5678", value=99),
            }
            pc = MagicMock()
            pc.getStats = AsyncMock(return_value=mock_stats_dict)

            webrtc_manager.sessions[session_id] = {
                "session_id": session_id,
                "device_id": device_id,
                "stream_types": stream_types,
                "connected": False,
                "pc": pc,
            }

            # Mock offer（需要包含 stream_map 以匹配后端端点的期望）
            offer = {
                "sdp": "v=0\r\no=- 0 0 IN IP4 0.0.0.0\r\ns=-\r\nt=0 0\r\n",
                "type": "offer",
                "stream_map": {str(i): st for i, st in enumerate(stream_types)},
            }

            return session_id, offer

        webrtc_manager.create_offer = mock_create_offer

        # 添加方法以处理 webrtc 操作
        async def mock_process_answer(session_id, sdp, type):
            """处理 WebRTC answer 的 mock 实现"""
            if session_id not in webrtc_manager.sessions or type != "answer" or not sdp:
                return False
            return True

        async def mock_add_ice_candidate(session_id, candidate, sdpMid, sdpMLineIndex):
            """添加 ICE 候选的 mock 实现"""
            if session_id not in webrtc_manager.sessions or not candidate:
                return False
            return True

        async def mock_close_session(session_id):
            """关闭会话的 mock 实现"""
            if session_id in webrtc_manager.sessions:
                del webrtc_manager.sessions[session_id]
            return True

        webrtc_manager.process_answer = mock_process_answer
        webrtc_manager.add_ice_candidate = mock_add_ice_candidate
        webrtc_manager.close_session = mock_close_session

        # 填充初始数据
        rs_manager.refresh_devices()

        return {"rs_manager": rs_manager, "webrtc_manager": webrtc_manager}

    # ----- RealSense API 测试 -----

    def test_get_devices(self, setup_mock_managers):
        """测试获取设备列表 API (/devices 端点）"""
        response = client.get("/api/devices")
        assert response.status_code == 200

        devices = response.json()
        assert len(devices) == 2
        assert devices[0]["name"] == "Test Device 1"
        assert devices[1]["name"] == "Test Device 2"

    def test_get_device_by_id(self, setup_mock_managers):
        """测试通过 ID 获取单个设备 (/devices/{device_id} 端点）"""
        response = client.get("/api/devices/device1")
        assert response.status_code == 200

        device = response.json()
        assert device["device_id"] == "device1"
        assert device["name"] == "Test Device 1"

        # 测试不存在的设备
        response = client.get("/api/devices/nonexistent")
        assert response.status_code == 404

    def test_get_sensors(self, setup_mock_managers):
        """测试获取设备传感器列表 (/devices/{device_id}/sensors 端点）"""
        response = client.get("/api/devices/device1/sensors")
        assert response.status_code == 200

        sensors = response.json()
        assert len(sensors) == 2
        assert sensors[0]["type"] in ["Depth Sensor", "RGB Camera"]
        assert sensors[1]["type"] in ["Depth Sensor", "RGB Camera"]

    def test_get_sensor_by_id(self, setup_mock_managers):
        """测试通过 ID 获取单个传感器 (/devices/{device_id}/sensors/{sensor_id} 端点）"""
        response = client.get("/api/devices/device1/sensors/device1-sensor-0")
        assert response.status_code == 200

        sensor = response.json()
        assert sensor["sensor_id"] == "device1-sensor-0"

        # 测试不存在的传感器
        response = client.get("/api/devices/device1/sensors/nonexistent")
        assert response.status_code == 404

    def test_get_sensor_options(self, setup_mock_managers):
        """测试获取传感器选项 (/devices/{device_id}/sensors/{sensor_id}/options 端点）"""
        response = client.get("/api/devices/device1/sensors/device1-sensor-0/options")
        assert response.status_code == 200

        options = response.json()
        assert len(options) > 0

    def test_get_option_by_id(self, setup_mock_managers):
        """测试获取单个选项 (/devices/{device_id}/sensors/{sensor_id}/options/{option_id} 端点）"""
        # 首先获取可用选项
        response = client.get("/api/devices/device1/sensors/device1-sensor-0/options")
        options = response.json()
        option_id = options[0]["option_id"]

        response = client.get(
            f"/api/devices/device1/sensors/device1-sensor-0/options/{option_id}"
        )
        assert response.status_code == 200

        option = response.json()
        assert option["option_id"] == option_id

    def test_set_option(self, setup_mock_managers):
        """测试设置传感器选项 (PUT /devices/{device_id}/sensors/{sensor_id}/options/{option_id} 端点）"""
        # 首先获取可用选项
        response = client.get("/api/devices/device1/sensors/device1-sensor-0/options")
        options = response.json()
        option_id = options[0]["option_id"]

        response = client.put(
            f"/api/devices/device1/sensors/device1-sensor-0/options/{option_id}",
            json={"value": 0.5},
        )
        assert response.status_code == 200

    def test_start_stream(self, setup_mock_managers):
        """测试启动视频流 (POST /devices/{device_id}/stream/start 端点）"""
        stream_config = {
            "configs": [
                {
                    "sensor_id": "device1-sensor-0",
                    "stream_type": "depth",
                    "format": "z16",
                    "resolution": {"width": 640, "height": 480},
                    "framerate": 30,
                }
            ]
        }

        response = client.post("/api/devices/device1/stream/start", json=stream_config)
        assert response.status_code == 200

        result = response.json()
        assert result["device_id"] == "device1"
        assert result["is_streaming"]
        assert "depth" in result["active_streams"]

    def test_stop_stream(self, setup_mock_managers):
        """测试停止视频流 (POST /devices/{device_id}/stream/stop 端点）"""
        # 首先启动流式传输
        stream_config = {
            "configs": [
                {
                    "sensor_id": "device1-sensor-0",
                    "stream_type": "depth",
                    "format": "z16",
                    "resolution": {"width": 640, "height": 480},
                    "framerate": 30,
                }
            ]
        }
        response = client.post("/api/devices/device1/stream/stop", json=stream_config)

        assert response.status_code == 200

        result = response.json()
        assert result["device_id"] == "device1"
        assert not result["is_streaming"]

    def test_get_stream_status(self, setup_mock_managers):
        """测试获取流式传输状态 (GET /devices/{device_id}/stream/status 端点）"""
        response = client.get("/api/devices/device1/stream/status")
        assert response.status_code == 200

        status = response.json()
        assert status["device_id"] == "device1"
        assert "is_streaming" in status

    # ----- WebRTC API 测试 -----

    @pytest.mark.asyncio
    async def test_create_webrtc_offer(self, setup_mock_managers):
        """测试创建 WebRTC offer (POST /webrtc/offer 端点）"""
        # 首先启动流式传输
        stream_config = {
            "configs": [
                {
                    "sensor_id": "device1-sensor-0",
                    "stream_type": "depth",
                    "format": "z16",
                    "resolution": {"width": 640, "height": 480},
                    "framerate": 30,
                }
            ]
        }
        client.post("/api/devices/device1/stream", json=stream_config)

        # 测试 WebRTC offer 创建
        webrtc_config = {"device_id": "device1", "stream_types": ["depth"]}

        response = client.post("/api/webrtc/offer", json=webrtc_config)
        assert response.status_code == 200

        result = response.json()
        assert "session_id" in result
        assert "sdp"
        assert "type"
        assert result["session_id"] == "test-session-device1"
        assert result["type"] == "offer"

    @pytest.mark.asyncio
    async def test_process_webrtc_answer(self, setup_mock_managers):
        """测试处理 WebRTC answer (POST /webrtc/answer 端点）"""
        # 首先创建 offer
        webrtc_config = {"device_id": "device1", "stream_types": ["depth"]}
        response = client.post("/api/webrtc/offer", json=webrtc_config)
        session_id = response.json()["session_id"]

        # 测试处理 answer
        answer = {
            "session_id": session_id,
            "sdp": "v=0\r\no=- 0 0 IN IP4 0.0.0.0\r\ns=-\r\nt=0 0\r\n",
            "type": "answer",
        }

        response = client.post("/api/webrtc/answer", json=answer)
        assert response.status_code == 200
        assert response.json()["success"]

    @pytest.mark.asyncio
    async def test_add_ice_candidate(self, setup_mock_managers):
        """测试添加 ICE 候选 (POST /webrtc/ice-candidates 端点）"""
        # 首先创建 offer
        webrtc_config = {"device_id": "device1", "stream_types": ["depth"]}
        response = client.post("/api/webrtc/offer", json=webrtc_config)
        session_id = response.json()["session_id"]

        # 测试添加 ICE 候选
        ice_candidate = {
            "session_id": session_id,
            "candidate": "candidate:0 1 UDP 2122260223 192.168.1.1 49152 typ host",
            "sdpMid": "0",
            "sdpMLineIndex": 0,
        }

        response = client.post("/api/webrtc/ice-candidates", json=ice_candidate)
        assert response.status_code == 200
        assert response.json()["success"]

    @pytest.mark.asyncio
    async def test_get_webrtc_session(self, setup_mock_managers):
        """测试获取 WebRTC 会话信息 (GET /webrtc/sessions/{session_id} 端点）"""
        # 首先创建 offer
        webrtc_config = {"device_id": "device1", "stream_types": ["depth"]}
        response = client.post("/api/webrtc/offer", json=webrtc_config)
        session_id = response.json()["session_id"]

        # 测试获取会话
        response = client.get(f"/api/webrtc/sessions/{session_id}")
        assert (status_code == 200)

        result = response.json()
        assert result["session_id"] == session_id
        assert result["device_id"] == "device1"
        assert "depth" in result["stream_types"]

    @pytest.mark.asyncio
    async def test_close_webrtc_session(self, setup_mock_managers):
        """测试关闭 WebRTC 会话 (DELETE /webrtc/sessions/{session_id} 端点）"""
        # 首先创建 offer
        webrtc_config = {"device_id": "device1", "stream_types": ["depth"]}
        response = client.post("/api/webrtc/offer", json=webrtc_config)
        session_id = response.json()["session_id"]

        # 测试关闭会话
        response = client.delete(f"/api/webrtc/sessions/{session_id}")
        assert response.status_code == 200
        assert response.json()["success"]

        # 验证会话已关闭
        response = client.get(f"/api/webrtc/sessions/{session_id}")
        assert response.status_code == 404


class TestRealSenseAPIIntegration:
    """
    集成测试套件（使用真实 RealSense 设备）

    这些测试绕过 mock，测试真实的 API 功能。
    需要连接真实的 RealSense 设备才能运行。
    """

    @classmethod
    def setup_class(cls):
        """设置集成测试环境"""
        # 创建不使用 mock 依赖项的独立测试客户端
        from main import app
        from fastapi.testclient import TestClient

        # 为这些测试清除任何现有的 mock 补丁
        cls.real_client = TestClient(app)

    @pytest.fixture
    def real_rs_manager(self):
        """为集成测试创建真实的 RealSenseManager 实例"""
        from app.services.socketio import sio
        from app.services.rs_manager import RealSenseManager

        # 创建真实的 RealSenseManager（非 mock）
        manager = RealSenseManager(sio)
        return manager

    def test_get_device_rs(self, real_rs_manager):
        """测试使用真实 RealSense API 获取设备信息"""
        # 获取真实设备
        devices = real_rs_manager.get_devices()

        # 测试获取第一个设备
        device = devices[0]
        assert device.device_id is not None
        assert device.name is not None
        assert device.serial_number is not None
        assert isinstance(device.sensors, list)
        assert len(device.sensors) > 0

        # 测试通过 ID 获取设备
        retrieved_device = real_rs_manager.get_device(device.device_id)
        assert retrieved_device.device_id == device.device_id
        assert retrieved_device.name == device.name
        assert retrieved_device.serial_number == device.serial_number

    def test_get_sensor_rs(self, real_rs_manager):
        """测试使用真实 RealSense API 获取传感器信息"""
        # 获取真实设备
        devices = real_rs_manager.get_devices()

        device = devices[0]
        device_id = device.device_id

        # 获取设备的传感器
        sensors = real_rs_manager.get_sensors(device_id)
        assert len(sensors) > 0

        # 测试获取第一个传感器
        sensor = sensors[0]
        assert sensor.sensor_id is not None
        assert sensor.name is not None
        assert sensor.type is not None
        assert isinstance(sensor.supported_stream_profiles, list)
        assert isinstance(sensor.options, list)

        # 测试通过 ID 获取传感器
        retrieved_sensor = real_rs_manager.get_sensor(device_id, sensor.sensor_id)
        assert retrieved_sensor.sensor_id == sensor.sensor_id
        assert retrieved_sensor.name == sensor.name
        assert retrieved_sensor.type == sensor.type

    def test_set_option_rs(self, real_rs_manager):
        """测试使用真实 RealSense API 设置传感器选项"""
        # 获取真实设备
        devices = real_rs_manager.get_devices()

        device = devices[0]
        device_id = device.device_id

        # 获取设备的传感器
        sensors = real_rs_manager.get_sensors(device_id)

        # 查找具有可写选项的传感器
        writable_option = None
        sensor_id = None
        original_value = None

        for sensor in sensors:
            for option in sensor.options:
                if not option.read_only and option.min_value != option.max_value:
                    writable_option = option
                    sensor_id = sensor.sensor_id
                    original_value = option.current_value
                    break
            if writable_option:
                break

        # 测试将选项设置为不同的值
        option_id = writable_option.option_id

        # 计算范围内的安全测试值
        min_val = writable_option.min_value
        max_val = writable_option.max_value
        step = writable_option.step

        # 选择与当前值不同的值，如果 step > 0 则遵循 step
        if step > 0:
            test_value = min_val + step
            if test_value == original_value and (min_val + 2 * step) <= max_val:
                test_value = min_val + 2 * step
        else:
            # 对于连续值，使用中点
            test_value = (min_val + max_val) / 2
            if abs(test_value - original_value) < 0.001:  # 与原始值太接近
                test_value = min_val + (max_val - min_val) * 0.75

        # 确保测试值在边界内
        test_value = max(min_val, min(max_val, test_value))

        try:
            # 设置选项
            success = real_rs_manager.set_sensor_option(device_id, sensor_id, option_id, test_value)
            assert success

            # 通过读回验证选项已设置
            updated_option = real_rs_manager.get_sensor_option(device_id, sensor_id, option_id)

            # 对于步进值，检查完全匹配；对于连续值，允许小误差
            if step > 0:
                assert updated_option.current_value == test_value
            else:
                assert abs(updated_option.current_value - test_value) < 0.01

        finally:
            # 恢复原始值
            try:
                real_rs_manager.set_sensor_option(device_id, sensor_id, option_id, original_value)
            except Exception:
                # 如果恢复失败，不要让测试失败
                pass

    # 端点的附加集成测试
    def test_devices_endpoint_rs(self):
        """测试使用真实设备的 /api/devices 端点"""
        # 通过创建新的应用实例暂时绕过 mock
        import importlib
        import sys

        # 从依赖项模块中移除 mock 补丁
        if 'app.api.dependencies' in sys.modules:
            importlib.reload(sys.modules['app.api.dependencies'])

        from main import app
        from fastapi.testclient import TestClient

        real_client = TestClient(app)

        response = real_client.get("/api/devices")

        if response.status_code == 500:
            pytest.skip("未连接 RealSense 设备或 RealSense 库问题")

        assert response.status_code == 200
        devices = response.json()

        if devices:  # 仅在设备已连接时测试
            assert isinstance(devices, list)
            device = devices[0]
            assert "device_id" in device
            assert "name" in device
            assert "serial_number" in device
            assert "sensors" in device

    def test_sensors_endpoint_rs(self):
        """测试使用真实设备的 /api/devices/{device_id}/sensors 端点"""
        import importlib
        import sys

        # 从依赖项模块中移除 mock 补丁
        if 'app.api.dependencies' in sys.modules:
            importlib.reload(sys.modules['app.api.dependencies'])

        from main import app
        from fastapi.testclient import TestClient

        real_client = TestClient(app)

        # 首先获取设备
        response = real_client.get("/api/devices")

        devices = response.json()
        device_id = devices[0]["device_id"]

        # 测试传感器端点
        response = real_client.get(f"/api/devices/{device_id}/sensors")
        assert response.status_code == 200

        sensors = response.json()
        assert isinstance(sensors, list)
        assert len(sensors) > 0

        sensor = sensors[0]
        assert "sensor_id" in sensor
        assert "name" in sensor
        assert "type" in sensor
        assert "supported_stream_profiles" in sensor
        assert "options" in sensor

    def test_options_endpoint_rs(self):
        """测试使用真实设备的选项端点"""
        import importlib
        import sys

        # 从依赖项模块中移除 mock 补丁
        if 'app.api.dependencies' in sys.modules:
            importlib.reload(sys.modules['app.api.dependencies'])

        from main import app
        from fastapi.testclient import TestClient

        real_client = TestClient(app)

        # 首先获取设备
        response = real_client.get("/api/devices")

        devices = response.json()

        device_id = devices[0]["device_id"]

        # 获取传感器
        response = real_client.get(f"/api/devices/{device_id}/sensors")
        sensors = response.json()
        sensor_id = sensors[0]["sensor_id"]

        # 测试选项端点
        response = real_client.get(f"/api/devices/{device_id}/sensors/{sensor_id}/options")
        assert response.status_code == 200

        options = response.json()
        assert isinstance(options, list)
        assert len(options) > 0

        option = options[0]
        assert "option_id" in option
        assert "name" in option
        assert "current_value" in option
        assert "min_value" in option
        assert "max_value" in option

        # 测试获取特定选项
        option_id = option["option_id"]
        response = real_client.get(f"/api/devices/{device_id}/sensors/{sensor_id}/options/{option_id}")
        assert response.status_code == 200

        retrieved retrieved_option = response.json()
        assert retrieved_option["option_id"] == option_id

    def test_webrtc_offer_rs(self, real_rs_manager):
        """测试使用真实设备创建 WebRTC offer"""
        import importlib
        import sys

        # 移除 mock 补丁
        if 'app.api.dependencies' in sys.modules:
            importlib.reload(sys.modules['app.api.dependencies'])

        from main import app
        from fastapi.testclient import TestClient

        real_client = TestClient(app)

        # 1. 获取设备
        response = real_client.get("/api/devices")
        if response.status_code != 200 or not response.json():
             pytest.skip("未找到设备")
        device_id = response.json()[0]["device_id"]

        # 2. 启动流
        stream_config = {
            "configs": [
                {
                    "sensor_id": "0", # 将被解析为第一个传感器或需要逻辑 ID
                    "stream_type": "depth",
                    "format": "z16",
                    "resolution": {"width": 640, "height": 480},
                    "framerate": 30
                }
            ]
        }
        # 需要正确的 sensor_id。从设备获取。
        sensors = real_client.get(f"/api/devices/{device_id}/sensors").json()
        stream_config["configs"][0]["sensor_id"] = sensors[0]["sensor_id"]

        # 为安全起见首先停止任何现有流
        real_client.post(f"/api/devices/{device_id}/stream/stop", json=stream_config)

        start_resp = real_client.post(f"/api/devices/{device_id}/stream/start", json=stream_config)
        assert start_resp.status_code == 200

        try:
            # 3. 创建 Offer
            webrtc_config = {"device_id": device_id, "stream_types": ["depth"]}
            response = real_client.post("/api/webrtc/offer", json=webrtc_config)

            assert response.status_code == 200
            result = response.json()
            assert "session_id" in result
            assert "sdp" in result
            assert result["type"] == "offer"

            session_id = result["session_id"]

            # 4. 清理会话
            real_client.delete(f"/api/webrtc/sessions/{session_id}")

        finally:
            # 5. 停止流
            real_client.post(f"/api/devices/{device_id}/stream/stop", json=stream_config)
