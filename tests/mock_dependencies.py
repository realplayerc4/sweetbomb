"""
Mock 依赖注入模块

本模块提供测试所需的依赖项模拟，用于隔离测试环境：
- 模拟 RealSenseManager 设备管理器
- 模拟 WebRTCManager 视频流管理器
- 提供自动化的依赖注入 fixture

通过使用 mock 对象，测试可以在没有真实硬件的情况下运行，
确保测试的独立性和可重复性。
"""
from unittest.mock import MagicMock
import pytest

import app.api.dependencies as dependencies
from app.services.rs_manager import RealSenseManager
from app.services.we. rtc_manager import WebRTCManager
from main import app


class DummyOfferStat:
    """WebRTC Offer 统计信息的模拟类"""
    def __init__(self, **entries):
        self.__dict__.update(entries)


# 创建用于 RealSenseManager 的 mock sio 实例
mock_sio = MagicMock()


# Pytest fixture 用于修补应用的依赖项
@pytest.fixture(autouse=True)
def patch_dependencies(monkeypatch):
    """
    使用 mock 版本替换应用的依赖项
    此 fixture 会自动在所有测试中使用
    """
    # 创建 mock 实例
    rs_manager = RealSenseManager(mock_sio)
    webrtc_manager = WebRTCManager(rs_manager)

    app.dependency_overrides[dependencies.get_realsense_manager] = lambda: rs_manager
    app.dependency_overrides[dependencies.get_webrtc_manager] = lambda: webrtc_manager

    # 同时修补原始模块中的全局变量，以防万一
    monkeypatch.setattr(dependencies, "_realsense_manager", rs_manager)
    monkeypatch.setattr(dependencies, "_webrtc_manager", webrtc_manager)

    yield {
        "rs_manager": rs_manager,
        "webrtc_manager": webrtc_manager,
    }

    # 清理依赖覆盖
    app.dependency_overrides = {}
