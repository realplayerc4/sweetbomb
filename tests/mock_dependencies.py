from unittest.mock import MagicMock
import pytest

import app.api.dependencies as dependencies
from app.services.rs_manager import RealSenseManager
from app.services.webrtc_manager import WebRTCManager
from main import app


class DummyOfferStat:
    def __init__(self, **entries):
        self.__dict__.update(entries)


# Create mock sio instance for RealSenseManager
mock_sio = MagicMock()


# Pytest fixture to patch dependencies in your app
@pytest.fixture(autouse=True)
def patch_dependencies(monkeypatch):
    """
    Patch the app's dependencies with our mock versions.
    This fixture will automatically be used in all tests.
    """
    # Create mock instances
    rs_manager = RealSenseManager(mock_sio)
    webrtc_manager = WebRTCManager(rs_manager)

    app.dependency_overrides[dependencies.get_realsense_manager] = lambda: rs_manager
    app.dependency_overrides[dependencies.get_webrtc_manager] = lambda: webrtc_manager

    # Also patch the global variables in the original module just in case
    monkeypatch.setattr(dependencies, "_realsense_manager", rs_manager)
    monkeypatch.setattr(dependencies, "_webrtc_manager", webrtc_manager)

    yield {
        "rs_manager": rs_manager,
        "webrtc_manager": webrtc_manager,
    }

    # Clean up overrides
    app.dependency_overrides = {}
