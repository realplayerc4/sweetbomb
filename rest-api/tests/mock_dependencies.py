from unittest.mock import MagicMock
import pytest

import app.api.dependencies as dependencies
from app.services.rs_manager import RealSenseManager
from app.services.webrtc_manager import WebRTCManager
from main import app


class DummyOfferStat:
    def __init__(self, **entries):
        self.__dict__.update(entries)

# Mock the dependencies
mock_oauth2_scheme = MagicMock()
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
    app.dependency_overrides[dependencies.oauth2_scheme] = lambda: "mock_token"

    # Also patch the global variables in the original module just in case
    monkeypatch.setattr(dependencies, "_realsense_manager", rs_manager)
    monkeypatch.setattr(dependencies, "_webrtc_manager", webrtc_manager)
    monkeypatch.setattr(dependencies, "oauth2_scheme", mock_oauth2_scheme)


    yield {
        "rs_manager": rs_manager,
        "webrtc_manager": webrtc_manager,
        "oauth2_scheme": mock_oauth2_scheme,
    }

    # Clean up overrides
    app.dependency_overrides = {}
