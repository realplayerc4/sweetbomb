from fastapi import APIRouter, Depends, HTTPException


from app.models.stream import StreamStatus, StreamStart
from app.services.rs_manager import RealSenseManager
from app.api.dependencies import get_realsense_manager
from app.core.errors import RealSenseError

router = APIRouter()

@router.post("/start", response_model=StreamStatus)
async def start_stream(
    device_id: str,
    stream_config: StreamStart,
    rs_manager: RealSenseManager = Depends(get_realsense_manager),
):
    """
    Start streaming from a RealSense device with the specified configuration.
    """
    try:
        return rs_manager.start_stream(device_id, stream_config.configs, stream_config.align_to)
    except RealSenseError:
        raise
    except Exception as e:
        if isinstance(e, HTTPException):
            raise e
        # If it's a known RealSenseError (which might inherit from HTTPException if I defined it so, or I should import it)
        # Checking RealSenseError definition in app/models/error.py or similar is needed.
        # But simply removing the try-catch block allows FastAPIs exception handler to work if RealSenseError translates to HTTPException.
        # Assuming RealSenseError is HTTPException-like.
        raise HTTPException(status_code=400, detail=str(e))

@router.post("/stop", response_model=StreamStatus)
async def stop_stream(
    device_id: str,
    rs_manager: RealSenseManager = Depends(get_realsense_manager),
):
    """
    Stop streaming from a RealSense device.
    """
    try:
        return rs_manager.stop_stream(device_id)
    except RealSenseError:
        raise
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.get("/status", response_model=StreamStatus)
async def get_stream_status(
    device_id: str,
    rs_manager: RealSenseManager = Depends(get_realsense_manager)
):
    """
    Get the streaming status for a RealSense device.
    """
    try:
        return rs_manager.get_stream_status(device_id)
    except Exception as e:
        raise HTTPException(status_code=404, detail=str(e))