import uvicorn
import asyncio
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.router import api_router
from app.core.errors import setup_exception_handlers
from config import settings
import socketio
from app.services.socketio import sio
from app.core.logging_config import setup_logging
from app.api.dependencies import get_realsense_manager, get_webrtc_manager

# Initialize logging
setup_logging()


# --- Create FastAPI App ---
# Initialize FastAPI app with title and OpenAPI URL
app = FastAPI(
    title=settings.PROJECT_NAME,
    openapi_url=f"{settings.API_V1_STR}/openapi.json",
)

# Set up CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Set up routers
app.include_router(api_router, prefix=settings.API_V1_STR)

# Set up exception handlers
setup_exception_handlers(app)


# --- Combine FastAPI and Socket.IO into a single ASGI App ---
# Mount the Socket.IO app (`sio`) onto the FastAPI app (`app`)
# The result `combined_app` is what Uvicorn will run.
# Note: socketio_path must match the client's path option
combined_app = socketio.ASGIApp(socketio_server=sio, other_asgi_app=app)


@app.on_event("startup")
async def startup_event():
    # 启动 WebRTC 清理循环
    webrtc_manager = get_webrtc_manager()
    await webrtc_manager.start_cleanup_loop()

    # 启动 10 分钟健康检查循环
    asyncio.create_task(health_check_loop())


async def health_check_loop():
    """定时对所有活跃流进行健康检查（每10分钟）"""
    print("[System] Health check loop started (Interval: 600s)")
    while True:
        await asyncio.sleep(600)
        try:
            rs_manager = get_realsense_manager()
            print("[System] Running scheduled 10-min health check...")
            rs_manager.check_all_streams_health()
        except Exception as e:
            print(f"[System] Error in health check loop: {str(e)}")


if __name__ == "__main__":
    uvicorn.run("main:combined_app", host="0.0.0.0", port=8000, reload=False, log_level="debug")
