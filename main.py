import uvicorn
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pathlib import Path

from app.api.router import api_router
from app.core.errors import setup_exception_handlers
from config import settings
import socketio
from app.services.socketio import sio
from app.api.dependencies import get_webrtc_manager


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

# --- Serve Frontend Static Files ---
FRONTEND_DIST = Path(__file__).parent / "ui" / "frontend" / "dist"
if FRONTEND_DIST.exists():
    # Mount assets directory
    assets_dir = FRONTEND_DIST / "assets"
    if assets_dir.exists():
        app.mount("/assets", StaticFiles(directory=str(assets_dir)), name="assets")

# SPA fallback handler - must be added last, catches all non-API routes
@app.middleware("http")
async def spa_middleware(request: Request, call_next):
    response = await call_next(request)

    # If 404 and not an API or socket path, serve SPA
    if response.status_code == 404:
        path = request.url.path
        if not path.startswith("/api") and not path.startswith("/socket") and FRONTEND_DIST.exists():
            # Check if it's a file request
            file_path = FRONTEND_DIST / path.lstrip("/")
            if file_path.exists() and file_path.is_file():
                return FileResponse(str(file_path))
            # Serve index.html for SPA routing
            return FileResponse(str(FRONTEND_DIST / "index.html"))

    return response


# --- Combine FastAPI and Socket.IO into a single ASGI App ---
# Mount the Socket.IO app (`sio`) onto the FastAPI app (`app`)
# The result `combined_app` is what Uvicorn will run.
combined_app = socketio.ASGIApp(socketio_server=sio, other_asgi_app=app, socketio_path='socket')

@app.on_event("startup")
async def startup_event():
    manager = get_webrtc_manager()
    await manager.start_cleanup_loop()

if __name__ == "__main__":
    uvicorn.run("main:combined_app", host="0.0.0.0", port=8000, reload=True, log_level="debug")
