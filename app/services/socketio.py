import socketio

# Create Socket.IO Server
sio = socketio.AsyncServer(
    async_mode="asgi",
    cors_allowed_origins='*',
    cors_credentials=True,
    logger=False,  # Disable verbose logging to reduce noise
    engineio_logger=True,  # Enable minimal engineio logging
    ping_timeout=60,  # Default ping timeout
    ping_interval=25,  # Default ping interval
    allow_upgrading=True,  # Allow WebSocket upgrades from HTTP
    http_compression=True,  # Enable HTTP compression
    compression_threshold=1024,  # Compress messages over this size
)

# Setup basic event handlers
@sio.event
async def connect(sid, environ):
    print(f"[Socket.IO] Client connected: {sid}")

@sio.event
async def disconnect(sid):
    print(f"[Socket.IO] Client disconnected: {sid}")
