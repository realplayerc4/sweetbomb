import socketio

# Create Socket.IO Server
sio = socketio.AsyncServer(
    async_mode="asgi",
    cors_allowed_origins='*',
    cors_credentials=True,
    logger=True,
    engineio_logger=True
)

# Setup basic event handlers
@sio.event
async def connect(sid, environ):
    print(f"[Socket.IO] Client connected: {sid}")
    await sio.emit('welcome', {'message': 'Connected to RealSense Metadata Server'}, to=sid)

@sio.event
async def disconnect(sid):
    print(f"[Socket.IO] Client disconnected: {sid}")
