import os
from typing import Optional

class Settings():
    API_V1_STR: str = "/api"
    PROJECT_NAME: str = "restrealsenseMonitor"

    # Security
    SECRET_KEY: str = os.getenv("SECRET_KEY", "your-secret-key-here")  # Change in production
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30

    # CORS
    CORS_ORIGINS: list = ["*"]

    # WebRTC
    STUN_SERVER: Optional[str] = None
    TURN_SERVER: Optional[str] = None
    TURN_USERNAME: Optional[str] = None
    TURN_PASSWORD: Optional[str] = None

settings = Settings()