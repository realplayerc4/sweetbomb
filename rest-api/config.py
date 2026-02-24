import os
from typing import Optional


class Settings:
    API_V1_STR: str = "/api"
    PROJECT_NAME: str = "restrealsenseMonitor"

    # CORS
    CORS_ORIGINS: list = ["*"]

    # WebRTC
    STUN_SERVER: Optional[str] = None
    TURN_SERVER: Optional[str] = None
    TURN_USERNAME: Optional[str] = None
    TURN_PASSWORD: Optional[str] = None


settings = Settings()
