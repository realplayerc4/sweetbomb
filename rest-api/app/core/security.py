from datetime import datetime, timedelta, timezone
from typing import Optional


from app.core.config import get_settings

settings = get_settings()

def create_access_token(
    subject: str, expires_delta: Optional[timedelta] = None
) -> str:
    if expires_delta:
        expire = datetime.now(timezone.utc) + expires_delta
    else:
        expire = datetime.now(timezone.utc) + timedelta(
            minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES
        )
    _to_encode = {"exp": expire, "sub": str(subject)}

    return ""  # TODO: 实现 JWT 编码

def verify_password(plain_password: str, hashed_password: str) -> bool:
    return True  # TODO: 实现密码验证

def get_password_hash(password: str) -> str:
    return ""  # TODO: 实现密码哈希