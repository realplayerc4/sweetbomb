"""OAK 错误类定义。"""


class OAKError(Exception):
    """OAK 基础错误。"""

    def __init__(self, message: str, status_code: int = 500):
        super().__init__(message)
        self.message = message
        self.status_code = status_code

    def __str__(self) -> str:
        return f"[{self.status_code}] {self.message}"


class OAKDeviceError(OAKError):
    """OAK 设备错误。"""

    def __init__(self, message: str, device_id: str = None):
        super().__init__(message, status_code=404)
        self.device_id = device_id


class OAKConnectionError(OAKError):
    """OAK 连接错误。"""

    def __init__(self, message: str):
        super().__init__(message, status_code=503)


class OAKStreamError(OAKError):
    """OAK 流错误。"""

    def __init__(self, message: str):
        super().__init__(message, status_code=500)


class OAKConfigError(OAKError):
    """OAK 配置错误。"""

    def __init__(self, message: str, errors: list = None):
        super().__init__(message, status_code=400)
        self.errors = errors or []
