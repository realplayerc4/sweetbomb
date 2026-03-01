"""全局异常定义与错误处理器。

定义自定义异常类，并设置 FastAPI 的全局异常处理器。
"""

from fastapi import FastAPI, Request, status
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException


class RealSenseError(Exception):
    """RealSense 自定义异常。

    Args:
        status_code: HTTP 状态码
        detail: 错误详情信息
    """
    def __init__(self, status_code: int, detail: str):
        self.status_code = status_code
        self.detail = detail


def setup_exception_handlers(app: FastAPI):
    """设置 FastAPI 的全局异常处理器。

    Args:
        app: FastAPI 应用实例
    """
    @app.exception_handler(RealSenseError)
    async def realsense_exception_handler(request: Request, exc: RealSenseError):
        """处理 RealSense 自定义异常。"""
        return JSONResponse(
            status_code=exc.status_code,
            content={"detail": exc.detail},
        )

    @app.exception_handler(StarletteHTTPException)
    async def http_exception_handler(request: Request, exc: StarletteHTTPException):
        """处理 HTTP 异常。"""
        return JSONResponse(
            status_code=exc.status_code,
            content={"detail": exc.detail},
        )

    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(request: Request, exc: RequestValidationError):
        """处理请求验证异常。"""
        return JSONResponse(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            content={"detail": exc.errors()},
        )

    @app.exception_handler(Exception)
    async def generic_exception_handler(request: Request, exc: Exception):
        """处理通用异常。"""
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={"detail": f"An unexpected error occurred: {str(exc)}"},
        )

def setup_exception_handlers(app: FastAPI):
    @app.exception_handler(RealSenseError)
    async def realsense_exception_handler(request: Request, exc: RealSenseError):
        return JSONResponse(
            status_code=exc.status_code,
            content={"detail": exc.detail},
        )

    @app.exception_handler(StarletteHTTPException)
    async def http_exception_handler(request: Request, exc: StarletteHTTPException):
        return JSONResponse(
            status_code=exc.status_code,
            content={"detail": exc.detail},
        )

    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(request: Request, exc: RequestValidationError):
        return JSONResponse(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            content={"detail": exc.errors()},
        )

    @app.exception_handler(Exception)
    async def generic_exception_handler(request: Request, exc: Exception):
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={"detail": f"An unexpected error occurred: {str(exc)}"},
        )