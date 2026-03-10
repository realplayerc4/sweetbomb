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
    """设置 FastAPI 的全局异常处理器。"""
    @app.exception_handler(RealSenseError)
    async def realsense_exception_handler(request: Request, exc: RealSenseError):
        print(f"[ERROR] RealSenseError: {exc.status_code} - {exc.detail}")
        return JSONResponse(
            status_code=exc.status_code,
            content={"detail": exc.detail},
        )

    @app.exception_handler(StarletteHTTPException)
    async def http_exception_handler(request: Request, exc: StarletteHTTPException):
        print(f"[ERROR] HTTPException: {exc.status_code} - {exc.detail}")
        return JSONResponse(
            status_code=exc.status_code,
            content={"detail": exc.detail},
        )

    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(request: Request, exc: RequestValidationError):
        errors = exc.errors()
        print(f"[ERROR] Validation error for {request.method} {request.url}: {errors}")
        return JSONResponse(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            content={"detail": errors},
        )

    @app.exception_handler(Exception)
    async def generic_exception_handler(request: Request, exc: Exception):
        import traceback
        trace = traceback.format_exc()
        print(f"[ERROR] Unhandled Exception for {request.method} {request.url}: {str(exc)}")
        print(f"[ERROR] Traceback: {trace}")
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={"detail": f"An unexpected error occurred: {str(exc)}"},
        )