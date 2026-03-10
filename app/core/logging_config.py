import logging
import os
from logging.handlers import RotatingFileHandler

def setup_logging():
    log_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "logs")
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    log_file = os.path.join(log_dir, "sweetbomb.log")
    
    # 基础配置
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            # 轮转文件处理器：最大10MB，保留5个备份
            RotatingFileHandler(
                log_file, 
                maxBytes=10*1024*1024, 
                backupCount=5,
                encoding='utf-8'
            ),
            # 同时输出到控制台
            logging.StreamHandler()
        ]
    )
    
    # 设置第三方库的日志级别，避免干扰
    logging.getLogger("uvicorn.access").setLevel(logging.WARNING)
    logging.getLogger("socketio").setLevel(logging.WARNING)
    logging.getLogger("engineio").setLevel(logging.WARNING)

    logger = logging.getLogger(__name__)
    logger.info("Logging initialized with RotatingFileHandler")
