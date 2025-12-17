import logging
import os
from logging.handlers import RotatingFileHandler

def setup_logging(log_file_name: str = "pipeline.log", level=logging.INFO):
    """
    Sets up a common logging configuration for the RAG pipeline.

    Logs to a file in the 'backend/logs/' directory and to the console.
    The log file will rotate when it reaches 5MB, keeping up to 5 old log files.
    """
    log_dir = os.path.join(os.path.dirname(__file__), '..', 'logs')
    os.makedirs(log_dir, exist_ok=True)
    log_file_path = os.path.join(log_dir, log_file_name)

    # Create logger
    logger = logging.getLogger('rag_pipeline')
    logger.setLevel(level)
    logger.propagate = False # Prevent logging to the root logger

    # Create formatter
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)

    # File handler
    file_handler = RotatingFileHandler(
        log_file_path,
        maxBytes=5 * 1024 * 1024, # 5 MB
        backupCount=5
    )
    file_handler.setLevel(level)
    file_handler.setFormatter(formatter)

    # Add handlers to logger
    if not logger.handlers: # Avoid adding duplicate handlers if setup is called multiple times
        logger.addHandler(console_handler)
        logger.addHandler(file_handler)
    
    return logger

# Example usage (for testing this module directly)
if __name__ == "__main__":
    logger = setup_logging()
    logger.info("This is an info message from logging_config.")
    logger.warning("This is a warning message.")
    logger.error("This is an error message.")

    logger_debug = setup_logging(log_file_name="debug.log", level=logging.DEBUG)
    logger_debug.debug("This is a debug message from a different logger.")
