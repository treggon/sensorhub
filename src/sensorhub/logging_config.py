
import logging
import os

def configure_logging():
    level = os.getenv('LOG_LEVEL', 'INFO').upper()
    logging.basicConfig(
        level=getattr(logging, level, logging.INFO),
        format='[%(asctime)s] [%(process)d] [%(levelname)s] %(name)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    gunicorn_logger = logging.getLogger('gunicorn.error')
    if gunicorn_logger.handlers:
        root = logging.getLogger()
        for h in gunicorn_logger.handlers:
            root.addHandler(h)
        root.setLevel(gunicorn_logger.level)
