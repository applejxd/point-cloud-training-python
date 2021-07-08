import inspect
import os
from logging import Logger, getLogger, DEBUG, Formatter, StreamHandler, FileHandler
from datetime import datetime


class SelfLogger:
    _main_logger = None
    formatter = Formatter("%(asctime)s - %(levelname)s - %(filename)s - %(message)s")

    def __new__(cls):
        raise NotImplementedError("Cannot initialize via Constructor")

    @classmethod
    def _get_file_handler(cls, name: str):
        log_file_name = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + name
        file_handler = FileHandler(f"{os.path.dirname(__file__)}/../logs/{log_file_name}.log")
        file_handler.setLevel(DEBUG)
        file_handler.setFormatter(cls.formatter)
        return file_handler

    @classmethod
    def _get_stream_handler(cls):
        stream_handler = StreamHandler()
        stream_handler.setLevel(DEBUG)
        stream_handler.setFormatter(cls.formatter)
        return stream_handler

    @classmethod
    def _get_main_logger(cls, name: str) -> Logger:
        logger_ins = getLogger(name)
        logger_ins.setLevel(DEBUG)
        logger_ins.propagate = False

        file_handler = cls._get_file_handler(name)
        logger_ins.addHandler(file_handler)
        stream_handler = cls._get_stream_handler()
        logger_ins.addHandler(stream_handler)

        return logger_ins

    @classmethod
    def get_logger(cls, file_path: str) -> Logger:
        file_name = os.path.splitext(os.path.basename(file_path))[0]
        if not cls._main_logger:
            cls._main_logger = cls._get_main_logger(file_name)
            return cls._main_logger
        else:
            return cls._main_logger.getChild(file_name)
