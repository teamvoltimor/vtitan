DETAILS_KEY = "details"
"""
Key name for structured log details in the log record. If a LogRecord has a 'details' attribute that is a non-empty dict, it will be included in the JSON log output under this key.
"""

EXEC_INFO_KEY = "exc_info"
"""
Key name for exception information in the log record. If a LogRecord has 'exc_info' set, the formatted exception information will be included in the JSON log output under this key.
"""
