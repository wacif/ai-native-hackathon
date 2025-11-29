"""
Utilities for the Physical AI & Humanoid Robotics Textbook backend.
"""

from .logger import logger, setup_logging
from .errors import (
    TextbookAPIException,
    DatabaseException,
    AuthenticationException,
    AuthorizationException,
    ResourceNotFoundException,
    ValidationException,
    ExternalServiceException,
    textbook_exception_handler,
    http_exception_handler,
    validation_exception_handler,
    generic_exception_handler,
)

__all__ = [
    # Logging
    "logger",
    "setup_logging",
    # Exceptions
    "TextbookAPIException",
    "DatabaseException",
    "AuthenticationException",
    "AuthorizationException",
    "ResourceNotFoundException",
    "ValidationException",
    "ExternalServiceException",
    # Exception handlers
    "textbook_exception_handler",
    "http_exception_handler",
    "validation_exception_handler",
    "generic_exception_handler",
]

