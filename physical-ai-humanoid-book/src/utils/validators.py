"""
Input validation and sanitization utilities for the Physical AI & Humanoid Robotics Book Platform.
"""
import re
from typing import Any, Optional
from urllib.parse import urlparse
import bleach
import html


def validate_email(email: str) -> bool:
    """
    Validate email format using regex.

    Args:
        email: Email string to validate

    Returns:
        True if valid, False otherwise
    """
    pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
    return re.match(pattern, email) is not None


def validate_url(url: str) -> bool:
    """
    Validate URL format.

    Args:
        url: URL string to validate

    Returns:
        True if valid, False otherwise
    """
    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def sanitize_input(text: str, allowed_tags: Optional[list] = None) -> str:
    """
    Sanitize input text to prevent XSS attacks.

    Args:
        text: Input text to sanitize
        allowed_tags: List of allowed HTML tags (defaults to safe subset)

    Returns:
        Sanitized text
    """
    if allowed_tags is None:
        # Default to a safe subset of tags
        allowed_tags = ['p', 'br', 'strong', 'em', 'u', 'ol', 'ul', 'li']

    # First, escape any HTML entities
    text = html.escape(text)

    # Then use bleach to allow only safe tags
    sanitized = bleach.clean(text, tags=allowed_tags, strip=True)

    return sanitized


def validate_content_length(text: str, min_length: int = 1, max_length: int = 10000) -> bool:
    """
    Validate content length is within acceptable bounds.

    Args:
        text: Text to validate
        min_length: Minimum allowed length
        max_length: Maximum allowed length

    Returns:
        True if within bounds, False otherwise
    """
    length = len(text)
    return min_length <= length <= max_length


def validate_integer_range(value: int, min_val: int = 0, max_val: int = 1000000) -> bool:
    """
    Validate integer is within acceptable range.

    Args:
        value: Integer value to validate
        min_val: Minimum allowed value
        max_val: Maximum allowed value

    Returns:
        True if within range, False otherwise
    """
    return min_val <= value <= max_val


def validate_file_extension(filename: str, allowed_extensions: list) -> bool:
    """
    Validate file extension is allowed.

    Args:
        filename: Name of the file to validate
        allowed_extensions: List of allowed extensions (e.g., ['.txt', '.md', '.pdf'])

    Returns:
        True if extension is allowed, False otherwise
    """
    import os
    _, ext = os.path.splitext(filename.lower())
    return ext in allowed_extensions


def validate_file_size(file_size_bytes: int, max_size_mb: int = 10) -> bool:
    """
    Validate file size is within allowed limits.

    Args:
        file_size_bytes: Size of file in bytes
        max_size_mb: Maximum allowed size in megabytes

    Returns:
        True if within limits, False otherwise
    """
    max_size_bytes = max_size_mb * 1024 * 1024  # Convert MB to bytes
    return file_size_bytes <= max_size_bytes


def sanitize_filename(filename: str) -> str:
    """
    Sanitize filename to prevent path traversal and other security issues.

    Args:
        filename: Filename to sanitize

    Returns:
        Sanitized filename
    """
    # Remove path separators to prevent directory traversal
    sanitized = re.sub(r'[\/\\]+', '_', filename)

    # Remove potentially dangerous characters
    sanitized = re.sub(r'[<>:"|?*]', '_', sanitized)

    # Limit length to prevent issues
    if len(sanitized) > 255:
        name, ext = sanitized.rsplit('.', 1) if '.' in sanitized else (sanitized, '')
        sanitized = name[:250] + ('.' + ext if ext else '')

    return sanitized


def validate_json_payload(payload: Any, required_fields: Optional[list] = None) -> tuple[bool, str]:
    """
    Validate JSON payload structure.

    Args:
        payload: JSON payload to validate
        required_fields: List of required field names

    Returns:
        Tuple of (is_valid, error_message)
    """
    if not isinstance(payload, dict):
        return False, "Payload must be a dictionary"

    if required_fields:
        missing_fields = [field for field in required_fields if field not in payload]
        if missing_fields:
            return False, f"Missing required fields: {', '.join(missing_fields)}"

    return True, ""


def validate_query_params(params: dict, allowed_params: list) -> tuple[bool, str]:
    """
    Validate query parameters to prevent unexpected parameters.

    Args:
        params: Dictionary of query parameters
        allowed_params: List of allowed parameter names

    Returns:
        Tuple of (is_valid, error_message)
    """
    invalid_params = [param for param in params.keys() if param not in allowed_params]
    if invalid_params:
        return False, f"Invalid query parameters: {', '.join(invalid_params)}"

    return True, ""


def escape_special_characters(text: str) -> str:
    """
    Escape special characters that could be used in injection attacks.

    Args:
        text: Text to escape

    Returns:
        Escaped text
    """
    # Escape common special characters
    escaped = text.replace('\\', '\\\\').replace('"', '\\"').replace("'", "\\'")
    return escaped


def validate_boolean(value: Any) -> bool:
    """
    Validate if a value can be converted to a boolean.

    Args:
        value: Value to validate

    Returns:
        True if value is boolean-like, False otherwise
    """
    if isinstance(value, bool):
        return True
    if isinstance(value, str):
        return value.lower() in ['true', 'false', '1', '0', 'yes', 'no']
    if isinstance(value, int):
        return value in [0, 1]
    return False


def convert_to_boolean(value: Any) -> bool:
    """
    Convert a value to boolean.

    Args:
        value: Value to convert

    Returns:
        Boolean representation of the value
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ['true', '1', 'yes', 'on', 't', 'y']
    if isinstance(value, int):
        return bool(value)
    return False