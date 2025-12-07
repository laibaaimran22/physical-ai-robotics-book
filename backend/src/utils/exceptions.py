from fastapi import HTTPException, status
from typing import Optional


class APIException(HTTPException):
    """Base API exception class."""

    def __init__(
        self,
        status_code: int,
        detail: str,
        headers: Optional[dict] = None
    ):
        super().__init__(status_code=status_code, detail=detail, headers=headers)


class BadRequestException(APIException):
    """Exception for bad requests (400)."""

    def __init__(self, detail: str = "Bad Request"):
        super().__init__(status_code=status.HTTP_400_BAD_REQUEST, detail=detail)


class UnauthorizedException(APIException):
    """Exception for unauthorized access (401)."""

    def __init__(self, detail: str = "Unauthorized"):
        super().__init__(status_code=status.HTTP_401_UNAUTHORIZED, detail=detail)


class ForbiddenException(APIException):
    """Exception for forbidden access (403)."""

    def __init__(self, detail: str = "Forbidden"):
        super().__init__(status_code=status.HTTP_403_FORBIDDEN, detail=detail)


class NotFoundException(APIException):
    """Exception for not found resources (404)."""

    def __init__(self, detail: str = "Not Found"):
        super().__init__(status_code=status.HTTP_404_NOT_FOUND, detail=detail)


class ConflictException(APIException):
    """Exception for conflicts (409)."""

    def __init__(self, detail: str = "Conflict"):
        super().__init__(status_code=status.HTTP_409_CONFLICT, detail=detail)


class InternalServerErrorException(APIException):
    """Exception for internal server errors (500)."""

    def __init__(self, detail: str = "Internal Server Error"):
        super().__init__(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=detail)


class RateLimitExceededException(APIException):
    """Exception for rate limit exceeded (429)."""

    def __init__(self, detail: str = "Rate Limit Exceeded"):
        super().__init__(status_code=status.HTTP_429_TOO_MANY_REQUESTS, detail=detail)


class ValidationErrorException(APIException):
    """Exception for validation errors (422)."""

    def __init__(self, detail: str = "Validation Error"):
        super().__init__(status_code=status.HTTP_422_UNPROCESSABLE_ENTITY, detail=detail)


# Custom exceptions for specific use cases
class UserNotFoundException(NotFoundException):
    """Exception raised when a user is not found."""

    def __init__(self, user_id: int):
        super().__init__(detail=f"User with ID {user_id} not found")


class IngestionJobNotFoundException(NotFoundException):
    """Exception raised when an ingestion job is not found."""

    def __init__(self, job_id: str):
        super().__init__(detail=f"Ingestion job with ID {job_id} not found")


class ContentNotFoundException(NotFoundException):
    """Exception raised when content is not found."""

    def __init__(self, content_id: int, content_type: str):
        super().__init__(detail=f"{content_type.capitalize()} with ID {content_id} not found")


class QuotaExceededException(RateLimitExceededException):
    """Exception raised when API quota is exceeded."""

    def __init__(self, quota_type: str, user_id: Optional[int] = None):
        user_info = f" for user {user_id}" if user_id else ""
        super().__init__(detail=f"Quota exceeded for {quota_type}{user_info}")