from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime


class BaseSchema(BaseModel):
    """Base Pydantic schema with common configurations."""

    class Config:
        from_attributes = True
        # Allow extra fields during development, restrict in production
        extra = "allow"  # Change to "forbid" in production


class ResponseBase(BaseModel):
    """Base response schema with standard fields."""
    success: bool = True
    message: Optional[str] = None
    data: Optional[Any] = None
    timestamp: datetime = datetime.now()


class PaginationParams(BaseModel):
    """Common pagination parameters."""
    skip: int = 0
    limit: int = 100


class PaginatedResponse(BaseModel):
    """Response schema for paginated results."""
    items: List[Any]
    total: int
    skip: int
    limit: int