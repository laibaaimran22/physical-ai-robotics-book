# Import SQLAlchemy models to register them with the Base metadata
from .user import User
from .book_metadata import BookMetadata
from .chapter import Chapter
from .lesson import Lesson
from .lesson_section import LessonSection
from .note import Note
from .highlight import Highlight
from .chat_history import ChatHistory
from .rag_query import RAGQuery
from .api_usage_quota import APIUsageQuota
from .embedding_document import EmbeddingDocument
from .ingestion_job import IngestionJob

# Keep the Pydantic schemas
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