"""
Database models module - imports all SQLAlchemy models to ensure they are registered with the Base.
This file ensures all models are loaded before SQLAlchemy tries to resolve foreign key relationships.
"""

# Import Base first
from .base import Base

# Import all models to ensure foreign key relationships are resolved
from ..models.book_metadata import BookMetadata
from ..models.chapter import Chapter
from ..models.lesson import Lesson
from ..models.lesson_section import LessonSection
from ..models.embedding_document import EmbeddingDocument
from ..models.user import User
from ..models.chat_history import ChatHistory
from ..models.api_usage_quota import APIUsageQuota
from ..models.rag_query import RAGQuery
from ..models.highlight import Highlight
from ..models.note import Note
from ..models.ingestion_job import IngestionJob

# Ensure all models are registered
__all__ = [
    "BookMetadata",
    "Chapter",
    "Lesson",
    "LessonSection",
    "EmbeddingDocument",
    "User",
    "ChatHistory",
    "APIUsageQuota",
    "RAGQuery",
    "Highlight",
    "Note",
    "IngestionJob"
]