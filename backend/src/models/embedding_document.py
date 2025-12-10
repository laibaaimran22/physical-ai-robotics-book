from sqlalchemy import Integer, String, Text, ForeignKey, JSON, ARRAY
from sqlalchemy.orm import Mapped, mapped_column, relationship
from typing import Optional, Dict, Any, List
from src.database.base import Base, TimestampMixin


class EmbeddingDocument(Base, TimestampMixin):
    """Model for linking content chunks to vector embeddings in Qdrant."""

    __tablename__ = "embedding_documents"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    qdrant_id: Mapped[str] = mapped_column(String, unique=True, nullable=False)  # ID in Qdrant
    content: Mapped[str] = mapped_column(Text, nullable=False)  # The actual content chunk
    content_type: Mapped[str] = mapped_column(String, nullable=False)  # "lesson", "lesson_section", etc.
    content_id: Mapped[int] = mapped_column(Integer, nullable=False)  # ID of the content in its table
    doc_metadata: Mapped[Dict[str, Any]] = mapped_column(JSON, nullable=False)  # Additional metadata for search
    embedding_model: Mapped[str] = mapped_column(String, default="all-MiniLM-L6-v2")
    tokens: Mapped[int] = mapped_column(Integer, nullable=False)

    # Foreign key references (linked to actual content)
    chapter_id: Mapped[Optional[int]] = mapped_column(Integer, ForeignKey("chapters.id"))
    lesson_id: Mapped[Optional[int]] = mapped_column(Integer, ForeignKey("lessons.id"))
    lesson_section_id: Mapped[Optional[int]] = mapped_column(Integer, ForeignKey("lesson_sections.id"))

    # Relationships - commented out to avoid circular import issues
    # chapter: Mapped[Optional["Chapter"]] = relationship("Chapter", back_populates="embedding_documents")
    # lesson: Mapped[Optional["Lesson"]] = relationship("Lesson", back_populates="embedding_documents")
    # lesson_section: Mapped[Optional["LessonSection"]] = relationship("LessonSection", back_populates="embedding_documents")