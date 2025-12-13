from sqlalchemy import Integer, String, Text
from sqlalchemy.orm import Mapped, mapped_column, relationship
from typing import Optional, List
from src.database.base import Base, TimestampMixin


class BookMetadata(Base, TimestampMixin):
    """Model for storing book metadata information."""

    __tablename__ = "book_metadata"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    title: Mapped[str] = mapped_column(String, nullable=False)
    description: Mapped[Optional[str]] = mapped_column(Text)
    version: Mapped[str] = mapped_column(String, nullable=False)  # Version of the book content
    total_chapters: Mapped[int] = mapped_column(Integer, default=0)
    total_lessons: Mapped[int] = mapped_column(Integer, default=0)
    total_words: Mapped[int] = mapped_column(Integer, default=0)
    language: Mapped[str] = mapped_column(String, default="en")

    # Relationships
    chapters: Mapped[List["Chapter"]] = relationship("Chapter", back_populates="book_metadata")