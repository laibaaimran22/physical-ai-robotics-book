from sqlalchemy import Integer, String, Text, ForeignKey
from sqlalchemy.orm import Mapped, mapped_column, relationship
from typing import Optional, List
from src.database.base import Base, TimestampMixin


class Chapter(Base, TimestampMixin):
    """Model for storing book chapters."""

    __tablename__ = "chapters"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    title: Mapped[str] = mapped_column(String, nullable=False)
    description: Mapped[Optional[str]] = mapped_column(Text)
    order_index: Mapped[int] = mapped_column(Integer, nullable=False)
    book_metadata_id: Mapped[int] = mapped_column(Integer, ForeignKey("book_metadata.id"), nullable=False)

    # Relationships
    book_metadata: Mapped["BookMetadata"] = relationship("BookMetadata", back_populates="chapters")
    lessons: Mapped[List["Lesson"]] = relationship("Lesson", back_populates="chapter", cascade="all, delete-orphan")