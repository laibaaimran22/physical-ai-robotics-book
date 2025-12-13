from sqlalchemy import Integer, String, Text, ForeignKey
from sqlalchemy.orm import Mapped, mapped_column, relationship
from typing import Optional, List
from src.database.base import Base, TimestampMixin


class LessonSection(Base, TimestampMixin):
    """Model for storing smaller sections within lessons."""

    __tablename__ = "lesson_sections"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    title: Mapped[str] = mapped_column(String, nullable=False)
    content: Mapped[str] = mapped_column(Text, nullable=False)  # Markdown content
    order_index: Mapped[int] = mapped_column(Integer, nullable=False)
    lesson_id: Mapped[int] = mapped_column(Integer, ForeignKey("lessons.id"), nullable=False)

    # Relationships
    lesson: Mapped["Lesson"] = relationship("Lesson", back_populates="lesson_sections")