from sqlalchemy import Integer, String, Text, Boolean, ForeignKey
from sqlalchemy.orm import Mapped, mapped_column
from typing import Optional
from src.database.base import Base, TimestampMixin


class Note(Base, TimestampMixin):
    """Model for storing user notes associated with content (optional feature)."""

    __tablename__ = "notes"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    user_id: Mapped[Optional[int]] = mapped_column(Integer, nullable=True)  # Optional for anonymous notes
    content_id: Mapped[int] = mapped_column(Integer, nullable=False)  # ID of the content (lesson/section)
    content_type: Mapped[str] = mapped_column(String, nullable=False)  # "lesson" or "lesson_section"
    title: Mapped[str] = mapped_column(String, nullable=False)
    content: Mapped[str] = mapped_column(Text, nullable=False)  # The note content
    is_public: Mapped[bool] = mapped_column(Boolean, default=False)  # Whether note is visible to others