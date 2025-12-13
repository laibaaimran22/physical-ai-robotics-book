from sqlalchemy import Integer, String, Text, Boolean
from sqlalchemy.orm import Mapped, mapped_column
from typing import Optional
from src.database.base import Base, TimestampMixin


class Highlight(Base, TimestampMixin):
    """Model for storing user highlights in lessons (optional feature)."""

    __tablename__ = "highlights"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    user_id: Mapped[Optional[int]] = mapped_column(Integer, nullable=True)  # Optional for anonymous highlights
    content_id: Mapped[int] = mapped_column(Integer, nullable=False)  # ID of the content (lesson/section)
    content_type: Mapped[str] = mapped_column(String, nullable=False)  # "lesson" or "lesson_section"
    text: Mapped[str] = mapped_column(Text, nullable=False)  # The highlighted text
    start_pos: Mapped[int] = mapped_column(Integer, nullable=False)  # Start position in content
    end_pos: Mapped[int] = mapped_column(Integer, nullable=False)  # End position in content
    note: Mapped[Optional[str]] = mapped_column(String)  # Optional note about the highlight