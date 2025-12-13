from sqlalchemy import Integer, String, Text, DateTime
from sqlalchemy.orm import Mapped, mapped_column
from typing import Optional
from datetime import datetime
from src.database.base import Base, TimestampMixin


class ChatHistory(Base, TimestampMixin):
    """Model for storing conversation history for users."""

    __tablename__ = "chat_history"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    user_id: Mapped[Optional[int]] = mapped_column(Integer, nullable=True)  # Optional for anonymous chats
    session_id: Mapped[str] = mapped_column(String, index=True, nullable=False)
    role: Mapped[str] = mapped_column(String, nullable=False)  # "user" or "assistant"
    content: Mapped[str] = mapped_column(Text, nullable=False)
    timestamp: Mapped[datetime] = mapped_column(DateTime, default=datetime.now())