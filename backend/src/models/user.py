from sqlalchemy import Integer, String, Boolean, DateTime, Text
from sqlalchemy.orm import Mapped, mapped_column
from typing import Optional
from datetime import datetime
from src.database.base import Base, TimestampMixin


class User(Base, TimestampMixin):
    """Model for platform users with optional authentication and authorization capabilities (for enhanced features)."""

    __tablename__ = "users"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    email: Mapped[str] = mapped_column(String, unique=True, index=True, nullable=False)
    hashed_password: Mapped[str] = mapped_column(String, nullable=False)
    full_name: Mapped[Optional[str]] = mapped_column(String)
    software_background_level: Mapped[Optional[str]] = mapped_column(String(50))  # beginner, intermediate, advanced
    hardware_background_level: Mapped[Optional[str]] = mapped_column(String(50))  # beginner, intermediate, advanced
    preferred_languages: Mapped[Optional[str]] = mapped_column(Text)  # JSON string of languages
    learning_goals: Mapped[Optional[str]] = mapped_column(Text)  # free text field
    personalization_preferences: Mapped[Optional[str]] = mapped_column(Text)  # JSON string for personalization settings
    is_active: Mapped[bool] = mapped_column(Boolean, default=True)
    is_admin: Mapped[bool] = mapped_column(Boolean, default=False)
    last_login: Mapped[Optional[datetime]] = mapped_column(DateTime)