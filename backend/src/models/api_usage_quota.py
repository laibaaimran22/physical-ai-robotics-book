from sqlalchemy import Integer, String, DateTime
from sqlalchemy.orm import Mapped, mapped_column
from typing import Optional
from datetime import datetime
from src.database.base import Base, TimestampMixin


class APIUsageQuota(Base, TimestampMixin):
    """Model for tracking API usage for users to enforce resource limits (optional feature)."""

    __tablename__ = "api_usage_quotas"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    user_id: Mapped[int] = mapped_column(Integer, nullable=False)  # ID of the user
    quota_type: Mapped[str] = mapped_column(String, nullable=False)  # "rag_query", "search", etc.
    quota_limit: Mapped[int] = mapped_column(Integer, nullable=False)  # Max allowed per period
    quota_used: Mapped[int] = mapped_column(Integer, default=0)  # Used in current period
    quota_period_start: Mapped[datetime] = mapped_column(DateTime, nullable=False)  # Period start time
    reset_interval_hours: Mapped[int] = mapped_column(Integer, default=24)  # Hours between resets