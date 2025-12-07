from sqlalchemy import Integer, String, Text, DateTime, ForeignKey
from sqlalchemy.orm import Mapped, mapped_column, relationship
from typing import Optional
from datetime import datetime
from src.database.base import Base, TimestampMixin


class IngestionJob(Base, TimestampMixin):
    """Model for tracking document ingestion jobs with status and results."""

    __tablename__ = "ingestion_jobs"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    job_id: Mapped[str] = mapped_column(String, unique=True, nullable=False)  # UUID for the job
    admin_user_id: Mapped[Optional[int]] = mapped_column(Integer, nullable=True)  # Who started the job (optional for public ingestion)
    file_path: Mapped[str] = mapped_column(String, nullable=False)  # Path to uploaded file
    file_name: Mapped[str] = mapped_column(String, nullable=False)
    file_type: Mapped[str] = mapped_column(String, nullable=False)  # "markdown", "html", "pdf", etc.
    status: Mapped[str] = mapped_column(String, default="pending")  # "pending", "processing", "completed", "failed"
    total_chunks: Mapped[Optional[int]] = mapped_column(Integer)  # Total chunks processed
    processed_chunks: Mapped[int] = mapped_column(Integer, default=0)  # Chunks completed
    error_message: Mapped[Optional[str]] = mapped_column(Text)  # Error if job failed
    started_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.now())
    completed_at: Mapped[Optional[datetime]] = mapped_column(DateTime)