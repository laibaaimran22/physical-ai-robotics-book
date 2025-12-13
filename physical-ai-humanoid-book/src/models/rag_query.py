from sqlalchemy import Integer, Text, JSON, Boolean
from sqlalchemy.orm import Mapped, mapped_column
from typing import Optional, List, Dict, Any
from src.database.base import Base, TimestampMixin


class RAGQuery(Base, TimestampMixin):
    """Model for tracking user RAG queries for analytics and improvement."""

    __tablename__ = "rag_queries"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    user_id: Mapped[Optional[int]] = mapped_column(Integer, nullable=True)  # Optional for anonymous queries
    query_text: Mapped[str] = mapped_column(Text, nullable=False)
    response_text: Mapped[str] = mapped_column(Text, nullable=False)
    context_chunks: Mapped[List[Dict[str, Any]]] = mapped_column(JSON, nullable=False)  # Retrieved chunks
    tokens_used: Mapped[int] = mapped_column(Integer, nullable=False)
    response_time_ms: Mapped[int] = mapped_column(Integer, nullable=False)
    is_hallucination: Mapped[bool] = mapped_column(Boolean, default=False)  # Flag for quality control
    feedback_score: Mapped[Optional[int]] = mapped_column(Integer)  # 1-5 rating