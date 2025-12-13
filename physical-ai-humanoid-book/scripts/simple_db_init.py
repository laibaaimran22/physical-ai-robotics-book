#!/usr/bin/env python3
"""
Simple database initialization script for the Physical AI & Humanoid Robotics Book Platform
"""
import sys
from pathlib import Path

# Add the backend/src directory to the Python path
sys.path.insert(0, str(Path(__file__).parent.parent))

from sqlalchemy import create_engine, text
from src.database.base import Base

def initialize_db():
    """Initialize the database with all required tables."""
    print("Initializing database...")

    # Import all models to register them with Base.metadata
    from src.models.user import User
    from src.models.book_metadata import BookMetadata
    from src.models.chapter import Chapter
    from src.models.lesson import Lesson
    from src.models.lesson_section import LessonSection
    from src.models.embedding_document import EmbeddingDocument
    from src.models.rag_query import RAGQuery
    from src.models.chat_history import ChatHistory
    from src.models.ingestion_job import IngestionJob
    from src.models.api_usage_quota import APIUsageQuota
    from src.models.highlight import Highlight
    from src.models.note import Note

    # Use a simple SQLite database for testing
    engine = create_engine("sqlite:///./book_platform.db", echo=True)

    try:
        # Create all tables based on SQLAlchemy models
        Base.metadata.create_all(engine)
        print("Database tables created successfully!")

        # Verify tables were created
        with engine.connect() as conn:
            result = conn.execute(text("SELECT name FROM sqlite_master WHERE type='table';"))
            tables = [row[0] for row in result.fetchall()]
            print(f"Created tables: {tables}")

        return True
    except Exception as e:
        print(f"Error initializing database: {e}")
        return False

if __name__ == "__main__":
    success = initialize_db()
    if success:
        print("Database initialization completed successfully!")
    else:
        print("Database initialization failed!")
        sys.exit(1)