#!/usr/bin/env python3
"""Script to initialize database tables."""

import asyncio
import sys
import os

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sqlalchemy.ext.asyncio import create_async_engine
from src.database.base import Base
from src.config.settings import settings

async def init_db():
    """Initialize database tables."""
    print("Initializing database tables...")

    # Get the database URL with proper async driver
    from src.config.database import db_url
    print(f"Using database URL: {db_url}")

    # Create engine with more comprehensive error handling
    engine = create_async_engine(
        db_url,
        echo=settings.DEBUG,
        pool_pre_ping=True,
        pool_size=5,
        max_overflow=10,
        pool_recycle=300,
    )

    try:
        # Test the connection first
        async with engine.begin() as conn:
            # Execute a simple query to test connection (using SQLAlchemy text)
            from sqlalchemy import text
            await conn.execute(text("SELECT 1"))
            print("Database connection successful!")

        # Create all tables
        async with engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)

        print("Database tables created successfully!")

    except Exception as e:
        print(f"Error initializing database: {e}")
        print("This may be due to:")
        print("1. Network connectivity issues to the database")
        print("2. Invalid database credentials in .env file")
        print("3. Database service being temporarily unavailable")
        print("\nFalling back to SQLite database...")

        # Fallback to SQLite
        sqlite_url = "sqlite+aiosqlite:///./book_platform.db"
        print(f"Creating tables in fallback database: {sqlite_url}")

        fallback_engine = create_async_engine(
            sqlite_url,
            echo=settings.DEBUG,
            pool_pre_ping=True,
        )

        async with fallback_engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)

        print("Fallback database tables created successfully!")

    finally:
        await engine.dispose()

if __name__ == "__main__":
    asyncio.run(init_db())