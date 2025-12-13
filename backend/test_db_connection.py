#!/usr/bin/env python3
"""Test script to check database connectivity."""

import asyncio
import sys
import os

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from sqlalchemy.ext.asyncio import create_async_engine
from src.config.settings import settings

async def test_db_connection():
    """Test database connection."""
    print("Testing database connection...")

    # Get the database URL with proper async driver
    from src.config.database import db_url
    print(f"Database URL: {db_url}")

    try:
        # Create engine
        engine = create_async_engine(
            db_url,
            echo=True,
            pool_pre_ping=True,
            pool_size=5,
            max_overflow=10,
            pool_recycle=300,
        )

        # Test the connection
        async with engine.begin() as conn:
            # Execute a simple query to test connection (using SQLAlchemy text)
            from sqlalchemy import text
            result = await conn.execute(text("SELECT 1 as test"))
            row = result.fetchone()
            print(f"Connection test result: {row[0] if row else 'No result'}")
            print("SUCCESS: Database connection successful!")

        await engine.dispose()
        return True

    except Exception as e:
        print(f"ERROR: Database connection failed: {e}")
        print("This could be due to:")
        print("1. Network connectivity issues to Neon")
        print("2. Invalid database credentials in .env file")
        print("3. Database service being temporarily unavailable")
        print("4. SSL/TLS configuration issues")
        return False

async def test_sqlite_fallback():
    """Test SQLite fallback connection."""
    print("\nTrying fallback to SQLite...")
    # Test SQLite connection
    sqlite_url = "sqlite+aiosqlite:///./book_platform.db"
    print(f"Testing SQLite fallback: {sqlite_url}")

    engine = create_async_engine(sqlite_url, echo=True)
    try:
        async with engine.begin() as conn:
            from sqlalchemy import text
            result = await conn.execute(text("SELECT 1 as test"))
            row = result.fetchone()
            print(f"SQLite connection test result: {row[0] if row else 'No result'}")
            print("SUCCESS: SQLite fallback connection successful!")
            return True
    except Exception as e:
        print(f"ERROR: SQLite fallback also failed: {e}")
        return False
    finally:
        await engine.dispose()


if __name__ == "__main__":
    success = asyncio.run(test_db_connection())
    if not success:
        asyncio.run(test_sqlite_fallback())