#!/usr/bin/env python3
"""Script to properly initialize database tables by importing models first."""

import asyncio
import sys
import os

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy import text

async def proper_init_db():
    """Properly initialize database tables by importing models first."""
    print("Properly initializing database tables...")

    # Import models first to register them with Base metadata
    import src.models
    print("Models imported successfully")

    # Get the database URL with proper async driver
    from src.config.database import db_url
    print(f"Using database URL: {db_url}")

    # Create engine with more comprehensive error handling
    engine = create_async_engine(
        db_url,
        echo=True,
        pool_pre_ping=True,
        pool_size=5,
        max_overflow=10,
        pool_recycle=300,
    )

    try:
        # Test the connection first
        async with engine.begin() as conn:
            from sqlalchemy import text
            await conn.execute(text("SELECT 1"))
            print("Database connection successful!")

        # Import Base after models to ensure tables are registered
        from src.database.base import Base

        # Create all tables
        async with engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)

        print("Database tables created successfully!")

        # Verify tables were created
        async with engine.begin() as conn:
            result = await conn.execute(
                text("SELECT table_name FROM information_schema.tables WHERE table_schema = 'public';")
            )
            all_tables = [row[0] for row in result.fetchall()]
            print(f"Tables in database after creation: {all_tables}")

            expected_tables = ['users', 'rag_queries', 'chat_history']
            existing_expected = [t for t in expected_tables if t in all_tables]
            print(f"Expected tables that exist: {existing_expected}")

    except Exception as e:
        print(f"ERROR: Database initialization failed: {e}")
        import traceback
        traceback.print_exc()

    finally:
        await engine.dispose()

if __name__ == "__main__":
    asyncio.run(proper_init_db())