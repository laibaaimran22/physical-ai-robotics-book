#!/usr/bin/env python3
"""Test script to verify tables were created properly."""

import asyncio
import sys
import os

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy import text
from src.config.settings import settings

async def verify_tables():
    """Verify that tables were created properly."""
    print("Verifying table creation...")

    # Get the database URL with proper async driver
    from src.config.database import db_url
    print(f"Database URL: {db_url}")

    try:
        # Create engine
        engine = create_async_engine(
            db_url,
            echo=False,
            pool_pre_ping=True,
            pool_size=5,
            max_overflow=10,
            pool_recycle=300,
        )

        async with engine.begin() as conn:
            # Get all tables including their schema
            result = await conn.execute(
                text("""
                SELECT table_schema, table_name
                FROM information_schema.tables
                WHERE table_name IN ('users', 'rag_queries', 'chat_history', 'api_usage_quota', 'book_metadata', 'chapter', 'lesson', 'lesson_section', 'note', 'highlight', 'embedding_document', 'ingestion_job')
                ORDER BY table_schema, table_name;
                """)
            )
            found_tables = [(row[0], row[1]) for row in result.fetchall()]
            print(f"Found application tables: {found_tables}")

            # Check specifically in public schema
            result = await conn.execute(
                text("""
                SELECT table_name
                FROM information_schema.tables
                WHERE table_schema = 'public'
                AND table_name IN ('users', 'rag_queries', 'chat_history', 'api_usage_quota', 'book_metadata', 'chapter', 'lesson', 'lesson_section', 'note', 'highlight', 'embedding_document', 'ingestion_job')
                ORDER BY table_name;
                """)
            )
            public_tables = [row[0] for row in result.fetchall()]
            print(f"Application tables in public schema: {public_tables}")

            # List all tables in public schema
            result = await conn.execute(
                text("SELECT table_name FROM information_schema.tables WHERE table_schema = 'public' ORDER BY table_name;")
            )
            all_public_tables = [row[0] for row in result.fetchall()]
            print(f"All tables in public schema: {all_public_tables}")

            # Check if any of our expected tables exist
            expected_tables = ['users', 'rag_queries', 'chat_history']
            existing_tables = [table for table in expected_tables if table in all_public_tables]
            missing_tables = [table for table in expected_tables if table not in all_public_tables]

            print(f"\nExpected tables: {expected_tables}")
            print(f"Existing tables: {existing_tables}")
            print(f"Missing tables: {missing_tables}")

            if not missing_tables:
                print("\nSUCCESS: All required tables exist in the database!")
                return True
            else:
                print(f"\nMISSING: Tables still missing: {missing_tables}")
                # Try to create tables directly as a test
                print("Attempting to create missing tables manually...")
                from src.database.base import Base
                await conn.run_sync(Base.metadata.create_all)
                print("Manual table creation attempted.")

                # Check again
                result = await conn.execute(
                    text("SELECT table_name FROM information_schema.tables WHERE table_schema = 'public';")
                )
                all_public_tables_after = [row[0] for row in result.fetchall()]
                print(f"Tables after manual creation: {all_public_tables_after}")

                existing_after = [table for table in expected_tables if table in all_public_tables_after]
                print(f"Tables now existing: {existing_after}")

                return len(expected_tables) == len(existing_after)

        await engine.dispose()

    except Exception as e:
        print(f"ERROR: Could not verify tables: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    asyncio.run(verify_tables())