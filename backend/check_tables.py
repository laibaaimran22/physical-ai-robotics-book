#!/usr/bin/env python3
"""Test script to check if required tables exist in the database."""

import asyncio
import sys
import os

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from sqlalchemy.ext.asyncio import create_async_engine
from sqlalchemy import text
from src.config.settings import settings

async def check_tables_exist():
    """Check if required tables exist in the database."""
    print("Checking if required tables exist...")

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

        # Check for required tables
        async with engine.begin() as conn:
            # Check if users table exists
            if "postgresql" in db_url:
                # For PostgreSQL
                result = await conn.execute(
                    text("SELECT EXISTS (SELECT FROM information_schema.tables WHERE table_name = 'users');")
                )
                users_exist = result.scalar()
                print(f"Users table exists: {users_exist}")

                # Check if rag_queries table exists
                result = await conn.execute(
                    text("SELECT EXISTS (SELECT FROM information_schema.tables WHERE table_name = 'rag_queries');")
                )
                rag_queries_exist = result.scalar()
                print(f"RAG Queries table exists: {rag_queries_exist}")

                # Check if chat_history table exists
                result = await conn.execute(
                    text("SELECT EXISTS (SELECT FROM information_schema.tables WHERE table_name = 'chat_history');")
                )
                chat_history_exist = result.scalar()
                print(f"Chat History table exists: {chat_history_exist}")

                # List all tables in the database
                result = await conn.execute(
                    text("SELECT table_name FROM information_schema.tables WHERE table_schema = 'public';")
                )
                all_tables = [row[0] for row in result.fetchall()]
                print(f"All tables in database: {all_tables}")

            elif "sqlite" in db_url:
                # For SQLite
                result = await conn.execute(text("SELECT name FROM sqlite_master WHERE type='table';"))
                all_tables = [row[0] for row in result.fetchall()]
                print(f"All tables in database: {all_tables}")

                users_exist = 'users' in all_tables
                rag_queries_exist = 'rag_queries' in all_tables
                chat_history_exist = 'chat_history' in all_tables

                print(f"Users table exists: {users_exist}")
                print(f"RAG Queries table exists: {rag_queries_exist}")
                print(f"Chat History table exists: {chat_history_exist}")

        await engine.dispose()

        # Summary
        all_tables_exist = users_exist and rag_queries_exist and chat_history_exist
        print(f"\nAll required tables exist: {all_tables_exist}")

        if all_tables_exist:
            print("✅ SUCCESS: All required tables exist in the database!")
        else:
            missing_tables = []
            if not users_exist:
                missing_tables.append("users")
            if not rag_queries_exist:
                missing_tables.append("rag_queries")
            if not chat_history_exist:
                missing_tables.append("chat_history")
            print(f"❌ MISSING: The following tables are missing: {missing_tables}")
            print("You need to run the database initialization script to create these tables.")

        return all_tables_exist

    except Exception as e:
        print(f"ERROR: Could not check tables: {e}")
        return False

if __name__ == "__main__":
    asyncio.run(check_tables_exist())