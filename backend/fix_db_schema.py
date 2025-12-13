#!/usr/bin/env python3
"""Script to fix database schema by adding missing columns."""

import asyncio
import sys
import os

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sqlalchemy import text
from src.config.database import engine

async def fix_database_schema():
    """Add missing columns to existing tables."""
    print("Fixing database schema...")

    async with engine.begin() as conn:
        # Add personalization_preferences column to users table if it doesn't exist
        try:
            await conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1
                        FROM information_schema.columns
                        WHERE table_name='users' AND column_name='personalization_preferences'
                    ) THEN
                        ALTER TABLE users ADD COLUMN personalization_preferences TEXT;
                    END IF;
                END $$;
            """))
            print("Added personalization_preferences column to users table")
        except Exception as e:
            print(f"Error adding personalization_preferences column: {e}")

        # Add other potentially missing columns based on the User model
        try:
            # Add software_background_level if missing
            await conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1
                        FROM information_schema.columns
                        WHERE table_name='users' AND column_name='software_background_level'
                    ) THEN
                        ALTER TABLE users ADD COLUMN software_background_level VARCHAR(50);
                    END IF;
                END $$;
            """))
            print("Added software_background_level column to users table")
        except Exception as e:
            print(f"Error adding software_background_level column: {e}")

        try:
            # Add hardware_background_level if missing
            await conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1
                        FROM information_schema.columns
                        WHERE table_name='users' AND column_name='hardware_background_level'
                    ) THEN
                        ALTER TABLE users ADD COLUMN hardware_background_level VARCHAR(50);
                    END IF;
                END $$;
            """))
            print("Added hardware_background_level column to users table")
        except Exception as e:
            print(f"Error adding hardware_background_level column: {e}")

        try:
            # Add preferred_languages if missing
            await conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1
                        FROM information_schema.columns
                        WHERE table_name='users' AND column_name='preferred_languages'
                    ) THEN
                        ALTER TABLE users ADD COLUMN preferred_languages TEXT;
                    END IF;
                END $$;
            """))
            print("Added preferred_languages column to users table")
        except Exception as e:
            print(f"Error adding preferred_languages column: {e}")

        try:
            # Add learning_goals if missing
            await conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1
                        FROM information_schema.columns
                        WHERE table_name='users' AND column_name='learning_goals'
                    ) THEN
                        ALTER TABLE users ADD COLUMN learning_goals TEXT;
                    END IF;
                END $$;
            """))
            print("Added learning_goals column to users table")
        except Exception as e:
            print(f"Error adding learning_goals column: {e}")

        try:
            # Add last_login if missing
            await conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1
                        FROM information_schema.columns
                        WHERE table_name='users' AND column_name='last_login'
                    ) THEN
                        ALTER TABLE users ADD COLUMN last_login TIMESTAMP;
                    END IF;
                END $$;
            """))
            print("Added last_login column to users table")
        except Exception as e:
            print(f"Error adding last_login column: {e}")

    print("Database schema fixes applied!")

if __name__ == "__main__":
    asyncio.run(fix_database_schema())