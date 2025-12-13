#!/usr/bin/env python3
"""Test using the same dependency injection as FastAPI"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from src.api.deps import get_db
from src.api.auth import register
import asyncio

async def test_with_deps():
    """Test using the same dependency injection as FastAPI"""
    print("Testing with dependency injection...")

    # Get the database session the same way FastAPI does
    db_generator = get_db()
    db = await db_generator.__anext__()

    try:
        result = await register(
            email='test_deps@example.com',
            password='test123',
            full_name='Test Deps User',
            software_background_level=None,
            hardware_background_level=None,
            preferred_languages=None,
            learning_goals=None,
            db=db
        )
        print(f"Success! Result: {result}")
    except Exception as e:
        print(f"Error with dependency injection: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Close the session
        await db.close()

if __name__ == "__main__":
    asyncio.run(test_with_deps())