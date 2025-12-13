#!/usr/bin/env python3
"""Debug the registration endpoint by calling the exact same function as the API"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

import asyncio
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from src.config.database import db_url
from src.api.auth import register

async def debug_endpoint():
    """Debug the exact endpoint function"""
    print("Testing the exact register endpoint function...")

    engine = create_async_engine(db_url)
    async_session = sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)

    async with async_session() as session:
        try:
            print("Calling register function with test data...")
            result = await register(
                email='test_endpoint_debug@example.com',
                password='test123',
                full_name='Test Endpoint Debug',
                software_background_level='beginner',
                hardware_background_level='beginner',
                preferred_languages='Python',
                learning_goals='Learn robotics',
                db=session
            )
            print(f"Success! Result: {result}")

        except Exception as e:
            print(f"Error in endpoint function: {e}")
            import traceback
            traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(debug_endpoint())