#!/usr/bin/env python3
"""Test the registration endpoint with detailed error logging"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

import asyncio
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from src.config.database import db_url
from src.services.auth_service import get_auth_service

async def test_registration_directly():
    """Test registration directly to see what happens"""
    print("Testing registration directly...")

    engine = create_async_engine(db_url)
    async_session = sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)

    async with async_session() as session:
        try:
            service = get_auth_service(session)
            print("+ Auth service created successfully")

            # Test registration with a new email
            user = await service.register_user(
                email='test_direct_debug@example.com',
                password='test123',
                full_name='Test Direct Debug User'
            )
            print(f"Registration result: {user}")

            if user:
                print(f"+ User created with ID: {user.id}")

                # Test token creation
                access_token = await service.create_access_token(user.id)
                refresh_token = await service.create_refresh_token(user.id)
                print(f"+ Tokens created successfully")
                print(f"Access token: {access_token[:30]}...")
                print(f"Refresh token: {refresh_token[:30]}...")
            else:
                print("- User registration failed - likely email already exists")

        except Exception as e:
            print(f"- Error in registration: {e}")
            import traceback
            traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_registration_directly())