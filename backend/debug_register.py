#!/usr/bin/env python3
"""Debug script to test user registration step by step."""

import asyncio
import sys
import os

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from src.config.database import db_url
from src.models.user import User
from src.database.crud.user import create_user as create_user_db
from src.utils.security import get_password_hash
from src.services.auth_service import get_auth_service

async def test_user_creation():
    """Test user creation step by step."""
    print("Testing user creation process...")

    # Create engine and session
    engine = create_async_engine(db_url)
    async_session = sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)

    async with async_session() as session:
        print("1. Testing password hashing...")
        try:
            password = "test123"
            hashed = get_password_hash(password)
            print(f"   + Password hashing successful: {hashed[:20]}...")
        except Exception as e:
            print(f"   X Password hashing failed: {e}")
            return False

        print("2. Testing user object creation...")
        try:
            user = User(
                email="test@example.com",
                hashed_password=password,  # This will be hashed in the CRUD function
                full_name="Test User",
                is_active=True,
                is_admin=False
            )
            print("   + User object created successfully")
        except Exception as e:
            print(f"   X User object creation failed: {e}")
            return False

        print("3. Testing database creation...")
        try:
            created_user = await create_user_db(session, user)
            print(f"   + User created in database: {created_user.email}")
        except Exception as e:
            print(f"   X Database creation failed: {e}")
            import traceback
            traceback.print_exc()
            return False

        print("4. Testing auth service registration...")
        try:
            auth_service = get_auth_service(session)
            registered_user = await auth_service.register_user(
                email="test2@example.com",
                password="test123",
                full_name="Test User 2"
            )
            print(f"   + User registered via auth service: {registered_user.email if registered_user else 'None'}")
        except Exception as e:
            print(f"   X Auth service registration failed: {e}")
            import traceback
            traceback.print_exc()
            return False

        await session.rollback()  # Rollback the test transactions
        return True

if __name__ == "__main__":
    success = asyncio.run(test_user_creation())
    print(f"\nUser creation test: {'SUCCESS' if success else 'FAILED'}")