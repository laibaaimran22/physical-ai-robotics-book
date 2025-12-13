from typing import Optional
from sqlalchemy.ext.asyncio import AsyncSession
from datetime import datetime, timedelta

from src.models.user import User
from src.database.crud.user import authenticate_user, create_user as create_user_db, get_user_by_email
from src.utils.security import create_access_token, create_refresh_token, verify_token, get_password_hash
from src.config.settings import settings


class AuthService:
    """Service for handling user authentication and authorization."""

    def __init__(self, db: AsyncSession):
        self.db = db

    async def authenticate_user(self, email: str, password: str) -> Optional[User]:
        """
        Authenticate a user by email and password.

        Args:
            email: User's email
            password: User's password

        Returns:
            User object if authentication is successful, None otherwise
        """
        user = await authenticate_user(self.db, email, password)
        if not user:
            return None

        # Update last login time
        user.last_login = datetime.utcnow()
        # In a real implementation, we would update the user in the database
        # For now, we'll just return the user object

        return user

    async def register_user(
        self,
        email: str,
        password: str,
        full_name: Optional[str] = None,
        software_background_level: Optional[str] = None,
        hardware_background_level: Optional[str] = None,
        preferred_languages: Optional[str] = None,
        learning_goals: Optional[str] = None
    ) -> Optional[User]:
        """
        Register a new user.

        Args:
            email: User's email
            password: User's password
            full_name: User's full name (optional)
            software_background_level: User's software background level (beginner/intermediate/advanced)
            hardware_background_level: User's hardware background level (beginner/intermediate/advanced)
            preferred_languages: User's preferred programming languages
            learning_goals: User's learning goals

        Returns:
            User object if registration is successful, None otherwise
        """
        # Check if user already exists
        existing_user = await get_user_by_email(self.db, email)
        if existing_user:
            return None

        # Create new user
        user = User(
            email=email,
            hashed_password=get_password_hash(password),  # Hash the password before storing
            full_name=full_name,
            software_background_level=software_background_level,
            hardware_background_level=hardware_background_level,
            preferred_languages=preferred_languages,
            learning_goals=learning_goals,
            personalization_preferences='{"chapters": {}}',  # Initialize with empty chapters object
            is_active=True,
            is_admin=False
        )

        created_user = await create_user_db(self.db, user)
        return created_user

    async def create_access_token(self, user_id: int) -> str:
        """
        Create an access token for a user.

        Args:
            user_id: ID of the user

        Returns:
            JWT access token
        """
        data = {"sub": str(user_id)}
        expires_delta = timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
        return create_access_token(data, expires_delta)

    async def create_refresh_token(self, user_id: int) -> str:
        """
        Create a refresh token for a user.

        Args:
            user_id: ID of the user

        Returns:
            JWT refresh token
        """
        data = {"sub": str(user_id)}
        expires_delta = timedelta(days=settings.REFRESH_TOKEN_EXPIRE_DAYS)
        return create_refresh_token(data, expires_delta)

    async def verify_token(self, token: str) -> Optional[dict]:
        """
        Verify a JWT token.

        Args:
            token: JWT token to verify

        Returns:
            Token payload if valid, None otherwise
        """
        return verify_token(token)

    async def get_current_user_from_token(self, token: str) -> Optional[User]:
        """
        Get the current user from a token.

        Args:
            token: JWT token

        Returns:
            User object if token is valid, None otherwise
        """
        payload = await self.verify_token(token)
        if not payload:
            return None

        user_id = int(payload.get("sub"))
        if not user_id:
            return None

        from src.database.crud.user import get_user
        user = await get_user(self.db, user_id)
        return user


# Convenience function to create an auth service instance
def get_auth_service(db: AsyncSession) -> AuthService:
    """Get an instance of the auth service."""
    return AuthService(db)