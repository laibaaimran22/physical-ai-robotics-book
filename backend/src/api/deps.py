from fastapi import Depends, HTTPException, status, Security
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Generator, Optional
import jwt
from jwt.exceptions import InvalidTokenError

from src.config.database import get_db_session
from src.config.settings import settings
from src.models.user import User
from src.database.crud.user import get_user as get_user_by_id


# Security scheme for JWT
security = HTTPBearer()


async def get_db() -> Generator[AsyncSession, None, None]:
    """Dependency to get database session."""
    async for session in get_db_session():
        try:
            yield session
        finally:
            await session.close()


async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db: AsyncSession = Depends(get_db)
) -> User:
    """Dependency to get the current authenticated user."""
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    try:
        payload = jwt.decode(
            credentials.credentials,
            settings.JWT_SECRET_KEY,
            algorithms=[settings.JWT_ALGORITHM]
        )
        user_id: str = payload.get("sub")
        if user_id is None:
            raise credentials_exception
    except InvalidTokenError:
        raise credentials_exception

    user = await get_user_by_id(db, user_id=int(user_id))
    if user is None:
        raise credentials_exception

    return user


async def get_optional_user(
    auth_header: Optional[HTTPAuthorizationCredentials] = Security(HTTPBearer(auto_error=False)),
    db: AsyncSession = Depends(get_db)
) -> Optional[User]:
    """Dependency to get the current user if authenticated (optional)."""
    if not auth_header or not auth_header.credentials:
        return None

    try:
        payload = jwt.decode(
            auth_header.credentials,
            settings.JWT_SECRET_KEY,
            algorithms=[settings.JWT_ALGORITHM]
        )
        user_id: str = payload.get("sub")
        if user_id is None:
            return None
    except (InvalidTokenError, jwt.ExpiredSignatureError):
        return None

    user = await get_user_by_id(db, user_id=int(user_id))
    return user if user else None


# Rate limiting dependency will be implemented later
async def check_rate_limit():
    """Dependency to check rate limits."""
    # This will be implemented in the middleware
    pass