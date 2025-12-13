from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import Optional

from src.models.user import User
from src.utils.security import get_password_hash


async def create_user(db: AsyncSession, user: User) -> User:
    """Create a new user."""
    # Hash the password before storing if it's not already hashed
    if hasattr(user, 'hashed_password') and user.hashed_password:
        # Check if password is already hashed - it should be 60+ characters for bcrypt or 96 for our pbkdf2 fallback
        # Bcrypt hashes start with $2b$, our fallback hashes are 96 hex characters (32 salt + 64 hash)
        is_already_hashed = (
            user.hashed_password.startswith('$2b$') or  # bcrypt
            len(user.hashed_password) == 96  # our pbkdf2 fallback (32 hex chars salt + 64 hex chars hash)
        )
        if not is_already_hashed:
            user.hashed_password = get_password_hash(user.hashed_password)

    db.add(user)
    await db.commit()
    await db.refresh(user)
    return user


async def get_user(db: AsyncSession, user_id: int) -> Optional[User]:
    """Get a user by ID."""
    result = await db.execute(select(User).filter(User.id == user_id))
    return result.scalar_one_or_none()


async def get_user_by_email(db: AsyncSession, email: str) -> Optional[User]:
    """Get a user by email."""
    result = await db.execute(select(User).filter(User.email == email))
    return result.scalar_one_or_none()


async def update_user(db: AsyncSession, user_id: int, **kwargs) -> Optional[User]:
    """Update a user."""
    result = await db.execute(select(User).filter(User.id == user_id))
    user = result.scalar_one_or_none()

    if not user:
        return None

    # Hash password if it's being updated
    if 'hashed_password' in kwargs:
        kwargs['hashed_password'] = get_password_hash(kwargs['hashed_password'])

    for key, value in kwargs.items():
        setattr(user, key, value)

    await db.commit()
    await db.refresh(user)
    return user


async def delete_user(db: AsyncSession, user_id: int) -> bool:
    """Delete a user."""
    result = await db.execute(select(User).filter(User.id == user_id))
    user = result.scalar_one_or_none()

    if not user:
        return False

    await db.delete(user)
    await db.commit()
    return True


async def authenticate_user(db: AsyncSession, email: str, password: str):
    """Authenticate a user by email and password."""
    from src.utils.security import verify_password

    user = await get_user_by_email(db, email)
    if not user or not verify_password(password, user.hashed_password):
        return None
    return user