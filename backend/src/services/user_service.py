from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional

from ..models.user import User
from ..database.crud.user import get_user as get_user_db


async def get_user_by_id(db: AsyncSession, user_id: int) -> Optional[User]:
    """
    Retrieve a user by their ID.

    Args:
        db: Database session
        user_id: ID of the user to retrieve

    Returns:
        User object if found, None otherwise
    """
    return await get_user_db(db, user_id)