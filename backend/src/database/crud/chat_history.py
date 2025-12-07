from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import Optional, List

from src.models.chat_history import ChatHistory


async def create_chat_history(db: AsyncSession, chat_history: ChatHistory) -> ChatHistory:
    """Create a new chat history record."""
    db.add(chat_history)
    await db.commit()
    await db.refresh(chat_history)
    return chat_history


async def get_chat_history(db: AsyncSession, chat_history_id: int) -> Optional[ChatHistory]:
    """Get a chat history record by ID."""
    result = await db.execute(select(ChatHistory).filter(ChatHistory.id == chat_history_id))
    return result.scalar_one_or_none()


async def get_chat_history_by_session(db: AsyncSession, session_id: str, skip: int = 0, limit: int = 100) -> List[ChatHistory]:
    """Get all chat history for a specific session with pagination."""
    result = await db.execute(
        select(ChatHistory)
        .filter(ChatHistory.session_id == session_id)
        .order_by(ChatHistory.timestamp.asc())
        .offset(skip)
        .limit(limit)
    )
    return result.scalars().all()


async def get_chat_history_by_user(db: AsyncSession, user_id: Optional[int], skip: int = 0, limit: int = 100) -> List[ChatHistory]:
    """Get all chat history for a specific user with pagination."""
    if user_id is None:
        # Get anonymous chat history
        result = await db.execute(
            select(ChatHistory)
            .filter(ChatHistory.user_id.is_(None))
            .order_by(ChatHistory.timestamp.desc())
            .offset(skip)
            .limit(limit)
        )
    else:
        # Get chat history for a specific user
        result = await db.execute(
            select(ChatHistory)
            .filter(ChatHistory.user_id == user_id)
            .order_by(ChatHistory.timestamp.desc())
            .offset(skip)
            .limit(limit)
        )
    return result.scalars().all()


async def get_all_chat_history(db: AsyncSession, skip: int = 0, limit: int = 100) -> List[ChatHistory]:
    """Get all chat history with pagination."""
    result = await db.execute(
        select(ChatHistory)
        .order_by(ChatHistory.timestamp.desc())
        .offset(skip)
        .limit(limit)
    )
    return result.scalars().all()


async def update_chat_history(db: AsyncSession, chat_history_id: int, **kwargs) -> Optional[ChatHistory]:
    """Update a chat history record."""
    result = await db.execute(select(ChatHistory).filter(ChatHistory.id == chat_history_id))
    chat_history = result.scalar_one_or_none()

    if not chat_history:
        return None

    for key, value in kwargs.items():
        setattr(chat_history, key, value)

    await db.commit()
    await db.refresh(chat_history)
    return chat_history


async def delete_chat_history(db: AsyncSession, chat_history_id: int) -> bool:
    """Delete a chat history record."""
    result = await db.execute(select(ChatHistory).filter(ChatHistory.id == chat_history_id))
    chat_history = result.scalar_one_or_none()

    if not chat_history:
        return False

    await db.delete(chat_history)
    await db.commit()
    return True


async def delete_chat_history_by_session(db: AsyncSession, session_id: str) -> int:
    """Delete all chat history for a specific session."""
    result = await db.execute(
        select(ChatHistory)
        .filter(ChatHistory.session_id == session_id)
    )
    histories = result.scalars().all()

    count = 0
    for history in histories:
        await db.delete(history)
        count += 1

    await db.commit()
    return count