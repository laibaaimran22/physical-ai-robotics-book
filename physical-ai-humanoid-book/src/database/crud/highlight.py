from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import Optional, List

from src.models.highlight import Highlight


async def create_highlight(db: AsyncSession, highlight: Highlight) -> Highlight:
    """Create a new highlight."""
    db.add(highlight)
    await db.commit()
    await db.refresh(highlight)
    return highlight


async def get_highlight(db: AsyncSession, highlight_id: int) -> Optional[Highlight]:
    """Get a highlight by ID."""
    result = await db.execute(select(Highlight).filter(Highlight.id == highlight_id))
    return result.scalar_one_or_none()


async def get_highlights_by_user(db: AsyncSession, user_id: Optional[int], skip: int = 0, limit: int = 100) -> List[Highlight]:
    """Get all highlights for a specific user with pagination."""
    if user_id is None:
        # Get anonymous highlights
        result = await db.execute(
            select(Highlight)
            .filter(Highlight.user_id.is_(None))
            .order_by(Highlight.created_at.desc())
            .offset(skip)
            .limit(limit)
        )
    else:
        # Get highlights for a specific user
        result = await db.execute(
            select(Highlight)
            .filter(Highlight.user_id == user_id)
            .order_by(Highlight.created_at.desc())
            .offset(skip)
            .limit(limit)
        )
    return result.scalars().all()


async def get_highlights_by_content(db: AsyncSession, content_id: int, content_type: str, skip: int = 0, limit: int = 100) -> List[Highlight]:
    """Get all highlights for a specific content with pagination."""
    result = await db.execute(
        select(Highlight)
        .filter(Highlight.content_id == content_id)
        .filter(Highlight.content_type == content_type)
        .order_by(Highlight.created_at.desc())
        .offset(skip)
        .limit(limit)
    )
    return result.scalars().all()


async def update_highlight(db: AsyncSession, highlight_id: int, **kwargs) -> Optional[Highlight]:
    """Update a highlight."""
    result = await db.execute(select(Highlight).filter(Highlight.id == highlight_id))
    highlight = result.scalar_one_or_none()

    if not highlight:
        return None

    for key, value in kwargs.items():
        setattr(highlight, key, value)

    await db.commit()
    await db.refresh(highlight)
    return highlight


async def delete_highlight(db: AsyncSession, highlight_id: int) -> bool:
    """Delete a highlight."""
    result = await db.execute(select(Highlight).filter(Highlight.id == highlight_id))
    highlight = result.scalar_one_or_none()

    if not highlight:
        return False

    await db.delete(highlight)
    await db.commit()
    return True


async def delete_highlights_by_user(db: AsyncSession, user_id: Optional[int]) -> int:
    """Delete all highlights for a specific user."""
    if user_id is None:
        # Delete anonymous highlights
        result = await db.execute(
            select(Highlight)
            .filter(Highlight.user_id.is_(None))
        )
    else:
        # Delete highlights for a specific user
        result = await db.execute(
            select(Highlight)
            .filter(Highlight.user_id == user_id)
        )

    highlights = result.scalars().all()
    count = 0
    for highlight in highlights:
        await db.delete(highlight)
        count += 1

    await db.commit()
    return count