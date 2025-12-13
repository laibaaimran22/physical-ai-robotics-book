from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy.orm import selectinload
from typing import Optional, List

from src.models.chapter import Chapter


async def create_chapter(db: AsyncSession, chapter: Chapter) -> Chapter:
    """Create a new chapter."""
    db.add(chapter)
    await db.commit()
    await db.refresh(chapter)
    return chapter


async def get_chapter(db: AsyncSession, chapter_id: int) -> Optional[Chapter]:
    """Get a chapter by ID."""
    result = await db.execute(
        select(Chapter)
        .options(selectinload(Chapter.lessons))
        .filter(Chapter.id == chapter_id)
    )
    return result.scalar_one_or_none()


async def get_chapter_by_title(db: AsyncSession, title: str) -> Optional[Chapter]:
    """Get a chapter by title."""
    result = await db.execute(select(Chapter).filter(Chapter.title == title))
    return result.scalar_one_or_none()


async def get_chapters_by_book(db: AsyncSession, book_metadata_id: int, skip: int = 0, limit: int = 100) -> List[Chapter]:
    """Get all chapters for a specific book with pagination."""
    result = await db.execute(
        select(Chapter)
        .filter(Chapter.book_metadata_id == book_metadata_id)
        .order_by(Chapter.order_index)
        .offset(skip)
        .limit(limit)
    )
    return result.scalars().all()


async def get_all_chapters(db: AsyncSession, skip: int = 0, limit: int = 100) -> List[Chapter]:
    """Get all chapters with pagination."""
    result = await db.execute(select(Chapter).order_by(Chapter.order_index).offset(skip).limit(limit))
    return result.scalars().all()


async def update_chapter(db: AsyncSession, chapter_id: int, **kwargs) -> Optional[Chapter]:
    """Update a chapter."""
    result = await db.execute(select(Chapter).filter(Chapter.id == chapter_id))
    chapter = result.scalar_one_or_none()

    if not chapter:
        return None

    for key, value in kwargs.items():
        setattr(chapter, key, value)

    await db.commit()
    await db.refresh(chapter)
    return chapter


async def delete_chapter(db: AsyncSession, chapter_id: int) -> bool:
    """Delete a chapter."""
    result = await db.execute(select(Chapter).filter(Chapter.id == chapter_id))
    chapter = result.scalar_one_or_none()

    if not chapter:
        return False

    await db.delete(chapter)
    await db.commit()
    return True