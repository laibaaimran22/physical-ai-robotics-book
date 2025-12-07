from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy.orm import selectinload
from typing import Optional, List

from src.models.lesson import Lesson


async def create_lesson(db: AsyncSession, lesson: Lesson) -> Lesson:
    """Create a new lesson."""
    db.add(lesson)
    await db.commit()
    await db.refresh(lesson)
    return lesson


async def get_lesson(db: AsyncSession, lesson_id: int) -> Optional[Lesson]:
    """Get a lesson by ID."""
    result = await db.execute(
        select(Lesson)
        .options(selectinload(Lesson.lesson_sections))
        .filter(Lesson.id == lesson_id)
    )
    return result.scalar_one_or_none()


async def get_lesson_by_title(db: AsyncSession, title: str) -> Optional[Lesson]:
    """Get a lesson by title."""
    result = await db.execute(select(Lesson).filter(Lesson.title == title))
    return result.scalar_one_or_none()


async def get_lessons_by_chapter(db: AsyncSession, chapter_id: int, skip: int = 0, limit: int = 100) -> List[Lesson]:
    """Get all lessons for a specific chapter with pagination."""
    result = await db.execute(
        select(Lesson)
        .filter(Lesson.chapter_id == chapter_id)
        .order_by(Lesson.order_index)
        .offset(skip)
        .limit(limit)
    )
    return result.scalars().all()


async def get_all_lessons(db: AsyncSession, skip: int = 0, limit: int = 100) -> List[Lesson]:
    """Get all lessons with pagination."""
    result = await db.execute(select(Lesson).order_by(Lesson.order_index).offset(skip).limit(limit))
    return result.scalars().all()


async def update_lesson(db: AsyncSession, lesson_id: int, **kwargs) -> Optional[Lesson]:
    """Update a lesson."""
    result = await db.execute(select(Lesson).filter(Lesson.id == lesson_id))
    lesson = result.scalar_one_or_none()

    if not lesson:
        return None

    for key, value in kwargs.items():
        setattr(lesson, key, value)

    await db.commit()
    await db.refresh(lesson)
    return lesson


async def delete_lesson(db: AsyncSession, lesson_id: int) -> bool:
    """Delete a lesson."""
    result = await db.execute(select(Lesson).filter(Lesson.id == lesson_id))
    lesson = result.scalar_one_or_none()

    if not lesson:
        return False

    await db.delete(lesson)
    await db.commit()
    return True