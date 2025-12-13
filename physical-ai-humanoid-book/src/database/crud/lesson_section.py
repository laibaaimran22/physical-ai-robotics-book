from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import Optional, List

from src.models.lesson_section import LessonSection


async def create_lesson_section(db: AsyncSession, lesson_section: LessonSection) -> LessonSection:
    """Create a new lesson section."""
    db.add(lesson_section)
    await db.commit()
    await db.refresh(lesson_section)
    return lesson_section


async def get_lesson_section(db: AsyncSession, lesson_section_id: int) -> Optional[LessonSection]:
    """Get a lesson section by ID."""
    result = await db.execute(select(LessonSection).filter(LessonSection.id == lesson_section_id))
    return result.scalar_one_or_none()


async def get_lesson_section_by_title(db: AsyncSession, title: str) -> Optional[LessonSection]:
    """Get a lesson section by title."""
    result = await db.execute(select(LessonSection).filter(LessonSection.title == title))
    return result.scalar_one_or_none()


async def get_lesson_sections_by_lesson(db: AsyncSession, lesson_id: int, skip: int = 0, limit: int = 100) -> List[LessonSection]:
    """Get all lesson sections for a specific lesson with pagination."""
    result = await db.execute(
        select(LessonSection)
        .filter(LessonSection.lesson_id == lesson_id)
        .order_by(LessonSection.order_index)
        .offset(skip)
        .limit(limit)
    )
    return result.scalars().all()


async def get_all_lesson_sections(db: AsyncSession, skip: int = 0, limit: int = 100) -> List[LessonSection]:
    """Get all lesson sections with pagination."""
    result = await db.execute(select(LessonSection).order_by(LessonSection.order_index).offset(skip).limit(limit))
    return result.scalars().all()


async def update_lesson_section(db: AsyncSession, lesson_section_id: int, **kwargs) -> Optional[LessonSection]:
    """Update a lesson section."""
    result = await db.execute(select(LessonSection).filter(LessonSection.id == lesson_section_id))
    lesson_section = result.scalar_one_or_none()

    if not lesson_section:
        return None

    for key, value in kwargs.items():
        setattr(lesson_section, key, value)

    await db.commit()
    await db.refresh(lesson_section)
    return lesson_section


async def delete_lesson_section(db: AsyncSession, lesson_section_id: int) -> bool:
    """Delete a lesson section."""
    result = await db.execute(select(LessonSection).filter(LessonSection.id == lesson_section_id))
    lesson_section = result.scalar_one_or_none()

    if not lesson_section:
        return False

    await db.delete(lesson_section)
    await db.commit()
    return True