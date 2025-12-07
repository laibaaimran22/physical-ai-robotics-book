from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import Optional, List

from src.models.note import Note


async def create_note(db: AsyncSession, note: Note) -> Note:
    """Create a new note."""
    db.add(note)
    await db.commit()
    await db.refresh(note)
    return note


async def get_note(db: AsyncSession, note_id: int) -> Optional[Note]:
    """Get a note by ID."""
    result = await db.execute(select(Note).filter(Note.id == note_id))
    return result.scalar_one_or_none()


async def get_notes_by_user(db: AsyncSession, user_id: Optional[int], skip: int = 0, limit: int = 100) -> List[Note]:
    """Get all notes for a specific user with pagination."""
    if user_id is None:
        # Get anonymous notes
        result = await db.execute(
            select(Note)
            .filter(Note.user_id.is_(None))
            .order_by(Note.created_at.desc())
            .offset(skip)
            .limit(limit)
        )
    else:
        # Get notes for a specific user
        result = await db.execute(
            select(Note)
            .filter(Note.user_id == user_id)
            .order_by(Note.created_at.desc())
            .offset(skip)
            .limit(limit)
        )
    return result.scalars().all()


async def get_notes_by_content(db: AsyncSession, content_id: int, content_type: str, skip: int = 0, limit: int = 100) -> List[Note]:
    """Get all notes for a specific content with pagination."""
    result = await db.execute(
        select(Note)
        .filter(Note.content_id == content_id)
        .filter(Note.content_type == content_type)
        .order_by(Note.created_at.desc())
        .offset(skip)
        .limit(limit)
    )
    return result.scalars().all()


async def update_note(db: AsyncSession, note_id: int, **kwargs) -> Optional[Note]:
    """Update a note."""
    result = await db.execute(select(Note).filter(Note.id == note_id))
    note = result.scalar_one_or_none()

    if not note:
        return None

    for key, value in kwargs.items():
        setattr(note, key, value)

    await db.commit()
    await db.refresh(note)
    return note


async def delete_note(db: AsyncSession, note_id: int) -> bool:
    """Delete a note."""
    result = await db.execute(select(Note).filter(Note.id == note_id))
    note = result.scalar_one_or_none()

    if not note:
        return False

    await db.delete(note)
    await db.commit()
    return True


async def delete_notes_by_user(db: AsyncSession, user_id: Optional[int]) -> int:
    """Delete all notes for a specific user."""
    if user_id is None:
        # Delete anonymous notes
        result = await db.execute(
            select(Note)
            .filter(Note.user_id.is_(None))
        )
    else:
        # Delete notes for a specific user
        result = await db.execute(
            select(Note)
            .filter(Note.user_id == user_id)
        )

    notes = result.scalars().all()
    count = 0
    for note in notes:
        await db.delete(note)
        count += 1

    await db.commit()
    return count