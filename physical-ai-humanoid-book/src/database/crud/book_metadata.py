from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import Optional, List

from src.models.book_metadata import BookMetadata


async def create_book_metadata(db: AsyncSession, book_metadata: BookMetadata) -> BookMetadata:
    """Create a new book metadata record."""
    db.add(book_metadata)
    await db.commit()
    await db.refresh(book_metadata)
    return book_metadata


async def get_book_metadata(db: AsyncSession, book_metadata_id: int) -> Optional[BookMetadata]:
    """Get a book metadata record by ID."""
    result = await db.execute(select(BookMetadata).filter(BookMetadata.id == book_metadata_id))
    return result.scalar_one_or_none()


async def get_book_metadata_by_title(db: AsyncSession, title: str) -> Optional[BookMetadata]:
    """Get a book metadata record by title."""
    result = await db.execute(select(BookMetadata).filter(BookMetadata.title == title))
    return result.scalar_one_or_none()


async def get_all_book_metadata(db: AsyncSession, skip: int = 0, limit: int = 100) -> List[BookMetadata]:
    """Get all book metadata records with pagination."""
    result = await db.execute(select(BookMetadata).offset(skip).limit(limit))
    return result.scalars().all()


async def update_book_metadata(db: AsyncSession, book_metadata_id: int, **kwargs) -> Optional[BookMetadata]:
    """Update a book metadata record."""
    result = await db.execute(select(BookMetadata).filter(BookMetadata.id == book_metadata_id))
    book_metadata = result.scalar_one_or_none()

    if not book_metadata:
        return None

    for key, value in kwargs.items():
        setattr(book_metadata, key, value)

    await db.commit()
    await db.refresh(book_metadata)
    return book_metadata


async def delete_book_metadata(db: AsyncSession, book_metadata_id: int) -> bool:
    """Delete a book metadata record."""
    result = await db.execute(select(BookMetadata).filter(BookMetadata.id == book_metadata_id))
    book_metadata = result.scalar_one_or_none()

    if not book_metadata:
        return False

    await db.delete(book_metadata)
    await db.commit()
    return True