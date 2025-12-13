from typing import List, Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select

from src.models.chapter import Chapter
from src.models.lesson import Lesson
from src.models.lesson_section import LessonSection
from src.models.book_metadata import BookMetadata
from src.database.crud.chapter import (
    create_chapter, get_chapter, get_chapters_by_book, get_all_chapters, update_chapter, delete_chapter
)
from src.database.crud.lesson import (
    create_lesson, get_lesson, get_lessons_by_chapter, get_all_lessons, update_lesson, delete_lesson
)
from src.database.crud.lesson_section import (
    create_lesson_section, get_lesson_section, get_lesson_sections_by_lesson, get_all_lesson_sections, update_lesson_section, delete_lesson_section
)
from src.database.crud.book_metadata import (
    create_book_metadata, get_book_metadata, get_book_metadata_by_title, get_all_book_metadata, update_book_metadata, delete_book_metadata
)


class ContentService:
    """Service for managing book content (chapters, lessons, sections)."""

    def __init__(self, db: AsyncSession):
        self.db = db

    # Book Metadata Operations
    async def create_book(self, title: str, description: Optional[str] = None, version: str = "1.0") -> BookMetadata:
        """Create a new book."""
        book_metadata = BookMetadata(
            title=title,
            description=description,
            version=version,
            total_chapters=0,
            total_lessons=0,
            total_words=0,
            language="en"
        )
        return await create_book_metadata(self.db, book_metadata)

    async def get_book(self, book_id: int) -> Optional[BookMetadata]:
        """Get a book by ID."""
        return await get_book_metadata(self.db, book_id)

    async def get_book_by_title(self, title: str) -> Optional[BookMetadata]:
        """Get a book by title."""
        return await get_book_metadata_by_title(self.db, title)

    async def get_all_books(self, skip: int = 0, limit: int = 100) -> List[BookMetadata]:
        """Get all books with pagination."""
        return await get_all_book_metadata(self.db, skip, limit)

    async def update_book(self, book_id: int, **kwargs) -> Optional[BookMetadata]:
        """Update a book."""
        return await update_book_metadata(self.db, book_id, **kwargs)

    async def delete_book(self, book_id: int) -> bool:
        """Delete a book."""
        return await delete_book_metadata(self.db, book_id)

    # Chapter Operations
    async def create_chapter(self, book_id: int, title: str, description: Optional[str] = None, order_index: int = 0) -> Chapter:
        """Create a new chapter in a book."""
        chapter = Chapter(
            title=title,
            description=description,
            order_index=order_index,
            book_metadata_id=book_id
        )
        return await create_chapter(self.db, chapter)

    async def get_chapter(self, chapter_id: int) -> Optional[Chapter]:
        """Get a chapter by ID."""
        return await get_chapter(self.db, chapter_id)

    async def get_chapters_for_book(self, book_id: int, skip: int = 0, limit: int = 100) -> List[Chapter]:
        """Get all chapters for a specific book."""
        return await get_chapters_by_book(self.db, book_id, skip, limit)

    async def update_chapter(self, chapter_id: int, **kwargs) -> Optional[Chapter]:
        """Update a chapter."""
        return await update_chapter(self.db, chapter_id, **kwargs)

    async def delete_chapter(self, chapter_id: int) -> bool:
        """Delete a chapter."""
        return await delete_chapter(self.db, chapter_id)

    # Lesson Operations
    async def create_lesson(self, chapter_id: int, title: str, content: Optional[str] = None,
                           description: Optional[str] = None, order_index: int = 0) -> Lesson:
        """Create a new lesson in a chapter."""
        lesson = Lesson(
            title=title,
            description=description,
            content=content,
            order_index=order_index,
            chapter_id=chapter_id
        )
        return await create_lesson(self.db, lesson)

    async def get_lesson(self, lesson_id: int) -> Optional[Lesson]:
        """Get a lesson by ID."""
        return await get_lesson(self.db, lesson_id)

    async def get_lessons_for_chapter(self, chapter_id: int, skip: int = 0, limit: int = 100) -> List[Lesson]:
        """Get all lessons for a specific chapter."""
        return await get_lessons_by_chapter(self.db, chapter_id, skip, limit)

    async def update_lesson(self, lesson_id: int, **kwargs) -> Optional[Lesson]:
        """Update a lesson."""
        return await update_lesson(self.db, lesson_id, **kwargs)

    async def delete_lesson(self, lesson_id: int) -> bool:
        """Delete a lesson."""
        return await delete_lesson(self.db, lesson_id)

    # Lesson Section Operations
    async def create_lesson_section(self, lesson_id: int, title: str, content: str, order_index: int = 0) -> LessonSection:
        """Create a new lesson section."""
        lesson_section = LessonSection(
            title=title,
            content=content,
            order_index=order_index,
            lesson_id=lesson_id
        )
        return await create_lesson_section(self.db, lesson_section)

    async def get_lesson_section(self, lesson_section_id: int) -> Optional[LessonSection]:
        """Get a lesson section by ID."""
        return await get_lesson_section(self.db, lesson_section_id)

    async def get_lesson_sections_for_lesson(self, lesson_id: int, skip: int = 0, limit: int = 100) -> List[LessonSection]:
        """Get all lesson sections for a specific lesson."""
        return await get_lesson_sections_by_lesson(self.db, lesson_id, skip, limit)

    async def update_lesson_section(self, lesson_section_id: int, **kwargs) -> Optional[LessonSection]:
        """Update a lesson section."""
        return await update_lesson_section(self.db, lesson_section_id, **kwargs)

    async def delete_lesson_section(self, lesson_section_id: int) -> bool:
        """Delete a lesson section."""
        return await delete_lesson_section(self.db, lesson_section_id)

    # Content Search Operations
    async def search_content(self, query: str, book_id: Optional[int] = None,
                            content_type: Optional[str] = None, skip: int = 0, limit: int = 100) -> List:
        """
        Search for content across books, chapters, lessons, and lesson sections.
        This would typically integrate with the RAG service for semantic search.
        """
        results = []

        # Search in lessons
        all_lessons = await get_all_lessons(self.db, skip=0, limit=1000)  # Get all lessons for searching
        for lesson in all_lessons:
            if lesson.content and query.lower() in lesson.content.lower():
                results.append({
                    "type": "lesson",
                    "id": lesson.id,
                    "title": lesson.title,
                    "content_preview": lesson.content[:200] + "..." if len(lesson.content) > 200 else lesson.content,
                    "chapter_id": lesson.chapter_id
                })

        # Search in lesson sections
        all_sections = await get_all_lesson_sections(self.db, skip=0, limit=1000)  # Get all sections for searching
        for section in all_sections:
            if query.lower() in section.content.lower():
                results.append({
                    "type": "lesson_section",
                    "id": section.id,
                    "title": section.title,
                    "content_preview": section.content[:200] + "..." if len(section.content) > 200 else section.content,
                    "lesson_id": section.lesson_id
                })

        # Limit and offset results
        start_idx = skip
        end_idx = min(skip + limit, len(results))
        return results[start_idx:end_idx]


# Convenience function to create a content service instance
def get_content_service(db: AsyncSession) -> ContentService:
    """Get an instance of the content service."""
    return ContentService(db)