from fastapi import APIRouter, HTTPException, Depends, Query
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional, List

from src.config.database import get_db_session as get_db
from src.services.content_service import get_content_service
from src.services.rag_service import get_rag_service
from src.api.deps import get_optional_user
from src.models.user import User  # Will be created later


router = APIRouter()


@router.get("/books")
async def get_all_books(
    skip: int = Query(0, ge=0),
    limit: int = Query(100, ge=1, le=1000),
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Get all books with pagination.
    """
    service = get_content_service(db)
    books = await service.get_all_books(skip=skip, limit=limit)
    return {
        "books": books,
        "skip": skip,
        "limit": limit,
        "total": len(books)  # In a real implementation, this would be the total count
    }


@router.get("/chapters/{book_id}")
async def get_chapters_for_book(
    book_id: int,
    skip: int = Query(0, ge=0),
    limit: int = Query(100, ge=1, le=1000),
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Get all chapters for a specific book.
    """
    service = get_content_service(db)
    chapters = await service.get_chapters_for_book(book_id=book_id, skip=skip, limit=limit)

    # Check if book exists
    book = await service.get_book(book_id)
    if not book:
        raise HTTPException(status_code=404, detail="Book not found")

    return {
        "chapters": chapters,
        "book_id": book_id,
        "skip": skip,
        "limit": limit,
        "total": len(chapters)  # In a real implementation, this would be the total count
    }


@router.get("/lessons/{chapter_id}")
async def get_lessons_for_chapter(
    chapter_id: int,
    skip: int = Query(0, ge=0),
    limit: int = Query(100, ge=1, le=1000),
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Get all lessons for a specific chapter.
    """
    service = get_content_service(db)
    lessons = await service.get_lessons_for_chapter(chapter_id=chapter_id, skip=skip, limit=limit)

    # Check if chapter exists
    from src.database.crud.chapter import get_chapter
    chapter = await get_chapter(db, chapter_id)
    if not chapter:
        raise HTTPException(status_code=404, detail="Chapter not found")

    return {
        "lessons": lessons,
        "chapter_id": chapter_id,
        "skip": skip,
        "limit": limit,
        "total": len(lessons)  # In a real implementation, this would be the total count
    }


@router.get("/lesson/{lesson_id}")
async def get_lesson(
    lesson_id: int,
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Get a specific lesson by ID.
    """
    service = get_content_service(db)
    lesson = await service.get_lesson(lesson_id)

    if not lesson:
        raise HTTPException(status_code=404, detail="Lesson not found")

    return lesson


@router.get("/search")
async def search_content(
    q: str = Query(..., min_length=1, description="Search query"),
    book_id: Optional[int] = Query(None, description="Filter by book ID"),
    content_type: Optional[str] = Query(None, description="Filter by content type (lesson, lesson_section)"),
    skip: int = Query(0, ge=0),
    limit: int = Query(10, ge=1, le=100),
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Search for content across books, chapters, lessons, and lesson sections.
    """
    if len(q) < 3:
        raise HTTPException(status_code=400, detail="Query must be at least 3 characters long")

    content_service = get_content_service(db)
    results = await content_service.search_content(
        query=q,
        book_id=book_id,
        content_type=content_type,
        skip=skip,
        limit=limit
    )

    # Also perform semantic search using RAG service
    rag_service = get_rag_service(db)
    semantic_results = await rag_service.semantic_search(query=q, top_k=limit)

    return {
        "text_search_results": results,
        "semantic_search_results": semantic_results,
        "query": q,
        "book_id": book_id,
        "content_type": content_type,
        "skip": skip,
        "limit": limit
    }


@router.get("/content/{content_id}")
async def get_content_by_id(
    content_id: int,
    content_type: str = Query(..., regex="^(lesson|lesson_section|chapter)$", description="Type of content"),
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Get content by ID and type.
    """
    service = get_content_service(db)

    if content_type == "lesson":
        content = await service.get_lesson(content_id)
        if not content:
            raise HTTPException(status_code=404, detail="Lesson not found")
    elif content_type == "lesson_section":
        content = await service.get_lesson_section(content_id)
        if not content:
            raise HTTPException(status_code=404, detail="Lesson section not found")
    elif content_type == "chapter":
        content = await service.get_chapter(content_id)
        if not content:
            raise HTTPException(status_code=404, detail="Chapter not found")
    else:
        raise HTTPException(status_code=400, detail="Invalid content type")

    return content