from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import Optional, List

from src.models.rag_query import RAGQuery


async def create_rag_query(db: AsyncSession, rag_query: RAGQuery) -> RAGQuery:
    """Create a new RAG query record."""
    db.add(rag_query)
    await db.commit()
    await db.refresh(rag_query)
    return rag_query


async def get_rag_query(db: AsyncSession, rag_query_id: int) -> Optional[RAGQuery]:
    """Get a RAG query by ID."""
    result = await db.execute(select(RAGQuery).filter(RAGQuery.id == rag_query_id))
    return result.scalar_one_or_none()


async def get_rag_queries_by_user(db: AsyncSession, user_id: Optional[int], skip: int = 0, limit: int = 100) -> List[RAGQuery]:
    """Get all RAG queries for a specific user with pagination."""
    if user_id is None:
        # Get anonymous queries
        result = await db.execute(
            select(RAGQuery)
            .filter(RAGQuery.user_id.is_(None))
            .order_by(RAGQuery.created_at.desc())
            .offset(skip)
            .limit(limit)
        )
    else:
        # Get queries for a specific user
        result = await db.execute(
            select(RAGQuery)
            .filter(RAGQuery.user_id == user_id)
            .order_by(RAGQuery.created_at.desc())
            .offset(skip)
            .limit(limit)
        )
    return result.scalars().all()


async def get_all_rag_queries(db: AsyncSession, skip: int = 0, limit: int = 100) -> List[RAGQuery]:
    """Get all RAG queries with pagination."""
    result = await db.execute(
        select(RAGQuery)
        .order_by(RAGQuery.created_at.desc())
        .offset(skip)
        .limit(limit)
    )
    return result.scalars().all()


async def update_rag_query(db: AsyncSession, rag_query_id: int, **kwargs) -> Optional[RAGQuery]:
    """Update a RAG query."""
    result = await db.execute(select(RAGQuery).filter(RAGQuery.id == rag_query_id))
    rag_query = result.scalar_one_or_none()

    if not rag_query:
        return None

    for key, value in kwargs.items():
        setattr(rag_query, key, value)

    await db.commit()
    await db.refresh(rag_query)
    return rag_query


async def delete_rag_query(db: AsyncSession, rag_query_id: int) -> bool:
    """Delete a RAG query."""
    result = await db.execute(select(RAGQuery).filter(RAGQuery.id == rag_query_id))
    rag_query = result.scalar_one_or_none()

    if not rag_query:
        return False

    await db.delete(rag_query)
    await db.commit()
    return True