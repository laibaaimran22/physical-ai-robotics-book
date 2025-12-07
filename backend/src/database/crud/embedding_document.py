from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import Optional, List

from src.models.embedding_document import EmbeddingDocument


async def create_embedding_document(db: AsyncSession, embedding_document: EmbeddingDocument) -> EmbeddingDocument:
    """Create a new embedding document."""
    db.add(embedding_document)
    await db.commit()
    await db.refresh(embedding_document)
    return embedding_document


async def get_embedding_document(db: AsyncSession, embedding_document_id: int) -> Optional[EmbeddingDocument]:
    """Get an embedding document by ID."""
    result = await db.execute(select(EmbeddingDocument).filter(EmbeddingDocument.id == embedding_document_id))
    return result.scalar_one_or_none()


async def get_embedding_document_by_qdrant_id(db: AsyncSession, qdrant_id: str) -> Optional[EmbeddingDocument]:
    """Get an embedding document by Qdrant ID."""
    result = await db.execute(select(EmbeddingDocument).filter(EmbeddingDocument.qdrant_id == qdrant_id))
    return result.scalar_one_or_none()


async def get_embedding_documents_by_content(db: AsyncSession, content_id: int, content_type: str, skip: int = 0, limit: int = 100) -> List[EmbeddingDocument]:
    """Get all embedding documents for a specific content with pagination."""
    result = await db.execute(
        select(EmbeddingDocument)
        .filter(EmbeddingDocument.content_id == content_id)
        .filter(EmbeddingDocument.content_type == content_type)
        .offset(skip)
        .limit(limit)
    )
    return result.scalars().all()


async def get_embedding_documents_by_type(db: AsyncSession, content_type: str, skip: int = 0, limit: int = 100) -> List[EmbeddingDocument]:
    """Get all embedding documents of a specific type with pagination."""
    result = await db.execute(
        select(EmbeddingDocument)
        .filter(EmbeddingDocument.content_type == content_type)
        .offset(skip)
        .limit(limit)
    )
    return result.scalars().all()


async def get_all_embedding_documents(db: AsyncSession, skip: int = 0, limit: int = 100) -> List[EmbeddingDocument]:
    """Get all embedding documents with pagination."""
    result = await db.execute(select(EmbeddingDocument).offset(skip).limit(limit))
    return result.scalars().all()


async def update_embedding_document(db: AsyncSession, embedding_document_id: int, **kwargs) -> Optional[EmbeddingDocument]:
    """Update an embedding document."""
    result = await db.execute(select(EmbeddingDocument).filter(EmbeddingDocument.id == embedding_document_id))
    embedding_document = result.scalar_one_or_none()

    if not embedding_document:
        return None

    for key, value in kwargs.items():
        setattr(embedding_document, key, value)

    await db.commit()
    await db.refresh(embedding_document)
    return embedding_document


async def delete_embedding_document(db: AsyncSession, embedding_document_id: int) -> bool:
    """Delete an embedding document."""
    result = await db.execute(select(EmbeddingDocument).filter(EmbeddingDocument.id == embedding_document_id))
    embedding_document = result.scalar_one_or_none()

    if not embedding_document:
        return False

    await db.delete(embedding_document)
    await db.commit()
    return True


async def delete_embedding_documents_by_content(db: AsyncSession, content_id: int, content_type: str) -> int:
    """Delete all embedding documents for a specific content."""
    result = await db.execute(
        select(EmbeddingDocument)
        .filter(EmbeddingDocument.content_id == content_id)
        .filter(EmbeddingDocument.content_type == content_type)
    )
    documents = result.scalars().all()

    count = 0
    for doc in documents:
        await db.delete(doc)
        count += 1

    await db.commit()
    return count