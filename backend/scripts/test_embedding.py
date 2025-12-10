#!/usr/bin/env python3
"""Simple test to debug the EmbeddingDocument creation issue."""

import asyncio
import sys
import os

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sqlalchemy.ext.asyncio import create_async_engine
from src.database.base import Base
from src.config.settings import settings
from src.database.session import AsyncSessionLocal
from src.models.embedding_document import EmbeddingDocument
from src.database.crud.embedding_document import create_embedding_document

async def test_embedding_creation():
    """Test creating an embedding document."""
    print("Testing EmbeddingDocument creation...")

    # Initialize the database (create tables if they don't exist)
    engine = create_async_engine(settings.DATABASE_URL)
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    print("Database initialized.")

    # Try to create a simple embedding document
    async with AsyncSessionLocal() as session:
        try:
            print("Creating EmbeddingDocument...")
            embedding_doc = EmbeddingDocument(
                qdrant_id="test_id",
                content="Test content for debugging",
                content_type="test",
                content_id=1,
                doc_metadata={"test": "value"},
                embedding_model="test_model",
                tokens=10
                # Not setting chapter_id, lesson_id, or lesson_section_id
            )

            print("Attempting to add to session...")
            session.add(embedding_doc)
            print("Attempting to commit...")
            await session.commit()
            print("Successfully created embedding document!")

            # Refresh to get the ID
            await session.refresh(embedding_doc)
            print(f"Created document ID: {embedding_doc.id}")

        except Exception as e:
            print(f"Error creating embedding document: {e}")
            import traceback
            traceback.print_exc()
            await session.rollback()

if __name__ == "__main__":
    asyncio.run(test_embedding_creation())