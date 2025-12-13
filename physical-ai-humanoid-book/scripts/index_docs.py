#!/usr/bin/env python3
"""
Script to index all documentation files into the vector store.
This script will read all markdown files from the physical-ai-robotics-book/docs directory
and index them into the Qdrant vector store for RAG functionality.
"""

import asyncio
import os
import sys
from pathlib import Path
from typing import List

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker

from src.config.settings import settings
from src.core.vector_store import get_vector_store
from src.core.embedding_client import embedding_client
from src.models.embedding_document import EmbeddingDocument
from src.database.session import AsyncSessionLocal
from src.utils.file_processing import extract_text_from_markdown
from src.utils.text_chunking import chunk_text
from src.database.crud.embedding_document import create_embedding_document


async def index_documentation_files():
    """Index all documentation files into the vector store."""
    print("Starting documentation indexing process...")

    # Initialize vector store
    vector_store = get_vector_store()

    # Define the path to the documentation files using absolute path
    # Get the project root directory (where both backend and physical-ai-robotics-book folders are)
    project_root = Path(__file__).parent.parent.parent  # This goes from backend/scripts to project root
    docs_path = project_root / "physical-ai-robotics-book" / "docs"

    if not docs_path.exists():
        print(f"Documentation directory not found: {docs_path.absolute()}")
        return

    # Find all markdown files
    md_files = list(docs_path.rglob("*.md"))
    print(f"Found {len(md_files)} markdown files to index")

    if not md_files:
        print("No markdown files found to index.")
        return

    # Use the existing AsyncSessionLocal from the database session
    async_session = AsyncSessionLocal

    indexed_count = 0

    for file_path in md_files:
        print(f"\nProcessing file: {file_path}")

        try:
            # Read and extract content from the markdown file with error handling
            try:
                content = extract_text_from_markdown(str(file_path))
            except UnicodeDecodeError:
                # If UTF-8 fails, try with different encoding or skip
                print(f"  UTF-8 decoding failed for {file_path}, trying with error handling...")
                try:
                    with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
                        raw_content = file.read()
                        # Convert markdown to HTML then extract text
                        import markdown
                        from bs4 import BeautifulSoup
                        html = markdown.markdown(raw_content)
                        soup = BeautifulSoup(html, 'html.parser')
                        content = soup.get_text()
                except Exception as e:
                    print(f"    Error reading file with error handling: {str(e)}")
                    continue

            if not content.strip():
                print(f"  Skipping empty file: {file_path}")
                continue

            # Chunk the content
            chunks = chunk_text(content, max_tokens=settings.CHUNK_SIZE_TOKENS,
                              min_tokens=settings.MIN_CHUNK_SIZE_TOKENS, strategy="sentence")

            print(f"  Generated {len(chunks)} chunks from {file_path}")

            # Process each chunk
            for i, chunk in enumerate(chunks):
                if not chunk.strip():
                    continue

                try:
                    # Generate embedding for the chunk
                    embedding = embedding_client.generate_embedding(chunk)

                    # First, add to vector store to get qdrant_id
                    qdrant_id = await vector_store.add_embedding(
                        content=chunk,
                        content_id=i,  # Using chunk index as content ID temporarily
                        content_type="markdown",
                        metadata={
                            "source_file": str(file_path.relative_to(docs_path)),
                            "chunk_index": i,
                            "original_file_type": "markdown",
                            "relative_path": str(file_path.relative_to(docs_path))
                        },
                        vector=embedding
                    )

                    # Create embedding document record with the qdrant_id
                    embedding_doc = EmbeddingDocument(
                        qdrant_id=qdrant_id,  # Now we have the qdrant_id from vector store
                        content=chunk,
                        content_type="markdown",
                        content_id=i,  # Using chunk index as content ID temporarily
                        doc_metadata={
                            "source_file": str(file_path.relative_to(docs_path)),
                            "chunk_index": i,
                            "original_file_type": "markdown",
                            "relative_path": str(file_path.relative_to(docs_path))
                        },
                        embedding_model=embedding_client.model_name,
                        tokens=len(chunk.split())  # Approximate token count
                        # For documentation files, we don't set chapter_id, lesson_id, or lesson_section_id
                        # They will default to None due to being optional in the model
                    )

                    # Save to database
                    async with async_session() as session:
                        created_doc = await create_embedding_document(session, embedding_doc)

                        print(f"    Indexed chunk {i+1}/{len(chunks)} (DB ID: {created_doc.id})")
                        indexed_count += 1

                except Exception as e:
                    print(f"    Error processing chunk {i}: {str(e)}")
                    continue

        except Exception as e:
            print(f"  Error processing file {file_path}: {str(e)}")
            continue

    print(f"\nIndexing completed! Total chunks indexed: {indexed_count}")

    # Print final count
    total_points = vector_store.get_total_points()
    print(f"Total documents now in vector store: {total_points}")


def main():
    """Main entry point."""
    print("Documentation Indexing Script")
    print("=" * 40)

    # Verify Qdrant connection first
    vector_store = get_vector_store()
    if vector_store.client is None:
        print("ERROR: Qdrant client is not configured properly!")
        print("Please check your QDRANT_URL and QDRANT_API_KEY in the .env file.")
        return

    print("Qdrant connection verified.")

    # Run the indexing process
    asyncio.run(index_documentation_files())


if __name__ == "__main__":
    main()