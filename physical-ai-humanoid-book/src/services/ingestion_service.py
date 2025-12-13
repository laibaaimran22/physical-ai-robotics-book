from typing import Optional, List, Dict, Any
from sqlalchemy.ext.asyncio import AsyncSession
import uuid
import logging
from datetime import datetime

from ..models.ingestion_job import IngestionJob
from ..models.embedding_document import EmbeddingDocument
from ..database.crud.ingestion_job import create_ingestion_job, get_ingestion_job_by_job_id, update_ingestion_job
from ..database.crud.embedding_document import create_embedding_document
from ..core.embedding_client import embedding_client
from ..core.vector_store import get_vector_store

# Get the vector store instance
vector_store = get_vector_store()
from ..utils.file_processing import extract_text_from_file, get_file_type
from ..utils.text_chunking import chunk_text
from ..config.settings import settings

logger = logging.getLogger(__name__)


class IngestionService:
    """Service for handling document ingestion and processing."""

    def __init__(self, db: AsyncSession):
        self.db = db

    async def ingest_file(self, file_path: str, file_name: str, user_id: int) -> str:
        """
        Start ingestion of a file.

        Args:
            file_path: Path to the file to be ingested
            file_name: Name of the file
            user_id: ID of the user initiating the ingestion

        Returns:
            Job ID of the created ingestion job
        """
        job_id = str(uuid.uuid4())

        ingestion_job = IngestionJob(
            job_id=job_id,
            user_id=user_id,
            file_path=file_path,
            file_name=file_name,
            file_type=get_file_type(file_path),
            status="pending"
        )

        await create_ingestion_job(self.db, ingestion_job)
        logger.info(f"Started ingestion job {job_id} for file {file_name}")

        # Process the ingestion asynchronously
        await self._process_ingestion_job(ingestion_job.id)

        return job_id

    async def _process_ingestion_job(self, job_id: int):
        """Process an ingestion job in the background."""
        ingestion_job = await get_ingestion_job_by_job_id(self.db, str(job_id))
        if not ingestion_job:
            logger.error(f"Ingestion job {job_id} not found")
            return False

        # Update status to processing
        await update_ingestion_job(self.db, ingestion_job.id, status="processing", started_at=datetime.utcnow())

        try:
            # Extract text from the file
            content = extract_text_from_file(ingestion_job.file_path)

            # Chunk the content
            chunks = chunk_text(content, max_tokens=settings.CHUNK_SIZE_TOKENS, min_tokens=settings.MIN_CHUNK_SIZE_TOKENS, strategy="sentence")

            # Update total chunks count
            await update_ingestion_job(self.db, ingestion_job.id, total_chunks=len(chunks))

            # Process each chunk
            processed_count = 0
            for i, chunk in enumerate(chunks):
                # Generate embedding for the chunk
                embedding = embedding_client.generate_embedding(chunk)

                # Create embedding document
                embedding_doc = EmbeddingDocument(
                    qdrant_id=str(uuid.uuid4()),
                    content=chunk,
                    content_type=ingestion_job.file_type,
                    content_id=ingestion_job.id,  # Using job ID as content ID for this ingestion
                    doc_metadata={  # Using the corrected field name
                        "source_file": ingestion_job.file_name,
                        "chunk_index": i,
                        "original_file_type": ingestion_job.file_type
                    },
                    embedding_model=embedding_client.model_name,
                    tokens=len(chunk.split())  # Approximate token count
                )

                # Save to database
                created_doc = await create_embedding_document(self.db, embedding_doc)

                # Save to vector store
                await vector_store.add_embedding(
                    content=chunk,
                    content_id=created_doc.id,
                    content_type=ingestion_job.file_type,
                    metadata=embedding_doc.doc_metadata,  # Using the corrected field name
                    vector=embedding
                )

                processed_count += 1

                # Update progress periodically
                if processed_count % 10 == 0:  # Update every 10 chunks
                    await update_ingestion_job(self.db, ingestion_job.id, processed_chunks=processed_count)

            # Update job status to completed
            await update_ingestion_job(
                self.db,
                ingestion_job.id,
                status="completed",
                processed_chunks=processed_count,
                completed_at=datetime.utcnow()
            )

            logger.info(f"Ingestion job {ingestion_job.job_id} completed successfully with {processed_count} chunks processed")
            return True

        except Exception as e:
            logger.error(f"Error processing ingestion job {ingestion_job.job_id}: {e}")
            await update_ingestion_job(
                self.db,
                ingestion_job.id,
                status="failed",
                error_message=str(e),
                completed_at=datetime.utcnow()
            )
            return False

    async def get_ingestion_job_status(self, job_id: str) -> Optional[Dict[str, Any]]:
        """
        Get the status of an ingestion job.

        Args:
            job_id: ID of the ingestion job

        Returns:
            Dictionary with job status information
        """
        ingestion_job = await get_ingestion_job_by_job_id(self.db, job_id)
        if not ingestion_job:
            return None

        return {
            "job_id": ingestion_job.job_id,
            "status": ingestion_job.status,
            "file_name": ingestion_job.file_name,
            "file_type": ingestion_job.file_type,
            "total_chunks": ingestion_job.total_chunks or 0,
            "processed_chunks": ingestion_job.processed_chunks or 0,
            "error_message": ingestion_job.error_message,
            "started_at": ingestion_job.started_at.isoformat() if ingestion_job.started_at else None,
            "completed_at": ingestion_job.completed_at.isoformat() if ingestion_job.completed_at else None
        }


def get_ingestion_service(db: AsyncSession) -> IngestionService:
    """Get an instance of the ingestion service."""
    return IngestionService(db)