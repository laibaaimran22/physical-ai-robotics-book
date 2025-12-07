from fastapi import APIRouter, UploadFile, File, HTTPException, Depends, BackgroundTasks
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional
import uuid
import os

from src.config.database import get_db_session as get_db
from src.services.ingestion_service import get_ingestion_service
from src.api.deps import get_current_user  # Will be implemented later when user model is created
from src.models.user import User  # Will be created later


router = APIRouter()


@router.post("/upload")
async def upload_file(
    file: UploadFile = File(...),
    db: AsyncSession = Depends(get_db),
    current_user: User = Depends(get_current_user)  # Optional authentication
):
    """
    Upload a content file for ingestion.
    Admin only endpoint for uploading files to be ingested.
    """
    # Validate file type
    allowed_extensions = {'.md', '.mdx', '.html', '.htm', '.txt', '.pdf'}
    file_ext = os.path.splitext(file.filename)[1].lower()

    if file_ext not in allowed_extensions:
        raise HTTPException(
            status_code=400,
            detail=f"File type {file_ext} not supported. Allowed types: {', '.join(allowed_extensions)}"
        )

    # Create uploads directory if it doesn't exist
    upload_dir = "uploads"
    os.makedirs(upload_dir, exist_ok=True)

    # Generate unique filename to avoid conflicts
    unique_filename = f"{uuid.uuid4()}_{file.filename}"
    file_path = os.path.join(upload_dir, unique_filename)

    # Save the uploaded file
    with open(file_path, "wb") as buffer:
        buffer.write(await file.read())

    return {
        "filename": file.filename,
        "file_path": file_path,
        "file_size": os.path.getsize(file_path),
        "file_type": file_ext
    }


@router.post("/run")
async def start_ingestion(
    background_tasks: BackgroundTasks,
    file_path: str,
    file_type: str,
    book_title: str,
    db: AsyncSession = Depends(get_db),
    current_user: User = Depends(get_current_user)  # Optional authentication
):
    """
    Start the ingestion process for an uploaded file.
    This runs the ingestion in the background.
    """
    from src.services.ingestion_service import IngestionService

    service = IngestionService(db)

    # Start the ingestion job
    job_id = await service.ingest_content_file(
        file_path=file_path,
        file_name=os.path.basename(file_path),
        book_title=book_title,
        admin_user_id=current_user.id if current_user else None
    )

    # In a real implementation, we would run the processing in the background
    # For now, we'll process it synchronously for demonstration
    success = await service.process_ingestion_job(job_id)

    if not success:
        raise HTTPException(
            status_code=500,
            detail="Failed to process the ingestion job"
        )

    return {
        "job_id": job_id,
        "status": "completed" if success else "failed",
        "message": f"Ingestion job {job_id} completed successfully" if success else "Ingestion job failed"
    }


@router.get("/status")
async def get_ingestion_status(
    job_id: str,
    db: AsyncSession = Depends(get_db),
    current_user: User = Depends(get_current_user)  # Optional authentication
):
    """
    Get the status of an ingestion job.
    """
    from src.services.ingestion_service import IngestionService

    service = IngestionService(db)
    status = await service.get_ingestion_job_status(job_id)

    if not status:
        raise HTTPException(
            status_code=404,
            detail=f"Ingestion job {job_id} not found"
        )

    return status


@router.delete("/remove/{job_id}")
async def remove_ingestion(
    job_id: str,
    db: AsyncSession = Depends(get_db),
    current_user: User = Depends(get_current_user)  # Optional authentication
):
    """
    Remove an ingestion job and its associated data.
    """
    from src.services.ingestion_service import IngestionService

    service = IngestionService(db)
    success = await service.cleanup_ingestion_job(job_id)

    if not success:
        raise HTTPException(
            status_code=404,
            detail=f"Ingestion job {job_id} not found or could not be removed"
        )

    return {
        "message": f"Ingestion job {job_id} and associated data removed successfully"
    }