from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import Optional, List

from src.models.ingestion_job import IngestionJob


async def create_ingestion_job(db: AsyncSession, ingestion_job: IngestionJob) -> IngestionJob:
    """Create a new ingestion job."""
    db.add(ingestion_job)
    await db.commit()
    await db.refresh(ingestion_job)
    return ingestion_job


async def get_ingestion_job(db: AsyncSession, ingestion_job_id: int) -> Optional[IngestionJob]:
    """Get an ingestion job by ID."""
    result = await db.execute(select(IngestionJob).filter(IngestionJob.id == ingestion_job_id))
    return result.scalar_one_or_none()


async def get_ingestion_job_by_job_id(db: AsyncSession, job_id: str) -> Optional[IngestionJob]:
    """Get an ingestion job by job ID string."""
    result = await db.execute(select(IngestionJob).filter(IngestionJob.job_id == job_id))
    return result.scalar_one_or_none()


async def get_ingestion_jobs_by_status(db: AsyncSession, status: str, skip: int = 0, limit: int = 100) -> List[IngestionJob]:
    """Get all ingestion jobs with a specific status with pagination."""
    result = await db.execute(
        select(IngestionJob)
        .filter(IngestionJob.status == status)
        .offset(skip)
        .limit(limit)
    )
    return result.scalars().all()


async def get_ingestion_jobs_by_user(db: AsyncSession, admin_user_id: Optional[int], skip: int = 0, limit: int = 100) -> List[IngestionJob]:
    """Get all ingestion jobs for a specific user with pagination."""
    if admin_user_id is None:
        # Get jobs without a specific user (public ingestion)
        result = await db.execute(
            select(IngestionJob)
            .filter(IngestionJob.admin_user_id.is_(None))
            .offset(skip)
            .limit(limit)
        )
    else:
        # Get jobs for a specific user
        result = await db.execute(
            select(IngestionJob)
            .filter(IngestionJob.admin_user_id == admin_user_id)
            .offset(skip)
            .limit(limit)
        )
    return result.scalars().all()


async def get_all_ingestion_jobs(db: AsyncSession, skip: int = 0, limit: int = 100) -> List[IngestionJob]:
    """Get all ingestion jobs with pagination."""
    result = await db.execute(select(IngestionJob).offset(skip).limit(limit))
    return result.scalars().all()


async def update_ingestion_job(db: AsyncSession, ingestion_job_id: int, **kwargs) -> Optional[IngestionJob]:
    """Update an ingestion job."""
    result = await db.execute(select(IngestionJob).filter(IngestionJob.id == ingestion_job_id))
    ingestion_job = result.scalar_one_or_none()

    if not ingestion_job:
        return None

    for key, value in kwargs.items():
        setattr(ingestion_job, key, value)

    await db.commit()
    await db.refresh(ingestion_job)
    return ingestion_job


async def delete_ingestion_job(db: AsyncSession, ingestion_job_id: int) -> bool:
    """Delete an ingestion job."""
    result = await db.execute(select(IngestionJob).filter(IngestionJob.id == ingestion_job_id))
    ingestion_job = result.scalar_one_or_none()

    if not ingestion_job:
        return False

    await db.delete(ingestion_job)
    await db.commit()
    return True