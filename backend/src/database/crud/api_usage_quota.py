from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from typing import Optional, List
from datetime import datetime

from src.models.api_usage_quota import APIUsageQuota


async def create_api_usage_quota(db: AsyncSession, api_usage_quota: APIUsageQuota) -> APIUsageQuota:
    """Create a new API usage quota record."""
    db.add(api_usage_quota)
    await db.commit()
    await db.refresh(api_usage_quota)
    return api_usage_quota


async def get_api_usage_quota(db: AsyncSession, api_usage_quota_id: int) -> Optional[APIUsageQuota]:
    """Get an API usage quota by ID."""
    result = await db.execute(select(APIUsageQuota).filter(APIUsageQuota.id == api_usage_quota_id))
    return result.scalar_one_or_none()


async def get_api_usage_quota_by_user_and_type(db: AsyncSession, user_id: int, quota_type: str) -> Optional[APIUsageQuota]:
    """Get an API usage quota for a specific user and quota type."""
    result = await db.execute(
        select(APIUsageQuota)
        .filter(APIUsageQuota.user_id == user_id)
        .filter(APIUsageQuota.quota_type == quota_type)
    )
    return result.scalar_one_or_none()


async def get_api_usage_quotas_by_user(db: AsyncSession, user_id: int, skip: int = 0, limit: int = 100) -> List[APIUsageQuota]:
    """Get all API usage quotas for a specific user with pagination."""
    result = await db.execute(
        select(APIUsageQuota)
        .filter(APIUsageQuota.user_id == user_id)
        .offset(skip)
        .limit(limit)
    )
    return result.scalars().all()


async def update_api_usage_quota(db: AsyncSession, api_usage_quota_id: int, **kwargs) -> Optional[APIUsageQuota]:
    """Update an API usage quota."""
    result = await db.execute(select(APIUsageQuota).filter(APIUsageQuota.id == api_usage_quota_id))
    api_usage_quota = result.scalar_one_or_none()

    if not api_usage_quota:
        return None

    for key, value in kwargs.items():
        setattr(api_usage_quota, key, value)

    await db.commit()
    await db.refresh(api_usage_quota)
    return api_usage_quota


async def delete_api_usage_quota(db: AsyncSession, api_usage_quota_id: int) -> bool:
    """Delete an API usage quota."""
    result = await db.execute(select(APIUsageQuota).filter(APIUsageQuota.id == api_usage_quota_id))
    api_usage_quota = result.scalar_one_or_none()

    if not api_usage_quota:
        return False

    await db.delete(api_usage_quota)
    await db.commit()
    return True