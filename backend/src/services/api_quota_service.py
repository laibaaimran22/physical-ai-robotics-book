from typing import Optional
from sqlalchemy.ext.asyncio import AsyncSession
from datetime import datetime, timedelta
import time

from src.models.api_usage_quota import APIUsageQuota
from src.database.crud.api_usage_quota import (
    create_api_usage_quota, get_api_usage_quota, update_api_usage_quota, get_api_usage_quota_by_user_and_type
)


class APIQuotaService:
    """Service for managing API usage quotas and rate limiting."""

    def __init__(self, db: AsyncSession):
        self.db = db

    async def check_quota(self, user_id: Optional[int], quota_type: str) -> dict:
        """
        Check if a user has exceeded their quota for a specific API type.

        Args:
            user_id: ID of the user (None for anonymous users)
            quota_type: Type of quota (e.g., 'rag_query', 'search', etc.)

        Returns:
            Dictionary with quota status information
        """
        # For anonymous users, use a default ID
        effective_user_id = user_id or 0

        # Get the user's quota record
        quota_record = await get_api_usage_quota_by_user_and_type(
            self.db, effective_user_id, quota_type
        )

        if not quota_record:
            # Create a new quota record with default limits
            quota_record = await self._create_default_quota(effective_user_id, quota_type)

        # Check if the quota period has expired and reset if needed
        if self._is_quota_period_expired(quota_record):
            quota_record = await self._reset_quota(quota_record)

        # Check if quota is exceeded
        is_exceeded = quota_record.quota_used >= quota_record.quota_limit

        return {
            "user_id": user_id,
            "quota_type": quota_type,
            "quota_limit": quota_record.quota_limit,
            "quota_used": quota_record.quota_used,
            "quota_remaining": max(0, quota_record.quota_limit - quota_record.quota_used),
            "is_exceeded": is_exceeded,
            "reset_time": quota_record.quota_period_start + timedelta(hours=quota_record.reset_interval_hours),
            "quota_record_id": quota_record.id
        }

    async def consume_quota(self, user_id: Optional[int], quota_type: str) -> bool:
        """
        Consume one unit of quota for a user.

        Args:
            user_id: ID of the user (None for anonymous users)
            quota_type: Type of quota to consume

        Returns:
            True if quota was successfully consumed, False if exceeded
        """
        quota_check = await self.check_quota(user_id, quota_type)

        if quota_check["is_exceeded"]:
            return False

        # For anonymous users, use a default ID
        effective_user_id = user_id or 0

        # Get the existing quota record
        quota_record = await get_api_usage_quota_by_user_and_type(
            self.db, effective_user_id, quota_type
        )

        if not quota_record:
            # Create a new quota record if it doesn't exist
            quota_record = await self._create_default_quota(effective_user_id, quota_type)

        # Update the quota usage
        updated_quota = await update_api_usage_quota(
            self.db,
            quota_record.id,
            quota_used=quota_record.quota_used + 1
        )

        return True

    async def reset_quota_for_user(self, user_id: Optional[int], quota_type: str) -> bool:
        """
        Manually reset quota for a user.

        Args:
            user_id: ID of the user (None for anonymous users)
            quota_type: Type of quota to reset

        Returns:
            True if reset was successful, False otherwise
        """
        effective_user_id = user_id or 0

        quota_record = await get_api_usage_quota_by_user_and_type(
            self.db, effective_user_id, quota_type
        )

        if not quota_record:
            return False

        # Reset the quota usage to 0 and update the period start time
        await update_api_usage_quota(
            self.db,
            quota_record.id,
            quota_used=0,
            quota_period_start=datetime.utcnow()
        )

        return True

    async def _create_default_quota(self, user_id: int, quota_type: str) -> APIUsageQuota:
        """
        Create a default quota record based on the quota type.

        Args:
            user_id: ID of the user
            quota_type: Type of quota

        Returns:
            Created APIUsageQuota record
        """
        # Define default limits based on quota type
        default_limits = {
            "rag_query": 30,  # 30 queries per day
            "search": 50,     # 50 searches per day
            "ingest": 10,     # 10 ingestion jobs per day
        }

        quota_limit = default_limits.get(quota_type, 100)  # Default to 100 if not specified

        new_quota = APIUsageQuota(
            user_id=user_id,
            quota_type=quota_type,
            quota_limit=quota_limit,
            quota_used=0,
            quota_period_start=datetime.utcnow(),
            reset_interval_hours=24  # Reset every 24 hours
        )

        return await create_api_usage_quota(self.db, new_quota)

    def _is_quota_period_expired(self, quota_record: APIUsageQuota) -> bool:
        """
        Check if the quota period has expired.

        Args:
            quota_record: The quota record to check

        Returns:
            True if the period has expired, False otherwise
        """
        period_end = quota_record.quota_period_start + timedelta(hours=quota_record.reset_interval_hours)
        return datetime.utcnow() > period_end

    async def _reset_quota(self, quota_record: APIUsageQuota) -> APIUsageQuota:
        """
        Reset the quota for a record.

        Args:
            quota_record: The quota record to reset

        Returns:
            Updated quota record
        """
        return await update_api_usage_quota(
            self.db,
            quota_record.id,
            quota_used=0,
            quota_period_start=datetime.utcnow()
        )

    async def get_usage_stats(self, user_id: Optional[int]) -> dict:
        """
        Get usage statistics for a user.

        Args:
            user_id: ID of the user (None for anonymous users)

        Returns:
            Dictionary with usage statistics
        """
        effective_user_id = user_id or 0

        # Get all quota records for the user
        from src.database.crud.api_usage_quota import get_api_usage_quotas_by_user
        quota_records = await get_api_usage_quotas_by_user(self.db, effective_user_id)

        stats = {}
        for record in quota_records:
            stats[record.quota_type] = {
                "limit": record.quota_limit,
                "used": record.quota_used,
                "remaining": max(0, record.quota_limit - record.quota_used),
                "reset_time": record.quota_period_start + timedelta(hours=record.reset_interval_hours)
            }

        # Add default quotas if not already present
        for default_type in ["rag_query", "search", "ingest"]:
            if default_type not in stats:
                default_limit = {"rag_query": 30, "search": 50, "ingest": 10}.get(default_type, 100)
                stats[default_type] = {
                    "limit": default_limit,
                    "used": 0,
                    "remaining": default_limit,
                    "reset_time": datetime.utcnow() + timedelta(hours=24)
                }

        return {
            "user_id": user_id,
            "timestamp": datetime.utcnow(),
            "stats": stats
        }

    async def track_usage(self, user_id: Optional[int], quota_type: str, amount: int = 1) -> bool:
        """
        Track usage for a specific quota type.

        Args:
            user_id: ID of the user (None for anonymous users)
            quota_type: Type of quota to track
            amount: Amount to increment (default 1)

        Returns:
            True if successful, False if quota exceeded
        """
        effective_user_id = user_id or 0

        # Check if quota would be exceeded
        quota_check = await self.check_quota(effective_user_id, quota_type)
        if quota_check["is_exceeded"]:
            return False

        # Get or create the quota record
        quota_record = await get_api_usage_quota_by_user_and_type(
            self.db, effective_user_id, quota_type
        )

        if not quota_record:
            # Create a new quota record if it doesn't exist
            quota_record = await self._create_default_quota(effective_user_id, quota_type)

        # Check if the period has expired and reset if needed
        if self._is_quota_period_expired(quota_record):
            quota_record = await self._reset_quota(quota_record)

        # Update the quota usage
        new_used = min(quota_record.quota_used + amount, quota_record.quota_limit)
        await update_api_usage_quota(
            self.db,
            quota_record.id,
            quota_used=new_used
        )

        return True

    async def get_all_user_quotas(self, user_id: Optional[int], skip: int = 0, limit: int = 100) -> dict:
        """
        Get all quota information for a user.

        Args:
            user_id: ID of the user (None for anonymous users)
            skip: Number of records to skip
            limit: Maximum number of records to return

        Returns:
            Dictionary with all quota information
        """
        effective_user_id = user_id or 0
        quota_records = await get_api_usage_quotas_by_user(self.db, effective_user_id, skip, limit)

        quotas = []
        for record in quota_records:
            is_expired = self._is_quota_period_expired(record)
            reset_time = record.quota_period_start + timedelta(hours=record.reset_interval_hours)

            quotas.append({
                "id": record.id,
                "quota_type": record.quota_type,
                "quota_limit": record.quota_limit,
                "quota_used": record.quota_used,
                "quota_remaining": max(0, record.quota_limit - record.quota_used),
                "quota_period_start": record.quota_period_start,
                "reset_interval_hours": record.reset_interval_hours,
                "reset_time": reset_time,
                "is_expired": is_expired
            })

        return {
            "quotas": quotas,
            "user_id": user_id,
            "total": len(quotas),
            "skip": skip,
            "limit": limit
        }


# Convenience function to create an API quota service instance
def get_api_quota_service(db: AsyncSession) -> APIQuotaService:
    """Get an instance of the API quota service."""
    return APIQuotaService(db)