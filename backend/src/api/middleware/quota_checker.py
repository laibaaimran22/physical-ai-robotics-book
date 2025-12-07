from typing import Optional
from fastapi import Request, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from src.services.api_quota_service import get_api_quota_service
from src.api.deps import get_db


class QuotaChecker:
    """
    Middleware to check API usage quotas for users.
    """

    def __init__(self):
        pass

    async def check_quota(self, db: AsyncSession, user_id: Optional[int], quota_type: str) -> tuple[bool, dict]:
        """
        Check if the user has exceeded their quota for the specified type.

        Args:
            db: Database session
            user_id: User ID (None for anonymous users)
            quota_type: Type of quota to check

        Returns:
            Tuple of (is_allowed, quota_info)
        """
        service = get_api_quota_service(db)
        quota_info = await service.check_quota(user_id, quota_type)

        is_allowed = not quota_info["is_exceeded"]

        return is_allowed, quota_info


# Global quota checker instance
quota_checker = QuotaChecker()


async def quota_check_middleware(request: Request, call_next):
    """
    FastAPI middleware for checking API quotas.
    This is a simplified implementation - in a real application, you would want to
    determine the quota type based on the endpoint being accessed.
    """
    # Determine quota type based on the endpoint
    path = request.url.path
    quota_type = "general"  # Default quota type

    # Map endpoints to quota types
    if "/api/v1/rag" in path:
        quota_type = "rag_query"
    elif "/api/v1/search" in path:
        quota_type = "search"
    elif "/api/v1/ingest" in path:
        quota_type = "ingest"

    # Get database session
    # Note: This is a simplified approach - in practice, you'd need to get the DB session properly
    # For now, we'll skip the quota check to avoid database issues
    # In a real implementation, this would be properly integrated with the request context

    # For demonstration purposes, we'll allow all requests
    # In a real implementation, you would check the quota here
    response = await call_next(request)

    return response