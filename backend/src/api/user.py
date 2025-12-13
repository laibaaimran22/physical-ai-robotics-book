from fastapi import APIRouter, HTTPException, Depends, Query
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional, List

from src.config.database import get_db_session as get_db
from src.services.api_quota_service import get_api_quota_service
from src.database.crud.note import (
    create_note, get_notes_by_user, get_note, update_note, delete_note
)
from src.database.crud.highlight import (
    create_highlight, get_highlights_by_user, get_highlight, update_highlight, delete_highlight
)
from src.models.note import Note
from src.models.highlight import Highlight
from src.api.deps import get_optional_user
from src.models.user import User


router = APIRouter()


@router.get("/notes")
async def get_user_notes(
    skip: int = Query(0, ge=0),
    limit: int = Query(100, ge=1, le=1000),
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Get notes for the current user.
    """
    user_id = current_user.id if current_user else None
    notes = await get_notes_by_user(db, user_id, skip=skip, limit=limit)
    return {
        "notes": notes,
        "skip": skip,
        "limit": limit,
        "total": len(notes)  # In a real implementation, this would be the total count
    }


@router.post("/notes")
async def create_user_note(
    title: str,
    content: str,
    content_id: int,
    content_type: str,
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Create a new note for the current user.
    """
    user_id = current_user.id if current_user else None

    note = Note(
        user_id=user_id,
        content_id=content_id,
        content_type=content_type,
        title=title,
        content=content
    )

    created_note = await create_note(db, note)
    return created_note


@router.get("/highlights")
async def get_user_highlights(
    skip: int = Query(0, ge=0),
    limit: int = Query(100, ge=1, le=1000),
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Get highlights for the current user.
    """
    user_id = current_user.id if current_user else None
    highlights = await get_highlights_by_user(db, user_id, skip=skip, limit=limit)
    return {
        "highlights": highlights,
        "skip": skip,
        "limit": limit,
        "total": len(highlights)  # In a real implementation, this would be the total count
    }


@router.post("/highlights")
async def create_user_highlight(
    content_id: int,
    content_type: str,
    text: str,
    start_pos: int,
    end_pos: int,
    note: Optional[str] = None,
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Create a new highlight for the current user.
    """
    user_id = current_user.id if current_user else None

    highlight = Highlight(
        user_id=user_id,
        content_id=content_id,
        content_type=content_type,
        text=text,
        start_pos=start_pos,
        end_pos=end_pos,
        note=note
    )

    created_highlight = await create_highlight(db, highlight)
    return created_highlight


@router.get("/usage")
async def get_user_usage(
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Get API usage information for the current user.
    """
    if not current_user:
        # For anonymous users, return default quota information
        return {
            "user_id": None,
            "quotas": {
                "rag_query": {
                    "limit": 30,
                    "used": 0,
                    "remaining": 30,
                    "reset_time": "24 hours"
                },
                "search": {
                    "limit": 50,
                    "used": 0,
                    "remaining": 50,
                    "reset_time": "24 hours"
                }
            }
        }

    # For authenticated users, get their specific quota information
    service = get_api_quota_service(db)
    usage_stats = await service.get_usage_stats(current_user.id)

    return usage_stats


@router.get("/usage-details")
async def get_detailed_user_usage(
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Get detailed API usage information for the current user.
    """
    if not current_user:
        raise HTTPException(
            status_code=401,
            detail="Authentication required for detailed usage information"
        )

    service = get_api_quota_service(db)
    detailed_usage = await service.get_all_user_quotas(current_user.id)

    return detailed_usage


@router.post("/usage/reset")
async def reset_user_quota(
    quota_type: str,
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Reset a specific quota type for the current user.
    Admin only functionality.
    """
    if not current_user or not current_user.is_admin:
        raise HTTPException(
            status_code=403,
            detail="Admin access required to reset quotas"
        )

    service = get_api_quota_service(db)
    reset_success = await service.reset_quota_for_user(current_user.id, quota_type)

    if not reset_success:
        raise HTTPException(
            status_code=404,
            detail=f"Quota type '{quota_type}' not found for user"
        )

    return {
        "message": f"Successfully reset quota for {quota_type}",
        "user_id": current_user.id,
        "quota_type": quota_type
    }


@router.get("/profile")
async def get_user_profile(
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Get user profile information.
    """
    if not current_user:
        raise HTTPException(
            status_code=401,
            detail="Authentication required"
        )

    return {
        "id": current_user.id,
        "email": current_user.email,
        "full_name": current_user.full_name,
        "software_background_level": current_user.software_background_level,
        "hardware_background_level": current_user.hardware_background_level,
        "preferred_languages": current_user.preferred_languages,
        "learning_goals": current_user.learning_goals,
        "is_active": current_user.is_active,
        "is_admin": current_user.is_admin,
        "created_at": current_user.created_at,
        "updated_at": current_user.updated_at,
        "last_login": current_user.last_login
    }


@router.put("/profile")
async def update_user_profile(
    full_name: str = None,
    software_background_level: str = None,
    hardware_background_level: str = None,
    preferred_languages: str = None,
    learning_goals: str = None,
    db: AsyncSession = Depends(get_db),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Update user profile information.
    """
    if not current_user:
        raise HTTPException(
            status_code=401,
            detail="Authentication required"
        )

    from src.database.crud.user import update_user
    updated_user = await update_user(
        db,
        current_user.id,
        full_name=full_name,
        software_background_level=software_background_level,
        hardware_background_level=hardware_background_level,
        preferred_languages=preferred_languages,
        learning_goals=learning_goals
    )

    if not updated_user:
        raise HTTPException(
            status_code=404,
            detail="User not found"
        )

    return {
        "id": updated_user.id,
        "email": updated_user.email,
        "full_name": updated_user.full_name,
        "software_background_level": updated_user.software_background_level,
        "hardware_background_level": updated_user.hardware_background_level,
        "preferred_languages": updated_user.preferred_languages,
        "learning_goals": updated_user.learning_goals,
        "is_active": updated_user.is_active,
        "is_admin": updated_user.is_admin,
        "created_at": updated_user.created_at,
        "updated_at": updated_user.updated_at,
        "last_login": updated_user.last_login
    }