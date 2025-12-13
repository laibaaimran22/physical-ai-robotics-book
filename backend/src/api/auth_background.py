from fastapi import APIRouter, HTTPException, Depends, status
from fastapi.security import HTTPBearer
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional, Dict, Any
from pydantic import BaseModel
import json

from src.config.database import get_db_session as get_db
from src.models.user import User
from src.api.deps import get_current_user as get_current_user_dep
from src.database.crud.user import update_user

router = APIRouter()
security = HTTPBearer()

# Pydantic models for background information
class BackgroundInfo(BaseModel):
    software_level: Optional[str] = None
    hardware_level: Optional[str] = None
    preferred_languages: Optional[str] = None  # JSON string for array
    goals: Optional[str] = None

class UpdateBackgroundRequest(BaseModel):
    software_level: Optional[str] = None
    hardware_level: Optional[str] = None
    preferred_languages: Optional[str] = None
    goals: Optional[str] = None

# Pydantic models for personalization preferences
class PersonalizationPreferences(BaseModel):
    difficulty_level: str = "auto"
    content_focus: str = "balanced"
    learning_style: str = "comprehensive"
    pace: str = "moderate"

class PersonalizationRequest(BaseModel):
    chapter_id: str
    chapter_title: str
    preferences: PersonalizationPreferences

# Endpoint to get user profile with background information
@router.get("/profile", response_model=dict)
async def get_user_profile(
    current_user: User = Depends(get_current_user_dep),
    db: AsyncSession = Depends(get_db)
):
    """
    Get user profile including background information.
    """
    return {
        "id": current_user.id,
        "email": current_user.email,
        "full_name": current_user.full_name,
        "software_level": current_user.software_background_level,
        "hardware_level": current_user.hardware_background_level,
        "preferred_languages": current_user.preferred_languages,
        "goals": current_user.learning_goals,
        "is_active": current_user.is_active,
        "is_admin": current_user.is_admin,
        "created_at": current_user.created_at,
        "updated_at": current_user.updated_at,
        "last_login": current_user.last_login
    }

# Endpoint to update user background information
@router.post("/profile/update")
async def update_user_background(
    request: UpdateBackgroundRequest,
    current_user: User = Depends(get_current_user_dep),
    db: AsyncSession = Depends(get_db)
):
    """
    Update user background information.
    """
    # Update user background information
    updated_user = await update_user(
        db,
        current_user.id,
        software_background_level=request.software_level,
        hardware_background_level=request.hardware_level,
        preferred_languages=request.preferred_languages,
        learning_goals=request.goals
    )

    if not updated_user:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    return {
        "id": updated_user.id,
        "email": updated_user.email,
        "full_name": updated_user.full_name,
        "software_level": updated_user.software_background_level,
        "hardware_level": updated_user.hardware_background_level,
        "preferred_languages": updated_user.preferred_languages,
        "goals": updated_user.learning_goals,
        "is_active": updated_user.is_active,
        "is_admin": updated_user.is_admin
    }

# Endpoint to get personalized content based on user background
@router.get("/personalize/{content_type}")
async def get_personalized_content(
    content_type: str,
    current_user: User = Depends(get_current_user_dep),
    db: AsyncSession = Depends(get_db)
):
    """
    Get personalized content recommendations based on user background.
    """
    # This endpoint would generate personalized content based on user's background
    personalization = {
        "content_type": content_type,
        "difficulty_level": "beginner",
        "recommended_topics": [],
        "learning_path": [],
        "resources": []
    }

    # Determine difficulty based on user's background levels
    software_level = current_user.software_background_level or "beginner"
    hardware_level = current_user.hardware_background_level or "beginner"

    # Set difficulty based on user's skill level
    if software_level == "advanced" or hardware_level == "advanced":
        personalization["difficulty_level"] = "advanced"
    elif software_level == "intermediate" or hardware_level == "intermediate":
        personalization["difficulty_level"] = "intermediate"
    else:
        personalization["difficulty_level"] = "beginner"

    # Generate recommendations based on user's goals and preferences
    if current_user.learning_goals:
        personalization["recommended_topics"].append(current_user.learning_goals)

    if current_user.preferred_languages:
        # Add language-specific recommendations
        personalization["resources"].append(f"Content in {current_user.preferred_languages}")

    # Add default recommendations based on skill level
    if personalization["difficulty_level"] == "beginner":
        personalization["recommended_topics"].extend([
            "Introduction to Physical AI",
            "Basic Robotics Concepts",
            "Getting Started with ROS2"
        ])
        personalization["learning_path"] = [
            "Start with fundamentals",
            "Practice basic examples",
            "Move to intermediate concepts"
        ]
    elif personalization["difficulty_level"] == "intermediate":
        personalization["recommended_topics"].extend([
            "Advanced ROS2 Concepts",
            "Machine Learning for Robotics",
            "Sensor Integration"
        ])
        personalization["learning_path"] = [
            "Review intermediate concepts",
            "Implement complex examples",
            "Explore advanced topics"
        ]
    else:  # advanced
        personalization["recommended_topics"].extend([
            "Research-level AI for Robotics",
            "Custom Robot Development",
            "Advanced Control Systems"
        ])
        personalization["learning_path"] = [
            "Explore cutting-edge research",
            "Develop custom solutions",
            "Contribute to the field"
        ]

    return personalization

# Endpoint to get chapter personalization preferences
@router.get("/personalize/{chapter_id}/preferences")
async def get_chapter_preferences(
    chapter_id: str,
    current_user: User = Depends(get_current_user_dep),
    db: AsyncSession = Depends(get_db)
):
    """
    Get personalization preferences for a specific chapter.
    """
    # Get user's personalization preferences from the database
    # Since we don't have a separate table for personalization preferences yet,
    # we'll extract them from the user's profile data as JSON
    preferences = {}

    if current_user.personalization_preferences:
        try:
            # personalization_preferences is stored as a JSON string in the user table
            preferences = json.loads(current_user.personalization_preferences)
        except json.JSONDecodeError:
            # If there's an error parsing the JSON, return default preferences
            preferences = {
                "difficulty_level": "auto",
                "content_focus": "balanced",
                "learning_style": "comprehensive",
                "pace": "moderate"
            }

    # Check if there are chapter-specific preferences
    chapter_prefs = {}
    if "chapters" in preferences:
        chapter_prefs = preferences["chapters"].get(chapter_id, {})

    # Merge with defaults if needed
    default_prefs = {
        "difficulty_level": "auto",
        "content_focus": "balanced",
        "learning_style": "comprehensive",
        "pace": "moderate"
    }

    # Update defaults with any existing chapter-specific preferences
    default_prefs.update(chapter_prefs)

    return {
        "chapter_id": chapter_id,
        "preferences": default_prefs
    }

# Endpoint to save chapter personalization preferences
@router.post("/personalize/{chapter_id}/preferences")
async def save_chapter_preferences(
    chapter_id: str,
    request: PersonalizationRequest,
    current_user: User = Depends(get_current_user_dep),
    db: AsyncSession = Depends(get_db)
):
    """
    Save personalization preferences for a specific chapter.
    """
    try:
        # Get existing personalization preferences
        existing_prefs = {}
        if current_user.personalization_preferences:
            try:
                existing_prefs = json.loads(current_user.personalization_preferences)
            except json.JSONDecodeError:
                existing_prefs = {}

        # Initialize chapters dict if it doesn't exist
        if "chapters" not in existing_prefs:
            existing_prefs["chapters"] = {}

        # Update preferences for this specific chapter
        existing_prefs["chapters"][chapter_id] = request.preferences.dict()

        # Save the updated preferences back to the user
        updated_user = await update_user(
            db,
            current_user.id,
            personalization_preferences=json.dumps(existing_prefs)
        )

        if not updated_user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="User not found"
            )

        return {
            "chapter_id": chapter_id,
            "preferences": request.preferences.dict(),
            "message": "Preferences saved successfully"
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error saving preferences: {str(e)}"
        )