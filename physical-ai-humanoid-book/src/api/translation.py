from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy.ext.asyncio import AsyncSession
from pydantic import BaseModel
from typing import Optional

from ..services.translation_service import get_translation_service
from ..api.deps import get_db_session, get_optional_user
from ..models.user import User

router = APIRouter()


class TranslationRequest(BaseModel):
    content: str
    source_language: Optional[str] = "en"
    target_language: Optional[str] = "ur"


class TranslationResponse(BaseModel):
    translated_content: str
    source_content: str
    source_language: str
    target_language: str
    success: bool
    error_message: Optional[str] = None


@router.post("/translate", response_model=TranslationResponse)
async def translate_content(
    request: TranslationRequest,
    db: AsyncSession = Depends(get_db_session),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Translate content from source language to target language using Google Gemini API.
    """
    if not request.content or len(request.content.strip()) == 0:
        raise HTTPException(
            status_code=400,
            detail="Content is required and cannot be empty"
        )

    if len(request.content) > 10000:
        raise HTTPException(
            status_code=400,
            detail="Content length exceeds maximum allowed length of 10000 characters"
        )

    translation_service = get_translation_service()

    try:
        result = await translation_service.translate_text(
            text=request.content,
            target_language=request.target_language,
            source_language=request.source_language
        )

        return TranslationResponse(
            translated_content=result.get("translated_text", ""),
            source_content=request.content,
            source_language=result.get("source_language", request.source_language),
            target_language=result.get("target_language", request.target_language),
            success=result.get("success", False),
            error_message=result.get("error", None)
        )

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error translating content: {str(e)}"
        )