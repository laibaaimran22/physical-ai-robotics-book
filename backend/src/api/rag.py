from fastapi import APIRouter, HTTPException, Depends, Query
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional, List, Dict, Any
from pydantic import BaseModel

from ..services.rag_service import get_rag_service
from ..config.settings import settings
from ..api.deps import get_db_session
from ..models.user import User
from ..api.deps import get_optional_user

router = APIRouter()


class UserBackgroundRequest(BaseModel):
    software_background_level: Optional[str] = None
    hardware_background_level: Optional[str] = None
    preferred_languages: Optional[str] = None
    learning_goals: Optional[str] = None


class RAGQueryRequest(BaseModel):
    query: str
    top_k: int = Query(5, ge=1, le=20, description="Number of results to retrieve")
    use_langchain: bool = Query(False, description="Whether to use LangChain-based RAG (if available)")
    user_background: Optional[UserBackgroundRequest] = None


class RAGQueryWithSelectedTextRequest(BaseModel):
    query: str
    selected_text: str
    user_background: Optional[UserBackgroundRequest] = None


@router.post("/rag/query")
async def rag_query(
    request: RAGQueryRequest,
    db: AsyncSession = Depends(get_db_session),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Query the RAG system for contextually relevant responses.
    """
    service = get_rag_service(db)
    user_id = current_user.id if current_user else None

    # Prepare user background information for personalization
    user_background = None
    if current_user:
        # Use user's profile information if available
        user_background = {
            "software_background_level": current_user.software_background_level,
            "hardware_background_level": current_user.hardware_background_level,
            "preferred_languages": current_user.preferred_languages,
            "learning_goals": current_user.learning_goals
        }
    elif request.user_background:
        # Use background information from request if user is not authenticated
        user_background = request.user_background.dict()

    try:
        result = await service.query_rag(
            query=request.query,
            user_id=user_id,
            user_background=user_background,
            top_k=request.top_k
        )
        return result
    except Exception as e:
        error_msg = str(e)
        # Log the full error for debugging
        import logging
        logging.error(f"Error in rag_query endpoint: {error_msg}", exc_info=True)

        # Check if it's an API rate limit error
        if "429" in error_msg or "rate limit" in error_msg.lower() or "exceeded" in error_msg.lower():
            return {
                "response": "The AI service is currently experiencing high demand. Please try again in a few moments.",
                "sources": [],
                "response_time_ms": 0,
                "tokens_used": 0,
                "warning": "Rate limit exceeded - using fallback response"
            }

        raise HTTPException(
            status_code=500,
            detail=f"Error processing RAG query: {str(e)}"
        )


@router.post("/rag/query-selected")
async def rag_query_with_selected_text(
    request: RAGQueryWithSelectedTextRequest,
    db: AsyncSession = Depends(get_db_session),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Query the RAG system with user-selected text as additional context.
    """
    service = get_rag_service(db)
    user_id = current_user.id if current_user else None

    # Prepare user background information for personalization
    user_background = None
    if current_user:
        # Use user's profile information if available
        user_background = {
            "software_background_level": current_user.software_background_level,
            "hardware_background_level": current_user.hardware_background_level,
            "preferred_languages": current_user.preferred_languages,
            "learning_goals": current_user.learning_goals
        }
    elif request.user_background:
        # Use background information from request if user is not authenticated
        user_background = request.user_background.dict()

    try:
        result = await service.query_with_selected_text(
            query=request.query,
            selected_text=request.selected_text,
            user_id=user_id,
            user_background=user_background
        )
        return result
    except Exception as e:
        error_msg = str(e)
        # Log the full error for debugging
        import logging
        logging.error(f"Error in rag_query_with_selected_text endpoint: {error_msg}", exc_info=True)

        # Check if it's an API rate limit error
        if "429" in error_msg or "rate limit" in error_msg.lower() or "exceeded" in error_msg.lower():
            return {
                "response": "The AI service is currently experiencing high demand. Please try again in a few moments.",
                "sources": [],
                "selected_text_used": request.selected_text,
                "response_time_ms": 0,
                "tokens_used": 0,
                "warning": "Rate limit exceeded - using fallback response"
            }

        raise HTTPException(
            status_code=500,
            detail=f"Error processing RAG query with selected text: {str(e)}"
        )


@router.post("/rag/search")
async def semantic_search(
    query: str,
    top_k: int = Query(10, ge=1, le=50, description="Number of results to retrieve"),
    content_type: Optional[str] = Query(None, description="Filter by content type"),
    db: AsyncSession = Depends(get_db_session),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Perform semantic search without generating a response.
    """
    service = get_rag_service(db)

    try:
        results = await service.semantic_search(
            query=query,
            top_k=top_k,
            content_type=content_type
        )
        return {
            "results": results,
            "query": query,
            "top_k": top_k,
            "content_type": content_type
        }
    except Exception as e:
        error_msg = str(e)
        # Log the full error for debugging
        import logging
        logging.error(f"Error in semantic_search endpoint: {error_msg}", exc_info=True)

        raise HTTPException(
            status_code=500,
            detail=f"Error performing semantic search: {str(e)}"
        )


@router.post("/rag/hybrid-search")
async def hybrid_search(
    query: str,
    keyword_query: Optional[str] = Query(None, description="Additional keyword query for hybrid search"),
    top_k: int = Query(10, ge=1, le=50, description="Number of results to retrieve"),
    db: AsyncSession = Depends(get_db_session),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Perform hybrid search combining semantic and keyword search.
    """
    service = get_rag_service(db)

    try:
        results = await service.hybrid_search(
            query=query,
            keyword_query=keyword_query,
            top_k=top_k
        )
        return {
            "results": results,
            "query": query,
            "keyword_query": keyword_query,
            "top_k": top_k
        }
    except Exception as e:
        error_msg = str(e)
        # Log the full error for debugging
        import logging
        logging.error(f"Error in hybrid_search endpoint: {error_msg}", exc_info=True)

        raise HTTPException(
            status_code=500,
            detail=f"Error performing hybrid search: {str(e)}"
        )