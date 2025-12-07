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


class RAGQueryRequest(BaseModel):
    query: str
    top_k: int = Query(5, ge=1, le=20, description="Number of results to retrieve")
    use_langchain: bool = Query(False, description="Whether to use LangChain-based RAG (if available)")


class RAGQueryWithSelectedTextRequest(BaseModel):
    query: str
    selected_text: str


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

    try:
        result = await service.query_rag(
            query=request.query,
            user_id=user_id,
            top_k=request.top_k
        )
        return result
    except Exception as e:
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

    try:
        result = await service.query_with_selected_text(
            query=request.query,
            selected_text=request.selected_text,
            user_id=user_id
        )
        return result
    except Exception as e:
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
        raise HTTPException(
            status_code=500,
            detail=f"Error performing hybrid search: {str(e)}"
        )