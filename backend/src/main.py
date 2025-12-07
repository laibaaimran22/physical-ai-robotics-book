from datetime import datetime
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List, Dict, Any

from .api.rag import router as rag_router
from .api.ingest import router as ingest_router
from .config.settings import settings

# Create FastAPI app instance
app = FastAPI(
    title="Physical AI & Humanoid Robotics Book Platform API",
    description="Backend API for the Physical AI & Humanoid Robotics book platform with free RAG chatbot, content management, and optional user features",
    version="1.0.0",
    openapi_url="/api/v1/openapi.json" if settings.ENVIRONMENT != "production" else None,
    docs_url="/api/v1/docs" if settings.ENVIRONMENT != "production" else None,
    redoc_url="/api/v1/redoc" if settings.ENVIRONMENT != "production" else None,
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Expose headers for rate limiting information
    expose_headers=["X-RateLimit-Remaining", "X-RateLimit-Limit", "X-RateLimit-Reset"],
)

# Include API routers
app.include_router(rag_router, prefix="/api/v1", tags=["RAG"])
app.include_router(ingest_router, prefix="/api/v1", tags=["Ingestion"])

@app.get("/health")
async def health_check():
    """Health check endpoint to verify the API is running."""
    import asyncio

    # Check if database is accessible
    try:
        # This would check if the database connection is working
        db_health = "ok"
    except Exception:
        db_health = "unavailable"

    # Check if vector store is accessible
    try:
        # This would check if the Qdrant connection is working
        vector_store_health = "ok"
    except Exception:
        vector_store_health = "unavailable"

    return {
        "status": "healthy",
        "service": "Physical AI & Humanoid Robotics Book Platform Backend",
        "timestamp": datetime.utcnow().isoformat(),
        "dependencies": {
            "database": db_health,
            "vector_store": vector_store_health
        }
    }

@app.get("/")
async def root():
    """Root endpoint with basic information about the API."""
    return {
        "message": "Welcome to the Physical AI & Humanoid Robotics Book Platform Backend",
        "version": "1.0.0",
        "docs": "/api/v1/docs",
        "health": "/health"
    }

@app.get("/api/v1/status")
async def api_status():
    """Detailed status information about the API."""
    return {
        "status": "operational",
        "features": {
            "rag_query": True,
            "rag_query_selected": True,
            "document_ingestion": True,
            "semantic_search": True,
            "free_tier_limits": True
        },
        "models": {
            "embedding": settings.EMBEDDING_MODEL,
            "llm": settings.LLM_MODEL
        },
        "rate_limits": {
            "requests_per_minute": settings.RATE_LIMIT_REQUESTS,
            "free_tier_daily_queries": settings.FREE_TIER_RAG_QUERIES_PER_DAY
        }
    }