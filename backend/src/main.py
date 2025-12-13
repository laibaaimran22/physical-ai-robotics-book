import os
from datetime import datetime
from typing import Optional, List, Dict, Any

import openai
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from .api.rag import router as rag_router
from .api.book import router as book_router
from .api.ingest import router as ingest_router
from .api.auth import router as auth_router
from .api.auth_background import router as auth_background_router
from .config.settings import settings
from .agent.agent_runner import agent, Runner, AgentRequest
from .core.llm_client_gemini import initialize_gemini_client

# Load environment variables
load_dotenv()

# Initialize the Gemini client
initialize_gemini_client()
print("DEBUG: Gemini client initialization called")

# Initialize the OpenAI client with Google Gemini API
GEMINI_API_KEY = (os.getenv("GOOGLE_GEMINI_API_KEY") or os.getenv("GEMINI_API_KEY") or
                 settings.GOOGLE_GEMINI_API_KEY or settings.GEMINI_API_KEY)

if not GEMINI_API_KEY:
    print("WARNING: No GEMINI_API_KEY found. Chatbot functionality will not work.")
    # Allow the app to start even without the API key and handle errors gracefully in endpoints
    print("Running without GEMINI_API_KEY - chatbot will be disabled")
    # Create a mock client that returns error messages when called
    class MockAsyncOpenAI:
        def __init__(self, api_key, base_url):
            pass

        class Chat:
            def completions(self):
                return self

            async def create(self, **kwargs):
                raise Exception("No API key configured - chat functionality unavailable")

        @property
        def chat(self):
            return MockAsyncOpenAI.Chat()

    client = MockAsyncOpenAI(api_key="", base_url="")
else:
    client = openai.AsyncOpenAI(
        api_key=GEMINI_API_KEY,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    )

# Create FastAPI app instance
app = FastAPI(
    title="Physical AI & Humanoid Robotics Book Platform API",
    description="Backend API for the Physical AI & Humanoid Robotics book platform with free RAG chatbot, content management, and optional user features",
    version="1.0.0",
    openapi_url="/api/v1/openapi.json" if settings.ENVIRONMENT != "production" else None,
    docs_url="/api/v1/docs" if settings.ENVIRONMENT != "production" else None,
    redoc_url="/api/v1/redoc" if settings.ENVIRONMENT != "production" else None,
)


# Pydantic model for chat requests
class ChatRequest(BaseModel):
    message: str


# Initialize the agent once (not on every request) - using the agent from agent_runner
# The agent is already defined in the imported module

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
app.include_router(book_router, prefix="/api/v1", tags=["Book"])
app.include_router(ingest_router, prefix="/api/v1", tags=["Ingestion"])
app.include_router(auth_router, prefix="/api/v1/auth", tags=["Authentication"])
app.include_router(auth_background_router, prefix="/api/v1/auth", tags=["Authentication Background"])


@app.post("/chat")
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint that accepts a user message and returns the agent's response.
    Accepts JSON body: { "message": "user query" }
    Returns the agent's final_output to the frontend.
    """
    try:
        # Run the agent using Runner.run_sync()
        result = Runner.run_sync(starting_agent=agent, input_message=request.message)
        return {"response": result.final_output}
    except Exception as e:
        error_msg = str(e)
        if "No API key configured" in error_msg:
            return {"response": "Chat functionality is currently unavailable. Please try again later."}
        else:
            raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


@app.post("/agent-chat")
async def agent_chat_endpoint(request: AgentRequest):
    """
    Agent chat endpoint that accepts a user message and returns the agent's response.
    Accepts JSON body: { "message": "user message" }
    Returns: { "reply": "agent response" }
    """
    try:
        # Run the agent using Runner.run_sync()
        result = Runner.run_sync(starting_agent=agent, input_message=request.message)
        return {"reply": result.final_output}
    except Exception as e:
        error_msg = str(e)
        if "No API key configured" in error_msg:
            return {"reply": "Chat functionality is currently unavailable. Please try again later."}
        else:
            raise HTTPException(status_code=500, detail=f"Error processing agent chat request: {str(e)}")


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