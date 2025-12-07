from pydantic_settings import BaseSettings
from typing import List, Optional
from pydantic import Field


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Application
    APP_NAME: str = "Physical AI & Humanoid Robotics Book Platform"
    ENVIRONMENT: str = Field(default="development", alias="APP_ENV")
    DEBUG: bool = Field(default=True)
    VERSION: str = "1.0.0"
    API_V1_STR: str = "/api/v1"

    # Server
    HOST: str = Field(default="0.0.0.0", alias="APP_HOST")
    PORT: int = Field(default=8000, alias="APP_PORT")

    # Database - Neon Serverless Postgres
    DATABASE_URL: str = Field(default="", alias="DATABASE_URL")

    # Qdrant - Qdrant Cloud Free Tier
    QDRANT_URL: str = Field(default="", alias="QDRANT_URL")
    QDRANT_API_KEY: Optional[str] = Field(default=None, alias="QDRANT_API_KEY")
    QDRANT_CLUSTER_ID: Optional[str] = Field(default=None, alias="QDRANT_CLUSTER_ID")

    # Optional API Keys
    COHERE_API_KEY: Optional[str] = Field(default=None, alias="COHERE_API_KEY")
    OPENAI_API_KEY: Optional[str] = Field(default=None, alias="OPENAI_API_KEY")

    # LLM Configuration
    LLM_MODEL: str = "gpt-3.5-turbo"
    TEMPERATURE: float = 0.3

    # Embedding Configuration
    EMBEDDING_MODEL: str = "text-embedding-3-small"
    EMBEDDING_DIMENSION: int = 1536

    # CORS
    ALLOWED_ORIGINS: List[str] = ["*"]

    # Rate Limiting
    RATE_LIMIT_REQUESTS: int = 100
    RATE_LIMIT_WINDOW: int = 60  # seconds

    # Free Tier Limits
    FREE_TIER_RAG_QUERIES_PER_DAY: int = 30
    FREE_TIER_SEARCH_QUERIES_PER_DAY: int = 50

    # Chunking Configuration
    CHUNK_SIZE_TOKENS: int = 400
    MIN_CHUNK_SIZE_TOKENS: int = 100
    MAX_CHUNK_SIZE_TOKENS: int = 500

    # JWT Configuration
    JWT_SECRET_KEY: str = "your-super-secret-jwt-key-change-in-production"
    JWT_ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30
    REFRESH_TOKEN_EXPIRE_DAYS: int = 7

    class Config:
        env_file = ".env"
        case_sensitive = True


# Create settings instance
settings = Settings()