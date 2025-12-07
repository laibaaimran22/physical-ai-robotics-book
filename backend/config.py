"""
Configuration management for different environments (dev, staging, prod)
"""
import os
from enum import Enum
from typing import Optional
from pydantic_settings import BaseSettings


class Environment(str, Enum):
    DEVELOPMENT = "development"
    STAGING = "staging"
    PRODUCTION = "production"


class Config(BaseSettings):
    """Base configuration class."""

    # Environment
    ENVIRONMENT: Environment = Environment.DEVELOPMENT
    DEBUG: bool = True

    # Database
    DATABASE_URL: str

    # Qdrant
    QDRANT_URL: str
    QDRANT_API_KEY: Optional[str] = None

    # API Keys
    OPENAI_API_KEY: Optional[str] = None
    HUGGINGFACE_API_KEY: Optional[str] = None
    OLLAMA_BASE_URL: str = "http://localhost:11434"

    # JWT
    JWT_SECRET_KEY: str
    JWT_ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30
    REFRESH_TOKEN_EXPIRE_DAYS: int = 7

    # Rate Limiting
    RATE_LIMIT_REQUESTS: int = 100
    RATE_LIMIT_WINDOW: int = 60  # seconds

    # API Limits (for free tier)
    FREE_TIER_RAG_QUERIES_PER_DAY: int = 30
    FREE_TIER_SEARCH_QUERIES_PER_DAY: int = 50
    FREE_TIER_NOTES_LIMIT: int = 50
    FREE_TIER_HIGHLIGHTS_LIMIT: int = 50
    FREE_TIER_BOOKMARKS_LIMIT: int = 10

    # Content processing
    CHUNK_SIZE_TOKENS: int = 400
    MIN_CHUNK_SIZE_TOKENS: int = 100
    MAX_CHUNK_SIZE_TOKENS: int = 500

    # Embedding model
    EMBEDDING_MODEL: str = "all-MiniLM-L6-v2"
    EMBEDDING_DIMENSION: int = 384

    # LLM Model
    LLM_MODEL: str = "llama2"

    class Config:
        env_file = ".env"
        case_sensitive = True


class DevConfig(Config):
    """Development configuration."""
    ENVIRONMENT: Environment = Environment.DEVELOPMENT
    DEBUG: bool = True


class StagingConfig(Config):
    """Staging configuration."""
    ENVIRONMENT: Environment = Environment.STAGING
    DEBUG: bool = False


class ProdConfig(Config):
    """Production configuration."""
    ENVIRONMENT: Environment = Environment.PRODUCTION
    DEBUG: bool = False


def get_config() -> Config:
    """Get configuration based on environment."""
    env = os.getenv("ENVIRONMENT", "development").lower()

    if env == "production":
        return ProdConfig()
    elif env == "staging":
        return StagingConfig()
    else:
        return DevConfig()


# Global config instance
config = get_config()