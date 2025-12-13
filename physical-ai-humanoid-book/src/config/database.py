from sqlalchemy import create_engine
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker, declarative_base
from typing import AsyncGenerator
import os

from .settings import settings

# Create async engine for Postgres - ensure URL uses asyncpg driver
# If DATABASE_URL is empty, use SQLite for Hugging Face Spaces compatibility
db_url = settings.DATABASE_URL
if not db_url or db_url.strip() == '':
    # Use SQLite in-memory database for Hugging Face Spaces
    db_url = "sqlite+aiosqlite:///:memory:"
    sync_db_url = "sqlite:///./app.db"  # Use file-based SQLite for sync operations
else:
    if "postgresql://" in db_url:
        db_url = db_url.replace("postgresql://", "postgresql+asyncpg://")
    elif "postgres://" in db_url:
        db_url = db_url.replace("postgres://", "postgresql+asyncpg://")
    sync_db_url = settings.DATABASE_URL.replace("+asyncpg", "+psycopg2") if "+asyncpg" in settings.DATABASE_URL else settings.DATABASE_URL

# Configure engine differently based on database type
if db_url.startswith("sqlite"):
    # SQLite doesn't support connection pooling parameters
    engine = create_async_engine(
        db_url,
        echo=settings.DEBUG,  # Set to True to see SQL queries in development
    )
else:
    # PostgreSQL supports connection pooling parameters
    engine = create_async_engine(
        db_url,
        echo=settings.DEBUG,  # Set to True to see SQL queries in development
        pool_pre_ping=True,  # Verify connections before use
        pool_size=5,  # Number of connection objects to maintain
        max_overflow=10,  # Additional connections beyond pool_size
        pool_recycle=300,  # Recycle connections after 5 minutes
    )

# Create async session factory
AsyncSessionLocal = sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False
)

# Create base class for declarative models
Base = declarative_base()

async def get_db_session() -> AsyncGenerator[AsyncSession, None]:
    """Dependency to get database session for FastAPI endpoints."""
    async with AsyncSessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()

# Synchronous engine for Alembic migrations and compatibility
if sync_db_url.startswith("sqlite"):
    # SQLite doesn't support connection pooling parameters
    sync_engine = create_engine(
        sync_db_url,
        echo=settings.DEBUG,
    )
else:
    # PostgreSQL supports connection pooling parameters
    sync_engine = create_engine(
        sync_db_url,
        echo=settings.DEBUG,
        pool_pre_ping=True,
        pool_size=5,
        max_overflow=10,
        pool_recycle=300,
    )

SyncSessionLocal = sessionmaker(
    sync_engine,
    expire_on_commit=False
)


def get_sync_db_session():
    """Dependency to get synchronous database session for legacy operations."""
    session = SyncSessionLocal()
    try:
        yield session
    finally:
        session.close()