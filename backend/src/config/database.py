from sqlalchemy import create_engine
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker, declarative_base
from typing import AsyncGenerator
import os

from .settings import settings

# Create async engine with fallback to SQLite for Hugging Face Spaces
db_url = settings.DATABASE_URL or os.getenv("DATABASE_URL", "")

# Check if we're running on Hugging Face Spaces or don't have a proper database URL
if not db_url or "sqlite" not in db_url.lower():
    # Check if it's a PostgreSQL URL and ensure it uses async driver
    if "postgresql://" in db_url:
        # Handle sslmode parameter for asyncpg
        if "?sslmode=require" in db_url:
            db_url = db_url.replace("postgresql://", "postgresql+asyncpg://").replace("?sslmode=require", "")
        else:
            db_url = db_url.replace("postgresql://", "postgresql+asyncpg://")
    elif "postgres://" in db_url:
        # Handle sslmode parameter for asyncpg
        if "?sslmode=require" in db_url:
            db_url = db_url.replace("postgres://", "postgresql+asyncpg://").replace("?sslmode=require", "")
        else:
            db_url = db_url.replace("postgres://", "postgresql+asyncpg://")
    elif db_url:  # If there's a non-SQLite URL but it's not PostgreSQL
        if "asyncpg" not in db_url:
            if "postgresql" not in db_url:
                clean_url = db_url.split("://", 1)[1]
                if "?sslmode=require" in clean_url:
                    clean_url = clean_url.replace("?sslmode=require", "")
                db_url = "postgresql+asyncpg://" + clean_url
            else:
                if "?sslmode=require" in db_url:
                    db_url = db_url.replace("postgresql://", "postgresql+asyncpg://").replace("?sslmode=require", "")
                else:
                    db_url = db_url.replace("postgresql://", "postgresql+asyncpg://")
    else:
        # Default to SQLite for Hugging Face Spaces compatibility
        db_url = "sqlite+aiosqlite:///./book_platform.db"

# Configure engine based on database type
if "sqlite" in db_url:
    # SQLite engine (for Hugging Face Spaces)
    engine = create_async_engine(
        db_url,
        echo=settings.DEBUG,
        pool_pre_ping=True,
    )
    # Synchronous engine for SQLite
    sync_engine = create_engine(
        db_url.replace("sqlite+aiosqlite://", "sqlite://"),
        echo=settings.DEBUG,
        pool_pre_ping=True,
    )
else:
    # PostgreSQL engine (for other environments)
    if "postgresql://" in db_url:
        db_url = db_url.replace("postgresql://", "postgresql+asyncpg://")
    elif "postgres://" in db_url:
        db_url = db_url.replace("postgres://", "postgresql+asyncpg://")

    engine = create_async_engine(
        db_url,
        echo=settings.DEBUG,
        pool_pre_ping=True,
        pool_size=5,
        max_overflow=10,
        pool_recycle=300,
    )

    # Synchronous engine for PostgreSQL
    sync_db_url = settings.DATABASE_URL
    if "postgresql+asyncpg://" in sync_db_url:
        # Remove sslmode from the asyncpg URL and add it back for psycopg2
        base_url = sync_db_url.replace("postgresql+asyncpg://", "postgresql://")
        if "?sslmode=require" not in base_url:
            base_url += "?sslmode=require"
        sync_db_url = base_url
    elif "postgresql://" in sync_db_url:
        # Ensure sslmode is included for psycopg2
        if "?sslmode=require" not in sync_db_url:
            sync_db_url += "?sslmode=require"
    elif "+asyncpg" in sync_db_url:
        sync_db_url = sync_db_url.replace("+asyncpg", "+psycopg2")
        if "?sslmode=require" not in sync_db_url:
            sync_db_url += "?sslmode=require"

    sync_engine = create_engine(
        sync_db_url,
        echo=settings.DEBUG,
        pool_pre_ping=True,
        pool_size=5,
        max_overflow=10,
        pool_recycle=300,
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