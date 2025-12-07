from sqlalchemy import create_engine
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker, declarative_base
from typing import AsyncGenerator
import os

from .settings import settings

# Create async engine for Postgres - ensure URL uses asyncpg driver
db_url = settings.DATABASE_URL
if "postgresql://" in db_url:
    db_url = db_url.replace("postgresql://", "postgresql+asyncpg://")
elif "postgres://" in db_url:
    db_url = db_url.replace("postgres://", "postgresql+asyncpg://")

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

# Synchronous engine for Alembic migrations
sync_engine = create_engine(
    settings.DATABASE_URL.replace("+asyncpg", "+psycopg2") if "+asyncpg" in settings.DATABASE_URL else settings.DATABASE_URL,
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