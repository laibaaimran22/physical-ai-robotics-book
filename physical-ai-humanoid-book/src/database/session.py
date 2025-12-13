from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from ..config.settings import settings


# Import all models to ensure foreign key relationships are resolved
from . import models  # This import ensures all models are registered with Base

# Create async engine for database
engine = create_async_engine(
    settings.DATABASE_URL,
    echo=settings.DEBUG,  # Set to True to see SQL queries in development
    pool_pre_ping=True,  # Verify connections before use
    pool_size=5,  # Number of connection objects to maintain
    max_overflow=10,  # Additional connections beyond pool_size
    pool_recycle=300,  # Recycle connections after 5 minutes
    # Enable foreign key constraints for SQLite
    connect_args={"check_same_thread": False} if "sqlite" in settings.DATABASE_URL.lower() else {},
)

# For SQLite, we need to enable foreign key constraints after connection
from sqlalchemy import event

if "sqlite" in settings.DATABASE_URL.lower():
    @event.listens_for(engine.sync_engine, "connect")
    def set_sqlite_pragma(dbapi_connection, connection_record):
        cursor = dbapi_connection.cursor()
        cursor.execute("PRAGMA foreign_keys=ON")
        cursor.close()

# Create async session factory
AsyncSessionLocal = sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False
)


async def get_db_session():
    """Dependency to get database session for FastAPI endpoints."""
    async with AsyncSessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()