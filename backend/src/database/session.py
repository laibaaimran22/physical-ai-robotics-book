from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from ..config.settings import settings


# Create async engine for Postgres
engine = create_async_engine(
    settings.DATABASE_URL,
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


async def get_db_session():
    """Dependency to get database session for FastAPI endpoints."""
    async with AsyncSessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()