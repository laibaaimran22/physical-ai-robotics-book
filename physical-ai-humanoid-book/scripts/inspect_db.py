#!/usr/bin/env python3
"""Script to inspect database tables."""

import asyncio
import sys
import os

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sqlalchemy import inspect
from sqlalchemy.ext.asyncio import create_async_engine
from src.config.settings import settings

async def inspect_db():
    """Inspect database tables."""
    # Create async engine
    engine = create_async_engine(
        settings.DATABASE_URL,
        echo=settings.DEBUG,
        pool_pre_ping=True,
        pool_size=5,
        max_overflow=10,
        pool_recycle=300,
    )

    async with engine.begin() as conn:
        # Run the inspection in a sync context
        def do_inspect(connection):
            insp = inspect(connection)
            return {
                'tables': insp.get_table_names(),
                'embedding_columns': insp.get_columns('embedding_documents') if 'embedding_documents' in insp.get_table_names() else None
            }

        result = await conn.run_sync(do_inspect)
        print('Tables:', result['tables'])
        if result['embedding_columns']:
            print('Embedding Documents columns:', result['embedding_columns'])
        else:
            print('embedding_documents table not found')

if __name__ == "__main__":
    asyncio.run(inspect_db())