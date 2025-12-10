#!/usr/bin/env python3
"""
Simple test to check if the Qdrant connection works
"""
import asyncio
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.core.vector_store import VectorStore
from src.core.embedding_client import embedding_client

async def test_vector_store():
    print("Testing vector store...")
    vector_store = VectorStore()

    print(f"Client initialized: {vector_store.client is not None}")
    print(f"Has query_points: {hasattr(vector_store.client, 'query_points')}")
    print(f"Has search: {hasattr(vector_store.client, 'search')}")

    # Try a simple embedding
    test_query = "Hello"
    query_vector = embedding_client.generate_embedding(test_query)
    print(f"Generated embedding of length: {len(query_vector)}")

    # Try to search (this is what's failing in the API)
    try:
        results = await vector_store.search_similar(
            query_vector=query_vector,
            limit=5
        )
        print(f"Search successful! Found {len(results)} results")
        print(f"Results: {results}")
    except Exception as e:
        print(f"Search failed with error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_vector_store())