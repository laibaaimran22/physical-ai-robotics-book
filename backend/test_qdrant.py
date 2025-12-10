#!/usr/bin/env python3
"""
Test script to check Qdrant client functionality
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from qdrant_client import QdrantClient
from qdrant_client.http import models
from src.config.settings import settings

print(f"Qdrant client version: {QdrantClient.__module__}")
print(f"Settings QDRANT_URL: {settings.QDRANT_URL}")
print(f"Settings QDRANT_API_KEY: {'***' if settings.QDRANT_API_KEY else 'None'}")

# Initialize client
if settings.QDRANT_API_KEY:
    client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
        prefer_grpc=False
    )
else:
    client = QdrantClient(url=settings.QDRANT_URL)

print(f"Available methods containing 'search' or 'query':")
methods = [method for method in dir(client) if 'search' in method.lower() or 'query' in method.lower()]
for method in methods:
    print(f"  - {method}")

print(f"\nHas 'search' method: {hasattr(client, 'search')}")
print(f"Has 'query_points' method: {hasattr(client, 'query_points')}")
print(f"Has 'search_points' method: {hasattr(client, 'search_points')}")

# Test creating a collection
try:
    collection_name = "test_collection"
    print(f"\nTesting collection operations...")

    # Try to get collection (might fail if doesn't exist)
    try:
        collection_info = client.get_collection(collection_name)
        print(f"Collection '{collection_name}' exists")
    except:
        print(f"Collection '{collection_name}' does not exist, creating...")
        # Create collection
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=1536,  # Standard embedding size
                distance=models.Distance.COSINE
            ),
        )
        print(f"Collection '{collection_name}' created")

    # Test search with a fake vector
    import random
    fake_vector = [random.random() for _ in range(1536)]

    print(f"\nTesting search methods...")

    # Try query_points method
    try:
        result = client.query_points(
            collection_name=collection_name,
            query=fake_vector,
            limit=5
        )
        print("query_points method works!")
        print(f"Result type: {type(result)}")
        if hasattr(result, 'points'):
            print(f"Number of points returned: {len(result.points)}")
    except Exception as e:
        print(f"query_points failed: {e}")

    # Try search method
    try:
        result = client.search(
            collection_name=collection_name,
            query_vector=fake_vector,
            limit=5
        )
        print("search method works!")
        print(f"Result type: {type(result)}")
        print(f"Number of results: {len(result)}")
    except Exception as e:
        print(f"search failed: {e}")

except Exception as e:
    print(f"Error during testing: {e}")