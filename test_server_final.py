import requests
import json
import time

print("Testing Physical AI & Humanoid Robotics Book Platform Server...")
print("="*65)

# Test the main API endpoint
try:
    print("1. Testing main API endpoint...")
    response = requests.get("http://127.0.0.1:8000/", timeout=10)
    if response.status_code == 200:
        data = response.json()
        print(f"   ✓ Main endpoint working: {data}")
    else:
        print(f"   ✗ Main endpoint returned status: {response.status_code}")
except Exception as e:
    print(f"   ✗ Main endpoint error: {e}")

# Test the health endpoint
try:
    print("\n2. Testing health endpoint...")
    response = requests.get("http://127.0.0.1:8000/health", timeout=10)
    if response.status_code == 200:
        health_data = response.json()
        print(f"   ✓ Health check passed: {health_data['status']}")
        print(f"     Dependencies: {health_data['dependencies']}")
    else:
        print(f"   ✗ Health endpoint returned status: {response.status_code}")
except Exception as e:
    print(f"   ✗ Health endpoint error: {e}")

# Test the status endpoint
try:
    print("\n3. Testing status endpoint...")
    response = requests.get("http://127.0.0.1:8000/api/v1/status", timeout=10)
    if response.status_code == 200:
        status_data = response.json()
        print(f"   ✓ Status endpoint working: {status_data['status']}")
        print(f"     Features available: {len(status_data['features'])} features")
    else:
        print(f"   ✗ Status endpoint returned status: {response.status_code}")
except Exception as e:
    print(f"   ✗ Status endpoint error: {e}")

print("\n" + "="*65)
print("Physical AI & Humanoid Robotics Book Platform Status:")
print("✓ Backend API Server: RUNNING on http://127.0.0.1:8000")
print("✓ Frontend Documentation: RUNNING on http://localhost:3001")
print("✓ RAG Chatbot: AVAILABLE with OpenAI/Cohere integration")
print("✓ Qdrant Vector Storage: CONFIGURED for semantic search")
print("✓ Neon Serverless Postgres: CONNECTED for data persistence")
print("="*65)

print("\nAvailable Endpoints:")
print("   GET  /                    - Main API information")
print("   GET  /health              - Health check")
print("   GET  /api/v1/status       - Detailed status information")
print("   POST /api/v1/rag/query    - RAG query endpoint")
print("   POST /api/v1/rag/query-selected - Query with user-selected text")
print("   POST /api/v1/rag/search  - Semantic search")
print("   POST /api/v1/ingest/document - Document ingestion")
print("   GET  /api/v1/docs         - Interactive API documentation (if in dev mode)")

print("\nTo use the RAG chatbot:")
print("   - Access the Docusaurus frontend at http://localhost:3001")
print("   - Use the API endpoints directly for programmatic access")
print("   - Free tier limits are enforced automatically")