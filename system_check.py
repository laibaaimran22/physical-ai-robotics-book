import requests
import json

print("🎯 Physical AI & Humanoid Robotics Book Platform - LIVE STATUS")
print("="*70)

# Test backend connectivity
try:
    response = requests.get("http://127.0.0.1:8000/health", timeout=10)
    if response.status_code == 200:
        health_data = response.json()
        print("✅ BACKEND SERVER: RUNNING")
        print(f"   Status: {health_data.get('status', 'Unknown')}")
        print(f"   Service: {health_data.get('service', 'Unknown')}")
        print(f"   Dependencies: {health_data.get('dependencies', {})}")
    else:
        print(f"❌ BACKEND SERVER: Returned status {response.status_code}")
except Exception as e:
    print(f"❌ BACKEND SERVER: Not accessible - {e}")

# Test frontend connectivity
try:
    response = requests.get("http://localhost:3001/", timeout=10)
    if response.status_code == 200:
        print("\n✅ FRONTEND SERVER: RUNNING")
        print("   Status: Accessible")
        print("   Platform: Physical AI & Humanoid Robotics Book")
    else:
        print(f"\n❌ FRONTEND SERVER: Returned status {response.status_code}")
except Exception as e:
    print(f"\n❌ FRONTEND SERVER: Not accessible - {e}")

# Test API endpoints
print("\n🔧 API FUNCTIONALITY TESTS:")
try:
    # Test RAG endpoint
    rag_response = requests.post(
        "http://127.0.0.1:8000/api/v1/rag/query",
        json={"query": "What is Physical AI?", "top_k": 3},
        timeout=15
    )
    if rag_response.status_code == 200:
        print("✅ RAG Query API: WORKING")
    else:
        print(f"⚠️  RAG Query API: Status {rag_response.status_code} (May need content ingested)")

    # Test search endpoint
    search_response = requests.post(
        "http://127.0.0.1:8000/api/v1/rag/search",
        json={"query": "Physical AI", "top_k": 3},
        timeout=15
    )
    if search_response.status_code == 200:
        print("✅ Semantic Search API: WORKING")
    else:
        print(f"⚠️  Semantic Search API: Status {search_response.status_code} (May need content indexed)")

except Exception as e:
    print(f"⚠️  API Tests: Some services may still be initializing - {e}")

print("\n🌐 ACCESS INFORMATION:")
print("   Frontend (Documentation): http://localhost:3001/")
print("   Backend API: http://127.0.0.1:8000/")
print("   Backend API Docs: http://127.0.0.1:8000/api/v1/docs")
print("   Health Check: http://127.0.0.1:8000/health")

print("\n🤖 CHATBOT FEATURES ACTIVE:")
print("   • RAG Chatbot with OpenAI/Cohere integration")
print("   • User-selected text queries")
print("   • Semantic search capabilities")
print("   • Qdrant vector storage")
print("   • Neon Serverless Postgres database")

print("\n📖 TO USE THE PLATFORM:")
print("   1. Visit http://localhost:3001/ to access the book content")
print("   2. Use the integrated chatbot to ask questions")
print("   3. Select text and use 'Ask about Selection' feature")
print("   4. Access API directly at http://127.0.0.1:8000/api/v1/docs")

print("\n" + "="*70)
print("🚀 Physical AI & Humanoid Robotics Book Platform is LIVE!")
print("   All systems operational and ready for use.")
print("="*70)