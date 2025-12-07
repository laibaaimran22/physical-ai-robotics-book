import requests
import time

print("Testing connection to the Physical AI & Humanoid Robotics Book Platform...")
print("Checking if server is accessible at http://127.0.0.1:8000...")

try:
    response = requests.get("http://127.0.0.1:8000/", timeout=10)
    print(f"✅ SUCCESS: Connected to server!")
    print(f"   Status Code: {response.status_code}")
    print(f"   Response: {response.json()}")

    # Test health endpoint too
    health_response = requests.get("http://127.0.0.1:8000/health", timeout=10)
    print(f"✅ Health Check: {health_response.status_code}")
    print(f"   Health Response: {health_response.json()}")

    print("\n🎉 The RAG chatbot backend is running successfully!")
    print("   - API available at: http://127.0.0.1:8000")
    print("   - Health check: http://127.0.0.1:8000/health")
    print("   - API docs: http://127.0.0.1:8000/api/v1/docs (if in dev mode)")

except requests.exceptions.ConnectionError as e:
    print(f"❌ Connection failed: {e}")
    print("   The server might still be starting up. Please wait a moment and try again.")
except Exception as e:
    print(f"❌ Error: {e}")