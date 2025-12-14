import os
import sys
import subprocess
import threading
import time
import requests
from pathlib import Path

def test_backend():
    # Change to the backend directory
    backend_dir = Path("C:\\Users\\laiba\\OneDrive\\Desktop\\hackathon-book\\backend")
    os.chdir(backend_dir)

    # Start the backend server in a subprocess
    print("Starting backend server...")
    process = subprocess.Popen([
        sys.executable, "-m", "uvicorn", "app:app",
        "--host", "0.0.0.0",
        "--port", "8000",
        "--reload"
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # Give the server some time to start
    time.sleep(5)

    try:
        # Test the health endpoint
        response = requests.get("http://localhost:8000/health", timeout=10)
        if response.status_code == 200:
            print("✅ Backend is running and health check passed!")
            print(f"Response: {response.json()}")
            return True
        else:
            print(f"❌ Backend health check failed with status {response.status_code}")
            return False
    except requests.exceptions.RequestException as e:
        print(f"❌ Could not connect to backend: {e}")
        return False
    finally:
        # Terminate the process
        process.terminate()
        process.wait()

if __name__ == "__main__":
    test_backend()