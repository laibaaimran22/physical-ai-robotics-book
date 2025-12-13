#!/usr/bin/env python3
"""Test registration with a fresh email using the same approach as test_with_server.py"""
import subprocess
import time
import requests
import signal
import sys
import os
from pathlib import Path

def main():
    # Start the uvicorn server as a subprocess
    server_process = subprocess.Popen([
        sys.executable, "-m", "uvicorn",
        "src.main:app",
        "--host", "127.0.0.1",
        "--port", "8000",
        "--log-level", "info"
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    try:
        # Wait for server to start
        print("Starting server...")
        time.sleep(5)  # Wait for server to start

        print("Server should be running, testing registration with fresh email...")

        # Test the registration endpoint with a completely fresh email
        response = requests.post(
            "http://127.0.0.1:8000/api/v1/auth/register",
            data={
                "email": "test_fresh_unique_123@example.com",
                "password": "test123",
                "full_name": "Test Fresh User 123"
            },
            timeout=10
        )

        print(f"Status Code: {response.status_code}")
        print(f"Response: {response.text}")

        if response.headers.get('content-type', '').startswith('application/json'):
            try:
                print(f"JSON Response: {response.json()}")
            except:
                print("Could not parse JSON response")

    except Exception as e:
        print(f"Error during test: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Terminate the server process
        print("Stopping server...")
        server_process.terminate()
        try:
            server_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server_process.kill()

if __name__ == "__main__":
    main()