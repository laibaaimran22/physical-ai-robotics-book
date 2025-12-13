#!/usr/bin/env python3
"""Test script that starts server and tests registration in one process"""
import asyncio
import subprocess
import time
import requests
import signal
import sys
import os
from pathlib import Path

# Add the backend directory to the path
sys.path.insert(0, str(Path(__file__).parent))

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

        print("Server should be running, testing registration...")

        # Test the registration endpoint
        response = requests.post(
            "http://127.0.0.1:8000/api/v1/auth/register",
            data={
                "email": "test_with_server@example.com",
                "password": "test123",
                "full_name": "Test With Server User"
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

        # Also test the chatbot endpoint
        chat_response = requests.post(
            "http://127.0.0.1:8000/api/v1/rag/query",
            json={"query": "hello"},
            headers={"Content-Type": "application/json"},
            timeout=10
        )

        print(f"Chat Status Code: {chat_response.status_code}")
        print(f"Chat Response: {chat_response.text[:200]}...")  # First 200 chars

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