#!/usr/bin/env python3
"""Script to start server and test registration in one go"""
import subprocess
import time
import requests
import threading
import signal
import sys
import os

def run_server():
    """Run the server in a subprocess"""
    server_process = subprocess.Popen([
        sys.executable, "-c",
        """
import uvicorn
import sys
import os
from pathlib import Path
sys.path.insert(0, str(Path('.').resolve()))
uvicorn.run('src.main:app', host='127.0.0.1', port=8000, log_level='debug')
        """
    ], cwd='backend', stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    return server_process

def test_registration():
    """Test the registration endpoint"""
    time.sleep(3)  # Wait for server to start

    try:
        response = requests.post(
            'http://127.0.0.1:8000/api/v1/auth/register',
            data={
                'email': 'test_thread@example.com',
                'password': 'test123',
                'full_name': 'Test Thread User'
            },
            timeout=10
        )
        print(f"Status Code: {response.status_code}")
        print(f"Response: {response.text}")
        print(f"Headers: {dict(response.headers)}")
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")

if __name__ == "__main__":
    # Start server
    server_proc = run_server()
    print("Server started, PID:", server_proc.pid)

    try:
        # Test registration in another thread
        test_registration()
    finally:
        # Kill the server
        server_proc.terminate()
        try:
            server_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server_proc.kill()
        print("Server stopped")