#!/usr/bin/env python3
"""Test script to verify Gemini API fixes work correctly."""

import asyncio
import requests
import time
import uvicorn
import threading
import sys
import os

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

from src.main import app

def run_server():
    uvicorn.run(app, host='127.0.0.1', port=8008, log_level='info', access_log=False)

def test_gemini_fixes():
    # Start server in a thread
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()

    # Wait a moment for server to start
    time.sleep(4)

    print('Server running on port 8008, testing Gemini fixes...')

    # Test 1: Register a new user
    print("\n1. Registering a fresh user...")
    user_data = {
        'email': 'gemini_test_user@example.com',
        'password': 'test123',
        'full_name': 'Gemini Test User',
        'software_background_level': 'intermediate',
        'hardware_background_level': 'beginner',
        'preferred_languages': 'Python',
        'learning_goals': 'Learn AI for robotics'
    }

    try:
        response = requests.post(
            'http://127.0.0.1:8008/api/v1/auth/register',
            data=user_data
        )
        print(f'Registration: {response.status_code}')
        if response.status_code == 200:
            token = response.json()['access_token']
            print('User registered successfully')
        else:
            print(f'Registration failed: {response.text}')
            # Try with a different email if user already exists
            user_data['email'] = 'gemini_test_user2@example.com'
            response = requests.post(
                'http://127.0.0.1:8008/api/v1/auth/register',
                data=user_data
            )
            if response.status_code == 200:
                token = response.json()['access_token']
                print('User registered successfully with alternative email')
            else:
                print(f'Alternative registration also failed: {response.text}')
                return False
    except Exception as e:
        print(f'Registration error: {e}')
        return False

    # Test 2: Make a RAG query to test the Gemini fixes
    print("\n2. Testing RAG query with Gemini fixes...")
    headers = {'Authorization': f'Bearer {token}'}
    query_data = {
        'query': 'What are the basics of robot kinematics?',
        'top_k': 2
    }

    try:
        response = requests.post(
            'http://127.0.0.1:8008/api/v1/rag/query',
            json=query_data,
            headers=headers
        )
        print(f'RAG query status: {response.status_code}')
        if response.status_code == 200:
            result = response.json()
            response_text = result['response']
            print(f'Response length: {len(response_text)} chars')
            print(f'First 200 chars: {response_text[:200]}...')

            # Check if it's still returning the error message
            if "encountered an error" in response_text or "couldn't generate" in response_text:
                print(f'ERROR: Still getting error response: {response_text[:100]}...')
            else:
                print('SUCCESS: Got a real response from Gemini!')
        else:
            print(f'Query failed: {response.text}')
    except Exception as e:
        print(f'Query error: {e}')

    print("\nGemini fixes test completed!")
    return True

if __name__ == "__main__":
    test_gemini_fixes()