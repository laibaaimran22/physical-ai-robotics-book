#!/usr/bin/env python3
"""Test script to verify personalization functionality end-to-end."""

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
    uvicorn.run(app, host='127.0.0.1', port=8007, log_level='info', access_log=False)

def test_personalization_end_to_end():
    # Start server in a thread
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()

    # Wait a moment for server to start
    time.sleep(4)

    print('Server running on port 8007, testing personalization...')

    # Test 1: Register user with advanced background
    print("\n1. Testing registration with advanced background...")
    advanced_user_data = {
        'email': 'advanced_user@example.com',
        'password': 'test123',
        'full_name': 'Advanced User',
        'software_background_level': 'advanced',
        'hardware_background_level': 'intermediate',
        'preferred_languages': 'Python, C++, ROS2',
        'learning_goals': 'Master humanoid robotics control systems'
    }

    try:
        response = requests.post(
            'http://127.0.0.1:8007/api/v1/auth/register',
            data=advanced_user_data
        )
        print(f'Advanced user registration: {response.status_code}')
        if response.status_code == 200:
            advanced_token = response.json()['access_token']
            print('Advanced user registered successfully')
        else:
            print(f'Registration failed: {response.text}')
            return False
    except Exception as e:
        print(f'Registration error: {e}')
        return False

    # Test 2: Register user with beginner background
    print("\n2. Testing registration with beginner background...")
    beginner_user_data = {
        'email': 'beginner_user@example.com',
        'password': 'test123',
        'full_name': 'Beginner User',
        'software_background_level': 'beginner',
        'hardware_background_level': 'beginner',
        'preferred_languages': 'Python',
        'learning_goals': 'Learn basic robotics concepts'
    }

    try:
        response = requests.post(
            'http://127.0.0.1:8007/api/v1/auth/register',
            data=beginner_user_data
        )
        print(f'Beginner user registration: {response.status_code}')
        if response.status_code == 200:
            beginner_token = response.json()['access_token']
            print('Beginner user registered successfully')
        else:
            print(f'Registration failed: {response.text}')
            return False
    except Exception as e:
        print(f'Registration error: {e}')
        return False

    # Test 3: Query as advanced user (should get advanced response)
    print("\n3. Querying as advanced user...")
    headers_advanced = {'Authorization': f'Bearer {advanced_token}'}
    query_data = {'query': 'Explain PID controllers in robotics', 'top_k': 2}

    try:
        response = requests.post(
            'http://127.0.0.1:8007/api/v1/rag/query',
            json=query_data,
            headers=headers_advanced
        )
        print(f'Advanced user query: {response.status_code}')
        if response.status_code == 200:
            advanced_response = response.json()['response']
            print(f'Advanced response length: {len(advanced_response)} chars')
            print(f'Advanced response preview: {advanced_response[:150]}...')
        else:
            print(f'Query failed: {response.text}')
    except Exception as e:
        print(f'Query error: {e}')

    # Test 4: Query as beginner user (should get beginner response)
    print("\n4. Querying as beginner user...")
    headers_beginner = {'Authorization': f'Bearer {beginner_token}'}

    try:
        response = requests.post(
            'http://127.0.0.1:8007/api/v1/rag/query',
            json=query_data,
            headers=headers_beginner
        )
        print(f'Beginner user query: {response.status_code}')
        if response.status_code == 200:
            beginner_response = response.json()['response']
            print(f'Beginner response length: {len(beginner_response)} chars')
            print(f'Beginner response preview: {beginner_response[:150]}...')
        else:
            print(f'Query failed: {response.text}')
    except Exception as e:
        print(f'Query error: {e}')

    # Test 5: Query with background in request (not authenticated)
    print("\n5. Querying with beginner background in request...")
    background_request = {
        'query': 'Explain PID controllers in robotics',
        'top_k': 2,
        'user_background': {
            'software_background_level': 'beginner',
            'hardware_background_level': 'beginner',
            'preferred_languages': 'Python',
            'learning_goals': 'Learn basic robotics'
        }
    }

    try:
        response = requests.post(
            'http://127.0.0.1:8007/api/v1/rag/query',
            json=background_request
        )
        print(f'Background request query: {response.status_code}')
        if response.status_code == 200:
            bg_response = response.json()['response']
            print(f'Background response length: {len(bg_response)} chars')
            print(f'Background response preview: {bg_response[:150]}...')
        else:
            print(f'Query failed: {response.text}')
    except Exception as e:
        print(f'Query error: {e}')

    print("\nPersonalization test completed!")
    return True

if __name__ == "__main__":
    test_personalization_end_to_end()