#!/usr/bin/env python3
"""Test script to check API endpoints and get detailed error information."""

import requests
import json

def test_register():
    """Test the register endpoint."""
    print("Testing register endpoint...")

    url = "http://127.0.0.1:8000/api/v1/auth/register"

    # Prepare form data
    form_data = {
        'email': 'test_final_verification@example.com',
        'password': 'test123',
        'full_name': 'Test User'
    }

    try:
        response = requests.post(url, data=form_data)
        print(f"Status Code: {response.status_code}")
        print(f"Response: {response.text}")
        if response.headers.get('content-type', '').startswith('application/json'):
            try:
                json_response = response.json()
                print(f"JSON Response: {json.dumps(json_response, indent=2)}")
            except:
                print("Could not parse JSON response")
        else:
            print("Response is not JSON")
    except Exception as e:
        print(f"Request failed: {e}")

def test_chatbot():
    """Test the chatbot endpoint."""
    print("\nTesting chatbot endpoint...")

    url = "http://127.0.0.1:8000/api/v1/rag/query"

    # Prepare JSON data
    json_data = {
        'query': 'hello'
    }

    headers = {
        'Content-Type': 'application/json'
    }

    try:
        response = requests.post(url, json=json_data, headers=headers)
        print(f"Status Code: {response.status_code}")
        print(f"Response: {response.text}")
        if response.headers.get('content-type', '').startswith('application/json'):
            try:
                json_response = response.json()
                print(f"JSON Response: {json.dumps(json_response, indent=2)}")
            except:
                print("Could not parse JSON response")
        else:
            print("Response is not JSON")
    except Exception as e:
        print(f"Request failed: {e}")

if __name__ == "__main__":
    test_register()
    test_chatbot()