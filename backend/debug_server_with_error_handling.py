#!/usr/bin/env python3
"""Server with comprehensive error handling for the register endpoint"""
import sys
import os
import logging
from pathlib import Path

# Set up logging
logging.basicConfig(level=logging.DEBUG)

# Add the backend directory to the path
sys.path.insert(0, str(Path(__file__).parent))

# Import and patch the register function to add comprehensive error handling
from src.api.auth import register as original_register

async def debug_register_with_error_handling(*args, **kwargs):
    """Debug version of register with comprehensive error handling"""
    try:
        print(f"DEBUG: Register function called with args: {args}")
        print(f"DEBUG: Register function called with kwargs: {dict((k, v) for k, v in kwargs.items() if k != 'db')}")

        result = await original_register(*args, **kwargs)

        print(f"DEBUG: Register function succeeded with result type: {type(result)}")
        if result and isinstance(result, dict):
            print(f"DEBUG: Register result keys: {list(result.keys())}")

        return result
    except Exception as e:
        print(f"DEBUG: Register function failed with error: {e}")
        import traceback
        traceback.print_exc()
        raise

# Replace the register function in the auth module
import src.api.auth
src.api.auth.register = debug_register_with_error_handling

# Now run the server
import uvicorn
if __name__ == "__main__":
    uvicorn.run(
        "src.main:app",
        host="127.0.0.1",
        port=8000,
        log_level="debug"
    )