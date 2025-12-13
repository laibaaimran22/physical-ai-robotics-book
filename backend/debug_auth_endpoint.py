#!/usr/bin/env python3
"""Server with comprehensive error handling"""
import sys
import os
import logging
import traceback
from pathlib import Path

# Set up logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

# Add the backend directory to the path
sys.path.insert(0, str(Path(__file__).parent))

# Set environment to catch all exceptions
import faulthandler
faulthandler.enable()

# Import and patch the register function to add error handling
from src.api.auth import register as original_register

async def debug_register(*args, **kwargs):
    try:
        print("DEBUG: Register function called with args:", args, "kwargs:", {k: v for k, v in kwargs.items() if k != 'db'})
        result = await original_register(*args, **kwargs)
        print("DEBUG: Register function succeeded with result:", type(result), str(result)[:100] if result else "None")
        return result
    except Exception as e:
        print("DEBUG: Register function failed with error:", str(e))
        import traceback
        traceback.print_exc()
        raise

# Replace the function
import src.api.auth
src.api.auth.register = debug_register

# Now run the server
import uvicorn
if __name__ == "__main__":
    uvicorn.run(
        "src.main:app",
        host="127.0.0.1",
        port=8000,
        log_level="debug",
        reload=False
    )