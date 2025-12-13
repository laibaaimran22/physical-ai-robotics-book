#!/usr/bin/env python3
"""Debug server with detailed error logging"""
import logging
import uvicorn
import sys
import os
from pathlib import Path

# Set up detailed logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Add the src directory to the Python path so we can import modules
sys.path.insert(0, str(Path(__file__).parent))

if __name__ == "__main__":
    # Set environment variable to make errors more visible
    os.environ['PYTHONUNBUFFERED'] = '1'

    # Start the uvicorn server with detailed logging
    uvicorn.run(
        "src.main:app",
        host="127.0.0.1",
        port=8000,
        reload=False,
        log_level="debug",
        access_log=True
    )