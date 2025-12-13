#!/usr/bin/env python3
"""Test script to run server with detailed logging"""
import uvicorn
import os
import sys

# Force UTF-8 encoding
os.environ['PYTHONIOENCODING'] = 'utf-8'
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')
if hasattr(sys.stderr, 'reconfigure'):
    sys.stderr.reconfigure(encoding='utf-8')

if __name__ == "__main__":
    # Add the backend directory to the path
    import os
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent))

    uvicorn.run(
        "backend.src.main:app",
        host="127.0.0.1",
        port=8000,
        log_level="debug",
        reload=False
    )