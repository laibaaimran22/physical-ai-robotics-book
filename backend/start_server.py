import uvicorn
import sys
import os
from pathlib import Path

# Add the src directory to the Python path so we can import modules
sys.path.insert(0, str(Path(__file__).parent))

if __name__ == "__main__":
    # Start the uvicorn server with the correct module path
    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        reload_dirs=["src"],
        log_level="info"
    )