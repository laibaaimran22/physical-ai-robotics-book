import os
from dotenv import load_dotenv
from src.main import app

# Load environment variables
load_dotenv()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", 8000)))