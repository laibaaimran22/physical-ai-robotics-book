import os
from dotenv import load_dotenv
from src.main import app

# Load environment variables
load_dotenv()

if __name__ == "__main__":
    import uvicorn
    # Hugging Face Spaces expects the app to run on port 7860 by default
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", 7860)))