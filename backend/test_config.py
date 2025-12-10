import os
from dotenv import load_dotenv
load_dotenv()

# Test the configuration
from src.config.settings import settings
print(f"GEMINI_API_KEY from settings: {'*' * 10 if settings.GEMINI_API_KEY else 'NOT FOUND'}")
print(f"GEMINI_MODEL_NAME from settings: {settings.GEMINI_MODEL_NAME}")

# Test importing the agent runner to see if it works
try:
    from src.agent.agent_runner import agent
    print(f"Agent model: {agent.model}")
    print("Agent loaded successfully!")
except Exception as e:
    print(f"Error loading agent: {e}")
    import traceback
    traceback.print_exc()

# Test importing the main app
try:
    from src.main import app
    print("Main app loaded successfully!")
except Exception as e:
    print(f"Error loading main app: {e}")
    import traceback
    traceback.print_exc()