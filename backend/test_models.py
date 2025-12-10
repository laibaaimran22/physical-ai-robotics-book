import os
import openai
from dotenv import load_dotenv
load_dotenv()

# Test the OpenAI-compatible endpoint to see what models are available
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
client = openai.AsyncOpenAI(
    api_key=GEMINI_API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

async def list_models():
    try:
        print("Trying to list models from OpenAI-compatible endpoint...")
        models = await client.models.list()
        print("Available models:")
        for model in models.data:
            print(f"  - {model.id}")
    except Exception as e:
        print(f"Error listing models: {e}")
        print("This suggests that the models.list() endpoint might not be supported")
        # Instead, let's try a few common model names
        common_models = [
            "gemini-pro",
            "gpt-3.5-turbo",  # fallback to check if endpoint works
            "gemini-1.0-pro",
            "gemini-1.0-pro-001",
            "gemini-1.5-pro-latest",
            "gemini-1.5-flash"
        ]

        for model_name in common_models:
            try:
                print(f"Testing model: {model_name}")
                response = await client.chat.completions.create(
                    model=model_name,
                    messages=[{"role": "user", "content": "Hi"}],
                    max_tokens=10
                )
                print(f"  SUCCESS: {model_name} works!")
                break
            except Exception as e:
                print(f"  FAILED: {model_name} - {str(e)[:100]}...")

import asyncio
asyncio.run(list_models())