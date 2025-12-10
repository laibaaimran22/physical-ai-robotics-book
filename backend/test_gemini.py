import os
import asyncio
from dotenv import load_dotenv
load_dotenv()

# Test the Gemini API key directly using the native Google Generative AI library
import google.generativeai as genai

# Get the API key from environment
GEMINI_API_KEY = (os.getenv("GOOGLE_GEMINI_API_KEY") or os.getenv("GEMINI_API_KEY"))
print(f"GEMINI_API_KEY found: {bool(GEMINI_API_KEY)}")

if GEMINI_API_KEY:
    try:
        # Configure the API key
        genai.configure(api_key=GEMINI_API_KEY)

        # List available models to see what's supported
        print("Available models:")
        for model in genai.list_models():
            print(f"  - {model.name}")

        # Try to create a generative model instance with an available model
        model = genai.GenerativeModel('gemini-pro-latest')

        print("Testing Gemini API connection with native library...")
        response = model.generate_content("Hello, how are you?")

        if response.text:
            print(f"Success! Response: {response.text[:100]}...")
        else:
            print("Got empty response from Gemini")

    except Exception as e:
        print(f"Error testing Gemini API with native library: {e}")
        import traceback
        traceback.print_exc()
else:
    print("No GEMINI_API_KEY found in environment")