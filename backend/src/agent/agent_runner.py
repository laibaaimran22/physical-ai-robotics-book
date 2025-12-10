import os
import asyncio
import concurrent.futures
from typing import Any
from pydantic import BaseModel
import openai
from dotenv import load_dotenv


# Load environment variables
load_dotenv()

# Try to get API key from environment or settings
GEMINI_API_KEY = os.getenv("GOOGLE_GEMINI_API_KEY") or os.getenv("GEMINI_API_KEY")

# Import settings to check for API key there as well
try:
    from ..config.settings import settings
    if not GEMINI_API_KEY:
        GEMINI_API_KEY = settings.GOOGLE_GEMINI_API_KEY or settings.GEMINI_API_KEY
except ImportError:
    pass

if not GEMINI_API_KEY:
    raise ValueError("GOOGLE_GEMINI_API_KEY or GEMINI_API_KEY environment variable or setting is required")

client = openai.AsyncOpenAI(
    api_key=GEMINI_API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)


class Agent:
    def __init__(self, name: str, instructions: str, model: str, openai_client):
        self.name = name
        self.instructions = instructions
        self.model = model
        self.openai_client = openai_client


class Runner:
    @staticmethod
    async def run_async(starting_agent: Agent, input_message: str):
        try:
            # Combine instructions with the user input
            messages = [
                {"role": "system", "content": starting_agent.instructions},
                {"role": "user", "content": input_message}
            ]

            # Call the OpenAI-compatible API
            response = await starting_agent.openai_client.chat.completions.create(
                model=starting_agent.model,
                messages=messages,
                temperature=0.7
            )

            final_output = response.choices[0].message.content
            return type('Result', (), {'final_output': final_output})()
        except Exception as e:
            raise e

    @staticmethod
    def run_sync(starting_agent: Agent, input_message: str):
        # Run the async function in a separate thread to avoid event loop conflicts
        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(
                lambda: asyncio.run(Runner.run_async(starting_agent, input_message))
            )
            return future.result()


# Initialize the agent once (not on every request)
agent = Agent(
    name="Assistant Agent",
    instructions="You're a helpful assistant for the Physical AI & Humanoid Robotics book. Answer questions about the book content and related topics.",
    model="gemini-pro-latest",  # Using the model name for OpenAI-compatible endpoint
    openai_client=client,
)


class AgentRequest(BaseModel):
    message: str