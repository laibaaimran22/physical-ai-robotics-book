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

# Create a mock client class for when API key is not available
class MockAsyncClient:
    async def chat(self):
        return MockChat()

    def __getattr__(self, name):
        return lambda *args, **kwargs: MockResponse()

class MockChat:
    def completions(self):
        return MockCompletions()

class MockCompletions:
    async def create(self, **kwargs):
        # Return a mock response with a helpful message
        content = "AI Assistant: This is a demo response. The actual AI service requires a valid API key to be set in the environment variables."
        return type('MockChoice', (), {'choices': [type('Choice', (), {'message': type('Message', (), {'content': content})()})]})()

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


# Create a mock agent runner that works without API key
class MockRunner:
    @staticmethod
    async def run_async(starting_agent: 'Agent', input_message: str):
        # Return a mock response for demo purposes
        final_output = f"Demo response for: {input_message}. To enable full AI functionality, please set your API key in the environment variables."
        return type('Result', (), {'final_output': final_output})()

    @staticmethod
    def run_sync(starting_agent: 'Agent', input_message: str):
        # Run the async function in a separate thread to avoid event loop conflicts
        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(
                lambda: asyncio.run(MockRunner.run_async(starting_agent, input_message))
            )
            return future.result()

# Initialize client based on whether API key is available
if GEMINI_API_KEY:
    client = openai.AsyncOpenAI(
        api_key=GEMINI_API_KEY,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    )
    RunnerClass = Runner
else:
    client = MockAsyncClient()
    RunnerClass = MockRunner
    print("WARNING: No API key found. Running in demo mode.")


# Initialize the agent once (not on every request)
agent = Agent(
    name="Assistant Agent",
    instructions="You're a helpful assistant for the Physical AI & Humanoid Robotics book. Answer questions about the book content and related topics.",
    model="gemini-pro-latest",  # Using the model name for OpenAI-compatible endpoint
    openai_client=client,
)


class AgentRequest(BaseModel):
    message: str