import os
import asyncio
import concurrent.futures
from typing import Any
from pydantic import BaseModel
from dotenv import load_dotenv


# Load environment variables
load_dotenv()

# Import the new configurable LLM client
from ..core.llm_client import get_llm_client


class Agent:
    def __init__(self, name: str, instructions: str, model: str = None):
        self.name = name
        self.instructions = instructions
        self.model = model
        # Get the configurable LLM client instead of the OpenAI client
        self.llm_client = get_llm_client()


class Runner:
    @staticmethod
    async def run_async(starting_agent: Agent, input_message: str):
        try:
            # Use the configurable LLM client instead of the OpenAI client
            # Combine instructions with the user input
            prompt = f"{starting_agent.instructions}\n\nUser: {input_message}\n\nAssistant:"

            # Use the configurable LLM client to generate the response
            final_output = await starting_agent.llm_client.generate_response(prompt)
            return type('Result', (), {'final_output': final_output})
        except Exception as e:
            error_msg = str(e)
            if "No API key configured" in error_msg or "quota" in error_msg.lower() or "rate limit" in error_msg.lower():
                return type('Result', (), {'final_output': "Chat functionality is currently unavailable. Please try again later."})
            else:
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
try:
    agent = Agent(
        name="Assistant Agent",
        instructions="You're a helpful assistant for the Physical AI & Humanoid Robotics book. Answer questions about the book content and related topics.",
    )
except Exception as e:
    print(f"Error initializing agent: {e}")
    # Create a mock agent that returns an error message
    class MockAgent:
        def __init__(self, name, instructions, model=None):
            self.name = name
            self.instructions = instructions
            self.model = model
            self.llm_client = get_llm_client()

    agent = MockAgent(
        name="Assistant Agent",
        instructions="You're a helpful assistant for the Physical AI & Humanoid Robotics book. Answer questions about the book content and related topics.",
    )


class AgentRequest(BaseModel):
    message: str