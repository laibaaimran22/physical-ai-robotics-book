import os
from dotenv import load_dotenv
load_dotenv()

# Test the agent directly
from src.agent.agent_runner import agent, Runner

print(f"Testing agent with model: {agent.model}")
print(f"Agent instructions: {agent.instructions[:50]}...")

try:
    # Test the agent with a simple message
    result = Runner.run_sync(starting_agent=agent, input_message="Hello, how are you?")
    print(f"Agent response: {result.final_output}")
    print("Agent test successful!")
except Exception as e:
    print(f"Error running agent: {e}")
    import traceback
    traceback.print_exc()