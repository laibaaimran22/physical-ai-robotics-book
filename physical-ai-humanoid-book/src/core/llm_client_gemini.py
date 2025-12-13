from typing import List, Dict, Any, Optional
import google.generativeai as genai
from ..config.settings import settings
import logging

logger = logging.getLogger(__name__)


class GeminiLLMClient:
    """Google Gemini LLM client for generating responses."""

    def __init__(self):
        self.api_key = settings.GOOGLE_GEMINI_API_KEY
        if not self.api_key:
            raise ValueError("GOOGLE_GEMINI_API_KEY environment variable is required")

        genai.configure(api_key=self.api_key)
        self.model_name = settings.GEMINI_MODEL_NAME
        self.temperature = settings.TEMPERATURE

        # Initialize the generative model
        # Use appropriate model name for native Google Generative AI library
        native_model_name = self.model_name
        if native_model_name == "gemini-pro-latest":
            native_model_name = "gemini-2.5-flash"  # Map to available native model
        elif native_model_name == "gemini-pro":
            native_model_name = "gemini-2.5-flash"  # Map to available native model
        elif not native_model_name or native_model_name not in ["gemini-pro", "gemini-1.0-pro", "gemini-1.5-pro", "gemini-pro-latest"]:
            native_model_name = "gemini-2.5-flash"  # Default to available model

        self.model = genai.GenerativeModel(
            model_name=native_model_name,
            generation_config={
                "temperature": self.temperature,
                "max_output_tokens": 2048,
            }
        )

    def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """
        Generate a response using Google Gemini.

        Args:
            prompt: The user's question/query
            context: Relevant context retrieved from RAG

        Returns:
            Generated response string
        """
        try:
            # Construct the full prompt with context if provided
            if context:
                full_prompt = f"""
                Context information:
                {context}

                Question: {prompt}

                Please provide a helpful and accurate response based on the context information provided above.
                """
            else:
                full_prompt = prompt

            # Generate response
            response = self.model.generate_content(full_prompt)

            if response.text:
                return response.text.strip()
            else:
                logger.warning("Gemini returned empty response")
                return "I couldn't generate a response. Please try asking your question again."

        except Exception as e:
            logger.error(f"Gemini API error: {str(e)}")
            return "I encountered an error while generating a response. Please try again."

    def generate_chat_response(self, messages: List[Dict[str, str]], context: Optional[str] = None) -> str:
        """
        Generate a response for chat conversations.

        Args:
            messages: List of messages in the conversation (role, content)
            context: Relevant context retrieved from RAG

        Returns:
            Generated response string
        """
        try:
            # Extract the user's last message as the prompt
            user_prompt = ""
            for msg in reversed(messages):
                if msg.get("role") == "user":
                    user_prompt = msg.get("content", "")
                    break

            # Construct the full prompt with context if provided
            if context:
                full_prompt = f"""
                Context information:
                {context}

                Previous conversation:
                {self.format_conversation_history(messages)}

                Question: {user_prompt}

                Please provide a helpful and accurate response based on the context information and conversation history provided above.
                """
            else:
                full_prompt = f"{self.format_conversation_history(messages)}\n\n{user_prompt}"

            # Generate response
            response = self.model.generate_content(full_prompt)

            if response.text:
                return response.text.strip()
            else:
                logger.warning("Gemini returned empty response for chat")
                return "I couldn't generate a response. Please try asking your question again."

        except Exception as e:
            logger.error(f"Gemini API error in chat: {str(e)}")
            return "I encountered an error while generating a response. Please try again."

    def format_conversation_history(self, messages: List[Dict[str, str]]) -> str:
        """Format conversation history for context."""
        formatted = []
        for msg in messages[-5:]:  # Use last 5 messages as context
            role = msg.get("role", "user")
            content = msg.get("content", "")
            formatted.append(f"{role.capitalize()}: {content}")
        return "\n".join(formatted)


# Global Gemini client instance
gemini_client = None

def initialize_gemini_client():
    """Initialize the Gemini client if API key is available."""
    global gemini_client
    if settings.GOOGLE_GEMINI_API_KEY:
        try:
            gemini_client = GeminiLLMClient()
            logger.info(f"Initialized Gemini client with model: {settings.GEMINI_MODEL_NAME}")
        except Exception as e:
            logger.error(f"Failed to initialize Gemini client: {e}")
            gemini_client = None
    else:
        logger.warning("No GOOGLE_GEMINI_API_KEY provided, Gemini client not initialized")