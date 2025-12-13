from typing import List, Dict, Any, Optional
import google.generativeai as genai
from ..config.settings import settings
import logging

logger = logging.getLogger(__name__)


class GeminiLLMClient:
    """Google Gemini LLM client for generating responses."""

    def __init__(self):
        print(f"DEBUG: Initializing GeminiLLMClient")
        self.api_key = settings.GOOGLE_GEMINI_API_KEY
        print(f"DEBUG: API key present: {bool(self.api_key)}")
        if not self.api_key:
            raise ValueError("GOOGLE_GEMINI_API_KEY environment variable is required")

        genai.configure(api_key=self.api_key)
        self.model_name = "gemini-1.5-flash"  # Use only this model
        self.temperature = settings.TEMPERATURE
        print(f"DEBUG: Model name: {self.model_name}")

        # Initialize the generative model with only gemini-1.5-flash
        try:
            self.model = genai.GenerativeModel(
                model_name=self.model_name,
                generation_config={
                    "temperature": self.temperature,
                    "max_output_tokens": 2048,
                }
            )
            print(f"DEBUG: Successfully initialized Gemini model: {self.model_name}")
            logger.info(f"Successfully initialized Gemini model: {self.model_name}")
        except Exception as e:
            print(f"DEBUG: Failed to initialize Gemini model '{self.model_name}': {str(e)}")
            logger.error(f"Failed to initialize Gemini model '{self.model_name}': {str(e)}")
            raise

    def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """
        Generate a response using Google Gemini.

        Args:
            prompt: The user's question/query
            context: Relevant context retrieved from RAG

        Returns:
            Generated response string
        """
        print(f"DEBUG generate_response: Model is None: {self.model is None}")
        if self.model:
            print(f"DEBUG generate_response: Model name being used: {self.model.model_name}")

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

        print(f"DEBUG generate_response: About to call generate_content with prompt length: {len(full_prompt)}")

        try:
            # Generate response
            response = self.model.generate_content(full_prompt)
            print(f"DEBUG generate_response: Got response object: {response is not None}")

            if response.text:
                result = response.text.strip()
                print(f"DEBUG generate_response: Returning response with length: {len(result)}")
                return result
            else:
                logger.warning("Gemini returned empty response")
                print("DEBUG generate_response: Response text is empty")
                return "I couldn't generate a response. Please try asking your question again."

        except Exception as e:
            print(f"DEBUG generate_response: Exception occurred: {e}")
            logger.error(f"Gemini API error: {str(e)}")
            # Return the real error message in development mode
            error_msg = f"Gemini API error: {str(e)}"
            print(f"DEBUG generate_response: Returning real error: {error_msg}")
            return error_msg

    def generate_chat_response(self, messages: List[Dict[str, str]], context: Optional[str] = None) -> str:
        """
        Generate a response for chat conversations.

        Args:
            messages: List of messages in the conversation (role, content)
            context: Relevant context retrieved from RAG

        Returns:
            Generated response string
        """
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

        try:
            # Generate response
            response = self.model.generate_content(full_prompt)

            if response.text:
                return response.text.strip()
            else:
                logger.warning("Gemini returned empty response for chat")
                return "I couldn't generate a response. Please try asking your question again."

        except Exception as e:
            logger.error(f"Gemini API error in chat: {str(e)}")
            # Return the real error message in development mode
            error_msg = f"Gemini API error in chat: {str(e)}"
            return error_msg

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
    print("DEBUG: initialize_gemini_client() function called")
    global gemini_client
    api_key_present = bool(settings.GOOGLE_GEMINI_API_KEY)
    print(f"DEBUG: GOOGLE_GEMINI_API_KEY present: {api_key_present}")
    print(f"DEBUG: Model name from settings: {settings.GEMINI_MODEL_NAME}")

    if settings.GOOGLE_GEMINI_API_KEY:
        try:
            print("DEBUG: Creating new GeminiLLMClient instance")
            gemini_client = GeminiLLMClient()
            # Log the actual model that was initialized
            if hasattr(gemini_client, 'model'):
                print(f"DEBUG: Successfully initialized Gemini client with model: {gemini_client.model.model_name}")
                logger.info(f"Initialized Gemini client with model: {gemini_client.model.model_name}")
            else:
                print(f"DEBUG: Initialized Gemini client with model: {settings.GEMINI_MODEL_NAME}")
                logger.info(f"Initialized Gemini client with model: {settings.GEMINI_MODEL_NAME}")
        except Exception as e:
            print(f"DEBUG: Failed to initialize Gemini client: {str(e)}")
            logger.error(f"Failed to initialize Gemini client: {e}")
            gemini_client = None
    else:
        print("DEBUG: No GOOGLE_GEMINI_API_KEY provided, Gemini client not initialized")
        logger.warning("No GOOGLE_GEMINI_API_KEY provided, Gemini client not initialized")
