from typing import List, Optional, Dict, Any
import requests
import logging
import httpx
from abc import ABC, abstractmethod
from ..config.settings import settings


logger = logging.getLogger(__name__)


class BaseLLMClient(ABC):
    """Abstract base class for LLM clients."""

    @abstractmethod
    async def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """Generate a response based on the prompt and optional context."""
        pass


class OpenAILLMClient(BaseLLMClient):
    """LLM client for OpenAI-compatible APIs."""

    def __init__(self, api_key: str, base_url: str = "https://api.openai.com/v1", model: str = "gpt-3.5-turbo"):
        self.api_key = api_key
        self.base_url = base_url
        self.model = model
        self.headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }

    async def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """Generate response using OpenAI-compatible API."""
        try:
            # Prepare the full prompt with context if provided
            full_prompt = prompt
            if context:
                full_prompt = f"Context: {context}\n\nQuestion: {prompt}"

            data = {
                "model": self.model,
                "messages": [
                    {"role": "system", "content": "You are a helpful AI assistant. Provide accurate and helpful responses based on the provided context."},
                    {"role": "user", "content": full_prompt}
                ],
                "temperature": 0.7,
                "max_tokens": 1000
            }

            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.post(
                    f"{self.base_url}/chat/completions",
                    headers=self.headers,
                    json=data
                )

                if response.status_code != 200:
                    raise Exception(f"API error: {response.status_code} - {response.text}")

                result = response.json()
                return result["choices"][0]["message"]["content"].strip()

        except Exception as e:
            logger.error(f"OpenAI API error: {str(e)}")
            raise


class OpenRouterLLMClient(BaseLLMClient):
    """LLM client for OpenRouter API."""

    def __init__(self, api_key: str, model: str = "openai/gpt-3.5-turbo"):
        self.api_key = api_key
        self.model = model
        self.headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }

    async def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """Generate response using OpenRouter API."""
        try:
            # Prepare the full prompt with context if provided
            full_prompt = prompt
            if context:
                full_prompt = f"Context: {context}\n\nQuestion: {prompt}"

            data = {
                "model": self.model,
                "messages": [
                    {"role": "system", "content": "You are a helpful AI assistant. Provide accurate and helpful responses based on the provided context."},
                    {"role": "user", "content": full_prompt}
                ],
                "temperature": 0.7,
                "max_tokens": 1000
            }

            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.post(
                    "https://openrouter.ai/api/v1/chat/completions",
                    headers=self.headers,
                    json=data
                )

                if response.status_code != 200:
                    raise Exception(f"OpenRouter API error: {response.status_code} - {response.text}")

                result = response.json()
                return result["choices"][0]["message"]["content"].strip()

        except Exception as e:
            logger.error(f"OpenRouter API error: {str(e)}")
            raise


class GeminiLLMClient(BaseLLMClient):
    """LLM client for Google Gemini API."""

    def __init__(self, api_key: str, model_name: str = "gemini-1.5-flash"):
        try:
            import google.generativeai as genai
            self.api_key = api_key
            self.model_name = model_name
            genai.configure(api_key=api_key)
            self.model = genai.GenerativeModel(
                model_name=f"models/{model_name}",
                generation_config={
                    "temperature": 0.7,
                    "max_output_tokens": 1000,
                }
            )
        except ImportError:
            logger.error("Google Generative AI library not available. Please install it with 'pip install google-generativeai'")
            raise
        except Exception as e:
            logger.error(f"Failed to initialize Gemini client: {e}")
            raise

    async def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """Generate response using Google Gemini API."""
        try:
            # Prepare the full prompt with context if provided
            full_prompt = prompt
            if context:
                full_prompt = f"Context: {context}\n\nQuestion: {prompt}"

            response = await self.model.generate_content_async(
                full_prompt,
                generation_config=self.model.generation_config
            )

            if response.text:
                return response.text.strip()
            else:
                logger.warning("Gemini returned empty response")
                return "I couldn't generate a response. The model returned an empty result."

        except Exception as e:
            logger.error(f"Gemini API error: {str(e)}")
            raise


class FallbackLLMClient(BaseLLMClient):
    """Fallback LLM client that tries multiple providers in sequence."""

    def __init__(self):
        self.clients = []

        # Initialize clients based on available API keys
        if settings.OPENROUTER_API_KEY:
            try:
                self.clients.append(OpenRouterLLMClient(
                    api_key=settings.OPENROUTER_API_KEY,
                    model="openai/gpt-3.5-turbo"  # Using a reliable, widely available model
                ))
                logger.info("Initialized OpenRouter client as primary provider")
            except Exception as e:
                logger.warning(f"Failed to initialize OpenRouter client: {e}")

        if settings.OPENAI_API_KEY:
            try:
                self.clients.append(OpenAILLMClient(
                    api_key=settings.OPENAI_API_KEY,
                    model="gpt-3.5-turbo"
                ))
                logger.info("Initialized OpenAI client as fallback provider")
            except Exception as e:
                logger.warning(f"Failed to initialize OpenAI client: {e}")

        if settings.GOOGLE_GEMINI_API_KEY:
            try:
                self.clients.append(GeminiLLMClient(
                    api_key=settings.GOOGLE_GEMINI_API_KEY,
                    model_name=settings.GEMINI_MODEL_NAME
                ))
                logger.info("Initialized Gemini client as fallback provider")
            except Exception as e:
                logger.warning(f"Failed to initialize Gemini client: {e}")

    async def generate_response(self, prompt: str, context: Optional[str] = None) -> str:
        """Generate response by trying clients in sequence until one succeeds."""
        for i, client in enumerate(self.clients):
            try:
                logger.info(f"Trying LLM client {i+1}/{len(self.clients)}: {type(client).__name__}")
                response = await client.generate_response(prompt, context)

                # Check if the response indicates an error or fallback message
                if (response and
                    "error" not in response.lower() and
                    "currently unavailable" not in response.lower() and
                    "try again later" not in response.lower() and
                    "fallback" not in response.lower() and
                    "implementation" not in response.lower()):
                    return response
                else:
                    logger.info(f"Client {type(client).__name__} returned error/fallback response, trying next...")
                    continue
            except Exception as e:
                logger.warning(f"Client {type(client).__name__} failed: {e}")
                continue

        # If all clients fail, return a fallback message
        return "I'm sorry, but I couldn't generate a response at this time. The LLM service may not be available due to API quota limits or other issues. Please try again later."


# Global LLM client instance
_llm_client = None


def get_llm_client() -> BaseLLMClient:
    """Get the configured LLM client instance."""
    global _llm_client
    if _llm_client is None:
        _llm_client = FallbackLLMClient()
    return _llm_client


def initialize_llm_client():
    """Initialize the LLM client with the configured settings."""
    global _llm_client
    _llm_client = FallbackLLMClient()
    logger.info("LLM client initialized with fallback strategy (OpenRouter > OpenAI > Gemini)")