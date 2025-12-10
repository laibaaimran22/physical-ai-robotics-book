"""
Google Gemini API client specifically for translation functionality
"""
from typing import Optional
import openai
from ..config.settings import settings
import logging

logger = logging.getLogger(__name__)


class GeminiTranslationClient:
    """
    Google Gemini API client specifically designed for translation tasks
    Uses the OpenAI-compatible endpoint for Google Gemini
    """

    def __init__(self):
        # Get API key from settings - check both possible variable names
        self.api_key = (settings.GOOGLE_GEMINI_API_KEY or
                       settings.GEMINI_API_KEY)

        if not self.api_key:
            raise ValueError("GOOGLE_GEMINI_API_KEY or GEMINI_API_KEY setting is required")

        # Initialize OpenAI client with Google Gemini API
        try:
            self.client = openai.AsyncOpenAI(
                api_key=self.api_key,
                base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
            )
            self.model_name = "gemini-pro"  # Using the model name for OpenAI-compatible endpoint
        except Exception as e:
            logger.error(f"Failed to initialize Gemini client: {e}")
            raise

    async def translate(self, content: str, source_language: str = "en", target_language: str = "ur") -> str:
        """
        Translate content from source language to target language using Google Gemini

        Args:
            content: The content to translate
            source_language: Source language code (default: en)
            target_language: Target language code (default: ur for Urdu)

        Returns:
            Translated content as string
        """
        try:
            # Prepare the translation prompt
            prompt = f"""
            Translate the following content from {source_language} to {target_language}.
            Preserve the original formatting, structure, and meaning as closely as possible.
            Return only the translated content without any additional explanations.

            Content to translate:
            {content}
            """

            # Call the translation API
            response = await self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": "You are a professional translator. Translate content accurately while preserving formatting and structure."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=4000
            )

            translated_text = response.choices[0].message.content
            return translated_text

        except Exception as e:
            logger.error(f"Translation failed: {str(e)}")
            raise


# Global client instance
translation_client = None


def initialize_translation_client():
    """Initialize the translation client if API key is available."""
    global translation_client
    api_key = (settings.GOOGLE_GEMINI_API_KEY or settings.GEMINI_API_KEY)

    if api_key:
        try:
            translation_client = GeminiTranslationClient()
            logger.info("Initialized Gemini translation client")
        except Exception as e:
            logger.error(f"Failed to initialize translation client: {e}")
            translation_client = None
    else:
        logger.info("No Gemini API key provided, translation client will use fallback")


def get_translation_client():
    """Get or initialize the translation client lazily."""
    global translation_client
    # Only initialize if we have an API key and it hasn't been initialized yet
    api_key = (settings.GOOGLE_GEMINI_API_KEY or settings.GEMINI_API_KEY)
    if api_key and translation_client is None:
        try:
            translation_client = GeminiTranslationClient()
            logger.info("Lazy-initialized Gemini translation client")
        except Exception as e:
            logger.error(f"Failed to lazy-initialize translation client: {e}")
            translation_client = None
    return translation_client