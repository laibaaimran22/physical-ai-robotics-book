import os
import google.generativeai as genai
from typing import Dict, Optional
from ..config.settings import settings
from pydantic import BaseModel


class TranslationResult(BaseModel):
    translated_text: str
    source_language: str
    target_language: str
    success: bool
    error: Optional[str] = None


class TranslationService:
    def __init__(self):
        # Get API key from environment or settings
        self.api_key = (
            os.getenv("GOOGLE_GEMINI_API_KEY") or
            os.getenv("GEMINI_API_KEY") or
            getattr(settings, 'GOOGLE_GEMINI_API_KEY', None) or
            getattr(settings, 'GEMINI_API_KEY', None) or
            getattr(settings, 'GOOGLE_API_KEY', None)
        )

        if self.api_key:
            genai.configure(api_key=self.api_key)
            self.model = genai.GenerativeModel(settings.GEMINI_MODEL_NAME if hasattr(settings, 'GEMINI_MODEL_NAME') else "gemini-pro")
        else:
            self.model = None

    async def translate_text(self, text: str, target_language: str = "ur", source_language: str = "en") -> Dict:
        """
        Translate text from source language to target language using Google Gemini.

        Args:
            text: Text to translate
            target_language: Target language code (e.g., 'ur' for Urdu, 'es' for Spanish)
            source_language: Source language code (default 'en' for English)

        Returns:
            Dictionary with translation result
        """
        if not self.model:
            # Return original text as fallback when no API key is available
            return {
                "translated_text": text,
                "source_language": source_language,
                "target_language": target_language,
                "success": False,
                "error": "No API key configured for translation service"
            }

        try:
            # Create a translation prompt for the Gemini model
            prompt = f"""
            You are a professional translator. Translate the following text from {source_language} to {target_language}.
            Only return the translated text without any additional commentary or explanations.

            Text to translate:
            {text}
            """

            # Generate content using the Gemini model
            response = await self.model.generate_content_async(prompt)

            if response and hasattr(response, 'text') and response.text:
                return {
                    "translated_text": response.text.strip(),
                    "source_language": source_language,
                    "target_language": target_language,
                    "success": True,
                    "error": None
                }
            else:
                return {
                    "translated_text": text,  # Fallback to original text
                    "source_language": source_language,
                    "target_language": target_language,
                    "success": False,
                    "error": "No translation received from model"
                }

        except Exception as e:
            return {
                "translated_text": text,  # Fallback to original text
                "source_language": source_language,
                "target_language": target_language,
                "success": False,
                "error": str(e)
            }


# Global instance
_translation_service = None


def get_translation_service():
    global _translation_service
    if _translation_service is None:
        _translation_service = TranslationService()
    return _translation_service