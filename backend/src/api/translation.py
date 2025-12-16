from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy.ext.asyncio import AsyncSession
from pydantic import BaseModel
from typing import Optional
import logging
import httpx
import json
from urllib.parse import quote

from ..config.settings import settings
from ..api.deps import get_db_session
from ..models.user import User
from ..api.deps import get_optional_user

logger = logging.getLogger(__name__)

router = APIRouter()


class TranslationRequest(BaseModel):
    text: str
    target_language: str = "ur"
    source_language: str = "en"


class TranslationResponse(BaseModel):
    original_text: str
    translated_text: str
    source_language: str
    target_language: str


@router.post("/translation", response_model=TranslationResponse)
async def translate_text(
    request: TranslationRequest,
    db: AsyncSession = Depends(get_db_session),
    current_user: Optional[User] = Depends(get_optional_user)
):
    """
    Translate text from source language to target language.
    Tries MyMemory first (free service), then OpenRouter, then Google Gemini, then OpenAI as fallbacks.
    """
    # Try MyMemory first (free translation service)
    try:
        logger.info("Attempting translation with MyMemory...")
        # URL encode the text to handle Unicode/Urdu characters properly
        encoded_text = quote(request.text, safe='')
        mymemory_url = f"https://api.mymemory.translated.net/get?q={encoded_text}&langpair={request.source_language}|{request.target_language}"

        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.get(mymemory_url)

            logger.info(f"MyMemory response status: {response.status_code}")

            if response.status_code == 200:
                result = response.json()
                translated_text = result.get("responseData", {}).get("translatedText", "")

                if translated_text and translated_text != request.text:  # Make sure we got a real translation
                    logger.info(f"MyMemory translation successful: {len(translated_text)} characters")
                    return TranslationResponse(
                        original_text=request.text,
                        translated_text=translated_text,
                        source_language=request.source_language,
                        target_language=request.target_language
                    )
                else:
                    logger.warning("MyMemory returned empty or identical translation")
            else:
                logger.warning(f"MyMemory API request failed with status {response.status_code}: {response.text}")

    except Exception as mymemory_error:
        logger.error(f"MyMemory translation failed: {str(mymemory_error)}")
        # Continue to try OpenRouter as fallback

    # Try OpenRouter as second option
    if settings.OPENROUTER_API_KEY:
        try:
            logger.info("Attempting translation with OpenRouter...")
            # Prepare the translation prompt for OpenRouter
            prompt = f"""
            Translate the following content from {request.source_language} to {request.target_language}.
            Preserve the original formatting, structure, and meaning as closely as possible.
            Return only the translated content without any additional explanations.

            Content to translate:
            {request.text}
            """

            # Call OpenRouter API
            headers = {
                "Authorization": f"Bearer {settings.OPENROUTER_API_KEY}",
                "Content-Type": "application/json"
            }

            data = {
                "model": "openai/gpt-3.5-turbo",  # Using a widely available model on OpenRouter
                "messages": [
                    {"role": "system", "content": "You are a helpful translation assistant. Translate the provided text accurately."},
                    {"role": "user", "content": prompt}
                ],
                "temperature": 0.3,
                "max_tokens": 4000
            }

            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.post(
                    "https://openrouter.ai/api/v1/chat/completions",
                    headers=headers,
                    json=data
                )

                logger.info(f"OpenRouter response status: {response.status_code}")

                if response.status_code != 200:
                    logger.error(f"OpenRouter API error: {response.status_code} - {response.text}")
                    raise Exception(f"OpenRouter API error: {response.status_code} - {response.text}")

                result = response.json()
                translated_text = result["choices"][0]["message"]["content"].strip()
                logger.info(f"OpenRouter translation successful: {len(translated_text)} characters")

                return TranslationResponse(
                    original_text=request.text,
                    translated_text=translated_text,
                    source_language=request.source_language,
                    target_language=request.target_language
                )
        except Exception as openrouter_error:
            logger.error(f"OpenRouter translation failed: {str(openrouter_error)}")
            # Continue to fallback to Google Gemini

    # Try Google Gemini as third option
    api_key = settings.GOOGLE_GEMINI_API_KEY or settings.GEMINI_API_KEY

    if api_key:
        try:
            logger.info("Attempting translation with Google Gemini...")
            import google.generativeai as genai

            # Configure the Google Generative AI client
            genai.configure(api_key=api_key)

            # Use the configured model name from settings
            # For Google Gemini API, the model name should not have the "models/" prefix in the SDK
            model_name = settings.GEMINI_MODEL_NAME

            # Initialize the generative model
            model = genai.GenerativeModel(model_name)

            # Prepare the translation prompt
            prompt = f"""
            Translate the following content from {request.source_language} to {request.target_language}.
            Preserve the original formatting, structure, and meaning as closely as possible.
            Return only the translated content without any additional explanations.

            Content to translate:
            {request.text}
            """

            # Call the translation API using the native Google Generative AI SDK
            response = await model.generate_content_async(
                prompt,
                generation_config=genai.GenerationConfig(
                    temperature=0.3,
                    max_output_tokens=4000
                )
            )

            translated_text = response.text
            logger.info(f"Gemini translation successful: {len(translated_text)} characters")

            return TranslationResponse(
                original_text=request.text,
                translated_text=translated_text,
                source_language=request.source_language,
                target_language=request.target_language
            )
        except Exception as gemini_error:
            logger.error(f"Gemini translation failed: {str(gemini_error)}")
            # Continue to fallback to OpenAI

    # Fallback to OpenAI if all other options fail
    if settings.OPENAI_API_KEY:
        try:
            logger.info("Attempting translation with OpenAI...")
            # Prepare the translation prompt for OpenAI
            prompt = f"""
            Translate the following content from {request.source_language} to {request.target_language}.
            Preserve the original formatting, structure, and meaning as closely as possible.
            Return only the translated content without any additional explanations.

            Content to translate:
            {request.text}
            """

            # Call OpenAI API
            headers = {
                "Authorization": f"Bearer {settings.OPENAI_API_KEY}",
                "Content-Type": "application/json"
            }

            data = {
                "model": "gpt-3.5-turbo",  # Using a widely available model
                "messages": [
                    {"role": "system", "content": "You are a helpful translation assistant. Translate the provided text accurately."},
                    {"role": "user", "content": prompt}
                ],
                "temperature": 0.3,
                "max_tokens": 4000
            }

            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.post(
                    "https://api.openai.com/v1/chat/completions",
                    headers=headers,
                    json=data
                )

                logger.info(f"OpenAI response status: {response.status_code}")

                if response.status_code != 200:
                    logger.error(f"OpenAI API error: {response.status_code} - {response.text}")
                    raise Exception(f"OpenAI API error: {response.status_code} - {response.text}")

                result = response.json()
                translated_text = result["choices"][0]["message"]["content"].strip()
                logger.info(f"OpenAI translation successful: {len(translated_text)} characters")

                return TranslationResponse(
                    original_text=request.text,
                    translated_text=translated_text,
                    source_language=request.source_language,
                    target_language=request.target_language
                )
        except Exception as openai_error:
            logger.error(f"OpenAI translation failed: {str(openai_error)}")

    # If all services fail, return an appropriate error response
    return TranslationResponse(
        original_text=request.text,
        translated_text="Translation service is currently unavailable. Please try again later.",
        source_language=request.source_language,
        target_language=request.target_language
    )


# Add the translation router to main app
def register_routes(app):
    """Helper function to register translation routes"""
    from ..api.deps import get_db_session
    app.include_router(router, prefix="/api/v1", tags=["Translation"])