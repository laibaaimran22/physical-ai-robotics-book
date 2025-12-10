from typing import List, Optional, Dict, Any
import requests
import logging
import torch
from ..config.settings import settings


# Try to import transformers components, but make them optional
try:
    from transformers import pipeline, AutoTokenizer
    TRANSFORMERS_AVAILABLE = True
except ImportError:
    pipeline = None
    AutoTokenizer = None
    TRANSFORMERS_AVAILABLE = False
    logger = logging.getLogger(__name__)
    logger.warning("Transformers library not available. Only Ollama will work.")

# Try to import OpenAI, but make it optional
try:
    import openai
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False
    logger = logging.getLogger(__name__)
    logger.warning("OpenAI library not available. OpenAI features will be disabled.")

# Try to import Google Generative AI, but make it optional
try:
    import google.generativeai as genai
    GEMINI_AVAILABLE = True
except ImportError:
    GEMINI_AVAILABLE = False
    logger = logging.getLogger(__name__)
    logger.warning("Google Generative AI library not available. Gemini features will be disabled.")


logger = logging.getLogger(__name__)


class LLMClient:
    """Client for interacting with various LLM models (OpenAI, Ollama, Hugging Face transformers, Google Gemini)."""

    def __init__(self):
        self.use_ollama = True  # Default to Ollama
        self.ollama_url = settings.OLLAMA_BASE_URL
        self.model_name = settings.OLLAMA_MODEL if not settings.OPENAI_API_KEY else settings.LLM_MODEL
        self.hf_pipeline = None
        self.tokenizer = None
        self.openai_client = None
        self.gemini_model = None

        # Initialize OpenAI client if available and API key is set
        if OPENAI_AVAILABLE and settings.OPENAI_API_KEY:
            try:
                self.openai_client = OpenAI(api_key=settings.OPENAI_API_KEY)
                logger.info("Initialized OpenAI client")
            except Exception as e:
                logger.warning(f"Failed to initialize OpenAI client: {e}")
                self.openai_client = None

        # Initialize Google Gemini client if available and API key is set
        if GEMINI_AVAILABLE and settings.GOOGLE_GEMINI_API_KEY:
            try:
                genai.configure(api_key=settings.GOOGLE_GEMINI_API_KEY)
                self.gemini_model = genai.GenerativeModel(
                    model_name=settings.GEMINI_MODEL_NAME,
                    generation_config={
                        "temperature": settings.TEMPERATURE,
                        "max_output_tokens": 2048,
                    }
                )
                logger.info(f"Initialized Google Gemini client with model: {settings.GEMINI_MODEL_NAME}")
            except Exception as e:
                logger.warning(f"Failed to initialize Google Gemini client: {e}")
                self.gemini_model = None

        # Try to initialize Hugging Face model as fallback if transformers is available
        if TRANSFORMERS_AVAILABLE:
            try:
                self.tokenizer = AutoTokenizer.from_pretrained(self.model_name)
                self.hf_pipeline = pipeline(
                    "text-generation",
                    model=self.model_name,
                    tokenizer=self.tokenizer,
                    torch_dtype=torch.float16 if torch.cuda.is_available() else None,
                    device_map="auto" if torch.cuda.is_available() else None
                )
                logger.info(f"Initialized Hugging Face pipeline with model: {self.model_name}")
            except Exception as e:
                logger.warning(f"Failed to initialize Hugging Face model {self.model_name}: {e}")
                # Try a default model
                try:
                    default_model = "microsoft/DialoGPT-medium"
                    self.tokenizer = AutoTokenizer.from_pretrained(default_model)
                    self.hf_pipeline = pipeline(
                        "text-generation",
                        model=default_model,
                        tokenizer=self.tokenizer
                    )
                    logger.info(f"Initialized Hugging Face pipeline with default model: {default_model}")
                except Exception as e2:
                    logger.error(f"Failed to initialize default Hugging Face model: {e2}")
                    self.hf_pipeline = None
        else:
            logger.info("Transformers not available, will use Ollama/OpenAI only")

    def generate_response(self, prompt: str, max_tokens: int = 500, temperature: float = 0.7) -> str:
        """
        Generate a response using available LLM.

        Args:
            prompt: Input prompt for the model
            max_tokens: Maximum tokens to generate
            temperature: Sampling temperature

        Returns:
            Generated response text
        """
        # First try Google Gemini if available
        if self.gemini_model:
            try:
                # Construct the full prompt
                full_prompt = f"{prompt}\n\nPlease provide a helpful and accurate response."

                response = self.gemini_model.generate_content(
                    full_prompt,
                    generation_config={
                        "max_output_tokens": max_tokens,
                        "temperature": temperature
                    }
                )

                if response.text:
                    return response.text.strip()
                else:
                    logger.warning("Gemini returned empty response")
            except Exception as e:
                logger.warning(f"Gemini request failed: {e}")

        # Then try OpenAI if available
        if self.openai_client:
            try:
                response = self.openai_client.chat.completions.create(
                    model=settings.OPENAI_MODEL,
                    messages=[{"role": "user", "content": prompt}],
                    max_tokens=max_tokens,
                    temperature=temperature
                )
                return response.choices[0].message.content
            except Exception as e:
                logger.warning(f"OpenAI request failed: {e}")

        # Then try Ollama
        if self._check_ollama_available():
            try:
                response = requests.post(
                    f"{self.ollama_url}/api/generate",
                    json={
                        "model": self.model_name,
                        "prompt": prompt,
                        "stream": False,
                        "options": {
                            "temperature": temperature,
                            "num_predict": max_tokens
                        }
                    },
                    timeout=30
                )
                if response.status_code == 200:
                    result = response.json()
                    return result.get("response", "")
            except Exception as e:
                logger.warning(f"Ollama request failed: {e}")

        # Fallback to Hugging Face transformers
        if self.hf_pipeline:
            try:
                # Truncate prompt if too long
                inputs = self.tokenizer.encode(prompt, return_tensors="pt", truncation=True, max_length=1024)

                with torch.no_grad():  # Disable gradient computation for inference
                    outputs = self.hf_pipeline(
                        self.tokenizer.decode(inputs[0], skip_special_tokens=True),
                        max_new_tokens=max_tokens,
                        temperature=temperature,
                        do_sample=True,
                        pad_token_id=self.tokenizer.eos_token_id
                    )

                # Extract just the generated part (not the original prompt)
                full_text = outputs[0]['generated_text']
                generated_text = full_text[len(prompt):].strip()

                return generated_text
            except Exception as e:
                logger.error(f"Hugging Face pipeline failed: {e}")

        # If all methods fail, return a default response
        return "I'm sorry, but I couldn't generate a response at this time. The LLM service may not be available."

    def chat(self, messages: List[Dict[str, str]], max_tokens: int = 500, temperature: float = 0.7) -> str:
        """
        Generate a chat response from a list of messages.

        Args:
            messages: List of messages in the format {"role": "user"/"assistant", "content": "message"}
            max_tokens: Maximum tokens to generate
            temperature: Sampling temperature

        Returns:
            Generated response text
        """
        # First try Google Gemini if available
        if self.gemini_model:
            try:
                # Format the conversation for Gemini
                formatted_conversation = ""
                for msg in messages:
                    role = msg.get("role", "user")
                    content = msg.get("content", "")
                    formatted_conversation += f"{role.capitalize()}: {content}\n"

                # Add instruction for assistant response
                full_prompt = f"{formatted_conversation}\nAssistant:"

                response = self.gemini_model.generate_content(
                    full_prompt,
                    generation_config={
                        "max_output_tokens": max_tokens,
                        "temperature": temperature
                    }
                )

                if response.text:
                    return response.text.strip()
                else:
                    logger.warning("Gemini returned empty response for chat")
            except Exception as e:
                logger.warning(f"Gemini chat request failed: {e}")

        # Then try OpenAI if available (it has better chat support)
        if self.openai_client:
            try:
                # Convert message format to match OpenAI requirements
                openai_messages = []
                for msg in messages:
                    role = msg.get("role", "user")
                    content = msg.get("content", "")
                    # Map assistant to assistant, user stays as user, and system as system
                    if role in ["user", "assistant", "system"]:
                        openai_messages.append({"role": role, "content": content})
                    else:
                        # Default to user if not a recognized role
                        openai_messages.append({"role": "user", "content": f"{role.capitalize()}: {content}"})

                response = self.openai_client.chat.completions.create(
                    model=settings.OPENAI_MODEL,
                    messages=openai_messages,
                    max_tokens=max_tokens,
                    temperature=temperature
                )
                return response.choices[0].message.content
            except Exception as e:
                logger.warning(f"OpenAI chat request failed: {e}")

        # Fallback to the original method for other LLMs
        # Format messages into a single prompt
        formatted_prompt = ""
        for message in messages:
            role = message.get("role", "user")
            content = message.get("content", "")
            formatted_prompt += f"{role.capitalize()}: {content}\n"

        formatted_prompt += "Assistant:"

        return self.generate_response(formatted_prompt, max_tokens, temperature)

    def _check_ollama_available(self) -> bool:
        """Check if Ollama service is available."""
        try:
            response = requests.get(f"{self.ollama_url}/api/tags", timeout=5)
            return response.status_code == 200
        except:
            return False

    def get_model_info(self) -> Dict[str, Any]:
        """Get information about the current model."""
        info = {
            "model_name": self.model_name,
            "available": False
        }

        if self.openai_client:
            info["backend"] = "openai"
            info["available"] = True
        elif self._check_ollama_available():
            info["backend"] = "ollama"
            info["available"] = True
        elif self.hf_pipeline:
            info["backend"] = "huggingface"
            info["available"] = True
        else:
            info["backend"] = "none"
            info["available"] = False

        return info


# Global LLM client instance
llm_client = LLMClient()