from typing import List, Optional
import numpy as np
import openai
from ..config.settings import settings
import logging


logger = logging.getLogger(__name__)


class EmbeddingClient:
    """Client for generating embeddings using various providers (OpenAI, local fallback)."""

    def __init__(self):
        self.model_name = settings.EMBEDDING_MODEL
        # Use 384 dimensions to match the existing Qdrant collection
        # This ensures compatibility with the existing vector store
        self.dimension = 384

        # Initialize clients based on available API keys
        self.openai_client = None
        self.gemini_api_key = None
        self.local_model = None
        self.sentence_transformers_available = False

        # Initialize OpenAI client if API key is provided
        if settings.OPENAI_API_KEY:
            try:
                self.openai_client = openai.OpenAI(api_key=settings.OPENAI_API_KEY)
                logger.info("Initialized OpenAI embedding client")
            except Exception as e:
                logger.warning(f"Failed to initialize OpenAI client: {e}")

        # Initialize Gemini client if API key is provided
        gemini_api_key = settings.GOOGLE_GEMINI_API_KEY or settings.GEMINI_API_KEY
        if gemini_api_key:
            try:
                import google.generativeai as genai
                genai.configure(api_key=gemini_api_key)
                # For embedding generation, we'll use the embedding models
                self.gemini_api_key = gemini_api_key
                logger.info("Initialized Gemini embedding client")
            except Exception as e:
                logger.warning(f"Failed to initialize Gemini client: {e}")
                self.gemini_api_key = None

        # Try to initialize local model as fallback (only if sentence_transformers is available)
        try:
            from sentence_transformers import SentenceTransformer
            self.local_model = SentenceTransformer('all-MiniLM-L6-v2')
            self.sentence_transformers_available = True
            logger.info("Initialized local embedding model: all-MiniLM-L6-v2")
        except ImportError:
            logger.warning("sentence-transformers not available, local embeddings disabled")
            self.local_model = None
            self.sentence_transformers_available = False
        except Exception as e:
            logger.error(f"Failed to initialize local embedding model: {e}")
            self.local_model = None
            self.sentence_transformers_available = False

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text using the best available provider.

        Args:
            text: Text to embed

        Returns:
            Embedding vector as a list of floats
        """
        # Try OpenAI first if available (primary method for Hugging Face Spaces)
        if self.openai_client:
            try:
                response = self.openai_client.embeddings.create(
                    input=text,
                    model=self.model_name
                )
                result = response.data[0].embedding
                # Ensure the result has the correct dimension
                if len(result) != self.dimension:
                    # If dimensions don't match, truncate or pad as needed
                    if len(result) > self.dimension:
                        result = result[:self.dimension]
                    else:
                        result.extend([0.0] * (self.dimension - len(result)))
                return result
            except Exception as e:
                logger.warning(f"OpenAI embedding failed: {e}")

        # Try local model if available and sentence_transformers is present
        if self.local_model and self.sentence_transformers_available:
            try:
                embedding = self.local_model.encode([text])
                result = embedding[0].tolist()
                # Ensure the result has the correct dimension
                if len(result) != self.dimension:
                    # If dimensions don't match, truncate or pad as needed
                    if len(result) > self.dimension:
                        result = result[:self.dimension]
                    else:
                        result.extend([0.0] * (self.dimension - len(result)))
                return result
            except Exception as e:
                logger.warning(f"Local embedding failed: {e}")

        # Try Gemini next if available
        if self.gemini_api_key:
            try:
                import google.generativeai as genai

                # Use the embedding generation function
                # Google's embedding models are different from the generative models
                result = genai.embed_content(
                    model="models/text-embedding-004",  # Google's latest text embedding model
                    content=text,
                    task_type="retrieval_document"  # or "retrieval_query" for queries
                )
                embedding = result['embedding']
                # Ensure the result has the correct dimension (384)
                if len(embedding) != self.dimension:
                    # If dimensions don't match, truncate or pad as needed
                    if len(embedding) > self.dimension:
                        embedding = embedding[:self.dimension]
                    else:
                        embedding.extend([0.0] * (self.dimension - len(embedding)))
                return embedding
            except Exception as e:
                logger.warning(f"Gemini embedding failed: {e}")

        # If all methods fail, return a zero vector (this should ideally not happen in production)
        logger.error("All embedding methods failed, returning zero vector")
        return [0.0] * self.dimension

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using the best available provider.

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors
        """
        # Try OpenAI first if available (primary method for Hugging Face Spaces)
        if self.openai_client:
            try:
                response = self.openai_client.embeddings.create(
                    input=texts,
                    model=self.model_name
                )
                results = [item.embedding for item in response.data]
                # Ensure all results have the correct dimension
                for i, result in enumerate(results):
                    if len(result) != self.dimension:
                        # If dimensions don't match, truncate or pad as needed
                        if len(result) > self.dimension:
                            results[i] = result[:self.dimension]
                        else:
                            results[i].extend([0.0] * (self.dimension - len(result)))
                return results
            except Exception as e:
                logger.warning(f"OpenAI batch embedding failed: {e}")

        # Try local model if available and sentence_transformers is present
        if self.local_model and self.sentence_transformers_available:
            try:
                embeddings = self.local_model.encode(texts)
                results = [embedding.tolist() for embedding in embeddings]
                # Ensure all results have the correct dimension
                for i, result in enumerate(results):
                    if len(result) != self.dimension:
                        # If dimensions don't match, truncate or pad as needed
                        if len(result) > self.dimension:
                            results[i] = result[:self.dimension]
                        else:
                            results[i].extend([0.0] * (self.dimension - len(result)))
                return results
            except Exception as e:
                logger.warning(f"Local batch embedding failed: {e}")

        # Try Gemini next if available
        if self.gemini_api_key:
            try:
                import google.generativeai as genai

                # Process embeddings one by one since genai.embed_content doesn't support batch
                embeddings = []
                for text in texts:
                    result = genai.embed_content(
                        model="models/text-embedding-004",  # Google's latest text embedding model
                        content=text,
                        task_type="retrieval_document"  # or "retrieval_query" for queries
                    )
                    embedding = result['embedding']
                    # Ensure the result has the correct dimension (384)
                    if len(embedding) != self.dimension:
                        # If dimensions don't match, truncate or pad as needed
                        if len(embedding) > self.dimension:
                            embedding = embedding[:self.dimension]
                        else:
                            embedding.extend([0.0] * (self.dimension - len(embedding)))
                    embeddings.append(embedding)
                return embeddings
            except Exception as e:
                logger.warning(f"Gemini batch embedding failed: {e}")

        # If all methods fail, return zero vectors
        logger.error("All batch embedding methods failed, returning zero vectors")
        return [[0.0] * self.dimension for _ in texts]

    def get_embedding_dimension(self) -> int:
        """
        Get the dimension of the embeddings.

        Returns:
            Dimension of the embedding vectors
        """
        return self.dimension

    def similarity(self, embedding1: List[float], embedding2: List[float]) -> float:
        """
        Calculate cosine similarity between two embeddings.

        Args:
            embedding1: First embedding vector
            embedding2: Second embedding vector

        Returns:
            Cosine similarity score
        """
        # Convert to numpy arrays
        emb1 = np.array(embedding1)
        emb2 = np.array(embedding2)

        # Calculate cosine similarity
        dot_product = np.dot(emb1, emb2)
        norm1 = np.linalg.norm(emb1)
        norm2 = np.linalg.norm(emb2)

        if norm1 == 0 or norm2 == 0:
            return 0.0

        return float(dot_product / (norm1 * norm2))


# Global embedding client instance
embedding_client = EmbeddingClient()