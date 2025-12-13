from typing import List, Optional
import numpy as np
import openai
from cohere import Client as CohereClient
from sentence_transformers import SentenceTransformer
from ..config.settings import settings
import logging


logger = logging.getLogger(__name__)


class EmbeddingClient:
    """Client for generating embeddings using various providers (OpenAI, Cohere, local)."""

    def __init__(self):
        self.model_name = settings.EMBEDDING_MODEL
        self.dimension = settings.EMBEDDING_DIMENSION

        # Initialize clients based on available API keys
        self.openai_client = None
        self.cohere_client = None
        self.local_model = None

        # Initialize OpenAI client if API key is provided
        if settings.OPENAI_API_KEY:
            try:
                self.openai_client = openai.OpenAI(api_key=settings.OPENAI_API_KEY)
                logger.info("Initialized OpenAI embedding client")
            except Exception as e:
                logger.warning(f"Failed to initialize OpenAI client: {e}")

        # Initialize Cohere client if API key is provided
        if settings.COHERE_API_KEY:
            try:
                self.cohere_client = CohereClient(api_key=settings.COHERE_API_KEY)
                logger.info("Initialized Cohere embedding client")
            except Exception as e:
                logger.warning(f"Failed to initialize Cohere client: {e}")

        # Initialize local model as fallback
        try:
            self.local_model = SentenceTransformer('all-MiniLM-L6-v2')
            logger.info("Initialized local embedding model: all-MiniLM-L6-v2")
        except Exception as e:
            logger.error(f"Failed to initialize local embedding model: {e}")
            self.local_model = None

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text using the best available provider.

        Args:
            text: Text to embed

        Returns:
            Embedding vector as a list of floats
        """
        # Try OpenAI first if available
        if self.openai_client:
            try:
                response = self.openai_client.embeddings.create(
                    input=text,
                    model=self.model_name
                )
                return response.data[0].embedding
            except Exception as e:
                logger.warning(f"OpenAI embedding failed: {e}")

        # Try Cohere next if available
        if self.cohere_client:
            try:
                response = self.cohere_client.embed(
                    texts=[text],
                    model=self.model_name
                )
                return response.embeddings[0]
            except Exception as e:
                logger.warning(f"Cohere embedding failed: {e}")

        # Fall back to local model
        if self.local_model:
            try:
                embedding = self.local_model.encode([text])
                return embedding[0].tolist()
            except Exception as e:
                logger.error(f"Local embedding failed: {e}")

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
        # Try OpenAI first if available
        if self.openai_client:
            try:
                response = self.openai_client.embeddings.create(
                    input=texts,
                    model=self.model_name
                )
                return [item.embedding for item in response.data]
            except Exception as e:
                logger.warning(f"OpenAI batch embedding failed: {e}")

        # Try Cohere next if available
        if self.cohere_client:
            try:
                response = self.cohere_client.embed(
                    texts=texts,
                    model=self.model_name
                )
                return response.embeddings
            except Exception as e:
                logger.warning(f"Cohere batch embedding failed: {e}")

        # Fall back to local model
        if self.local_model:
            try:
                embeddings = self.local_model.encode(texts)
                return [embedding.tolist() for embedding in embeddings]
            except Exception as e:
                logger.error(f"Local batch embedding failed: {e}")

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