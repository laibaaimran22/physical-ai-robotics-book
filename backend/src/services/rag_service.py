from typing import List, Dict, Any, Optional
from sqlalchemy.ext.asyncio import AsyncSession
import time
from datetime import datetime
import asyncio
import logging

from ..core.embedding_client import embedding_client
from ..core.vector_store import get_vector_store
from ..models.rag_query import RAGQuery
from ..models.chat_history import ChatHistory
from ..database.crud.rag_query import create_rag_query
from ..database.crud.chat_history import create_chat_history
from ..config.settings import settings
from .personalization_service import get_personalization_service, UserProfile

logger = logging.getLogger(__name__)


class RAGService:
    """Service for handling RAG (Retrieval Augmented Generation) functionality."""

    def __init__(self, db: AsyncSession):
        self.db = db
        self.personalization_service = get_personalization_service()

    async def query_rag(self, query: str, user_id: Optional[int] = None, user_background: Optional[Dict[str, Any]] = None, top_k: int = 5) -> Dict[str, Any]:
        """
        Query the RAG system to get contextually relevant responses.

        Args:
            query: User's query
            user_id: ID of the user making the query (optional)
            user_background: User's background information for personalization (optional)
            top_k: Number of top results to retrieve from vector store

        Returns:
            Dictionary with response and context information
        """
        start_time = time.time()

        # Generate embedding for the query
        query_embedding = embedding_client.generate_embedding(query)

        # Search for similar content in the vector store
        vector_store = get_vector_store()
        similar_chunks = await vector_store.search_similar(
            query_vector=query_embedding,
            limit=top_k
        )

        if not similar_chunks:
            response = "I couldn't find any relevant information to answer your question."
        else:
            # Prepare context from similar chunks
            context = "\n\n".join([chunk["content"] for chunk in similar_chunks])

            print(f"DEBUG RAG: Creating personalized prompt with user_background: {user_background}")
            # Prepare the prompt with context and user background for personalization
            prompt = self._create_personalized_prompt(query, context, user_background)
            print(f"DEBUG RAG: Generated personalized prompt, length: {len(prompt)}")

            # Generate response using the configurable LLM client with fallbacks
            from ..core.llm_client import get_llm_client

            try:
                llm_client = get_llm_client()
                response_text = await llm_client.generate_response(prompt, context)
                print(f"DEBUG RAG: Generated response using configurable LLM client: {bool(response_text)}")

                # If the response indicates an error, log it but still return it
                if "error" in response_text.lower() or "couldn't generate" in response_text.lower() or "currently experiencing high demand" in response_text.lower():
                    print(f"DEBUG RAG: Response contained error message: {response_text}")
            except Exception as e:
                print(f"DEBUG RAG: Configurable LLM client failed: {e}")
                logger.error(f"Configurable LLM client failed: {e}")
                # Fallback response
                response_text = f"Based on the context provided, I found {len(similar_chunks)} relevant pieces of information. In a full implementation with an LLM configured, I would generate a comprehensive answer for you."

        response_time_ms = int((time.time() - start_time) * 1000)
        tokens_used = len(response_text.split())

        # Create RAG query record for analytics (with error handling)
        try:
            rag_query = RAGQuery(
                user_id=user_id,
                query_text=query,
                response_text=response_text,
                context_chunks=similar_chunks,
                tokens_used=tokens_used,
                response_time_ms=response_time_ms,
                is_hallucination=False  # This would be determined by a more sophisticated check
            )

            await create_rag_query(self.db, rag_query)
        except Exception as e:
            logger.error(f"Failed to save RAG query to database: {e}")
            # Continue without saving the query record to avoid breaking the response
            pass

        # Add to chat history
        try:
            chat_history = ChatHistory(
                user_id=user_id,
                session_id=f"rag_session_{int(time.time())}",  # Simple session ID
                role="assistant",
                content=response_text
            )
            await create_chat_history(self.db, chat_history)
        except Exception as e:
            logger.error(f"Failed to save chat history to database: {e}")
            # Continue without saving the chat history to avoid breaking the response
            pass

        return {
            "response": response_text,
            "sources": similar_chunks,
            "response_time_ms": response_time_ms,
            "tokens_used": tokens_used
        }

    def _create_personalized_prompt(self, query: str, context: str, user_background: Optional[Dict[str, Any]] = None) -> str:
        """
        Create a personalized prompt based on user background information.

        Args:
            query: User's query
            context: Retrieved context from vector store
            user_background: User's background information for personalization

        Returns:
            Personalized prompt string
        """
        if not user_background:
            # Default prompt if no user background
            return f"""
            Based on the following context, please answer the question.
            If the context doesn't contain enough information, say so.

            Context:
            {context}

            Question: {query}

            Answer:
            """

        # Create a UserProfile object from the user_background dict
        user_profile = UserProfile(
            software_level=user_background.get("software_background_level", "beginner"),
            hardware_level=user_background.get("hardware_background_level", "beginner"),
            preferred_languages=user_background.get("preferred_languages", ""),
            goals=user_background.get("learning_goals", "")
        )

        # Use the personalization service to create a tailored prompt
        base_prompt = f"""
        Based on the following context, please answer the question:

        Context:
        {context}

        Question: {query}
        """

        return self.personalization_service.get_personalized_prompt(base_prompt, user_profile)

    async def query_with_selected_text(self, query: str, selected_text: str, user_id: Optional[int] = None, user_background: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Query the RAG system with user-selected text as additional context.

        Args:
            query: User's query
            selected_text: Text selected by the user
            user_id: ID of the user making the query (optional)
            user_background: User's background information for personalization (optional)

        Returns:
            Dictionary with response and context information
        """
        start_time = time.time()

        # Combine the query and selected text for better context
        enhanced_query = f"Context: {selected_text}\n\nQuestion: {query}"

        # Generate embedding for the combined query
        query_embedding = embedding_client.generate_embedding(enhanced_query)

        # Search for similar content in the vector store
        vector_store = get_vector_store()
        similar_chunks = await vector_store.search_similar(
            query_vector=query_embedding,
            limit=5  # Get top 5 similar chunks
        )

        # Include the selected text as primary context
        context = f"User-Selected Text: {selected_text}\n\nSimilar Content from Book:\n"
        context += "\n\n".join([f"- {chunk['content']}" for chunk in similar_chunks])

        print(f"DEBUG RAG 2: Creating personalized prompt with user_background: {user_background}")
        # Prepare the prompt with both selected text and similar chunks, plus user background for personalization
        prompt = self._create_personalized_prompt_with_selected_text(query, selected_text, context, user_background)
        print(f"DEBUG RAG 2: Generated personalized prompt, length: {len(prompt)}")

        # Generate response using the configurable LLM client with fallbacks
        from ..core.llm_client import get_llm_client

        try:
            llm_client = get_llm_client()
            response_text = await llm_client.generate_response(prompt, context)
            print(f"DEBUG RAG 2: Generated response using configurable LLM client: {bool(response_text)}")

            # If the response indicates an error, log it but still return it
            if "error" in response_text.lower() or "couldn't generate" in response_text.lower() or "currently experiencing high demand" in response_text.lower():
                print(f"DEBUG RAG 2: Response contained error message: {response_text}")
        except Exception as e:
            print(f"DEBUG RAG 2: Configurable LLM client failed: {e}")
            logger.error(f"Configurable LLM client failed: {e}")
            # Fallback response
            response_text = f"Based on the selected text and similar content, I found {len(similar_chunks)} relevant pieces of information. In a full implementation with an LLM configured, I would generate a comprehensive answer for you."

        response_time_ms = int((time.time() - start_time) * 1000)
        tokens_used = len(response_text.split())

        # Create RAG query record (with error handling)
        try:
            rag_query = RAGQuery(
                user_id=user_id,
                query_text=query,
                response_text=response_text,
                context_chunks=similar_chunks,
                tokens_used=tokens_used,
                response_time_ms=response_time_ms,
                is_hallucination=False
            )

            await create_rag_query(self.db, rag_query)
        except Exception as e:
            logger.error(f"Failed to save RAG query to database: {e}")
            # Continue without saving the query record to avoid breaking the response
            pass

        # Add to chat history
        try:
            chat_history = ChatHistory(
                user_id=user_id,
                session_id=f"selected_text_session_{int(time.time())}",
                role="assistant",
                content=response_text
            )
            await create_chat_history(self.db, chat_history)
        except Exception as e:
            logger.error(f"Failed to save chat history to database: {e}")
            # Continue without saving the chat history to avoid breaking the response
            pass

        return {
            "response": response_text,
            "sources": similar_chunks,
            "selected_text_used": selected_text,
            "response_time_ms": response_time_ms,
            "tokens_used": tokens_used
        }

    def _create_personalized_prompt_with_selected_text(self, query: str, selected_text: str, context: str, user_background: Optional[Dict[str, Any]] = None) -> str:
        """
        Create a personalized prompt based on user background information and selected text.

        Args:
            query: User's query
            selected_text: Text selected by the user
            context: Retrieved context from vector store
            user_background: User's background information for personalization

        Returns:
            Personalized prompt string
        """
        if not user_background:
            # Default prompt if no user background
            return f"""
            Based on the user-selected text and additional similar content from the book, please answer the question.
            Prioritize information from the selected text, but supplement with similar content if needed.
            If the provided information doesn't contain enough details, say so.

            {context}

            Question: {query}

            Answer:
            """

        # Create a UserProfile object from the user_background dict
        user_profile = UserProfile(
            software_level=user_background.get("software_background_level", "beginner"),
            hardware_level=user_background.get("hardware_background_level", "beginner"),
            preferred_languages=user_background.get("preferred_languages", ""),
            goals=user_background.get("learning_goals", "")
        )

        # Use the personalization service to create a tailored prompt
        base_prompt = f"""
        Based on the user-selected text and additional similar content from the book, please answer the question.

        {context}

        Question: {query}
        """

        return self.personalization_service.get_personalized_prompt(base_prompt, user_profile)

    async def semantic_search(self, query: str, top_k: int = 10, content_type: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        Perform semantic search without generating a response.

        Args:
            query: Search query
            top_k: Number of results to retrieve
            content_type: Filter by content type (optional)

        Returns:
            List of search results
        """
        start_time = time.time()

        # Generate embedding for the query
        query_embedding = embedding_client.generate_embedding(query)

        # Search for similar content in the vector store
        vector_store = get_vector_store()
        results = await vector_store.search_similar(
            query_vector=query_embedding,
            limit=top_k,
            content_type=content_type
        )

        search_time_ms = int((time.time() - start_time) * 1000)

        return {
            "results": results,
            "query": query,
            "top_k": top_k,
            "content_type": content_type,
            "search_time_ms": search_time_ms
        }

    async def hybrid_search(self, query: str, keyword_query: Optional[str] = None, top_k: int = 10) -> List[Dict[str, Any]]:
        """
        Perform hybrid search combining semantic and keyword search.

        Args:
            query: Semantic search query
            keyword_query: Keyword search query (for hybrid search)
            top_k: Number of results to retrieve

        Returns:
            List of search results
        """
        start_time = time.time()

        # Generate embedding for the semantic part of the query
        query_embedding = embedding_client.generate_embedding(query)

        # Perform hybrid search using both semantic and keyword queries
        vector_store = get_vector_store()
        results = await vector_store.search_similar(
            query_vector=query_embedding,
            limit=top_k,
            keyword_query=keyword_query  # For hybrid search
        )

        search_time_ms = int((time.time() - start_time) * 1000)

        return {
            "results": results,
            "query": query,
            "keyword_query": keyword_query,
            "top_k": top_k,
            "search_time_ms": search_time_ms
        }

    async def validate_response_accuracy(self, query: str, response: str, sources: List[Dict[str, Any]]) -> bool:
        """
        Validate if the response is accurate based on the provided sources.

        Args:
            query: Original query
            response: Generated response
            sources: Sources used to generate the response

        Returns:
            True if response appears accurate, False otherwise
        """
        # Simple validation: check if response contains information from sources
        response_lower = response.lower()

        # Count how many sources have content that appears in the response
        relevant_sources = 0
        for source in sources:
            source_content = source.get("content", "").lower()
            if len(source_content) > 10:  # Only check non-trivial content
                if source_content in response_lower:
                    relevant_sources += 1

        # If no sources are reflected in the response, it might be hallucinated
        return relevant_sources > 0

    async def detect_hallucinations(self, query: str, response: str, sources: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Detect potential hallucinations in the response based on the provided sources.

        Args:
            query: Original query
            response: Generated response
            sources: Sources used to generate the response

        Returns:
            Dictionary with hallucination detection results
        """
        # Check if response contains information not in sources
        response_sentences = [s.strip() for s in response.split('.') if s.strip()]

        source_text = " ".join([source.get("content", "") for source in sources]).lower()
        source_text_words = set(source_text.split())

        potential_hallucinations = []

        for sentence in response_sentences:
            sentence_words = set(sentence.lower().split())
            # Find words in the sentence that are not in the sources
            non_source_words = sentence_words - source_text_words

            # Only consider it a potential hallucination if there are significant non-source words
            # and the sentence is substantial
            if len(sentence) > 20 and len(non_source_words) / len(sentence_words) > 0.5:
                potential_hallucinations.append({
                    "sentence": sentence,
                    "non_source_words": list(non_source_words),
                    "confidence": len(non_source_words) / len(sentence_words)
                })

        # Also check for factual consistency by looking for strong indicators of uncertainty
        uncertainty_indicators = ["maybe", "perhaps", "might be", "could be", "i think", "i believe", "possibly"]
        uncertainty_detected = any(indicator in response.lower() for indicator in uncertainty_indicators)

        return {
            "is_likely_hallucinated": len(potential_hallucinations) > 0,
            "potential_hallucinations": potential_hallucinations,
            "uncertainty_indicators": uncertainty_detected,
            "accuracy_score": 1.0 - min(len(potential_hallucinations) / len(response_sentences), 1.0) if response_sentences else 1.0
        }


def get_rag_service(db: AsyncSession) -> RAGService:
    """Get an instance of the RAG service."""
    return RAGService(db)