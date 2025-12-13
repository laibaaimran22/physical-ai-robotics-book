from typing import List, Dict, Any, Optional
from langchain_openai import OpenAIEmbeddings, ChatOpenAI
from langchain_community.vectorstores import Qdrant
from langchain_core.tools import tool
from langchain_core.runnables import RunnablePassthrough
from langchain_core.output_parsers import StrOutputParser
from langchain.prompts import PromptTemplate
from ..config.settings import settings
import logging

logger = logging.getLogger(__name__)

class LangChainRAGService:
    """Advanced RAG service using LangChain for better retrieval and generation."""

    def __init__(self):
        self.qdrant_instance = None
        self.retrieval_chain = None
        self.embeddings = None
        self.llm = None

        # Initialize OpenAI components if API key is available
        if settings.OPENAI_API_KEY:
            try:
                self.embeddings = OpenAIEmbeddings(
                    openai_api_key=settings.OPENAI_API_KEY,
                    model="text-embedding-ada-002"
                )
                self.llm = ChatOpenAI(
                    openai_api_key=settings.OPENAI_API_KEY,
                    model_name=settings.OPENAI_MODEL,
                    temperature=0.3
                )
                logger.info("Initialized OpenAI embeddings and LLM for LangChain RAG")
            except Exception as e:
                logger.warning(f"Failed to initialize OpenAI components for LangChain: {e}")
        else:
            logger.info("OpenAI API key not provided, LangChain RAG will not be available")

    def initialize_qdrant(self, collection_name: str = "book_embeddings"):
        """Initialize Qdrant vector store for LangChain."""
        if not self.embeddings:
            logger.error("Cannot initialize LangChain RAG without embeddings")
            return False

        try:
            # Initialize Qdrant vector store for LangChain
            self.qdrant_instance = Qdrant.from_existing_collection(
                collection_name=collection_name,
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                embeddings=self.embeddings
            )
            logger.info(f"Initialized Qdrant vector store for LangChain with collection: {collection_name}")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant for LangChain: {e}")
            return False

    def create_retrieval_chain(self, chain_type: str = "stuff"):
        """Create a retrieval chain for question answering."""
        if not self.qdrant_instance or not self.llm:
            logger.error("Cannot create retrieval chain without Qdrant instance and LLM")
            return False

        try:
            # Create a custom prompt for book-related questions
            custom_prompt = PromptTemplate(
                input_variables=["context", "question"],
                template="""
                You are an expert assistant for the Physical AI and Humanoid Robotics book.
                Use the following pieces of context to answer the question at the end.
                If you don't know the answer, just say that you don't know, don't try to make up an answer.

                Context: {context}

                Question: {question}

                Helpful Answer:"""
            )

            # Create the document chain
            document_chain = create_stuff_documents_chain(
                llm=self.llm,
                prompt=custom_prompt
            )

            # Create the retrieval chain
            self.retrieval_chain = create_retrieval_chain(
                retriever=self.qdrant_instance.as_retriever(
                    search_kwargs={"k": 5}  # Retrieve top 5 similar documents
                ),
                combine_docs_chain=document_chain
            )
            logger.info("Created retrieval chain for LangChain RAG")
            return True
        except Exception as e:
            logger.error(f"Failed to create retrieval chain: {e}")
            return False

    def query(self, question: str, top_k: int = 5) -> Dict[str, Any]:
        """Query the RAG system."""
        if not self.retrieval_chain:
            logger.error("Retrieval chain not initialized")
            return {
                "response": "RAG system is not properly initialized.",
                "sources": [],
                "query": question
            }

        try:
            # Perform the query
            result = self.retrieval_chain.invoke({"input": question})

            # Extract sources (format depends on the new LangChain structure)
            sources = []
            if "context" in result:
                for doc in result["context"]:
                    sources.append({
                        "content": doc.page_content,
                        "metadata": doc.metadata,
                        "score": getattr(doc, 'score', 1.0)  # Default score if not available
                    })

            return {
                "response": result.get("answer", result.get("output", "No response generated")),
                "sources": sources,
                "query": question,
                "top_k": top_k
            }
        except Exception as e:
            logger.error(f"Error querying RAG system: {e}")
            return {
                "response": f"Error processing query: {str(e)}",
                "sources": [],
                "query": question
            }

    def query_with_context(self, question: str, context: str) -> Dict[str, Any]:
        """Query with additional context (e.g., user-selected text)."""
        if not self.llm:
            logger.error("LLM not initialized for context-based query")
            return {
                "response": "LLM is not properly initialized.",
                "query": question,
                "context_used": context
            }

        try:
            # Create a custom prompt that includes both retrieved context and user-provided context
            prompt = f"""
            You are an expert assistant for the Physical AI and Humanoid Robotics book.
            Use the following context to answer the question. The user has also provided
            specific text that should be considered in your response.

            Retrieved Context: {context}

            User's Selected Text: {context}

            Question: {question}

            Please provide a helpful and accurate answer based on the provided information.
            """

            # Use the LLM directly for this custom query
            response = self.llm.invoke(prompt)

            return {
                "response": response.content if hasattr(response, 'content') else str(response),
                "query": question,
                "context_used": context
            }
        except Exception as e:
            logger.error(f"Error querying with context: {e}")
            return {
                "response": f"Error processing query with context: {str(e)}",
                "query": question,
                "context_used": context
            }

# Global LangChain RAG service instance
langchain_rag_service = LangChainRAGService()