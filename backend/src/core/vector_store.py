from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Optional, Any
import uuid
from ..config.settings import settings
import logging


logger = logging.getLogger(__name__)


class VectorStore:
    """Qdrant vector store client for managing embeddings and semantic search."""

    def __init__(self):
        # Initialize Qdrant client - check if URL is provided
        if not settings.QDRANT_URL or settings.QDRANT_URL.strip() == "":
            # If no QDRANT_URL is provided, create a mock client that returns empty results
            self.client = None
            logger.warning("QDRANT_URL not provided - vector store will return empty results")
        else:
            if settings.QDRANT_API_KEY:
                self.client = QdrantClient(
                    url=settings.QDRANT_URL,
                    api_key=settings.QDRANT_API_KEY
                )
            else:
                self.client = QdrantClient(
                    url=settings.QDRANT_URL
                )

        self.collection_name = "book_embeddings"
        self.embedding_size = settings.EMBEDDING_DIMENSION

    def create_collection(self):
        """Create the embeddings collection if it doesn't exist."""
        if self.client is None:
            logger.warning("QDRANT_URL not configured - skipping collection creation")
            return

        try:
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' already exists")
        except:
            # Collection doesn't exist, create it
            logger.info(f"Creating collection '{self.collection_name}'...")
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.embedding_size,
                    distance=models.Distance.COSINE
                ),
            )

            # Create payload index for content type
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="content_type",
                field_schema=models.PayloadSchemaType.KEYWORD
            )

            # Create payload index for content_id
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="content_id",
                field_schema=models.PayloadSchemaType.INTEGER
            )

            logger.info(f"Collection '{self.collection_name}' created successfully with indexes")

    async def add_embedding(
        self,
        content: str,
        content_id: int,
        content_type: str,
        metadata: Dict[str, Any],
        vector: List[float]
    ) -> str:
        """Add a single embedding to the collection."""
        if self.client is None:
            logger.warning("QDRANT_URL not configured - skipping embedding addition")
            return str(uuid.uuid4())

        point_id = str(uuid.uuid4())

        self.client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=point_id,
                    vector=vector,
                    payload={
                        "content": content,
                        "content_id": content_id,
                        "content_type": content_type,
                        "metadata": metadata
                    }
                )
            ]
        )

        logger.debug(f"Added embedding with ID: {point_id}")
        return point_id

    async def add_embeddings_batch(
        self,
        contents: List[str],
        content_ids: List[int],
        content_types: List[str],
        metadatas: List[Dict[str, Any]],
        vectors: List[List[float]]
    ) -> List[str]:
        """Add multiple embeddings to the collection."""
        if self.client is None:
            logger.warning("QDRANT_URL not configured - skipping batch embedding addition")
            return [str(uuid.uuid4()) for _ in range(len(contents))]

        point_ids = [str(uuid.uuid4()) for _ in range(len(contents))]

        points = []
        for i in range(len(contents)):
            points.append(
                models.PointStruct(
                    id=point_ids[i],
                    vector=vectors[i],
                    payload={
                        "content": contents[i],
                        "content_id": content_ids[i],
                        "content_type": content_types[i],
                        "metadata": metadatas[i]
                    }
                )
            )

        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        logger.debug(f"Added {len(contents)} embeddings in batch")
        return point_ids

    async def search_similar(
        self,
        query_vector: List[float],
        limit: int = 10,
        content_type: Optional[str] = None,
        content_id: Optional[int] = None,
        keyword_query: Optional[str] = None  # For hybrid search
    ) -> List[Dict[str, Any]]:
        """Search for similar content based on the query vector with optional keyword filtering."""

        # Build filters
        filters = []
        if content_type:
            filters.append(
                models.FieldCondition(
                    key="content_type",
                    match=models.MatchValue(value=content_type)
                )
            )

        if content_id is not None:
            filters.append(
                models.FieldCondition(
                    key="content_id",
                    match=models.MatchValue(value=content_id)
                )
            )

        # If keyword query is provided, perform keyword search as well (hybrid search)
        if keyword_query:
            # For hybrid search, we'll use the keyword as a filter condition
            # Qdrant supports full-text search through its payload filtering
            # This is a simplified approach - in a real implementation you'd use more sophisticated hybrid search
            keyword_filters = [
                models.FieldCondition(
                    key="content",
                    match=models.MatchText(text=keyword_query)
                )
            ]

            # Combine with other filters
            if filters:
                filter_obj = models.Filter(
                    must=filters,
                    should=keyword_filters,
                    min_should_match=1
                )
            else:
                filter_obj = models.Filter(should=keyword_filters, min_should_match=1)
        else:
            filter_obj = models.Filter(must=filters) if filters else None

        # Perform the search - check if Qdrant client is available
        if self.client is None:
            # Qdrant client is not initialized, return empty results
            logger.warning("QDRANT_URL not configured - returning empty search results")
            results = []
        else:
            # Qdrant client exists, try to perform search
            try:
                results = self.client.search(
                    collection_name=self.collection_name,
                    query_vector=query_vector,
                    query_filter=filter_obj,
                    limit=limit
                )
            except AttributeError as e:
                # This handles the case where QdrantClient doesn't have 'search' method
                logger.error(f"Qdrant client does not have search method: {e}")
                results = []
            except Exception as e:
                # Handle other search-related errors
                logger.error(f"Qdrant search failed: {e}")
                results = []

        # Format results
        formatted_results = []
        for result in results:
            formatted_results.append({
                "id": result.id,
                "content": result.payload["content"],
                "content_id": result.payload["content_id"],
                "content_type": result.payload["content_type"],
                "metadata": result.payload["metadata"],
                "score": result.score
            })

        logger.debug(f"Found {len(formatted_results)} similar results")
        return formatted_results

    async def delete_embedding(self, point_id: str):
        """Delete a specific embedding by ID."""
        if self.client is None:
            logger.warning("QDRANT_URL not configured - skipping embedding deletion")
            return

        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.PointIdsList(points=[point_id])
        )
        logger.debug(f"Deleted embedding with ID: {point_id}")

    async def delete_by_content_id(self, content_id: int, content_type: str):
        """Delete embeddings associated with a specific content ID and type."""
        if self.client is None:
            logger.warning("QDRANT_URL not configured - skipping embedding deletion by content ID")
            return

        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.FilterSelector(
                filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="content_id",
                            match=models.MatchValue(value=content_id)
                        ),
                        models.FieldCondition(
                            key="content_type",
                            match=models.MatchValue(value=content_type)
                        )
                    ]
                )
            )
        )
        logger.debug(f"Deleted embeddings with content_id: {content_id} and content_type: {content_type}")

    async def get_embedding(self, point_id: str) -> Optional[Dict[str, Any]]:
        """Get a specific embedding by ID."""
        if self.client is None:
            logger.warning("QDRANT_URL not configured - cannot retrieve embedding")
            return None

        points = self.client.retrieve(
            collection_name=self.collection_name,
            ids=[point_id]
        )

        if points:
            point = points[0]
            return {
                "id": point.id,
                "content": point.payload["content"],
                "content_id": point.payload["content_id"],
                "content_type": point.payload["content_type"],
                "metadata": point.payload["metadata"]
            }

        return None

    def get_total_points(self) -> int:
        """Get the total number of points in the collection."""
        if self.client is None:
            logger.warning("QDRANT_URL not configured - returning 0 total points")
            return 0

        try:
            collection_info = self.client.get_collection(self.collection_name)
            return collection_info.points_count
        except:
            return 0


# Global vector store instance
vector_store = VectorStore()

# Initialize the collection when the module is loaded
vector_store.create_collection()