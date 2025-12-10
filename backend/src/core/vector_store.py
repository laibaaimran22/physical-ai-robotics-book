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
                # For Qdrant Cloud, use the proper initialization
                self.client = QdrantClient(
                    url=settings.QDRANT_URL,
                    api_key=settings.QDRANT_API_KEY,
                    prefer_grpc=False  # Use REST API for cloud instances
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
        if self.client is None:
            logger.warning("QDRANT_URL not configured - returning empty results")
            return []

        # Build the filter conditions
        must_conditions = []
        if content_type:
            must_conditions.append(
                models.FieldCondition(
                    key="content_type",
                    match=models.MatchValue(value=content_type)
                )
            )
        if content_id is not None:
            must_conditions.append(
                models.FieldCondition(
                    key="content_id",
                    match=models.MatchValue(value=content_id)
                )
            )

        # Create the filter if there are conditions
        query_filter = models.Filter(must=must_conditions) if must_conditions else None

        try:
            # Use the newer query_points method instead of the deprecated search method
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                query_filter=query_filter,
                limit=limit
            )

            # The query_points method returns a QueryResponse object with a points attribute
            # Check if results is a QueryResponse object or has a points attribute
            if hasattr(search_results, 'points'):
                scored_points = search_results.points
            else:
                # If it's already a list-like object, use it directly
                scored_points = search_results

            # Convert the results to the expected format
            similar_chunks = []
            for scored_point in scored_points:
                # Handle both possible result formats depending on Qdrant client version
                if hasattr(scored_point, 'payload'):  # Newer format
                    content_data = scored_point.payload
                    point_id = scored_point.id
                    score = scored_point.score
                elif isinstance(scored_point, dict):  # Older format or different return
                    content_data = scored_point.get('payload', {})
                    point_id = scored_point.get('id')
                    score = scored_point.get('score')
                else:
                    # Try to access attributes directly
                    content_data = getattr(scored_point, 'payload', {})
                    point_id = getattr(scored_point, 'id', None)
                    score = getattr(scored_point, 'score', None)

                similar_chunks.append({
                    "id": point_id,
                    "content": content_data.get("content", ""),
                    "content_id": content_data.get("content_id"),
                    "content_type": content_data.get("content_type"),
                    "metadata": content_data.get("metadata", {}),
                    "score": score  # Include the similarity score
                })

            logger.info(f"Found {len(similar_chunks)} similar chunks")
            return similar_chunks

        except AttributeError as e:
            # Handle the case where the Qdrant client doesn't have the expected method
            logger.error(f"Qdrant client attribute error: {e}")
            # Return empty results to avoid breaking the system
            return []
        except Exception as e:
            logger.error(f"Error searching for similar content: {e}")
            import traceback
            traceback.print_exc()
            return []

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


# Global vector store instance (lazy initialization to avoid import-time errors)
def get_vector_store():
    """Get or create the vector store instance with lazy initialization."""
    if not hasattr(get_vector_store, '_instance'):
        get_vector_store._instance = VectorStore()
        # Initialize the collection on first access
        get_vector_store._instance.create_collection()
    return get_vector_store._instance