from qdrant_client import QdrantClient
from qdrant_client.http import models
from src.config.settings import settings

def initialize_qdrant_collection():
    """Initialize the Qdrant collection for book embeddings."""
    print("Initializing Qdrant collection...")

    # Initialize Qdrant client
    if settings.QDRANT_API_KEY:
        client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            prefer_grpc=True
        )
    else:
        client = QdrantClient(
            url=settings.QDRANT_URL,
            prefer_grpc=True
        )

    collection_name = "book_embeddings"
    embedding_size = settings.EMBEDDING_DIMENSION

    try:
        # Check if collection exists
        client.get_collection(collection_name)
        print(f"Collection '{collection_name}' already exists")
    except:
        # Collection doesn't exist, create it
        print(f"Creating collection '{collection_name}'...")
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=embedding_size,
                distance=models.Distance.COSINE
            ),
        )

        # Create payload index for content type
        client.create_payload_index(
            collection_name=collection_name,
            field_name="content_type",
            field_schema=models.PayloadSchemaType.KEYWORD
        )

        # Create payload index for content_id
        client.create_payload_index(
            collection_name=collection_name,
            field_name="content_id",
            field_schema=models.PayloadSchemaType.INTEGER
        )

        print(f"Collection '{collection_name}' created successfully with indexes")

    print("Qdrant initialization completed!")

if __name__ == "__main__":
    initialize_qdrant_collection()