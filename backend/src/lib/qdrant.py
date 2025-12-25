"""Qdrant Cloud client configuration for vector database operations."""

from functools import lru_cache

from qdrant_client import QdrantClient
from qdrant_client.http import models as qdrant_models

from src.lib.config import get_settings

settings = get_settings()

# Collection configuration
TEXTBOOK_COLLECTION = "textbook_chunks"
VECTOR_SIZE = 1536  # OpenAI text-embedding-3-small


@lru_cache
def get_qdrant_client() -> QdrantClient:
    """Get cached Qdrant client instance."""
    if settings.qdrant_api_key:
        return QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
    else:
        # Local development without API key
        return QdrantClient(url=settings.qdrant_url)


async def init_collection() -> None:
    """Initialize the textbook_chunks collection if it doesn't exist."""
    client = get_qdrant_client()

    collections = client.get_collections().collections
    collection_names = [c.name for c in collections]

    if TEXTBOOK_COLLECTION not in collection_names:
        client.create_collection(
            collection_name=TEXTBOOK_COLLECTION,
            vectors_config=qdrant_models.VectorParams(
                size=VECTOR_SIZE,
                distance=qdrant_models.Distance.COSINE,
            ),
            optimizers_config=qdrant_models.OptimizersConfigDiff(
                default_segment_number=2,
            ),
            hnsw_config=qdrant_models.HnswConfigDiff(
                m=16,
                ef_construct=100,
            ),
        )


async def search_similar(
    query_vector: list[float],
    limit: int = 5,
    module_id: str | None = None,
    chapter_id: str | None = None,
) -> list[qdrant_models.ScoredPoint]:
    """
    Search for similar chunks in the textbook collection.

    Args:
        query_vector: The embedding vector to search with
        limit: Maximum number of results to return
        module_id: Optional filter by module
        chapter_id: Optional filter by chapter

    Returns:
        List of scored points with payloads
    """
    client = get_qdrant_client()

    # Build filter conditions
    filter_conditions = []
    if module_id:
        filter_conditions.append(
            qdrant_models.FieldCondition(
                key="module_id",
                match=qdrant_models.MatchValue(value=module_id),
            )
        )
    if chapter_id:
        filter_conditions.append(
            qdrant_models.FieldCondition(
                key="chapter_id",
                match=qdrant_models.MatchValue(value=chapter_id),
            )
        )

    query_filter = None
    if filter_conditions:
        query_filter = qdrant_models.Filter(must=filter_conditions)

    results = client.search(
        collection_name=TEXTBOOK_COLLECTION,
        query_vector=query_vector,
        limit=limit,
        query_filter=query_filter,
        with_payload=True,
    )

    return results


async def upsert_chunks(
    chunks: list[dict],
) -> None:
    """
    Upsert chunks into the textbook collection.

    Args:
        chunks: List of chunk dictionaries with 'id', 'vector', and 'payload' keys
    """
    client = get_qdrant_client()

    points = [
        qdrant_models.PointStruct(
            id=chunk["id"],
            vector=chunk["vector"],
            payload=chunk["payload"],
        )
        for chunk in chunks
    ]

    client.upsert(
        collection_name=TEXTBOOK_COLLECTION,
        points=points,
    )


async def delete_by_chapter(chapter_id: str) -> None:
    """Delete all chunks for a specific chapter."""
    client = get_qdrant_client()

    client.delete(
        collection_name=TEXTBOOK_COLLECTION,
        points_selector=qdrant_models.FilterSelector(
            filter=qdrant_models.Filter(
                must=[
                    qdrant_models.FieldCondition(
                        key="chapter_id",
                        match=qdrant_models.MatchValue(value=chapter_id),
                    )
                ]
            )
        ),
    )
