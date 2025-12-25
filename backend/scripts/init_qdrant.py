"""Initialize Qdrant collection for textbook content.

Run with: python -m scripts.init_qdrant
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from qdrant_client.http import models as qdrant_models

from src.lib.config import get_settings
from src.lib.qdrant import (
    get_qdrant_client,
    TEXTBOOK_COLLECTION,
    VECTOR_SIZE,
)


def create_collection() -> None:
    """Create the textbook_chunks collection with optimal settings."""
    client = get_qdrant_client()
    settings = get_settings()

    # Check if collection exists
    collections = client.get_collections().collections
    collection_names = [c.name for c in collections]

    if TEXTBOOK_COLLECTION in collection_names:
        print(f"Collection '{TEXTBOOK_COLLECTION}' already exists.")
        info = client.get_collection(TEXTBOOK_COLLECTION)
        print(f"  Vectors count: {info.vectors_count}")
        print(f"  Points count: {info.points_count}")

        response = input("Do you want to recreate it? (y/N): ")
        if response.lower() != "y":
            print("Keeping existing collection.")
            return

        print(f"Deleting collection '{TEXTBOOK_COLLECTION}'...")
        client.delete_collection(TEXTBOOK_COLLECTION)

    print(f"Creating collection '{TEXTBOOK_COLLECTION}'...")

    # Create collection with optimized settings for RAG
    client.create_collection(
        collection_name=TEXTBOOK_COLLECTION,
        vectors_config=qdrant_models.VectorParams(
            size=VECTOR_SIZE,  # OpenAI text-embedding-3-small
            distance=qdrant_models.Distance.COSINE,
            on_disk=False,  # Keep vectors in memory for fast retrieval
        ),
        optimizers_config=qdrant_models.OptimizersConfigDiff(
            default_segment_number=2,
            indexing_threshold=20000,  # Start indexing after 20k vectors
        ),
        hnsw_config=qdrant_models.HnswConfigDiff(
            m=16,  # Number of edges per node
            ef_construct=100,  # Search accuracy during indexing
            full_scan_threshold=10000,  # Use brute force below this
        ),
        # Enable payload indexing for filtering
        on_disk_payload=False,
    )

    # Create payload indexes for efficient filtering
    print("Creating payload indexes...")

    client.create_payload_index(
        collection_name=TEXTBOOK_COLLECTION,
        field_name="module_id",
        field_schema=qdrant_models.PayloadSchemaType.KEYWORD,
    )

    client.create_payload_index(
        collection_name=TEXTBOOK_COLLECTION,
        field_name="chapter_id",
        field_schema=qdrant_models.PayloadSchemaType.KEYWORD,
    )

    client.create_payload_index(
        collection_name=TEXTBOOK_COLLECTION,
        field_name="difficulty",
        field_schema=qdrant_models.PayloadSchemaType.KEYWORD,
    )

    print(f"Collection '{TEXTBOOK_COLLECTION}' created successfully!")
    print()
    print("Collection configuration:")
    print(f"  Vector size: {VECTOR_SIZE}")
    print(f"  Distance metric: COSINE")
    print(f"  Indexed fields: module_id, chapter_id, difficulty")
    print()
    print("Expected payload schema for each point:")
    print("  {")
    print('    "module_id": "ros2",')
    print('    "chapter_id": "ros2-nodes-topics",')
    print('    "difficulty": "beginner",')
    print('    "text": "The original chunk text...",')
    print('    "title": "Chapter title",')
    print('    "slug": "chapter-slug",')
    print('    "chunk_index": 0')
    print("  }")


def verify_collection() -> None:
    """Verify the collection is properly configured."""
    client = get_qdrant_client()

    print()
    print("Verifying collection...")

    try:
        info = client.get_collection(TEXTBOOK_COLLECTION)
        print(f"  Status: {info.status}")
        print(f"  Vectors count: {info.vectors_count}")
        print(f"  Points count: {info.points_count}")
        print(f"  Indexed vectors count: {info.indexed_vectors_count}")
        print("  Collection is ready for use!")
    except Exception as e:
        print(f"  Error: {e}")
        print("  Collection may not exist or there's a connection issue.")


def main() -> None:
    """Run the Qdrant initialization script."""
    settings = get_settings()

    print("Qdrant Collection Initialization")
    print("=" * 40)
    print()
    print(f"Qdrant URL: {settings.qdrant_url}")
    print(f"Collection: {TEXTBOOK_COLLECTION}")
    print()

    create_collection()
    verify_collection()


if __name__ == "__main__":
    main()
