"""Verify indexing script to check Qdrant collection status.

Run with: python -m scripts.verify_index [--query "test query"]
"""

import argparse
import asyncio
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lib.qdrant import (
    get_qdrant_client,
    TEXTBOOK_COLLECTION,
    search_similar,
)
from src.services.embedding_service import get_embedding


def verify_collection() -> dict:
    """Verify collection exists and get stats."""
    client = get_qdrant_client()

    try:
        info = client.get_collection(TEXTBOOK_COLLECTION)
        return {
            "exists": True,
            "status": str(info.status),
            "vectors_count": info.vectors_count,
            "points_count": info.points_count,
            "indexed_vectors_count": info.indexed_vectors_count,
        }
    except Exception as e:
        return {
            "exists": False,
            "error": str(e),
        }


def get_sample_points(limit: int = 5) -> list[dict]:
    """Get sample points from the collection."""
    client = get_qdrant_client()

    try:
        results = client.scroll(
            collection_name=TEXTBOOK_COLLECTION,
            limit=limit,
            with_payload=True,
            with_vectors=False,
        )
        points = []
        for point in results[0]:
            points.append({
                "id": point.id,
                "module_id": point.payload.get("module_id"),
                "chapter_id": point.payload.get("chapter_id"),
                "title": point.payload.get("title"),
                "text_preview": point.payload.get("text", "")[:100] + "...",
            })
        return points
    except Exception as e:
        return [{"error": str(e)}]


def get_chapter_counts() -> dict[str, int]:
    """Get count of chunks per chapter."""
    client = get_qdrant_client()

    try:
        # Scroll through all points to count by chapter
        counts: dict[str, int] = {}
        offset = None

        while True:
            results = client.scroll(
                collection_name=TEXTBOOK_COLLECTION,
                limit=100,
                offset=offset,
                with_payload=["chapter_id"],
                with_vectors=False,
            )
            points, next_offset = results

            if not points:
                break

            for point in points:
                chapter_id = point.payload.get("chapter_id", "unknown")
                counts[chapter_id] = counts.get(chapter_id, 0) + 1

            offset = next_offset
            if offset is None:
                break

        return dict(sorted(counts.items()))
    except Exception as e:
        return {"error": str(e)}


async def test_search(query: str) -> list[dict]:
    """Test semantic search with a query."""
    print(f"\nTesting search: '{query}'")
    print("-" * 40)

    # Get query embedding
    query_vector = await get_embedding(query)

    # Search
    results = await search_similar(query_vector, limit=3)

    search_results = []
    for i, result in enumerate(results, 1):
        search_results.append({
            "rank": i,
            "score": round(result.score, 4),
            "chapter_id": result.payload.get("chapter_id"),
            "title": result.payload.get("title"),
            "text_preview": result.payload.get("text", "")[:200] + "...",
        })
        print(f"\n{i}. Score: {result.score:.4f}")
        print(f"   Chapter: {result.payload.get('chapter_id')}")
        print(f"   Title: {result.payload.get('title')}")
        print(f"   Text: {result.payload.get('text', '')[:150]}...")

    return search_results


async def main(query: str | None = None):
    """Main verification function."""
    print("=" * 60)
    print("Index Verification Script")
    print("=" * 60)
    print()

    # Check collection status
    print("Collection Status:")
    print("-" * 40)
    stats = verify_collection()
    for key, value in stats.items():
        print(f"  {key}: {value}")

    if not stats.get("exists"):
        print("\nCollection does not exist. Run index_content.py first.")
        return

    if stats.get("points_count", 0) == 0:
        print("\nCollection is empty. Run index_content.py to index content.")
        return

    # Show chapter counts
    print("\nChunks per Chapter:")
    print("-" * 40)
    counts = get_chapter_counts()
    if "error" not in counts:
        for chapter_id, count in counts.items():
            print(f"  {chapter_id}: {count} chunks")
        print(f"\n  Total chapters: {len(counts)}")
    else:
        print(f"  Error: {counts['error']}")

    # Show sample points
    print("\nSample Points:")
    print("-" * 40)
    samples = get_sample_points(3)
    for sample in samples:
        if "error" in sample:
            print(f"  Error: {sample['error']}")
        else:
            print(f"  - {sample['chapter_id']}: {sample['text_preview']}")

    # Test search if query provided
    if query:
        await test_search(query)

    print()
    print("=" * 60)
    print("Verification complete!")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Verify Qdrant index status")
    parser.add_argument(
        "--query",
        type=str,
        help="Test search with this query",
    )
    args = parser.parse_args()

    asyncio.run(main(query=args.query))
