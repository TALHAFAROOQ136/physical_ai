"""Content indexing script to chunk and embed MDX files.

Run with: python -m scripts.index_content [--dry-run] [--chapter CHAPTER_ID]
"""

import argparse
import asyncio
import re
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from src.lib.config import get_settings
from src.lib.qdrant import (
    get_qdrant_client,
    TEXTBOOK_COLLECTION,
    upsert_chunks,
    delete_by_chapter,
)
from src.services.embedding_service import (
    get_embeddings_batch,
    chunk_text,
    generate_chunk_id,
)

settings = get_settings()

# Path to docs folder (relative to backend)
DOCS_PATH = Path(__file__).parent.parent.parent / "frontend" / "docs"

# Chapter metadata extracted from frontmatter
CHAPTER_MAPPING = {
    "intro/index.md": {"module_id": "intro", "chapter_id": "intro-index"},
    "intro/prerequisites.md": {"module_id": "intro", "chapter_id": "intro-prerequisites"},
    "intro/setup.md": {"module_id": "intro", "chapter_id": "intro-setup"},
    "module-1-ros2/index.md": {"module_id": "ros2", "chapter_id": "ros2-index"},
    "module-1-ros2/nodes-and-topics.md": {"module_id": "ros2", "chapter_id": "ros2-nodes-topics"},
    "module-1-ros2/services-and-actions.md": {"module_id": "ros2", "chapter_id": "ros2-services-actions"},
    "module-1-ros2/urdf-basics.md": {"module_id": "ros2", "chapter_id": "ros2-urdf"},
    "module-2-simulation/index.md": {"module_id": "simulation", "chapter_id": "sim-index"},
    "module-2-simulation/gazebo-basics.md": {"module_id": "simulation", "chapter_id": "sim-gazebo"},
    "module-2-simulation/unity-integration.md": {"module_id": "simulation", "chapter_id": "sim-unity"},
    "module-3-isaac/index.md": {"module_id": "isaac", "chapter_id": "isaac-index"},
    "module-3-isaac/isaac-sim-setup.md": {"module_id": "isaac", "chapter_id": "isaac-setup"},
    "module-3-isaac/perception-pipelines.md": {"module_id": "isaac", "chapter_id": "isaac-perception"},
    "module-4-vla/index.md": {"module_id": "vla", "chapter_id": "vla-index"},
    "module-4-vla/voice-interfaces.md": {"module_id": "vla", "chapter_id": "vla-voice"},
    "module-4-vla/llm-planning.md": {"module_id": "vla", "chapter_id": "vla-llm"},
    "capstone/index.md": {"module_id": "capstone", "chapter_id": "capstone-index"},
    "capstone/milestone-1.md": {"module_id": "capstone", "chapter_id": "capstone-m1"},
    "capstone/milestone-2.md": {"module_id": "capstone", "chapter_id": "capstone-m2"},
    "capstone/milestone-3.md": {"module_id": "capstone", "chapter_id": "capstone-m3"},
    "capstone/milestone-4.md": {"module_id": "capstone", "chapter_id": "capstone-m4"},
    "capstone/milestone-5.md": {"module_id": "capstone", "chapter_id": "capstone-m5"},
}


def extract_frontmatter(content: str) -> tuple[dict, str]:
    """Extract YAML frontmatter from markdown content."""
    frontmatter = {}
    body = content

    if content.startswith("---"):
        parts = content.split("---", 2)
        if len(parts) >= 3:
            import yaml
            try:
                frontmatter = yaml.safe_load(parts[1]) or {}
            except Exception:
                pass
            body = parts[2]

    return frontmatter, body


def clean_markdown(content: str) -> str:
    """Remove markdown syntax for cleaner embedding text."""
    # Remove code blocks
    content = re.sub(r"```[\s\S]*?```", "[code block]", content)
    # Remove inline code
    content = re.sub(r"`[^`]+`", "", content)
    # Remove links but keep text
    content = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", content)
    # Remove images
    content = re.sub(r"!\[[^\]]*\]\([^)]+\)", "", content)
    # Remove HTML tags
    content = re.sub(r"<[^>]+>", "", content)
    # Remove frontmatter markers
    content = re.sub(r"^---[\s\S]*?---", "", content)
    # Normalize whitespace
    content = re.sub(r"\n{3,}", "\n\n", content)
    return content.strip()


def extract_title(frontmatter: dict, content: str) -> str:
    """Extract title from frontmatter or first heading."""
    if "title" in frontmatter:
        return frontmatter["title"]

    # Find first heading
    match = re.search(r"^#\s+(.+)$", content, re.MULTILINE)
    if match:
        return match.group(1)

    return "Untitled"


async def index_chapter(
    file_path: Path,
    metadata: dict,
    dry_run: bool = False,
) -> int:
    """
    Index a single chapter file.

    Returns:
        Number of chunks indexed
    """
    print(f"  Processing: {file_path.name}")

    # Read file content
    content = file_path.read_text(encoding="utf-8")

    # Extract frontmatter and clean content
    frontmatter, body = extract_frontmatter(content)
    title = extract_title(frontmatter, body)
    clean_content = clean_markdown(body)

    if len(clean_content) < 100:
        print(f"    Skipping: Content too short ({len(clean_content)} chars)")
        return 0

    # Chunk the content
    chunks = chunk_text(clean_content, chunk_size=1000, overlap=200)
    print(f"    Created {len(chunks)} chunks")

    if dry_run:
        return len(chunks)

    # Delete existing chunks for this chapter
    chapter_id = metadata["chapter_id"]
    await delete_by_chapter(chapter_id)

    # Generate embeddings in batch
    chunk_texts = [c["text"] for c in chunks]
    embeddings = await get_embeddings_batch(chunk_texts)

    # Prepare points for Qdrant
    points = []
    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        point_id = generate_chunk_id(chapter_id, i)
        points.append({
            "id": point_id,
            "vector": embedding,
            "payload": {
                "module_id": metadata["module_id"],
                "chapter_id": chapter_id,
                "title": title,
                "slug": file_path.stem,
                "text": chunk["text"],
                "chunk_index": i,
                "difficulty": frontmatter.get("difficulty", "beginner"),
            },
        })

    # Upsert to Qdrant
    await upsert_chunks(points)
    print(f"    Indexed {len(points)} chunks to Qdrant")

    return len(points)


async def main(dry_run: bool = False, chapter_filter: str | None = None):
    """Main indexing function."""
    print("=" * 60)
    print("Content Indexing Script")
    print("=" * 60)
    print()
    print(f"Docs path: {DOCS_PATH}")
    print(f"Dry run: {dry_run}")
    if chapter_filter:
        print(f"Chapter filter: {chapter_filter}")
    print()

    if not DOCS_PATH.exists():
        print(f"ERROR: Docs path not found: {DOCS_PATH}")
        return

    total_chunks = 0
    total_files = 0

    for rel_path, metadata in CHAPTER_MAPPING.items():
        # Apply chapter filter if specified
        if chapter_filter and chapter_filter not in metadata["chapter_id"]:
            continue

        file_path = DOCS_PATH / rel_path

        if not file_path.exists():
            print(f"  SKIP: File not found: {rel_path}")
            continue

        try:
            chunks = await index_chapter(file_path, metadata, dry_run)
            total_chunks += chunks
            total_files += 1
        except Exception as e:
            print(f"  ERROR: {e}")

    print()
    print("=" * 60)
    print(f"Indexing complete!")
    print(f"  Files processed: {total_files}")
    print(f"  Total chunks: {total_chunks}")
    if dry_run:
        print("  (Dry run - no data written)")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Index textbook content for RAG")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run without writing to Qdrant",
    )
    parser.add_argument(
        "--chapter",
        type=str,
        help="Only index chapters matching this ID pattern",
    )
    args = parser.parse_args()

    asyncio.run(main(dry_run=args.dry_run, chapter_filter=args.chapter))
