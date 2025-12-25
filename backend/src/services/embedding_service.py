"""Embedding service using OpenAI text-embedding-3-small."""

import hashlib
from functools import lru_cache

from openai import AsyncOpenAI

from src.lib.config import get_settings

settings = get_settings()

# OpenAI client singleton
_client: AsyncOpenAI | None = None


def get_openai_client() -> AsyncOpenAI:
    """Get or create OpenAI client."""
    global _client
    if _client is None:
        _client = AsyncOpenAI(api_key=settings.openai_api_key)
    return _client


# Embedding model configuration
EMBEDDING_MODEL = "text-embedding-3-small"
EMBEDDING_DIMENSIONS = 1536
MAX_TOKENS_PER_CHUNK = 8000  # text-embedding-3-small limit


async def get_embedding(text: str) -> list[float]:
    """
    Generate embedding for a single text.

    Args:
        text: Text to embed (max ~8000 tokens)

    Returns:
        List of floats representing the embedding vector
    """
    client = get_openai_client()

    response = await client.embeddings.create(
        model=EMBEDDING_MODEL,
        input=text,
        dimensions=EMBEDDING_DIMENSIONS,
    )

    return response.data[0].embedding


async def get_embeddings_batch(texts: list[str]) -> list[list[float]]:
    """
    Generate embeddings for multiple texts in a single API call.

    Args:
        texts: List of texts to embed (max 2048 per batch)

    Returns:
        List of embedding vectors in same order as input
    """
    client = get_openai_client()

    # OpenAI batch limit is 2048 texts
    if len(texts) > 2048:
        raise ValueError("Batch size exceeds OpenAI limit of 2048 texts")

    response = await client.embeddings.create(
        model=EMBEDDING_MODEL,
        input=texts,
        dimensions=EMBEDDING_DIMENSIONS,
    )

    # Sort by index to ensure correct order
    sorted_data = sorted(response.data, key=lambda x: x.index)
    return [item.embedding for item in sorted_data]


def chunk_text(
    text: str,
    chunk_size: int = 1000,
    overlap: int = 200,
) -> list[dict]:
    """
    Split text into overlapping chunks for embedding.

    Args:
        text: Full text to chunk
        chunk_size: Target characters per chunk
        overlap: Overlap between chunks

    Returns:
        List of dicts with 'text', 'start', 'end' keys
    """
    chunks = []
    start = 0

    while start < len(text):
        # Find chunk end
        end = start + chunk_size

        # Try to break at paragraph or sentence boundary
        if end < len(text):
            # Look for paragraph break
            para_break = text.rfind("\n\n", start, end)
            if para_break > start + chunk_size // 2:
                end = para_break + 2
            else:
                # Look for sentence break
                sentence_break = max(
                    text.rfind(". ", start, end),
                    text.rfind("! ", start, end),
                    text.rfind("? ", start, end),
                )
                if sentence_break > start + chunk_size // 2:
                    end = sentence_break + 2

        chunk_text = text[start:end].strip()
        if chunk_text:
            chunks.append({
                "text": chunk_text,
                "start": start,
                "end": end,
            })

        # Move start with overlap
        start = end - overlap
        if start <= chunks[-1]["start"] if chunks else 0:
            start = end  # Avoid infinite loop

    return chunks


def generate_chunk_id(chapter_id: str, chunk_index: int) -> str:
    """Generate a deterministic ID for a chunk."""
    content = f"{chapter_id}:{chunk_index}"
    return hashlib.sha256(content.encode()).hexdigest()[:32]


async def compute_similarity(
    query_embedding: list[float],
    document_embedding: list[float],
) -> float:
    """
    Compute cosine similarity between two embeddings.

    Args:
        query_embedding: Query vector
        document_embedding: Document vector

    Returns:
        Similarity score between 0 and 1
    """
    dot_product = sum(a * b for a, b in zip(query_embedding, document_embedding))
    query_norm = sum(a * a for a in query_embedding) ** 0.5
    doc_norm = sum(b * b for b in document_embedding) ** 0.5

    if query_norm == 0 or doc_norm == 0:
        return 0.0

    return dot_product / (query_norm * doc_norm)
