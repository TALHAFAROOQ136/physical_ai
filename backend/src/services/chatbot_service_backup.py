"""Chatbot service with RAG (Retrieval-Augmented Generation)."""

import uuid
from typing import AsyncGenerator, Any
from datetime import datetime

from openai import AsyncOpenAI
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from src.lib.config import get_settings
from src.lib.qdrant import search_similar
from src.models.conversation import Conversation
from src.models.message import Message
from src.services.embedding_service import get_embedding, get_openai_client

settings = get_settings()

# System prompt for the chatbot
SYSTEM_PROMPT = """You are an expert teaching assistant for the "Physical AI & Humanoid Robotics" textbook. Your role is to help students understand robotics concepts including:

- ROS 2 (Robot Operating System 2)
- Gazebo and Unity simulation
- NVIDIA Isaac Sim and perception
- Vision-Language-Action (VLA) models
- Humanoid robot development

Guidelines:
1. Be concise but thorough in your explanations
2. Use code examples when helpful (Python, C++, YAML, bash)
3. Reference specific textbook chapters when relevant
4. Encourage hands-on learning and experimentation
5. If you don't know something, say so honestly
6. Adapt explanations to the student's background level

When answering, you may be provided with relevant context from the textbook. Use this context to provide accurate, textbook-aligned answers. Always cite sources when using provided context."""

MAX_CONTEXT_MESSAGES = 10
MAX_RETRIEVED_CHUNKS = 5


async def create_conversation(
    session: AsyncSession,
    user_id: uuid.UUID | None = None,
    chapter_context: str | None = None,
    context: dict[str, Any] | None = None,
) -> Conversation:
    """
    Create a new conversation.

    Args:
        session: Database session
        user_id: Optional user ID
        chapter_context: Optional chapter the user is viewing
        context: Optional conversation context

    Returns:
        Created Conversation object
    """
    conversation = Conversation(
        user_id=user_id,
        chapter_context=chapter_context,
        context=context or {},
    )
    session.add(conversation)
    await session.commit()
    await session.refresh(conversation)
    return conversation


async def get_conversation(
    session: AsyncSession,
    conversation_id: uuid.UUID,
) -> Conversation | None:
    """Get a conversation by ID."""
    result = await session.execute(
        select(Conversation).where(Conversation.id == conversation_id)
    )
    return result.scalar_one_or_none()


async def get_conversation_messages(
    session: AsyncSession,
    conversation_id: uuid.UUID,
    limit: int = MAX_CONTEXT_MESSAGES,
) -> list[Message]:
    """Get recent messages from a conversation."""
    result = await session.execute(
        select(Message)
        .where(Message.conversation_id == conversation_id)
        .order_by(Message.created_at.desc())
        .limit(limit)
    )
    messages = list(result.scalars().all())
    messages.reverse()  # Return in chronological order
    return messages


async def add_message(
    session: AsyncSession,
    conversation_id: uuid.UUID,
    role: str,
    content: str,
    citations: list[dict[str, Any]] | None = None,
    metadata: dict[str, Any] | None = None,
) -> Message:
    """
    Add a message to a conversation.

    Args:
        session: Database session
        conversation_id: Conversation ID
        role: 'user' or 'assistant'
        content: Message content
        citations: Optional citations
        metadata: Optional metadata

    Returns:
        Created Message object
    """
    message = Message(
        conversation_id=conversation_id,
        role=role,
        content=content,
        citations=citations or [],
        metadata=metadata or {},
    )
    session.add(message)

    # Update conversation message count and timestamp
    conversation = await get_conversation(session, conversation_id)
    if conversation:
        conversation.increment_message_count()

    await session.commit()
    await session.refresh(message)
    return message


async def retrieve_context(
    query: str,
    chapter_filter: str | None = None,
    limit: int = MAX_RETRIEVED_CHUNKS,
) -> list[dict[str, Any]]:
    """
    Retrieve relevant context from the vector store.

    Args:
        query: User's question
        chapter_filter: Optional chapter to filter by
        limit: Maximum chunks to retrieve

    Returns:
        List of relevant context chunks with metadata
    """
    # Get query embedding
    query_embedding = await get_embedding(query)

    # Search for similar chunks
    results = await search_similar(
        query_vector=query_embedding,
        limit=limit,
        chapter_id=chapter_filter,
    )

    # Format results
    context_chunks = []
    for result in results:
        if result.score < 0.5:  # Filter low-relevance results
            continue
        context_chunks.append({
            "chapter_id": result.payload.get("chapter_id"),
            "title": result.payload.get("title"),
            "text": result.payload.get("text"),
            "score": result.score,
        })

    return context_chunks


def format_context_for_prompt(chunks: list[dict[str, Any]]) -> str:
    """Format retrieved chunks for inclusion in prompt."""
    if not chunks:
        return ""

    context_parts = ["Here is relevant context from the textbook:\n"]
    for i, chunk in enumerate(chunks, 1):
        context_parts.append(
            f"[Source {i}: {chunk['title']} ({chunk['chapter_id']})]\n"
            f"{chunk['text']}\n"
        )

    return "\n".join(context_parts)


def format_citations(chunks: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Format chunks as citation objects."""
    return [
        {
            "chapter_id": chunk["chapter_id"],
            "title": chunk["title"],
            "relevance_score": round(chunk["score"], 3),
        }
        for chunk in chunks
    ]


async def generate_response(
    query: str,
    conversation_history: list[Message],
    context_chunks: list[dict[str, Any]],
) -> str:
    """
    Generate a response using OpenAI GPT-4.

    Args:
        query: User's question
        conversation_history: Previous messages
        context_chunks: Retrieved context

    Returns:
        Generated response text
    """
    client = get_openai_client()

    # Build messages list
    messages = [{"role": "system", "content": SYSTEM_PROMPT}]

    # Add retrieved context if available
    context_text = format_context_for_prompt(context_chunks)
    if context_text:
        messages.append({
            "role": "system",
            "content": context_text,
        })

    # Add conversation history
    for msg in conversation_history[-MAX_CONTEXT_MESSAGES:]:
        messages.append({
            "role": msg.role,
            "content": msg.content,
        })

    # Add current query
    messages.append({"role": "user", "content": query})

    # Generate response
    response = await client.chat.completions.create(
        model=settings.openai_model,
        messages=messages,
        max_tokens=1500,
        temperature=0.7,
    )

    return response.choices[0].message.content or ""


async def generate_response_stream(
    query: str,
    conversation_history: list[Message],
    context_chunks: list[dict[str, Any]],
) -> AsyncGenerator[str, None]:
    """
    Generate a streaming response using OpenAI GPT-4.

    Args:
        query: User's question
        conversation_history: Previous messages
        context_chunks: Retrieved context

    Yields:
        Response text chunks
    """
    client = get_openai_client()

    # Build messages list
    messages = [{"role": "system", "content": SYSTEM_PROMPT}]

    # Add retrieved context if available
    context_text = format_context_for_prompt(context_chunks)
    if context_text:
        messages.append({
            "role": "system",
            "content": context_text,
        })

    # Add conversation history
    for msg in conversation_history[-MAX_CONTEXT_MESSAGES:]:
        messages.append({
            "role": msg.role,
            "content": msg.content,
        })

    # Add current query
    messages.append({"role": "user", "content": query})

    # Generate streaming response
    stream = await client.chat.completions.create(
        model=settings.openai_model,
        messages=messages,
        max_tokens=1500,
        temperature=0.7,
        stream=True,
    )

    async for chunk in stream:
        if chunk.choices[0].delta.content:
            yield chunk.choices[0].delta.content


async def chat(
    session: AsyncSession,
    conversation_id: uuid.UUID,
    query: str,
    chapter_context: str | None = None,
) -> tuple[str, list[dict[str, Any]]]:
    """
    Process a chat message and generate a response.

    Args:
        session: Database session
        conversation_id: Conversation ID
        query: User's question
        chapter_context: Optional chapter filter

    Returns:
        Tuple of (response text, citations)
    """
    # Get conversation history
    history = await get_conversation_messages(session, conversation_id)

    # Add user message
    await add_message(session, conversation_id, "user", query)

    # Retrieve relevant context
    context_chunks = await retrieve_context(query, chapter_context)

    # Generate response
    response = await generate_response(query, history, context_chunks)

    # Format citations
    citations = format_citations(context_chunks)

    # Add assistant message
    await add_message(
        session,
        conversation_id,
        "assistant",
        response,
        citations=citations,
    )

    return response, citations


async def chat_stream(
    session: AsyncSession,
    conversation_id: uuid.UUID,
    query: str,
    chapter_context: str | None = None,
) -> AsyncGenerator[dict[str, Any], None]:
    """
    Process a chat message and stream the response.

    Args:
        session: Database session
        conversation_id: Conversation ID
        query: User's question
        chapter_context: Optional chapter filter

    Yields:
        Dict with 'type' ('chunk', 'citations', 'done') and data
    """
    # Get conversation history
    history = await get_conversation_messages(session, conversation_id)

    # Add user message
    await add_message(session, conversation_id, "user", query)

    # Retrieve relevant context
    context_chunks = await retrieve_context(query, chapter_context)

    # Yield citations first
    citations = format_citations(context_chunks)
    if citations:
        yield {"type": "citations", "data": citations}

    # Stream response
    full_response = ""
    async for chunk in generate_response_stream(query, history, context_chunks):
        full_response += chunk
        yield {"type": "chunk", "data": chunk}

    # Add assistant message with full response
    await add_message(
        session,
        conversation_id,
        "assistant",
        full_response,
        citations=citations,
    )

    yield {"type": "done", "data": None}


async def explain_text(
    selected_text: str,
    chapter_id: str | None = None,
) -> tuple[str, list[dict[str, Any]]]:
    """
    Explain selected text from the textbook.

    Args:
        selected_text: Text the user selected
        chapter_id: Chapter the text is from

    Returns:
        Tuple of (explanation, citations)
    """
    client = get_openai_client()

    # Retrieve additional context
    context_chunks = await retrieve_context(
        f"Explain: {selected_text}",
        chapter_filter=chapter_id,
        limit=3,
    )

    # Build prompt
    context_text = format_context_for_prompt(context_chunks)

    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "system", "content": context_text} if context_text else None,
        {
            "role": "user",
            "content": f"Please explain this text from the textbook in simple terms:\n\n\"{selected_text}\"\n\nProvide a clear, concise explanation that helps a student understand this concept.",
        },
    ]
    messages = [m for m in messages if m]  # Remove None

    response = await client.chat.completions.create(
        model=settings.openai_model,
        messages=messages,
        max_tokens=500,
        temperature=0.7,
    )

    explanation = response.choices[0].message.content or ""
    citations = format_citations(context_chunks)

    return explanation, citations

