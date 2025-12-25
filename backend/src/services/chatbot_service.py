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

# System prompt for the chatbot - English version
SYSTEM_PROMPT_EN = """You are an expert teaching assistant for the "Physical AI & Humanoid Robotics" textbook. Your role is to help students understand robotics concepts including:

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

# System prompt for the chatbot - Urdu version
SYSTEM_PROMPT_UR = """آپ "Physical AI & Humanoid Robotics" نصابی کتاب کے لیے ایک ماہر teaching assistant ہیں۔ آپ کا کام طلباء کو robotics کے تصورات سمجھنے میں مدد کرنا ہے جن میں شامل ہیں:

- ROS 2 (Robot Operating System 2)
- Gazebo اور Unity simulation
- NVIDIA Isaac Sim اور perception
- Vision-Language-Action (VLA) models
- Humanoid robot development

ہدایات:
1. اپنی وضاحتوں میں مختصر لیکن جامع رہیں
2. جب مددگار ہو تو code examples استعمال کریں (Python, C++, YAML, bash)
3. متعلقہ textbook chapters کا حوالہ دیں
4. hands-on learning اور تجربات کی حوصلہ افزائی کریں
5. اگر آپ کچھ نہیں جانتے تو ایمانداری سے کہیں
6. طالب علم کے پس منظر کی سطح کے مطابق وضاحتیں کریں

جواب دیتے وقت، آپ کو نصابی کتاب سے متعلقہ سیاق و سباق فراہم کیا جا سکتا ہے۔ درست، textbook-aligned جوابات فراہم کرنے کے لیے اس سیاق و سباق کا استعمال کریں۔ فراہم کردہ سیاق و سباق استعمال کرتے وقت ہمیشہ ذرائع کا حوالہ دیں۔

اہم: تمام جوابات اردو میں دیں۔ تاہم، تکنیکی اصطلاحات (جیسے ROS 2, node, topic, URDF) اور code انگریزی میں رکھیں۔"""

# Default system prompt (English)
SYSTEM_PROMPT = SYSTEM_PROMPT_EN

# Language codes
LANGUAGE_ENGLISH = "en"
LANGUAGE_URDU = "ur"

# Capstone-specific context for enhanced chatbot support
CAPSTONE_CONTEXT = """
Additional context for capstone project support:

The capstone project is a comprehensive 5-milestone project where students build a complete humanoid robot system:

**Milestone 1: System Design** (1 week)
- Define functional and non-functional requirements
- Create system architecture diagrams
- Select technologies (ROS 2 Humble, Isaac Sim, GPT-4, Whisper)
- Document design decisions

**Milestone 2: ROS 2 Foundation** (1 week)
- Create package structure with nodes for voice, planner, controller, perception
- Define custom messages, services, and actions
- Set up launch files and parameters
- Implement core node communication

**Milestone 3: Simulation Setup** (1 week)
- Build Gazebo/Isaac Sim world files
- Import robot URDF with sensors (camera, LIDAR)
- Configure sensor plugins
- Test basic robot behaviors

**Milestone 4: AI Integration** (1 week)
- Implement perception pipeline with object detection
- Create LLM-powered task planner using GPT-4
- Build voice interface using Whisper
- Integrate end-to-end pipeline

**Milestone 5: Presentation** (1 week)
- Create comprehensive documentation
- Record 5-minute demo video
- Prepare presentation slides
- Conduct final code review

Common capstone troubleshooting topics:
- ROS 2 node communication issues
- Gazebo plugin configuration
- Isaac Sim installation and setup
- LLM integration with ROS 2
- Voice recognition latency
- Simulation performance optimization
"""


def get_system_prompt(
    language: str = LANGUAGE_ENGLISH,
    include_capstone: bool = False,
) -> str:
    """
    Get the appropriate system prompt based on language preference.

    Args:
        language: Language code ('en' or 'ur')
        include_capstone: Whether to include capstone-specific context

    Returns:
        System prompt in the appropriate language
    """
    base_prompt = SYSTEM_PROMPT_UR if language == LANGUAGE_URDU else SYSTEM_PROMPT_EN

    if include_capstone:
        return f"{base_prompt}\n\n{CAPSTONE_CONTEXT}"

    return base_prompt


def is_capstone_related(query: str, chapter_context: str | None = None) -> bool:
    """
    Determine if a query is related to the capstone project.

    Args:
        query: User's question
        chapter_context: Optional chapter context

    Returns:
        True if the query is capstone-related
    """
    capstone_keywords = [
        "capstone", "milestone", "final project", "humanoid project",
        "project design", "system architecture", "deliverable",
        "presentation", "demo video", "grading",
    ]

    query_lower = query.lower()

    # Check if viewing capstone chapter
    if chapter_context and "capstone" in chapter_context.lower():
        return True

    # Check for capstone keywords in query
    return any(keyword in query_lower for keyword in capstone_keywords)


def detect_language(text: str) -> str:
    """
    Detect the language of the input text.

    Simple heuristic: check for Urdu Unicode characters.
    For production, consider using a library like langdetect.

    Args:
        text: Input text to analyze

    Returns:
        Detected language code ('en' or 'ur')
    """
    # Urdu Unicode range: 0600-06FF (Arabic script used for Urdu)
    urdu_chars = sum(1 for char in text if '\u0600' <= char <= '\u06FF')
    # If more than 10% of characters are Urdu, consider it Urdu
    if len(text) > 0 and urdu_chars / len(text) > 0.1:
        return LANGUAGE_URDU
    return LANGUAGE_ENGLISH

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
    language: str = LANGUAGE_ENGLISH,
    user_background: dict[str, Any] | None = None,
    chapter_context: str | None = None,
) -> str:
    """
    Generate a response using OpenAI GPT-4.

    Args:
        query: User's question
        conversation_history: Previous messages
        context_chunks: Retrieved context
        language: Response language ('en' or 'ur')
        user_background: User's background for personalization
        chapter_context: Optional chapter context for capstone detection

    Returns:
        Generated response text
    """
    client = get_openai_client()

    # Check if capstone-related and include extra context
    include_capstone = is_capstone_related(query, chapter_context)

    # Build system prompt with language preference and capstone context
    system_prompt = get_system_prompt(language, include_capstone=include_capstone)

    # Add user background context if available
    if user_background:
        background_context = format_user_background(user_background)
        system_prompt = f"{system_prompt}\n\nUser Background:\n{background_context}"

    # Build messages list
    messages = [{"role": "system", "content": system_prompt}]

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


def format_user_background(background: dict[str, Any]) -> str:
    """
    Format user background information for the system prompt.

    Args:
        background: User's background assessment data

    Returns:
        Formatted background string
    """
    parts = []

    python_level = background.get("python_level")
    if python_level:
        parts.append(f"- Python skill level: {python_level}")

    ros_experience = background.get("ros_experience")
    if ros_experience:
        parts.append(f"- ROS experience: {ros_experience}")

    hardware_access = background.get("hardware_access")
    if hardware_access:
        parts.append(f"- Hardware access: {hardware_access}")

    learning_goals = background.get("learning_goals")
    if learning_goals:
        parts.append(f"- Learning goals: {', '.join(learning_goals)}")

    return "\n".join(parts) if parts else "No background information available"


async def generate_response_stream(
    query: str,
    conversation_history: list[Message],
    context_chunks: list[dict[str, Any]],
    language: str = LANGUAGE_ENGLISH,
    user_background: dict[str, Any] | None = None,
    chapter_context: str | None = None,
) -> AsyncGenerator[str, None]:
    """
    Generate a streaming response using OpenAI GPT-4.

    Args:
        query: User's question
        conversation_history: Previous messages
        context_chunks: Retrieved context
        language: Response language ('en' or 'ur')
        user_background: User's background for personalization
        chapter_context: Optional chapter context for capstone detection

    Yields:
        Response text chunks
    """
    client = get_openai_client()

    # Check if capstone-related and include extra context
    include_capstone = is_capstone_related(query, chapter_context)

    # Build system prompt with language preference and capstone context
    system_prompt = get_system_prompt(language, include_capstone=include_capstone)

    # Add user background context if available
    if user_background:
        background_context = format_user_background(user_background)
        system_prompt = f"{system_prompt}\n\nUser Background:\n{background_context}"

    # Build messages list
    messages = [{"role": "system", "content": system_prompt}]

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
    language: str | None = None,
    user_background: dict[str, Any] | None = None,
) -> tuple[str, list[dict[str, Any]]]:
    """
    Process a chat message and generate a response.

    Args:
        session: Database session
        conversation_id: Conversation ID
        query: User's question
        chapter_context: Optional chapter filter
        language: Preferred response language (auto-detected if None)
        user_background: User's background for personalization

    Returns:
        Tuple of (response text, citations)
    """
    # Auto-detect language if not specified
    response_language = language or detect_language(query)

    # Get conversation history
    history = await get_conversation_messages(session, conversation_id)

    # Add user message
    await add_message(session, conversation_id, "user", query)

    # Retrieve relevant context
    context_chunks = await retrieve_context(query, chapter_context)

    # Generate response with language preference and capstone context
    response = await generate_response(
        query,
        history,
        context_chunks,
        language=response_language,
        user_background=user_background,
        chapter_context=chapter_context,
    )

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
    language: str | None = None,
    user_background: dict[str, Any] | None = None,
) -> AsyncGenerator[dict[str, Any], None]:
    """
    Process a chat message and stream the response.

    Args:
        session: Database session
        conversation_id: Conversation ID
        query: User's question
        chapter_context: Optional chapter filter
        language: Preferred response language (auto-detected if None)
        user_background: User's background for personalization

    Yields:
        Dict with 'type' ('chunk', 'citations', 'done') and data
    """
    # Auto-detect language if not specified
    response_language = language or detect_language(query)

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

    # Stream response with language preference and capstone context
    full_response = ""
    async for chunk in generate_response_stream(
        query,
        history,
        context_chunks,
        language=response_language,
        user_background=user_background,
        chapter_context=chapter_context,
    ):
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
    language: str | None = None,
) -> tuple[str, list[dict[str, Any]]]:
    """
    Explain selected text from the textbook.

    Args:
        selected_text: Text the user selected
        chapter_id: Chapter the text is from
        language: Preferred response language (auto-detected if None)

    Returns:
        Tuple of (explanation, citations)
    """
    client = get_openai_client()

    # Auto-detect language if not specified
    response_language = language or detect_language(selected_text)

    # Retrieve additional context
    context_chunks = await retrieve_context(
        f"Explain: {selected_text}",
        chapter_filter=chapter_id,
        limit=3,
    )

    # Build prompt with language preference
    system_prompt = get_system_prompt(response_language)
    context_text = format_context_for_prompt(context_chunks)

    # Build user prompt based on language
    if response_language == LANGUAGE_URDU:
        user_prompt = f"براہ کرم نصابی کتاب سے اس متن کی آسان الفاظ میں وضاحت کریں:\n\n\"{selected_text}\"\n\nایک واضح، مختصر وضاحت فراہم کریں جو طالب علم کو اس تصور کو سمجھنے میں مدد کرے۔ تکنیکی اصطلاحات انگریزی میں رکھیں۔"
    else:
        user_prompt = f"Please explain this text from the textbook in simple terms:\n\n\"{selected_text}\"\n\nProvide a clear, concise explanation that helps a student understand this concept."

    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "system", "content": context_text} if context_text else None,
        {"role": "user", "content": user_prompt},
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
