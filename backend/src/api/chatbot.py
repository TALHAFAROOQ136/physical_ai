"""Chatbot API endpoints with RAG support."""

import uuid
from typing import Any

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field
from sqlalchemy.ext.asyncio import AsyncSession

from src.lib.database import get_db
from src.services.chatbot_service import (
    add_message,
    chat,
    chat_stream,
    create_conversation,
    explain_text,
    get_conversation,
    get_conversation_messages,
)

router = APIRouter()


# Request/Response Models
class CreateConversationRequest(BaseModel):
    """Request to create a new conversation."""

    user_id: uuid.UUID | None = Field(None, description="Optional user ID")
    chapter_context: str | None = Field(None, description="Current chapter ID")
    context: dict[str, Any] | None = Field(None, description="Additional context")


class ConversationResponse(BaseModel):
    """Conversation response model."""

    id: uuid.UUID
    chapter_context: str | None
    message_count: int
    created_at: str
    updated_at: str

    class Config:
        from_attributes = True


class SendMessageRequest(BaseModel):
    """Request to send a message."""

    message: str = Field(..., min_length=1, max_length=2000, description="User message")
    chapter_context: str | None = Field(None, description="Current chapter for context")
    stream: bool = Field(True, description="Whether to stream the response")


class MessageResponse(BaseModel):
    """Individual message response."""

    id: uuid.UUID
    role: str
    content: str
    citations: list[dict[str, Any]]
    created_at: str

    class Config:
        from_attributes = True


class ChatResponse(BaseModel):
    """Non-streaming chat response."""

    message: MessageResponse
    citations: list[dict[str, Any]]


class ExplainRequest(BaseModel):
    """Request to explain selected text."""

    text: str = Field(..., min_length=1, max_length=1000, description="Text to explain")
    chapter_id: str | None = Field(None, description="Chapter the text is from")


class ExplainResponse(BaseModel):
    """Explanation response."""

    explanation: str
    citations: list[dict[str, Any]]


class FeedbackRequest(BaseModel):
    """Request to submit feedback on a message."""

    message_id: uuid.UUID = Field(..., description="Message to provide feedback on")
    rating: int = Field(..., ge=1, le=5, description="Rating 1-5")
    comment: str | None = Field(None, max_length=500, description="Optional comment")


class FeedbackResponse(BaseModel):
    """Feedback submission response."""

    success: bool
    message: str


# T051: POST /chatbot/conversations - Create new conversation
@router.post(
    "/conversations",
    response_model=ConversationResponse,
    status_code=status.HTTP_201_CREATED,
    summary="Create a new conversation",
    description="Start a new chatbot conversation session.",
)
async def create_new_conversation(
    request: CreateConversationRequest,
    session: AsyncSession = Depends(get_db),
) -> ConversationResponse:
    """Create a new conversation."""
    conversation = await create_conversation(
        session=session,
        user_id=request.user_id,
        chapter_context=request.chapter_context,
        context=request.context,
    )
    return ConversationResponse(
        id=conversation.id,
        chapter_context=conversation.chapter_context,
        message_count=conversation.message_count,
        created_at=conversation.created_at.isoformat(),
        updated_at=conversation.updated_at.isoformat(),
    )


# T052: POST /chatbot/conversations/{id}/messages - Send message with streaming
@router.post(
    "/conversations/{conversation_id}/messages",
    summary="Send a message",
    description="Send a message and receive a response (optionally streamed).",
)
async def send_message(
    conversation_id: uuid.UUID,
    request: SendMessageRequest,
    session: AsyncSession = Depends(get_db),
) -> StreamingResponse | ChatResponse:
    """Send a message and get a response."""
    # Verify conversation exists
    conversation = await get_conversation(session, conversation_id)
    if not conversation:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Conversation not found",
        )

    if request.stream:
        # Return streaming response using SSE
        async def event_generator():
            """Generate SSE events."""
            async for event in chat_stream(
                session=session,
                conversation_id=conversation_id,
                query=request.message,
                chapter_context=request.chapter_context,
            ):
                event_type = event["type"]
                data = event["data"]

                if event_type == "citations":
                    # Send citations as JSON
                    import json

                    yield f"event: citations\ndata: {json.dumps(data)}\n\n"
                elif event_type == "chunk":
                    # Send text chunk
                    yield f"event: chunk\ndata: {data}\n\n"
                elif event_type == "done":
                    yield "event: done\ndata: null\n\n"

        return StreamingResponse(
            event_generator(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",
            },
        )
    else:
        # Non-streaming response
        response_text, citations = await chat(
            session=session,
            conversation_id=conversation_id,
            query=request.message,
            chapter_context=request.chapter_context,
        )

        # Get the last assistant message
        messages = await get_conversation_messages(session, conversation_id, limit=1)
        last_message = messages[-1] if messages else None

        if not last_message:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Failed to save response",
            )

        return ChatResponse(
            message=MessageResponse(
                id=last_message.id,
                role=last_message.role,
                content=last_message.content,
                citations=last_message.citations,
                created_at=last_message.created_at.isoformat(),
            ),
            citations=citations,
        )


# GET conversation messages
@router.get(
    "/conversations/{conversation_id}/messages",
    response_model=list[MessageResponse],
    summary="Get conversation messages",
    description="Retrieve all messages in a conversation.",
)
async def get_messages(
    conversation_id: uuid.UUID,
    limit: int = 50,
    session: AsyncSession = Depends(get_db),
) -> list[MessageResponse]:
    """Get messages from a conversation."""
    # Verify conversation exists
    conversation = await get_conversation(session, conversation_id)
    if not conversation:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Conversation not found",
        )

    messages = await get_conversation_messages(session, conversation_id, limit=limit)

    return [
        MessageResponse(
            id=msg.id,
            role=msg.role,
            content=msg.content,
            citations=msg.citations,
            created_at=msg.created_at.isoformat(),
        )
        for msg in messages
    ]


# T053: POST /chatbot/explain - Explain selected text
@router.post(
    "/explain",
    response_model=ExplainResponse,
    summary="Explain text",
    description="Get an explanation for selected text from the textbook.",
)
async def explain_selected_text(
    request: ExplainRequest,
) -> ExplainResponse:
    """Explain selected text."""
    explanation, citations = await explain_text(
        selected_text=request.text,
        chapter_id=request.chapter_id,
    )

    return ExplainResponse(
        explanation=explanation,
        citations=citations,
    )


# T054: POST /chatbot/feedback - Submit feedback
@router.post(
    "/feedback",
    response_model=FeedbackResponse,
    summary="Submit feedback",
    description="Submit feedback on an assistant message.",
)
async def submit_feedback(
    request: FeedbackRequest,
    session: AsyncSession = Depends(get_db),
) -> FeedbackResponse:
    """Submit feedback on a message."""
    # In a full implementation, we'd store this in a feedback table
    # For now, we'll add it to the message metadata
    from sqlalchemy import select

    from src.models.message import Message

    result = await session.execute(
        select(Message).where(Message.id == request.message_id)
    )
    message = result.scalar_one_or_none()

    if not message:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Message not found",
        )

    if message.role != "assistant":
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Can only provide feedback on assistant messages",
        )

    # Update message metadata with feedback
    feedback_data = {
        "rating": request.rating,
        "comment": request.comment,
    }
    message.metadata = {**message.metadata, "feedback": feedback_data}
    await session.commit()

    return FeedbackResponse(
        success=True,
        message="Feedback submitted successfully",
    )


# GET conversation by ID
@router.get(
    "/conversations/{conversation_id}",
    response_model=ConversationResponse,
    summary="Get conversation",
    description="Get conversation details by ID.",
)
async def get_conversation_by_id(
    conversation_id: uuid.UUID,
    session: AsyncSession = Depends(get_db),
) -> ConversationResponse:
    """Get a conversation by ID."""
    conversation = await get_conversation(session, conversation_id)
    if not conversation:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Conversation not found",
        )

    return ConversationResponse(
        id=conversation.id,
        chapter_context=conversation.chapter_context,
        message_count=conversation.message_count,
        created_at=conversation.created_at.isoformat(),
        updated_at=conversation.updated_at.isoformat(),
    )
