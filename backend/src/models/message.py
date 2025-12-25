"""Message model for individual chat messages."""

from datetime import datetime
from typing import TYPE_CHECKING, Any, Literal
import uuid

from sqlalchemy import DateTime, ForeignKey, String, Text, func
from sqlalchemy.dialects.postgresql import JSONB, UUID
from sqlalchemy.orm import Mapped, mapped_column, relationship

from src.models.base import Base, UUIDMixin

if TYPE_CHECKING:
    from src.models.conversation import Conversation


MessageRole = Literal["user", "assistant"]


class Message(Base, UUIDMixin):
    """
    Individual messages within a conversation.

    Attributes:
        id: Unique message identifier (UUID)
        conversation_id: Parent conversation reference
        role: 'user' or 'assistant'
        content: Message text content
        citations: Source citations array (JSONB)
        metadata: Additional metadata (JSONB)
        created_at: Message timestamp
    """

    __tablename__ = "messages"

    conversation_id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True),
        ForeignKey("conversations.id", ondelete="CASCADE"),
        nullable=False,
        index=True,
    )
    role: Mapped[str] = mapped_column(
        String(20),
        nullable=False,
    )
    content: Mapped[str] = mapped_column(
        Text,
        nullable=False,
    )
    citations: Mapped[list[dict[str, Any]]] = mapped_column(
        JSONB,
        nullable=False,
        default=list,
        server_default="[]",
    )
    metadata: Mapped[dict[str, Any]] = mapped_column(
        JSONB,
        nullable=False,
        default=dict,
        server_default="{}",
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        server_default=func.now(),
        nullable=False,
    )

    # Relationships
    conversation: Mapped["Conversation"] = relationship(
        "Conversation",
        back_populates="messages",
    )

    def __repr__(self) -> str:
        preview = self.content[:50] + "..." if len(self.content) > 50 else self.content
        return f"<Message(id={self.id!r}, role={self.role!r}, content={preview!r})>"

    @property
    def is_user(self) -> bool:
        """Check if message is from user."""
        return self.role == "user"

    @property
    def is_assistant(self) -> bool:
        """Check if message is from assistant."""
        return self.role == "assistant"
