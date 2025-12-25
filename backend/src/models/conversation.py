"""Conversation model for chatbot sessions."""

from datetime import datetime
from typing import TYPE_CHECKING, Any
import uuid

from sqlalchemy import DateTime, ForeignKey, Integer, String, func
from sqlalchemy.dialects.postgresql import JSONB, UUID
from sqlalchemy.orm import Mapped, mapped_column, relationship

from src.models.base import Base, TimestampMixin, UUIDMixin

if TYPE_CHECKING:
    from src.models.message import Message
    from src.models.user import User


class Conversation(Base, UUIDMixin, TimestampMixin):
    """
    Stores chatbot conversation sessions.

    Attributes:
        id: Unique conversation identifier (UUID)
        user_id: Optional user reference (NULL for anonymous)
        context: Conversation context (JSONB)
        chapter_context: Chapter user was viewing
        message_count: Number of messages in conversation
    """

    __tablename__ = "conversations"

    user_id: Mapped[uuid.UUID | None] = mapped_column(
        UUID(as_uuid=True),
        ForeignKey("users.id", ondelete="SET NULL"),
        nullable=True,
        index=True,
    )
    context: Mapped[dict[str, Any]] = mapped_column(
        JSONB,
        nullable=False,
        default=dict,
        server_default="{}",
    )
    chapter_context: Mapped[str | None] = mapped_column(
        String(100),
        nullable=True,
    )
    message_count: Mapped[int] = mapped_column(
        Integer,
        nullable=False,
        default=0,
        server_default="0",
    )

    # Relationships
    messages: Mapped[list["Message"]] = relationship(
        "Message",
        back_populates="conversation",
        order_by="Message.created_at",
        cascade="all, delete-orphan",
    )
    user: Mapped["User | None"] = relationship(
        "User",
        back_populates="conversations",
    )

    def __repr__(self) -> str:
        return f"<Conversation(id={self.id!r}, messages={self.message_count})>"

    def increment_message_count(self) -> None:
        """Increment the message count."""
        self.message_count += 1
