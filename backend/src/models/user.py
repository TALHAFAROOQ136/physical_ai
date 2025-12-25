"""User model representing a registered learner on the platform."""

from typing import TYPE_CHECKING, Any

from sqlalchemy import Boolean, String
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.orm import Mapped, mapped_column, relationship

from src.models.base import Base, TimestampMixin, UUIDMixin

if TYPE_CHECKING:
    from src.models.conversation import Conversation
    from src.models.progress import Progress


class User(Base, UUIDMixin, TimestampMixin):
    """
    Represents a registered learner on the platform.

    Attributes:
        id: UUID primary key
        email: User's email address (unique)
        password_hash: Argon2id hashed password
        display_name: Optional display name
        background: Background assessment data (JSONB)
        language_pref: 'en' or 'ur'
        email_verified: Email verification status
        created_at: Account creation time
        updated_at: Last profile update
    """

    __tablename__ = "users"

    email: Mapped[str] = mapped_column(
        String(255),
        unique=True,
        nullable=False,
        index=True,
    )
    password_hash: Mapped[str] = mapped_column(String(255), nullable=False)
    display_name: Mapped[str | None] = mapped_column(String(100), nullable=True)
    background: Mapped[dict[str, Any]] = mapped_column(
        JSONB,
        nullable=False,
        default=dict,
        server_default="{}",
    )
    language_pref: Mapped[str] = mapped_column(
        String(10),
        nullable=False,
        default="en",
        server_default="en",
    )
    email_verified: Mapped[bool] = mapped_column(
        Boolean,
        nullable=False,
        default=False,
        server_default="false",
    )

    # Relationships
    progress: Mapped[list["Progress"]] = relationship(
        "Progress",
        back_populates="user",
        cascade="all, delete-orphan",
    )
    conversations: Mapped[list["Conversation"]] = relationship(
        "Conversation",
        back_populates="user",
        cascade="all, delete-orphan",
    )

    def __repr__(self) -> str:
        return f"<User(id={self.id!r}, email={self.email!r})>"

    @property
    def has_completed_assessment(self) -> bool:
        """Check if user has completed background assessment."""
        return self.background.get("completed_assessment", False)
