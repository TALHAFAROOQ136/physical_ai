"""Progress model for tracking user completion status for chapters."""

import uuid
from datetime import datetime
from typing import TYPE_CHECKING

from sqlalchemy import DateTime, ForeignKey, Integer, String, UniqueConstraint
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import Mapped, mapped_column, relationship

from src.models.base import Base, TimestampMixin, UUIDMixin

if TYPE_CHECKING:
    from src.models.chapter import Chapter
    from src.models.user import User


class ProgressStatus:
    """Progress status constants."""

    NOT_STARTED = "not_started"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"

    ALL = [NOT_STARTED, IN_PROGRESS, COMPLETED]


class Progress(Base, UUIDMixin, TimestampMixin):
    """
    Tracks user completion status for chapters.

    Attributes:
        id: UUID primary key
        user_id: User reference (FK)
        chapter_id: Chapter reference (FK)
        status: Progress status ('not_started', 'in_progress', 'completed')
        completed_at: When marked complete
        reading_time_seconds: Accumulated reading time
        last_position: Last scroll position/section
        created_at: First interaction time
        updated_at: Last update time
    """

    __tablename__ = "progress"

    user_id: Mapped[uuid.UUID] = mapped_column(
        UUID(as_uuid=True),
        ForeignKey("users.id", ondelete="CASCADE"),
        nullable=False,
        index=True,
    )
    chapter_id: Mapped[str] = mapped_column(
        String(100),
        ForeignKey("chapters.id", ondelete="CASCADE"),
        nullable=False,
        index=True,
    )
    status: Mapped[str] = mapped_column(
        String(20),
        nullable=False,
        default=ProgressStatus.NOT_STARTED,
        server_default=ProgressStatus.NOT_STARTED,
    )
    completed_at: Mapped[datetime | None] = mapped_column(
        DateTime(timezone=True),
        nullable=True,
    )
    reading_time_seconds: Mapped[int] = mapped_column(
        Integer,
        nullable=False,
        default=0,
        server_default="0",
    )
    last_position: Mapped[str | None] = mapped_column(
        String(100),
        nullable=True,
    )

    # Relationships
    user: Mapped["User"] = relationship("User", back_populates="progress")
    chapter: Mapped["Chapter"] = relationship("Chapter")

    # Constraints
    __table_args__ = (
        UniqueConstraint("user_id", "chapter_id", name="uq_progress_user_chapter"),
    )

    def __repr__(self) -> str:
        return f"<Progress(user_id={self.user_id!r}, chapter_id={self.chapter_id!r}, status={self.status!r})>"

    def mark_complete(self) -> None:
        """Mark this progress as completed."""
        self.status = ProgressStatus.COMPLETED
        self.completed_at = datetime.now()

    def mark_in_progress(self) -> None:
        """Mark this progress as in progress."""
        if self.status == ProgressStatus.NOT_STARTED:
            self.status = ProgressStatus.IN_PROGRESS

    def add_reading_time(self, seconds: int) -> None:
        """Add reading time to this progress."""
        if seconds > 0:
            self.reading_time_seconds += seconds
