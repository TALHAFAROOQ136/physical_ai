"""Chapter model representing a unit of textbook content."""

from typing import TYPE_CHECKING, Any

from sqlalchemy import ForeignKey, Integer, String
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.orm import Mapped, mapped_column, relationship

from src.models.base import Base

if TYPE_CHECKING:
    from src.models.module import Module


class Chapter(Base):
    """
    A unit of textbook content within a module.

    Attributes:
        id: Stable identifier (e.g., 'ros2-nodes')
        module_id: Parent module reference
        title: Chapter display title
        slug: URL-friendly slug
        order_sequence: Order within module
        estimated_time_min: Estimated reading time in minutes
        objectives: Chapter learning objectives (JSONB)
        prerequisites: Required prior chapters (JSONB array)
        difficulty: 'beginner', 'intermediate', 'advanced'
    """

    __tablename__ = "chapters"

    id: Mapped[str] = mapped_column(String(100), primary_key=True)
    module_id: Mapped[str] = mapped_column(
        String(50),
        ForeignKey("modules.id", ondelete="CASCADE"),
        nullable=False,
        index=True,
    )
    title: Mapped[str] = mapped_column(String(200), nullable=False)
    slug: Mapped[str] = mapped_column(String(100), nullable=False, unique=True, index=True)
    order_sequence: Mapped[int] = mapped_column(Integer, nullable=False)
    estimated_time_min: Mapped[int] = mapped_column(Integer, nullable=False, default=15)
    objectives: Mapped[dict[str, Any]] = mapped_column(JSONB, nullable=False, default=list)
    prerequisites: Mapped[dict[str, Any]] = mapped_column(JSONB, nullable=False, default=list)
    difficulty: Mapped[str] = mapped_column(String(20), nullable=False, default="beginner")

    # Relationships
    module: Mapped["Module"] = relationship("Module", back_populates="chapters")

    def __repr__(self) -> str:
        return f"<Chapter(id={self.id!r}, title={self.title!r})>"
