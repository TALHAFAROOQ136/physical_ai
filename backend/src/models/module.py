"""Module model representing a learning unit grouping chapters."""

from typing import TYPE_CHECKING, Any

from sqlalchemy import Integer, String, Text
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.orm import Mapped, mapped_column, relationship

from src.models.base import Base

if TYPE_CHECKING:
    from src.models.chapter import Chapter


class Module(Base):
    """
    Groups related chapters into a learning unit.

    Attributes:
        id: Stable identifier (e.g., 'ros2', 'simulation')
        title: Display title
        description: Module overview
        order_sequence: Display order (1-4)
        objectives: Learning objectives array (JSONB)
        icon: Icon identifier for UI
    """

    __tablename__ = "modules"

    id: Mapped[str] = mapped_column(String(50), primary_key=True)
    title: Mapped[str] = mapped_column(String(200), nullable=False)
    description: Mapped[str] = mapped_column(Text, nullable=False)
    order_sequence: Mapped[int] = mapped_column(Integer, nullable=False, unique=True)
    objectives: Mapped[dict[str, Any]] = mapped_column(JSONB, nullable=False, default=list)
    icon: Mapped[str | None] = mapped_column(String(50), nullable=True)

    # Relationships
    chapters: Mapped[list["Chapter"]] = relationship(
        "Chapter",
        back_populates="module",
        order_by="Chapter.order_sequence",
    )

    def __repr__(self) -> str:
        return f"<Module(id={self.id!r}, title={self.title!r})>"
