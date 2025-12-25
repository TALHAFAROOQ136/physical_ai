"""Initial schema for modules and chapters.

Revision ID: 001_initial_schema
Revises:
Create Date: 2024-12-23

"""

from typing import Sequence

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision: str = "001_initial_schema"
down_revision: str | None = None
branch_labels: Sequence[str] | None = None
depends_on: Sequence[str] | None = None


def upgrade() -> None:
    """Create modules and chapters tables."""
    # Create modules table
    op.create_table(
        "modules",
        sa.Column("id", sa.String(50), primary_key=True),
        sa.Column("title", sa.String(200), nullable=False),
        sa.Column("description", sa.Text(), nullable=False),
        sa.Column("order_sequence", sa.Integer(), nullable=False, unique=True),
        sa.Column(
            "objectives",
            postgresql.JSONB(astext_type=sa.Text()),
            nullable=False,
            server_default="[]",
        ),
        sa.Column("icon", sa.String(50), nullable=True),
    )

    # Create chapters table
    op.create_table(
        "chapters",
        sa.Column("id", sa.String(100), primary_key=True),
        sa.Column(
            "module_id",
            sa.String(50),
            sa.ForeignKey("modules.id", ondelete="CASCADE"),
            nullable=False,
            index=True,
        ),
        sa.Column("title", sa.String(200), nullable=False),
        sa.Column("slug", sa.String(100), nullable=False, unique=True, index=True),
        sa.Column("order_sequence", sa.Integer(), nullable=False),
        sa.Column("estimated_time_min", sa.Integer(), nullable=False, server_default="15"),
        sa.Column(
            "objectives",
            postgresql.JSONB(astext_type=sa.Text()),
            nullable=False,
            server_default="[]",
        ),
        sa.Column(
            "prerequisites",
            postgresql.JSONB(astext_type=sa.Text()),
            nullable=False,
            server_default="[]",
        ),
        sa.Column("difficulty", sa.String(20), nullable=False, server_default="beginner"),
    )

    # Create composite index for chapter ordering within module
    op.create_index(
        "ix_chapters_module_order",
        "chapters",
        ["module_id", "order_sequence"],
    )


def downgrade() -> None:
    """Drop modules and chapters tables."""
    op.drop_index("ix_chapters_module_order", table_name="chapters")
    op.drop_table("chapters")
    op.drop_table("modules")
