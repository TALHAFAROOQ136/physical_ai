"""Add users and progress tables.

Revision ID: 003_users_progress
Revises: 002_conversations
Create Date: 2024-12-24

"""

from typing import Sequence

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision: str = "003_users_progress"
down_revision: str = "002_conversations"
branch_labels: Sequence[str] | None = None
depends_on: Sequence[str] | None = None


def upgrade() -> None:
    """Create users and progress tables."""
    # Enable UUID extension if not exists
    op.execute('CREATE EXTENSION IF NOT EXISTS "uuid-ossp"')

    # Create users table
    op.create_table(
        "users",
        sa.Column(
            "id",
            postgresql.UUID(as_uuid=True),
            primary_key=True,
            server_default=sa.text("uuid_generate_v4()"),
            index=True,
        ),
        sa.Column("email", sa.String(255), nullable=False, unique=True, index=True),
        sa.Column("password_hash", sa.String(255), nullable=False),
        sa.Column("display_name", sa.String(100), nullable=True),
        sa.Column(
            "background",
            postgresql.JSONB(astext_type=sa.Text()),
            nullable=False,
            server_default="{}",
        ),
        sa.Column("language_pref", sa.String(10), nullable=False, server_default="en"),
        sa.Column("email_verified", sa.Boolean(), nullable=False, server_default="false"),
        sa.Column(
            "created_at",
            sa.DateTime(timezone=True),
            nullable=False,
            server_default=sa.func.now(),
        ),
        sa.Column(
            "updated_at",
            sa.DateTime(timezone=True),
            nullable=False,
            server_default=sa.func.now(),
            onupdate=sa.func.now(),
        ),
    )

    # Create index on created_at for user queries
    op.create_index(
        "ix_users_created",
        "users",
        ["created_at"],
    )

    # Create progress table
    op.create_table(
        "progress",
        sa.Column(
            "id",
            postgresql.UUID(as_uuid=True),
            primary_key=True,
            server_default=sa.text("uuid_generate_v4()"),
        ),
        sa.Column(
            "user_id",
            postgresql.UUID(as_uuid=True),
            sa.ForeignKey("users.id", ondelete="CASCADE"),
            nullable=False,
            index=True,
        ),
        sa.Column(
            "chapter_id",
            sa.String(100),
            sa.ForeignKey("chapters.id", ondelete="CASCADE"),
            nullable=False,
            index=True,
        ),
        sa.Column(
            "status",
            sa.String(20),
            nullable=False,
            server_default="not_started",
        ),
        sa.Column("completed_at", sa.DateTime(timezone=True), nullable=True),
        sa.Column("reading_time_seconds", sa.Integer(), nullable=False, server_default="0"),
        sa.Column("last_position", sa.String(100), nullable=True),
        sa.Column(
            "created_at",
            sa.DateTime(timezone=True),
            nullable=False,
            server_default=sa.func.now(),
        ),
        sa.Column(
            "updated_at",
            sa.DateTime(timezone=True),
            nullable=False,
            server_default=sa.func.now(),
            onupdate=sa.func.now(),
        ),
    )

    # Create unique constraint for user-chapter pair
    op.create_unique_constraint(
        "uq_progress_user_chapter",
        "progress",
        ["user_id", "chapter_id"],
    )

    # Create composite index for progress queries
    op.create_index(
        "ix_progress_user_chapter",
        "progress",
        ["user_id", "chapter_id"],
    )

    # Create index for status filtering
    op.create_index(
        "ix_progress_user_status",
        "progress",
        ["user_id", "status"],
    )

    # Add check constraint for status values
    op.create_check_constraint(
        "ck_progress_status",
        "progress",
        "status IN ('not_started', 'in_progress', 'completed')",
    )

    # Add check constraint for language preference
    op.create_check_constraint(
        "ck_users_language_pref",
        "users",
        "language_pref IN ('en', 'ur')",
    )

    # Create sessions table for Better-Auth
    op.create_table(
        "sessions",
        sa.Column("id", sa.String(255), primary_key=True),
        sa.Column(
            "user_id",
            postgresql.UUID(as_uuid=True),
            sa.ForeignKey("users.id", ondelete="CASCADE"),
            nullable=False,
            index=True,
        ),
        sa.Column("expires_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column("user_agent", sa.Text(), nullable=True),
        sa.Column("ip_address", sa.String(45), nullable=True),
        sa.Column(
            "created_at",
            sa.DateTime(timezone=True),
            nullable=False,
            server_default=sa.func.now(),
        ),
    )

    # Create index on expires_at for cleanup queries
    op.create_index(
        "ix_sessions_expires",
        "sessions",
        ["expires_at"],
    )

    # Create password reset tokens table
    op.create_table(
        "password_reset_tokens",
        sa.Column("id", sa.String(255), primary_key=True),
        sa.Column(
            "user_id",
            postgresql.UUID(as_uuid=True),
            sa.ForeignKey("users.id", ondelete="CASCADE"),
            nullable=False,
            index=True,
        ),
        sa.Column("expires_at", sa.DateTime(timezone=True), nullable=False),
        sa.Column(
            "created_at",
            sa.DateTime(timezone=True),
            nullable=False,
            server_default=sa.func.now(),
        ),
    )

    # Add FK constraint for conversations.user_id now that users table exists
    op.create_foreign_key(
        "fk_conversations_user_id",
        "conversations",
        "users",
        ["user_id"],
        ["id"],
        ondelete="SET NULL",
    )


def downgrade() -> None:
    """Drop users and progress tables."""
    # Drop FK constraint for conversations.user_id first
    op.drop_constraint("fk_conversations_user_id", "conversations", type_="foreignkey")
    op.drop_table("password_reset_tokens")
    op.drop_index("ix_sessions_expires", table_name="sessions")
    op.drop_table("sessions")
    op.drop_constraint("ck_users_language_pref", "users", type_="check")
    op.drop_constraint("ck_progress_status", "progress", type_="check")
    op.drop_index("ix_progress_user_status", table_name="progress")
    op.drop_index("ix_progress_user_chapter", table_name="progress")
    op.drop_constraint("uq_progress_user_chapter", "progress", type_="unique")
    op.drop_table("progress")
    op.drop_index("ix_users_created", table_name="users")
    op.drop_table("users")
