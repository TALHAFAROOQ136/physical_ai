"""Authentication service for user management and session handling."""

import secrets
import uuid
from datetime import datetime, timedelta, timezone
from typing import Any

from argon2 import PasswordHasher
from argon2.exceptions import VerifyMismatchError
from sqlalchemy import select, delete
from sqlalchemy.ext.asyncio import AsyncSession

from src.lib.config import get_settings
from src.models.user import User

settings = get_settings()
ph = PasswordHasher()

# Session duration
SESSION_DURATION_DAYS = 30
PASSWORD_RESET_DURATION_HOURS = 1


class AuthError(Exception):
    """Base exception for authentication errors."""

    def __init__(self, message: str, code: str = "AUTH_ERROR"):
        self.message = message
        self.code = code
        super().__init__(message)


class InvalidCredentialsError(AuthError):
    """Raised when credentials are invalid."""

    def __init__(self):
        super().__init__("Invalid email or password", "INVALID_CREDENTIALS")


class EmailAlreadyExistsError(AuthError):
    """Raised when email is already registered."""

    def __init__(self):
        super().__init__("Email already registered", "EMAIL_EXISTS")


class InvalidTokenError(AuthError):
    """Raised when token is invalid or expired."""

    def __init__(self):
        super().__init__("Invalid or expired token", "INVALID_TOKEN")


class WeakPasswordError(AuthError):
    """Raised when password doesn't meet requirements."""

    def __init__(self, message: str = "Password does not meet requirements"):
        super().__init__(message, "WEAK_PASSWORD")


def validate_password(password: str) -> None:
    """
    Validate password meets requirements.

    Requirements:
    - At least 8 characters
    - At least 1 number
    - At least 1 special character
    """
    if len(password) < 8:
        raise WeakPasswordError("Password must be at least 8 characters")

    has_number = any(c.isdigit() for c in password)
    has_special = any(c in "!@#$%^&*()_+-=[]{}|;:',.<>?/`~" for c in password)

    if not has_number:
        raise WeakPasswordError("Password must contain at least one number")
    if not has_special:
        raise WeakPasswordError("Password must contain at least one special character")


def hash_password(password: str) -> str:
    """Hash a password using Argon2id."""
    return ph.hash(password)


def verify_password(password: str, password_hash: str) -> bool:
    """Verify a password against its hash."""
    try:
        ph.verify(password_hash, password)
        return True
    except VerifyMismatchError:
        return False


def generate_session_token() -> str:
    """Generate a secure session token."""
    return secrets.token_urlsafe(32)


def generate_reset_token() -> str:
    """Generate a secure password reset token."""
    return secrets.token_urlsafe(32)


async def create_user(
    db: AsyncSession,
    email: str,
    password: str,
    display_name: str | None = None,
) -> User:
    """
    Create a new user account.

    Args:
        db: Database session
        email: User's email address
        password: Plain text password
        display_name: Optional display name

    Returns:
        Created User object

    Raises:
        EmailAlreadyExistsError: If email is already registered
        WeakPasswordError: If password doesn't meet requirements
    """
    # Validate password
    validate_password(password)

    # Check if email exists
    result = await db.execute(select(User).where(User.email == email.lower()))
    if result.scalar_one_or_none():
        raise EmailAlreadyExistsError()

    # Create user
    user = User(
        email=email.lower(),
        password_hash=hash_password(password),
        display_name=display_name,
    )
    db.add(user)
    await db.flush()
    await db.refresh(user)

    return user


async def authenticate_user(
    db: AsyncSession,
    email: str,
    password: str,
) -> User:
    """
    Authenticate a user with email and password.

    Args:
        db: Database session
        email: User's email address
        password: Plain text password

    Returns:
        Authenticated User object

    Raises:
        InvalidCredentialsError: If credentials are invalid
    """
    result = await db.execute(select(User).where(User.email == email.lower()))
    user = result.scalar_one_or_none()

    if not user or not verify_password(password, user.password_hash):
        raise InvalidCredentialsError()

    return user


async def get_user_by_id(db: AsyncSession, user_id: uuid.UUID) -> User | None:
    """Get a user by their ID."""
    result = await db.execute(select(User).where(User.id == user_id))
    return result.scalar_one_or_none()


async def get_user_by_email(db: AsyncSession, email: str) -> User | None:
    """Get a user by their email address."""
    result = await db.execute(select(User).where(User.email == email.lower()))
    return result.scalar_one_or_none()


async def update_user(
    db: AsyncSession,
    user: User,
    **kwargs: Any,
) -> User:
    """
    Update a user's profile.

    Args:
        db: Database session
        user: User to update
        **kwargs: Fields to update

    Returns:
        Updated User object
    """
    allowed_fields = {"display_name", "language_pref"}

    for key, value in kwargs.items():
        if key in allowed_fields:
            setattr(user, key, value)

    await db.flush()
    await db.refresh(user)
    return user


async def update_user_background(
    db: AsyncSession,
    user: User,
    background: dict[str, Any],
) -> User:
    """
    Update a user's background assessment.

    Args:
        db: Database session
        user: User to update
        background: Background assessment data

    Returns:
        Updated User object
    """
    user.background = background
    await db.flush()
    await db.refresh(user)
    return user


async def update_password(
    db: AsyncSession,
    user: User,
    new_password: str,
) -> None:
    """
    Update a user's password.

    Args:
        db: Database session
        user: User to update
        new_password: New plain text password

    Raises:
        WeakPasswordError: If password doesn't meet requirements
    """
    validate_password(new_password)
    user.password_hash = hash_password(new_password)
    await db.flush()


async def delete_user(db: AsyncSession, user: User) -> None:
    """
    Delete a user account and all associated data.

    Args:
        db: Database session
        user: User to delete
    """
    await db.delete(user)
    await db.flush()


# Session management using database
class Session:
    """Session model for database storage."""

    def __init__(
        self,
        id: str,
        user_id: uuid.UUID,
        expires_at: datetime,
        user_agent: str | None = None,
        ip_address: str | None = None,
        created_at: datetime | None = None,
    ):
        self.id = id
        self.user_id = user_id
        self.expires_at = expires_at
        self.user_agent = user_agent
        self.ip_address = ip_address
        self.created_at = created_at or datetime.now(timezone.utc)


async def create_session(
    db: AsyncSession,
    user: User,
    user_agent: str | None = None,
    ip_address: str | None = None,
) -> Session:
    """
    Create a new session for a user.

    Args:
        db: Database session
        user: User to create session for
        user_agent: Browser user agent string
        ip_address: Client IP address

    Returns:
        Created Session object
    """
    session = Session(
        id=generate_session_token(),
        user_id=user.id,
        expires_at=datetime.now(timezone.utc) + timedelta(days=SESSION_DURATION_DAYS),
        user_agent=user_agent,
        ip_address=ip_address,
    )

    # Insert session into database
    await db.execute(
        """
        INSERT INTO sessions (id, user_id, expires_at, user_agent, ip_address, created_at)
        VALUES (:id, :user_id, :expires_at, :user_agent, :ip_address, :created_at)
        """,
        {
            "id": session.id,
            "user_id": session.user_id,
            "expires_at": session.expires_at,
            "user_agent": session.user_agent,
            "ip_address": session.ip_address,
            "created_at": session.created_at,
        },
    )
    await db.flush()

    return session


async def get_session(db: AsyncSession, session_id: str) -> tuple[Session, User] | None:
    """
    Get a session and its associated user.

    Args:
        db: Database session
        session_id: Session token

    Returns:
        Tuple of (Session, User) if valid, None otherwise
    """
    result = await db.execute(
        """
        SELECT s.id, s.user_id, s.expires_at, s.user_agent, s.ip_address, s.created_at
        FROM sessions s
        WHERE s.id = :session_id AND s.expires_at > :now
        """,
        {"session_id": session_id, "now": datetime.now(timezone.utc)},
    )
    row = result.fetchone()

    if not row:
        return None

    session = Session(
        id=row[0],
        user_id=row[1],
        expires_at=row[2],
        user_agent=row[3],
        ip_address=row[4],
        created_at=row[5],
    )

    user = await get_user_by_id(db, session.user_id)
    if not user:
        return None

    return session, user


async def delete_session(db: AsyncSession, session_id: str) -> None:
    """Delete a session."""
    await db.execute(
        "DELETE FROM sessions WHERE id = :session_id",
        {"session_id": session_id},
    )
    await db.flush()


async def delete_all_user_sessions(db: AsyncSession, user_id: uuid.UUID) -> None:
    """Delete all sessions for a user."""
    await db.execute(
        "DELETE FROM sessions WHERE user_id = :user_id",
        {"user_id": user_id},
    )
    await db.flush()


# Password reset tokens
async def create_password_reset_token(db: AsyncSession, user: User) -> str:
    """
    Create a password reset token for a user.

    Args:
        db: Database session
        user: User requesting password reset

    Returns:
        Password reset token
    """
    token = generate_reset_token()
    expires_at = datetime.now(timezone.utc) + timedelta(hours=PASSWORD_RESET_DURATION_HOURS)

    await db.execute(
        """
        INSERT INTO password_reset_tokens (id, user_id, expires_at, created_at)
        VALUES (:id, :user_id, :expires_at, :created_at)
        """,
        {
            "id": token,
            "user_id": user.id,
            "expires_at": expires_at,
            "created_at": datetime.now(timezone.utc),
        },
    )
    await db.flush()

    return token


async def verify_password_reset_token(
    db: AsyncSession,
    token: str,
) -> User | None:
    """
    Verify a password reset token and return the associated user.

    Args:
        db: Database session
        token: Password reset token

    Returns:
        User if token is valid, None otherwise
    """
    result = await db.execute(
        """
        SELECT user_id FROM password_reset_tokens
        WHERE id = :token AND expires_at > :now
        """,
        {"token": token, "now": datetime.now(timezone.utc)},
    )
    row = result.fetchone()

    if not row:
        return None

    return await get_user_by_id(db, row[0])


async def delete_password_reset_token(db: AsyncSession, token: str) -> None:
    """Delete a password reset token."""
    await db.execute(
        "DELETE FROM password_reset_tokens WHERE id = :token",
        {"token": token},
    )
    await db.flush()


async def reset_password(
    db: AsyncSession,
    token: str,
    new_password: str,
) -> User:
    """
    Reset a user's password using a reset token.

    Args:
        db: Database session
        token: Password reset token
        new_password: New plain text password

    Returns:
        Updated User object

    Raises:
        InvalidTokenError: If token is invalid or expired
        WeakPasswordError: If password doesn't meet requirements
    """
    user = await verify_password_reset_token(db, token)
    if not user:
        raise InvalidTokenError()

    await update_password(db, user, new_password)
    await delete_password_reset_token(db, token)

    # Invalidate all existing sessions
    await delete_all_user_sessions(db, user.id)

    return user
