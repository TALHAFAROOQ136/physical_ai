"""Authentication API endpoints."""

from datetime import datetime
from typing import Annotated

from fastapi import APIRouter, Cookie, Depends, HTTPException, Request, Response, status
from pydantic import BaseModel, EmailStr, Field
from sqlalchemy.ext.asyncio import AsyncSession

from src.lib.database import get_db
from src.services import auth_service

router = APIRouter()


# Request/Response Models
class SignupRequest(BaseModel):
    """Request model for user signup."""

    email: EmailStr
    password: str = Field(..., min_length=8)
    display_name: str | None = Field(None, max_length=100)


class SigninRequest(BaseModel):
    """Request model for user signin."""

    email: EmailStr
    password: str


class ForgotPasswordRequest(BaseModel):
    """Request model for password reset request."""

    email: EmailStr


class ResetPasswordRequest(BaseModel):
    """Request model for password reset."""

    token: str
    new_password: str = Field(..., min_length=8)


class UserResponse(BaseModel):
    """Response model for user data."""

    id: str
    email: str
    display_name: str | None
    email_verified: bool
    created_at: datetime

    class Config:
        from_attributes = True


class SessionResponse(BaseModel):
    """Response model for session data."""

    id: str
    expires_at: datetime


class AuthResponse(BaseModel):
    """Response model for authentication."""

    user: UserResponse
    session: SessionResponse


class SessionInfoResponse(BaseModel):
    """Response model for session info."""

    user: UserResponse
    session_id: str
    expires_at: datetime


class ErrorResponse(BaseModel):
    """Response model for errors."""

    code: str
    message: str
    details: dict | None = None


# Helper functions
def get_session_cookie(session: str | None = Cookie(None)) -> str | None:
    """Extract session token from cookie."""
    return session


async def get_current_user(
    db: Annotated[AsyncSession, Depends(get_db)],
    session_token: Annotated[str | None, Depends(get_session_cookie)],
) -> tuple[auth_service.Session, auth_service.User]:
    """
    Dependency to get the current authenticated user.

    Raises:
        HTTPException: If not authenticated
    """
    if not session_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"code": "NOT_AUTHENTICATED", "message": "Not authenticated"},
        )

    result = await auth_service.get_session(db, session_token)
    if not result:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"code": "SESSION_EXPIRED", "message": "Session expired or invalid"},
        )

    return result


def set_session_cookie(response: Response, session: auth_service.Session) -> None:
    """Set session cookie on response."""
    response.set_cookie(
        key="session",
        value=session.id,
        httponly=True,
        secure=True,
        samesite="strict",
        max_age=int((session.expires_at - datetime.utcnow()).total_seconds()),
    )


def clear_session_cookie(response: Response) -> None:
    """Clear session cookie from response."""
    response.delete_cookie(key="session", httponly=True, secure=True, samesite="strict")


# Endpoints
@router.post(
    "/signup",
    response_model=AuthResponse,
    status_code=status.HTTP_201_CREATED,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid input"},
        409: {"model": ErrorResponse, "description": "Email already registered"},
    },
)
async def signup(
    request: Request,
    response: Response,
    body: SignupRequest,
    db: Annotated[AsyncSession, Depends(get_db)],
) -> AuthResponse:
    """Register a new user account."""
    try:
        user = await auth_service.create_user(
            db=db,
            email=body.email,
            password=body.password,
            display_name=body.display_name,
        )
    except auth_service.EmailAlreadyExistsError as e:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail={"code": e.code, "message": e.message},
        )
    except auth_service.WeakPasswordError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"code": e.code, "message": e.message},
        )

    # Create session
    session = await auth_service.create_session(
        db=db,
        user=user,
        user_agent=request.headers.get("user-agent"),
        ip_address=request.client.host if request.client else None,
    )

    set_session_cookie(response, session)

    return AuthResponse(
        user=UserResponse(
            id=str(user.id),
            email=user.email,
            display_name=user.display_name,
            email_verified=user.email_verified,
            created_at=user.created_at,
        ),
        session=SessionResponse(
            id=session.id,
            expires_at=session.expires_at,
        ),
    )


@router.post(
    "/signin",
    response_model=AuthResponse,
    responses={
        401: {"model": ErrorResponse, "description": "Invalid credentials"},
    },
)
async def signin(
    request: Request,
    response: Response,
    body: SigninRequest,
    db: Annotated[AsyncSession, Depends(get_db)],
) -> AuthResponse:
    """Authenticate a user and create a session."""
    try:
        user = await auth_service.authenticate_user(
            db=db,
            email=body.email,
            password=body.password,
        )
    except auth_service.InvalidCredentialsError as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"code": e.code, "message": e.message},
        )

    # Create session
    session = await auth_service.create_session(
        db=db,
        user=user,
        user_agent=request.headers.get("user-agent"),
        ip_address=request.client.host if request.client else None,
    )

    set_session_cookie(response, session)

    return AuthResponse(
        user=UserResponse(
            id=str(user.id),
            email=user.email,
            display_name=user.display_name,
            email_verified=user.email_verified,
            created_at=user.created_at,
        ),
        session=SessionResponse(
            id=session.id,
            expires_at=session.expires_at,
        ),
    )


@router.post("/signout")
async def signout(
    response: Response,
    db: Annotated[AsyncSession, Depends(get_db)],
    auth: Annotated[tuple, Depends(get_current_user)],
) -> dict[str, str]:
    """Sign out the current session."""
    session, _ = auth
    await auth_service.delete_session(db, session.id)
    clear_session_cookie(response)
    return {"message": "Signed out successfully"}


@router.post("/signout-all")
async def signout_all(
    response: Response,
    db: Annotated[AsyncSession, Depends(get_db)],
    auth: Annotated[tuple, Depends(get_current_user)],
) -> dict[str, str]:
    """Sign out from all devices."""
    _, user = auth
    await auth_service.delete_all_user_sessions(db, user.id)
    clear_session_cookie(response)
    return {"message": "Signed out from all devices"}


@router.post(
    "/forgot-password",
    responses={
        429: {"model": ErrorResponse, "description": "Too many requests"},
    },
)
async def forgot_password(
    body: ForgotPasswordRequest,
    db: Annotated[AsyncSession, Depends(get_db)],
) -> dict[str, str]:
    """Request a password reset email."""
    user = await auth_service.get_user_by_email(db, body.email)

    if user:
        # Create reset token
        token = await auth_service.create_password_reset_token(db, user)
        # TODO: Send email with reset link
        # In production, this would send an email
        # For now, we just return success (don't reveal if email exists)

    # Always return success to prevent email enumeration
    return {"message": "If an account with that email exists, a reset link has been sent"}


@router.post(
    "/reset-password",
    responses={
        400: {"model": ErrorResponse, "description": "Invalid or expired token"},
    },
)
async def reset_password(
    body: ResetPasswordRequest,
    db: Annotated[AsyncSession, Depends(get_db)],
) -> dict[str, str]:
    """Reset password using a reset token."""
    try:
        await auth_service.reset_password(
            db=db,
            token=body.token,
            new_password=body.new_password,
        )
    except auth_service.InvalidTokenError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"code": e.code, "message": e.message},
        )
    except auth_service.WeakPasswordError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={"code": e.code, "message": e.message},
        )

    return {"message": "Password reset successfully"}


@router.get(
    "/session",
    response_model=SessionInfoResponse,
    responses={
        401: {"model": ErrorResponse, "description": "Not authenticated"},
    },
)
async def get_session_info(
    auth: Annotated[tuple, Depends(get_current_user)],
) -> SessionInfoResponse:
    """Get current session information."""
    session, user = auth

    return SessionInfoResponse(
        user=UserResponse(
            id=str(user.id),
            email=user.email,
            display_name=user.display_name,
            email_verified=user.email_verified,
            created_at=user.created_at,
        ),
        session_id=session.id,
        expires_at=session.expires_at,
    )
