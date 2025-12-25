"""Users API endpoints for profile and background management."""

from datetime import datetime
from typing import Annotated, Any

from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel, EmailStr, Field
from sqlalchemy.ext.asyncio import AsyncSession

from src.lib.database import get_db
from src.services import auth_service
from src.api.auth import get_current_user, ErrorResponse

router = APIRouter()


# Request/Response Models
class BackgroundAssessment(BaseModel):
    """Background assessment data model."""

    python_level: str | None = Field(None, pattern="^(beginner|intermediate|advanced)$")
    ros_experience: str | None = Field(None, pattern="^(none|basic|experienced)$")
    hardware_access: str | None = Field(None, pattern="^(simulation_only|jetson|full_kit)$")
    learning_goals: list[str] | None = None
    completed_assessment: bool = False

    class Config:
        from_attributes = True


class UserPreferences(BaseModel):
    """User preferences model."""

    language_preference: str | None = Field(None, pattern="^(en|ur)$")
    email_notifications: bool | None = None
    dark_mode: bool | None = None


class UserProfile(BaseModel):
    """User profile response model."""

    id: str
    email: str
    display_name: str | None
    email_verified: bool
    background: BackgroundAssessment
    language_preference: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class UpdateProfileRequest(BaseModel):
    """Request model for profile update."""

    display_name: str | None = Field(None, max_length=100)


class DeleteAccountRequest(BaseModel):
    """Request model for account deletion."""

    confirm_password: str


# Endpoints
@router.get(
    "/me",
    response_model=UserProfile,
    responses={
        401: {"model": ErrorResponse, "description": "Not authenticated"},
    },
)
async def get_current_user_profile(
    auth: Annotated[tuple, Depends(get_current_user)],
) -> UserProfile:
    """Get current user profile."""
    _, user = auth

    # Parse background data
    bg = user.background or {}
    background = BackgroundAssessment(
        python_level=bg.get("python_level"),
        ros_experience=bg.get("ros_experience"),
        hardware_access=bg.get("hardware_access"),
        learning_goals=bg.get("learning_goals", []),
        completed_assessment=bg.get("completed_assessment", False),
    )

    return UserProfile(
        id=str(user.id),
        email=user.email,
        display_name=user.display_name,
        email_verified=user.email_verified,
        background=background,
        language_preference=user.language_pref,
        created_at=user.created_at,
        updated_at=user.updated_at,
    )


@router.patch(
    "/me",
    response_model=UserProfile,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid input"},
        401: {"model": ErrorResponse, "description": "Not authenticated"},
    },
)
async def update_current_user_profile(
    body: UpdateProfileRequest,
    db: Annotated[AsyncSession, Depends(get_db)],
    auth: Annotated[tuple, Depends(get_current_user)],
) -> UserProfile:
    """Update current user profile."""
    _, user = auth

    # Update user
    update_data = {}
    if body.display_name is not None:
        update_data["display_name"] = body.display_name

    if update_data:
        user = await auth_service.update_user(db, user, **update_data)

    # Parse background data
    bg = user.background or {}
    background = BackgroundAssessment(
        python_level=bg.get("python_level"),
        ros_experience=bg.get("ros_experience"),
        hardware_access=bg.get("hardware_access"),
        learning_goals=bg.get("learning_goals", []),
        completed_assessment=bg.get("completed_assessment", False),
    )

    return UserProfile(
        id=str(user.id),
        email=user.email,
        display_name=user.display_name,
        email_verified=user.email_verified,
        background=background,
        language_preference=user.language_pref,
        created_at=user.created_at,
        updated_at=user.updated_at,
    )


@router.delete(
    "/me",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={
        401: {"model": ErrorResponse, "description": "Not authenticated or wrong password"},
    },
)
async def delete_current_user_account(
    body: DeleteAccountRequest,
    db: Annotated[AsyncSession, Depends(get_db)],
    auth: Annotated[tuple, Depends(get_current_user)],
) -> None:
    """Delete current user account."""
    _, user = auth

    # Verify password
    if not auth_service.verify_password(body.confirm_password, user.password_hash):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail={"code": "INVALID_PASSWORD", "message": "Password is incorrect"},
        )

    # Delete user (cascades to related data)
    await auth_service.delete_user(db, user)


@router.get(
    "/me/background",
    response_model=BackgroundAssessment,
    responses={
        401: {"model": ErrorResponse, "description": "Not authenticated"},
    },
)
async def get_user_background(
    auth: Annotated[tuple, Depends(get_current_user)],
) -> BackgroundAssessment:
    """Get user background assessment."""
    _, user = auth

    bg = user.background or {}
    return BackgroundAssessment(
        python_level=bg.get("python_level"),
        ros_experience=bg.get("ros_experience"),
        hardware_access=bg.get("hardware_access"),
        learning_goals=bg.get("learning_goals", []),
        completed_assessment=bg.get("completed_assessment", False),
    )


@router.put(
    "/me/background",
    response_model=BackgroundAssessment,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid input"},
        401: {"model": ErrorResponse, "description": "Not authenticated"},
    },
)
async def update_user_background(
    body: BackgroundAssessment,
    db: Annotated[AsyncSession, Depends(get_db)],
    auth: Annotated[tuple, Depends(get_current_user)],
) -> BackgroundAssessment:
    """Update user background assessment."""
    _, user = auth

    # Validate learning goals
    valid_goals = {"ros2", "simulation", "isaac", "vla", "capstone"}
    if body.learning_goals:
        invalid_goals = set(body.learning_goals) - valid_goals
        if invalid_goals:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail={
                    "code": "INVALID_GOALS",
                    "message": f"Invalid learning goals: {', '.join(invalid_goals)}",
                },
            )

    # Update background
    background_data: dict[str, Any] = {
        "python_level": body.python_level,
        "ros_experience": body.ros_experience,
        "hardware_access": body.hardware_access,
        "learning_goals": body.learning_goals or [],
        "completed_assessment": body.completed_assessment,
    }

    user = await auth_service.update_user_background(db, user, background_data)

    return BackgroundAssessment(
        python_level=background_data["python_level"],
        ros_experience=background_data["ros_experience"],
        hardware_access=background_data["hardware_access"],
        learning_goals=background_data["learning_goals"],
        completed_assessment=background_data["completed_assessment"],
    )


@router.patch(
    "/me/preferences",
    response_model=UserPreferences,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid input"},
        401: {"model": ErrorResponse, "description": "Not authenticated"},
    },
)
async def update_user_preferences(
    body: UserPreferences,
    db: Annotated[AsyncSession, Depends(get_db)],
    auth: Annotated[tuple, Depends(get_current_user)],
) -> UserPreferences:
    """Update user preferences."""
    _, user = auth

    # Update language preference if provided
    if body.language_preference:
        user = await auth_service.update_user(
            db, user, language_pref=body.language_preference
        )

    # Note: email_notifications and dark_mode would be stored elsewhere
    # For now, we return what was passed

    return UserPreferences(
        language_preference=user.language_pref,
        email_notifications=body.email_notifications,
        dark_mode=body.dark_mode,
    )
