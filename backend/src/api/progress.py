"""Progress API endpoints for chapter tracking and dashboard."""

from datetime import datetime
from typing import Annotated, Any

from fastapi import APIRouter, Depends, HTTPException, Query, status
from pydantic import BaseModel, Field
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from src.lib.database import get_db
from src.models.chapter import Chapter
from src.models.progress import ProgressStatus
from src.services import progress_service
from src.api.auth import get_current_user, ErrorResponse

router = APIRouter()


# Request/Response Models
class ChapterProgress(BaseModel):
    """Chapter progress response model."""

    chapter_id: str
    module_id: str
    status: str
    completed_at: datetime | None
    reading_time_seconds: int
    last_position: str | None
    updated_at: datetime

    class Config:
        from_attributes = True


class ProgressSummary(BaseModel):
    """Progress summary response model."""

    total_chapters: int
    completed_chapters: int
    in_progress_chapters: int
    total_reading_time_minutes: int
    chapters: list[ChapterProgress]


class UpdateProgressRequest(BaseModel):
    """Request model for progress update."""

    status: str | None = Field(None, pattern="^(in_progress|completed)$")
    last_position: str | None = None
    additional_reading_time: int | None = Field(None, ge=0)


class HeartbeatRequest(BaseModel):
    """Request model for reading heartbeat."""

    chapter_id: str
    position: str | None = None
    duration_seconds: int = Field(default=30, ge=1, le=300)


class OverallProgress(BaseModel):
    """Overall progress stats."""

    completed: int
    in_progress: int
    not_started: int
    total_chapters: int
    percent_complete: float


class ModuleProgress(BaseModel):
    """Module progress stats."""

    module_id: str
    module_title: str
    completed: int
    total: int
    percent_complete: float


class RecentActivity(BaseModel):
    """Recent activity item."""

    chapter_id: str
    chapter_title: str
    action: str
    timestamp: str


class Dashboard(BaseModel):
    """Dashboard response model."""

    overall_progress: OverallProgress
    module_progress: list[ModuleProgress]
    recent_activity: list[RecentActivity]
    total_reading_time_minutes: int
    current_streak: int


class NextChapter(BaseModel):
    """Next chapter recommendation."""

    chapter_id: str
    title: str
    module_id: str
    reason: str


class SkipSuggestion(BaseModel):
    """Skip suggestion item."""

    chapter_id: str
    title: str
    reason: str


class FocusArea(BaseModel):
    """Focus area recommendation."""

    module_id: str
    module_title: str
    priority: str


class Recommendations(BaseModel):
    """Recommendations response model."""

    next_chapter: NextChapter | None
    skip_suggestions: list[SkipSuggestion]
    focus_areas: list[FocusArea]


# Helper function
async def verify_chapter_exists(db: AsyncSession, chapter_id: str) -> Chapter:
    """Verify a chapter exists and return it."""
    result = await db.execute(select(Chapter).where(Chapter.id == chapter_id))
    chapter = result.scalar_one_or_none()
    if not chapter:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={"code": "CHAPTER_NOT_FOUND", "message": f"Chapter '{chapter_id}' not found"},
        )
    return chapter


# Endpoints
@router.get(
    "",
    response_model=ProgressSummary,
    responses={
        401: {"model": ErrorResponse, "description": "Not authenticated"},
    },
)
async def get_all_progress(
    db: Annotated[AsyncSession, Depends(get_db)],
    auth: Annotated[tuple, Depends(get_current_user)],
    module_id: str | None = Query(None, description="Filter by module ID"),
) -> ProgressSummary:
    """Get all progress for current user."""
    _, user = auth

    # Get summary
    summary = await progress_service.get_progress_summary(db, user.id, module_id)

    # Get all chapters
    chapter_query = select(Chapter)
    if module_id:
        chapter_query = chapter_query.where(Chapter.module_id == module_id)
    result = await db.execute(chapter_query)
    all_chapters = list(result.scalars().all())

    # Get progress records
    progress_records = await progress_service.get_user_progress(db, user.id, module_id)
    progress_map = {p.chapter_id: p for p in progress_records}

    # Build chapter progress list
    chapters = []
    for chapter in all_chapters:
        progress = progress_map.get(chapter.id)
        if progress:
            chapters.append(ChapterProgress(
                chapter_id=chapter.id,
                module_id=chapter.module_id,
                status=progress.status,
                completed_at=progress.completed_at,
                reading_time_seconds=progress.reading_time_seconds,
                last_position=progress.last_position,
                updated_at=progress.updated_at,
            ))
        else:
            chapters.append(ChapterProgress(
                chapter_id=chapter.id,
                module_id=chapter.module_id,
                status=ProgressStatus.NOT_STARTED,
                completed_at=None,
                reading_time_seconds=0,
                last_position=None,
                updated_at=datetime.utcnow(),
            ))

    return ProgressSummary(
        total_chapters=summary["total_chapters"],
        completed_chapters=summary["completed_chapters"],
        in_progress_chapters=summary["in_progress_chapters"],
        total_reading_time_minutes=summary["total_reading_time_minutes"],
        chapters=chapters,
    )


@router.get(
    "/chapters/{chapter_id}",
    response_model=ChapterProgress,
    responses={
        401: {"model": ErrorResponse, "description": "Not authenticated"},
        404: {"model": ErrorResponse, "description": "Chapter not found"},
    },
)
async def get_chapter_progress(
    chapter_id: str,
    db: Annotated[AsyncSession, Depends(get_db)],
    auth: Annotated[tuple, Depends(get_current_user)],
) -> ChapterProgress:
    """Get progress for a specific chapter."""
    _, user = auth

    # Verify chapter exists
    chapter = await verify_chapter_exists(db, chapter_id)

    # Get progress
    progress = await progress_service.get_chapter_progress(db, user.id, chapter_id)

    if progress:
        return ChapterProgress(
            chapter_id=chapter.id,
            module_id=chapter.module_id,
            status=progress.status,
            completed_at=progress.completed_at,
            reading_time_seconds=progress.reading_time_seconds,
            last_position=progress.last_position,
            updated_at=progress.updated_at,
        )
    else:
        return ChapterProgress(
            chapter_id=chapter.id,
            module_id=chapter.module_id,
            status=ProgressStatus.NOT_STARTED,
            completed_at=None,
            reading_time_seconds=0,
            last_position=None,
            updated_at=datetime.utcnow(),
        )


@router.patch(
    "/chapters/{chapter_id}",
    response_model=ChapterProgress,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid input"},
        401: {"model": ErrorResponse, "description": "Not authenticated"},
        404: {"model": ErrorResponse, "description": "Chapter not found"},
    },
)
async def update_chapter_progress(
    chapter_id: str,
    body: UpdateProgressRequest,
    db: Annotated[AsyncSession, Depends(get_db)],
    auth: Annotated[tuple, Depends(get_current_user)],
) -> ChapterProgress:
    """Update progress for a chapter."""
    _, user = auth

    # Verify chapter exists
    chapter = await verify_chapter_exists(db, chapter_id)

    # Update progress
    progress = await progress_service.update_chapter_progress(
        db=db,
        user_id=user.id,
        chapter_id=chapter_id,
        status=body.status,
        last_position=body.last_position,
        additional_reading_time=body.additional_reading_time,
    )

    return ChapterProgress(
        chapter_id=chapter.id,
        module_id=chapter.module_id,
        status=progress.status,
        completed_at=progress.completed_at,
        reading_time_seconds=progress.reading_time_seconds,
        last_position=progress.last_position,
        updated_at=progress.updated_at,
    )


@router.post(
    "/chapters/{chapter_id}/complete",
    response_model=ChapterProgress,
    responses={
        401: {"model": ErrorResponse, "description": "Not authenticated"},
        404: {"model": ErrorResponse, "description": "Chapter not found"},
    },
)
async def mark_chapter_complete(
    chapter_id: str,
    db: Annotated[AsyncSession, Depends(get_db)],
    auth: Annotated[tuple, Depends(get_current_user)],
) -> ChapterProgress:
    """Mark a chapter as complete."""
    _, user = auth

    # Verify chapter exists
    chapter = await verify_chapter_exists(db, chapter_id)

    # Mark complete
    progress = await progress_service.mark_chapter_complete(db, user.id, chapter_id)

    return ChapterProgress(
        chapter_id=chapter.id,
        module_id=chapter.module_id,
        status=progress.status,
        completed_at=progress.completed_at,
        reading_time_seconds=progress.reading_time_seconds,
        last_position=progress.last_position,
        updated_at=progress.updated_at,
    )


@router.delete(
    "/chapters/{chapter_id}/complete",
    response_model=ChapterProgress,
    responses={
        401: {"model": ErrorResponse, "description": "Not authenticated"},
    },
)
async def unmark_chapter_complete(
    chapter_id: str,
    db: Annotated[AsyncSession, Depends(get_db)],
    auth: Annotated[tuple, Depends(get_current_user)],
) -> ChapterProgress:
    """Unmark chapter completion."""
    _, user = auth

    # Get chapter (don't require it exists - might have been deleted)
    result = await db.execute(select(Chapter).where(Chapter.id == chapter_id))
    chapter = result.scalar_one_or_none()

    # Unmark complete
    progress = await progress_service.unmark_chapter_complete(db, user.id, chapter_id)

    return ChapterProgress(
        chapter_id=chapter_id,
        module_id=chapter.module_id if chapter else "",
        status=progress.status,
        completed_at=progress.completed_at,
        reading_time_seconds=progress.reading_time_seconds,
        last_position=progress.last_position,
        updated_at=progress.updated_at,
    )


@router.get(
    "/dashboard",
    response_model=Dashboard,
    responses={
        401: {"model": ErrorResponse, "description": "Not authenticated"},
    },
)
async def get_dashboard(
    db: Annotated[AsyncSession, Depends(get_db)],
    auth: Annotated[tuple, Depends(get_current_user)],
) -> Dashboard:
    """Get progress dashboard data."""
    _, user = auth

    data = await progress_service.get_dashboard_data(db, user.id)

    return Dashboard(
        overall_progress=OverallProgress(**data["overall_progress"]),
        module_progress=[ModuleProgress(**mp) for mp in data["module_progress"]],
        recent_activity=[RecentActivity(**ra) for ra in data["recent_activity"]],
        total_reading_time_minutes=data["total_reading_time_minutes"],
        current_streak=data["current_streak"],
    )


@router.get(
    "/recommendations",
    response_model=Recommendations,
    responses={
        401: {"model": ErrorResponse, "description": "Not authenticated"},
    },
)
async def get_recommendations(
    db: Annotated[AsyncSession, Depends(get_db)],
    auth: Annotated[tuple, Depends(get_current_user)],
) -> Recommendations:
    """Get personalized content recommendations."""
    _, user = auth

    data = await progress_service.get_recommendations(db, user)

    return Recommendations(
        next_chapter=NextChapter(**data["next_chapter"]) if data["next_chapter"] else None,
        skip_suggestions=[SkipSuggestion(**ss) for ss in data["skip_suggestions"]],
        focus_areas=[FocusArea(**fa) for fa in data["focus_areas"]],
    )


@router.post(
    "/heartbeat",
    responses={
        401: {"model": ErrorResponse, "description": "Not authenticated"},
    },
)
async def heartbeat(
    body: HeartbeatRequest,
    db: Annotated[AsyncSession, Depends(get_db)],
    auth: Annotated[tuple, Depends(get_current_user)],
) -> dict[str, str]:
    """Update reading activity (called periodically)."""
    _, user = auth

    await progress_service.record_heartbeat(
        db=db,
        user_id=user.id,
        chapter_id=body.chapter_id,
        position=body.position,
        duration_seconds=body.duration_seconds,
    )

    return {"status": "recorded"}
