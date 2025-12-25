"""Progress tracking service for chapter completion and reading activity."""

import uuid
from datetime import datetime, timedelta, timezone
from typing import Any

from sqlalchemy import func, select, and_
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload

from src.models.progress import Progress, ProgressStatus
from src.models.chapter import Chapter
from src.models.module import Module
from src.models.user import User


async def get_user_progress(
    db: AsyncSession,
    user_id: uuid.UUID,
    module_id: str | None = None,
) -> list[Progress]:
    """
    Get all progress records for a user.

    Args:
        db: Database session
        user_id: User ID
        module_id: Optional filter by module

    Returns:
        List of Progress objects
    """
    query = select(Progress).where(Progress.user_id == user_id)

    if module_id:
        query = query.join(Chapter).where(Chapter.module_id == module_id)

    result = await db.execute(query)
    return list(result.scalars().all())


async def get_chapter_progress(
    db: AsyncSession,
    user_id: uuid.UUID,
    chapter_id: str,
) -> Progress | None:
    """
    Get progress for a specific chapter.

    Args:
        db: Database session
        user_id: User ID
        chapter_id: Chapter ID

    Returns:
        Progress object or None
    """
    result = await db.execute(
        select(Progress).where(
            and_(
                Progress.user_id == user_id,
                Progress.chapter_id == chapter_id,
            )
        )
    )
    return result.scalar_one_or_none()


async def get_or_create_progress(
    db: AsyncSession,
    user_id: uuid.UUID,
    chapter_id: str,
) -> Progress:
    """
    Get existing progress or create new one.

    Args:
        db: Database session
        user_id: User ID
        chapter_id: Chapter ID

    Returns:
        Progress object
    """
    progress = await get_chapter_progress(db, user_id, chapter_id)

    if not progress:
        progress = Progress(
            user_id=user_id,
            chapter_id=chapter_id,
            status=ProgressStatus.NOT_STARTED,
        )
        db.add(progress)
        await db.flush()
        await db.refresh(progress)

    return progress


async def update_chapter_progress(
    db: AsyncSession,
    user_id: uuid.UUID,
    chapter_id: str,
    status: str | None = None,
    last_position: str | None = None,
    additional_reading_time: int | None = None,
) -> Progress:
    """
    Update progress for a chapter.

    Args:
        db: Database session
        user_id: User ID
        chapter_id: Chapter ID
        status: New status
        last_position: Last scroll position
        additional_reading_time: Seconds to add

    Returns:
        Updated Progress object
    """
    progress = await get_or_create_progress(db, user_id, chapter_id)

    if status:
        if status == ProgressStatus.COMPLETED:
            progress.mark_complete()
        elif status == ProgressStatus.IN_PROGRESS:
            progress.mark_in_progress()

    if last_position:
        progress.last_position = last_position

    if additional_reading_time:
        progress.add_reading_time(additional_reading_time)

    await db.flush()
    await db.refresh(progress)
    return progress


async def mark_chapter_complete(
    db: AsyncSession,
    user_id: uuid.UUID,
    chapter_id: str,
) -> Progress:
    """
    Mark a chapter as complete.

    Args:
        db: Database session
        user_id: User ID
        chapter_id: Chapter ID

    Returns:
        Updated Progress object
    """
    progress = await get_or_create_progress(db, user_id, chapter_id)
    progress.mark_complete()
    await db.flush()
    await db.refresh(progress)
    return progress


async def unmark_chapter_complete(
    db: AsyncSession,
    user_id: uuid.UUID,
    chapter_id: str,
) -> Progress:
    """
    Unmark a chapter completion (reset to in_progress).

    Args:
        db: Database session
        user_id: User ID
        chapter_id: Chapter ID

    Returns:
        Updated Progress object
    """
    progress = await get_or_create_progress(db, user_id, chapter_id)
    progress.status = ProgressStatus.IN_PROGRESS
    progress.completed_at = None
    await db.flush()
    await db.refresh(progress)
    return progress


async def record_heartbeat(
    db: AsyncSession,
    user_id: uuid.UUID,
    chapter_id: str,
    position: str | None = None,
    duration_seconds: int = 30,
) -> None:
    """
    Record reading activity heartbeat.

    Args:
        db: Database session
        user_id: User ID
        chapter_id: Chapter ID
        position: Current scroll position
        duration_seconds: Time since last heartbeat
    """
    progress = await get_or_create_progress(db, user_id, chapter_id)

    # Mark as in progress if not started
    if progress.status == ProgressStatus.NOT_STARTED:
        progress.mark_in_progress()

    # Update position and reading time
    if position:
        progress.last_position = position

    progress.add_reading_time(duration_seconds)

    await db.flush()


async def get_progress_summary(
    db: AsyncSession,
    user_id: uuid.UUID,
    module_id: str | None = None,
) -> dict[str, Any]:
    """
    Get progress summary for a user.

    Args:
        db: Database session
        user_id: User ID
        module_id: Optional filter by module

    Returns:
        Summary dict with counts and totals
    """
    # Get all chapters
    chapter_query = select(Chapter)
    if module_id:
        chapter_query = chapter_query.where(Chapter.module_id == module_id)
    result = await db.execute(chapter_query)
    all_chapters = list(result.scalars().all())

    # Get user progress
    progress_records = await get_user_progress(db, user_id, module_id)
    progress_map = {p.chapter_id: p for p in progress_records}

    # Count statuses
    completed = 0
    in_progress = 0
    total_reading_time = 0

    for chapter in all_chapters:
        progress = progress_map.get(chapter.id)
        if progress:
            if progress.status == ProgressStatus.COMPLETED:
                completed += 1
            elif progress.status == ProgressStatus.IN_PROGRESS:
                in_progress += 1
            total_reading_time += progress.reading_time_seconds

    return {
        "total_chapters": len(all_chapters),
        "completed_chapters": completed,
        "in_progress_chapters": in_progress,
        "total_reading_time_minutes": total_reading_time // 60,
    }


async def get_dashboard_data(
    db: AsyncSession,
    user_id: uuid.UUID,
) -> dict[str, Any]:
    """
    Get dashboard data for a user.

    Args:
        db: Database session
        user_id: User ID

    Returns:
        Dashboard dict with overall and module progress
    """
    # Get all modules
    result = await db.execute(
        select(Module).order_by(Module.order_sequence)
    )
    modules = list(result.scalars().all())

    # Get all chapters
    result = await db.execute(select(Chapter))
    all_chapters = list(result.scalars().all())

    # Get user progress
    progress_records = await get_user_progress(db, user_id)
    progress_map = {p.chapter_id: p for p in progress_records}

    # Calculate overall progress
    completed = 0
    in_progress = 0
    not_started = 0
    total_reading_time = 0

    for chapter in all_chapters:
        progress = progress_map.get(chapter.id)
        if progress:
            if progress.status == ProgressStatus.COMPLETED:
                completed += 1
            elif progress.status == ProgressStatus.IN_PROGRESS:
                in_progress += 1
            else:
                not_started += 1
            total_reading_time += progress.reading_time_seconds
        else:
            not_started += 1

    total_chapters = len(all_chapters)
    percent_complete = (completed / total_chapters * 100) if total_chapters > 0 else 0

    # Calculate module progress
    module_progress = []
    for module in modules:
        module_chapters = [c for c in all_chapters if c.module_id == module.id]
        module_completed = sum(
            1 for c in module_chapters
            if progress_map.get(c.id) and progress_map[c.id].status == ProgressStatus.COMPLETED
        )
        module_total = len(module_chapters)
        module_percent = (module_completed / module_total * 100) if module_total > 0 else 0

        module_progress.append({
            "module_id": module.id,
            "module_title": module.title,
            "completed": module_completed,
            "total": module_total,
            "percent_complete": round(module_percent, 1),
        })

    # Get recent activity (last 10 updates)
    recent_query = (
        select(Progress)
        .where(Progress.user_id == user_id)
        .where(Progress.status != ProgressStatus.NOT_STARTED)
        .order_by(Progress.updated_at.desc())
        .limit(10)
    )
    result = await db.execute(recent_query)
    recent_records = list(result.scalars().all())

    # Build recent activity list
    recent_activity = []
    chapter_map = {c.id: c for c in all_chapters}
    for progress in recent_records:
        chapter = chapter_map.get(progress.chapter_id)
        if chapter:
            action = "completed" if progress.status == ProgressStatus.COMPLETED else "continued"
            if progress.reading_time_seconds < 60:
                action = "started"

            recent_activity.append({
                "chapter_id": chapter.id,
                "chapter_title": chapter.title,
                "action": action,
                "timestamp": progress.updated_at.isoformat(),
            })

    # Calculate streak (days with activity)
    streak = await _calculate_streak(db, user_id)

    return {
        "overall_progress": {
            "completed": completed,
            "in_progress": in_progress,
            "not_started": not_started,
            "total_chapters": total_chapters,
            "percent_complete": round(percent_complete, 1),
        },
        "module_progress": module_progress,
        "recent_activity": recent_activity,
        "total_reading_time_minutes": total_reading_time // 60,
        "current_streak": streak,
    }


async def _calculate_streak(db: AsyncSession, user_id: uuid.UUID) -> int:
    """Calculate current streak (consecutive days with activity)."""
    today = datetime.now(timezone.utc).date()
    streak = 0
    check_date = today

    while True:
        # Check if there's any progress update on this date
        start_of_day = datetime.combine(check_date, datetime.min.time()).replace(tzinfo=timezone.utc)
        end_of_day = datetime.combine(check_date, datetime.max.time()).replace(tzinfo=timezone.utc)

        result = await db.execute(
            select(func.count(Progress.id))
            .where(Progress.user_id == user_id)
            .where(Progress.updated_at >= start_of_day)
            .where(Progress.updated_at <= end_of_day)
        )
        count = result.scalar()

        if count and count > 0:
            streak += 1
            check_date = check_date - timedelta(days=1)
        else:
            # No activity on this day
            if check_date == today:
                # Today has no activity yet, check yesterday
                check_date = check_date - timedelta(days=1)
            else:
                # Streak is broken
                break

        # Safety limit
        if streak > 365:
            break

    return streak


async def get_recommendations(
    db: AsyncSession,
    user: User,
) -> dict[str, Any]:
    """
    Get personalized content recommendations.

    Args:
        db: Database session
        user: User object

    Returns:
        Recommendations dict
    """
    # Get user's background
    background = user.background or {}
    python_level = background.get("python_level", "beginner")
    ros_experience = background.get("ros_experience", "none")
    learning_goals = background.get("learning_goals", [])

    # Get all chapters ordered by module and sequence
    result = await db.execute(
        select(Chapter)
        .join(Module)
        .order_by(Module.order_sequence, Chapter.order_sequence)
    )
    all_chapters = list(result.scalars().all())

    # Get user progress
    progress_records = await get_user_progress(db, user.id)
    progress_map = {p.chapter_id: p for p in progress_records}

    # Find next chapter (first non-completed chapter in order)
    next_chapter = None
    for chapter in all_chapters:
        progress = progress_map.get(chapter.id)
        if not progress or progress.status != ProgressStatus.COMPLETED:
            result = await db.execute(
                select(Module).where(Module.id == chapter.module_id)
            )
            module = result.scalar_one_or_none()
            next_chapter = {
                "chapter_id": chapter.id,
                "title": chapter.title,
                "module_id": chapter.module_id,
                "reason": f"Continue your learning journey in {module.title if module else 'the textbook'}",
            }
            break

    # Generate skip suggestions based on background
    skip_suggestions = []
    if python_level == "advanced":
        # Suggest skipping intro Python content
        for chapter in all_chapters:
            if "intro" in chapter.id.lower() or "basics" in chapter.id.lower():
                if chapter.difficulty == "beginner":
                    skip_suggestions.append({
                        "chapter_id": chapter.id,
                        "title": chapter.title,
                        "reason": "Your advanced Python skills likely cover this content",
                    })

    if ros_experience == "experienced":
        # Suggest skipping basic ROS chapters
        for chapter in all_chapters:
            if "ros2" in chapter.id.lower() and chapter.difficulty == "beginner":
                skip_suggestions.append({
                    "chapter_id": chapter.id,
                    "title": chapter.title,
                    "reason": "Your ROS experience likely covers this content",
                })

    # Generate focus areas based on goals
    focus_areas = []
    goal_module_map = {
        "ros2": "ros2",
        "simulation": "simulation",
        "isaac": "isaac",
        "vla": "vla",
        "capstone": "capstone",
    }

    result = await db.execute(select(Module))
    modules = {m.id: m for m in result.scalars().all()}

    for goal in learning_goals:
        module_id = goal_module_map.get(goal)
        if module_id and module_id in modules:
            module = modules[module_id]
            priority = "high" if goal in learning_goals[:2] else "medium"
            focus_areas.append({
                "module_id": module.id,
                "module_title": module.title,
                "priority": priority,
            })

    # Add modules not in goals as low priority
    for module_id, module in modules.items():
        if module_id not in [fa["module_id"] for fa in focus_areas]:
            focus_areas.append({
                "module_id": module.id,
                "module_title": module.title,
                "priority": "low",
            })

    return {
        "next_chapter": next_chapter,
        "skip_suggestions": skip_suggestions[:5],  # Limit to 5
        "focus_areas": focus_areas,
    }
