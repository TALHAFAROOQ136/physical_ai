"""Recommendation service for personalized content suggestions."""

from typing import Any
import uuid

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from src.models.chapter import Chapter
from src.models.module import Module
from src.models.progress import Progress, ProgressStatus
from src.models.user import User


def build_user_background_prompt(background: dict[str, Any] | None) -> str:
    """
    Build a prompt section describing the user's background.

    Args:
        background: User background data (python_level, ros_experience, etc.)

    Returns:
        Formatted prompt string for user background context
    """
    if not background:
        return ""

    python_level = background.get("python_level", "unknown")
    ros_experience = background.get("ros_experience", "unknown")
    hardware_access = background.get("hardware_access", "unknown")
    learning_goals = background.get("learning_goals", [])

    parts = ["Student Background:"]

    # Python level descriptions
    python_desc = {
        "beginner": "beginner in Python (explain code step by step)",
        "intermediate": "intermediate Python programmer (comfortable with classes and modules)",
        "advanced": "advanced Python programmer (skip basic explanations)",
    }
    if python_level in python_desc:
        parts.append(f"- Python: {python_desc[python_level]}")

    # ROS experience descriptions
    ros_desc = {
        "none": "no ROS experience (explain ROS concepts thoroughly)",
        "basic": "basic ROS knowledge (familiar with topics and nodes)",
        "experienced": "experienced with ROS (can skip fundamentals)",
    }
    if ros_experience in ros_desc:
        parts.append(f"- ROS: {ros_desc[ros_experience]}")

    # Hardware access
    hardware_desc = {
        "simulation_only": "working in simulation only",
        "jetson": "has Jetson hardware",
        "full_kit": "has full robotics kit",
    }
    if hardware_access in hardware_desc:
        parts.append(f"- Hardware: {hardware_desc[hardware_access]}")

    # Learning goals
    if learning_goals:
        goals_str = ", ".join(learning_goals)
        parts.append(f"- Focused on: {goals_str}")

    if len(parts) == 1:
        return ""

    parts.append("\nAdjust your explanations to match this student's level.")
    return "\n".join(parts)


async def get_user_progress(
    db: AsyncSession,
    user_id: uuid.UUID,
) -> list[Progress]:
    """Get all progress records for a user."""
    query = select(Progress).where(Progress.user_id == user_id)
    result = await db.execute(query)
    return list(result.scalars().all())


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
        Recommendations dict with next_chapter, skip_suggestions, focus_areas
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


async def get_personalized_path(
    db: AsyncSession,
    user: User,
) -> dict[str, Any]:
    """
    Get a personalized learning path based on user background.

    Args:
        db: Database session
        user: User object

    Returns:
        Personalized learning path with suggested order and skips
    """
    background = user.background or {}
    recommendations = await get_recommendations(db, user)

    # Get all chapters
    result = await db.execute(
        select(Chapter)
        .join(Module)
        .order_by(Module.order_sequence, Chapter.order_sequence)
    )
    all_chapters = list(result.scalars().all())

    # Get progress
    progress_records = await get_user_progress(db, user.id)
    progress_map = {p.chapter_id: p for p in progress_records}

    # Build personalized path
    skip_ids = {s["chapter_id"] for s in recommendations["skip_suggestions"]}

    path = []
    for chapter in all_chapters:
        progress = progress_map.get(chapter.id)
        status = progress.status if progress else ProgressStatus.NOT_STARTED

        path.append({
            "chapter_id": chapter.id,
            "title": chapter.title,
            "module_id": chapter.module_id,
            "status": status,
            "suggested_skip": chapter.id in skip_ids,
            "difficulty": chapter.difficulty,
        })

    return {
        "path": path,
        "next_chapter": recommendations["next_chapter"],
        "skip_suggestions": recommendations["skip_suggestions"],
        "focus_areas": recommendations["focus_areas"],
    }
