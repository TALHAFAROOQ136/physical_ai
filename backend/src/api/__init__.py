"""API routes for the Physical AI Textbook platform."""

from fastapi import APIRouter

from src.api.chatbot import router as chatbot_router
from src.api.auth import router as auth_router
from src.api.users import router as users_router
from src.api.progress import router as progress_router

router = APIRouter()


@router.get("/")
async def api_root() -> dict[str, str]:
    """API root endpoint."""
    return {"api": "v1", "status": "active"}


# Include routers
router.include_router(chatbot_router, prefix="/chatbot", tags=["Chatbot"])
router.include_router(auth_router, prefix="/auth", tags=["Authentication"])
router.include_router(users_router, prefix="/users", tags=["Users"])
router.include_router(progress_router, prefix="/progress", tags=["Progress"])
