"""
API router configuration for the Physical AI & Humanoid Robotics Textbook backend.
"""

from fastapi import APIRouter

# Create main API router
api_router = APIRouter(prefix="/api/v1")

# Import routers from individual modules (to be created)
# from .auth import router as auth_router
# from .chatbot import router as chatbot_router
# from .personalization import router as personalization_router
# from .translation import router as translation_router

# Register routers
# api_router.include_router(auth_router, prefix="/auth", tags=["authentication"])
# api_router.include_router(chatbot_router, prefix="/chatbot", tags=["chatbot"])
# api_router.include_router(personalization_router, prefix="/personalization", tags=["personalization"])
# api_router.include_router(translation_router, prefix="/translation", tags=["translation"])

@api_router.get("/health")
async def health_check():
    """API health check endpoint."""
    return {
        "status": "healthy",
        "version": "1.0.0",
        "service": "Physical AI & Humanoid Robotics Textbook API"
    }

