"""FastAPI application entry point."""

import asyncio
from contextlib import asynccontextmanager
from typing import AsyncGenerator

from fastapi import FastAPI, Request, Response
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.base import BaseHTTPMiddleware

from src.lib.config import get_settings
from src.lib.database import close_db
from src.lib.cache import get_cache
from src.api import router as api_router

settings = get_settings()


class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """Middleware to add security headers to all responses."""

    async def dispatch(self, request: Request, call_next) -> Response:
        response = await call_next(request)

        # Security headers
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["X-XSS-Protection"] = "1; mode=block"
        response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"

        # Content Security Policy (adjust as needed)
        if settings.is_production:
            response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"

        return response


class CacheControlMiddleware(BaseHTTPMiddleware):
    """Middleware to add cache control headers for static API responses."""

    # Paths that can be cached
    CACHEABLE_PATHS = {
        "/v1/modules": 3600,  # 1 hour
        "/v1/chapters": 3600,  # 1 hour
        "/health": 60,  # 1 minute
    }

    async def dispatch(self, request: Request, call_next) -> Response:
        response = await call_next(request)

        # Add cache headers for cacheable GET requests
        if request.method == "GET":
            path = request.url.path
            for cacheable_path, max_age in self.CACHEABLE_PATHS.items():
                if path.startswith(cacheable_path):
                    response.headers["Cache-Control"] = f"public, max-age={max_age}"
                    break
            else:
                # Default: no caching for dynamic content
                if "/chatbot" in path or "/progress" in path or "/auth" in path:
                    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"

        return response


class RateLimitHeadersMiddleware(BaseHTTPMiddleware):
    """Middleware to add rate limiting headers to responses."""

    async def dispatch(self, request: Request, call_next) -> Response:
        response = await call_next(request)

        # Add rate limit headers (actual limiting done by rate_limiter.py)
        response.headers["X-RateLimit-Limit"] = str(settings.rate_limit_per_minute)
        # Note: X-RateLimit-Remaining and X-RateLimit-Reset should be set by the rate limiter

        return response


async def cleanup_cache_task() -> None:
    """Background task to periodically clean up expired cache entries."""
    cache = get_cache()
    while True:
        await asyncio.sleep(300)  # Every 5 minutes
        await cache.cleanup_expired()


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """Application lifespan handler for startup and shutdown events."""
    # Startup
    cleanup_task = asyncio.create_task(cleanup_cache_task())
    yield
    # Shutdown
    cleanup_task.cancel()
    try:
        await cleanup_task
    except asyncio.CancelledError:
        pass
    await close_db()


app = FastAPI(
    title="Physical AI Textbook API",
    description="Backend API for the Physical AI & Humanoid Robotics Textbook platform",
    version="0.1.0",
    docs_url="/docs" if settings.debug else None,
    redoc_url="/redoc" if settings.debug else None,
    lifespan=lifespan,
)

# Security headers middleware (runs last, wraps response)
app.add_middleware(SecurityHeadersMiddleware)

# Rate limit headers middleware
app.add_middleware(RateLimitHeadersMiddleware)

# Cache control middleware
app.add_middleware(CacheControlMiddleware)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "PATCH", "DELETE", "OPTIONS"],
    allow_headers=["Authorization", "Content-Type", "X-Requested-With"],
    expose_headers=["X-RateLimit-Limit", "X-RateLimit-Remaining", "X-RateLimit-Reset"],
)

# Include API routers
app.include_router(api_router, prefix="/v1")


@app.get("/health")
async def health_check() -> dict[str, str]:
    """Health check endpoint."""
    return {"status": "healthy", "environment": settings.environment}


@app.get("/")
async def root() -> dict[str, str]:
    """Root endpoint."""
    return {
        "name": "Physical AI Textbook API",
        "version": "0.1.0",
        "docs": "/docs" if settings.debug else "disabled",
    }
