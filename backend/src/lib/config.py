"""Application configuration using pydantic-settings."""

from functools import lru_cache
from typing import Literal

from pydantic import field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
    )

    # Database
    database_url: str = "postgresql+asyncpg://localhost/textbook"

    # Vector Database
    qdrant_url: str = "http://localhost:6333"
    qdrant_api_key: str | None = None

    # OpenAI
    openai_api_key: str = ""
    openai_model: str = "gpt-4-turbo"

    # Better-Auth
    better_auth_secret: str = "development-secret-change-in-production"
    better_auth_url: str = "http://localhost:8000"

    # Environment
    environment: Literal["development", "staging", "production", "test"] = "development"
    debug: bool = True

    # CORS - Security: Restrict origins in production
    cors_origins: str = "http://localhost:3000"

    # Rate Limiting
    rate_limit_per_minute: int = 100

    # Session Security
    session_cookie_name: str = "textbook_session"
    session_max_age_hours: int = 24
    session_secure_cookie: bool = True  # Require HTTPS in production

    @field_validator("better_auth_secret")
    @classmethod
    def validate_auth_secret(cls, v: str, info) -> str:
        """Validate auth secret is not default in production."""
        # We can't access other fields directly in validators,
        # so we check the environment variable
        import os
        env = os.getenv("ENVIRONMENT", "development")
        if env == "production" and v == "development-secret-change-in-production":
            raise ValueError("Must set BETTER_AUTH_SECRET in production")
        if len(v) < 32:
            raise ValueError("BETTER_AUTH_SECRET must be at least 32 characters")
        return v

    @property
    def cors_origins_list(self) -> list[str]:
        """Parse comma-separated CORS origins."""
        origins = [origin.strip() for origin in self.cors_origins.split(",")]
        # Filter out wildcards in production
        if self.is_production:
            origins = [o for o in origins if o != "*"]
        return origins

    @property
    def is_production(self) -> bool:
        """Check if running in production."""
        return self.environment == "production"

    @property
    def cookie_settings(self) -> dict:
        """Get secure cookie settings based on environment."""
        return {
            "httponly": True,
            "secure": self.is_production or self.session_secure_cookie,
            "samesite": "lax",
            "max_age": self.session_max_age_hours * 3600,
        }


@lru_cache
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
