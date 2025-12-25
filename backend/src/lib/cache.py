"""In-memory caching utilities for API responses.

This module provides a simple in-memory cache with TTL support for
caching static data like module lists, chapter metadata, etc.
"""

import asyncio
from datetime import datetime, timedelta
from functools import wraps
from typing import Any, Callable, TypeVar

T = TypeVar("T")


class CacheEntry:
    """A cache entry with TTL support."""

    def __init__(self, value: Any, ttl_seconds: int):
        self.value = value
        self.expires_at = datetime.utcnow() + timedelta(seconds=ttl_seconds)

    def is_expired(self) -> bool:
        """Check if the cache entry has expired."""
        return datetime.utcnow() > self.expires_at


class InMemoryCache:
    """
    Simple in-memory cache with TTL support.

    Thread-safe for async operations using asyncio locks.
    """

    def __init__(self):
        self._cache: dict[str, CacheEntry] = {}
        self._lock = asyncio.Lock()

    async def get(self, key: str) -> Any | None:
        """
        Get a value from the cache.

        Returns None if the key doesn't exist or has expired.
        """
        async with self._lock:
            entry = self._cache.get(key)
            if entry is None:
                return None
            if entry.is_expired():
                del self._cache[key]
                return None
            return entry.value

    async def set(self, key: str, value: Any, ttl_seconds: int = 300) -> None:
        """
        Set a value in the cache with TTL.

        Args:
            key: Cache key
            value: Value to cache
            ttl_seconds: Time-to-live in seconds (default: 5 minutes)
        """
        async with self._lock:
            self._cache[key] = CacheEntry(value, ttl_seconds)

    async def delete(self, key: str) -> None:
        """Delete a key from the cache."""
        async with self._lock:
            self._cache.pop(key, None)

    async def clear(self) -> None:
        """Clear all cached entries."""
        async with self._lock:
            self._cache.clear()

    async def cleanup_expired(self) -> int:
        """
        Remove all expired entries from the cache.

        Returns:
            Number of entries removed
        """
        async with self._lock:
            expired_keys = [
                key for key, entry in self._cache.items() if entry.is_expired()
            ]
            for key in expired_keys:
                del self._cache[key]
            return len(expired_keys)


# Global cache instance
_cache = InMemoryCache()


def get_cache() -> InMemoryCache:
    """Get the global cache instance."""
    return _cache


def cached(
    ttl_seconds: int = 300,
    key_prefix: str = "",
) -> Callable[[Callable[..., T]], Callable[..., T]]:
    """
    Decorator for caching async function results.

    Args:
        ttl_seconds: Cache TTL in seconds (default: 5 minutes)
        key_prefix: Prefix for the cache key

    Usage:
        @cached(ttl_seconds=600, key_prefix="modules")
        async def get_modules():
            # expensive operation
            return modules
    """

    def decorator(func: Callable[..., T]) -> Callable[..., T]:
        @wraps(func)
        async def wrapper(*args: Any, **kwargs: Any) -> T:
            # Generate cache key from function name and arguments
            cache_key = f"{key_prefix}:{func.__name__}"
            if args:
                cache_key += f":{':'.join(str(a) for a in args)}"
            if kwargs:
                cache_key += f":{':'.join(f'{k}={v}' for k, v in sorted(kwargs.items()))}"

            cache = get_cache()

            # Try to get from cache
            cached_value = await cache.get(cache_key)
            if cached_value is not None:
                return cached_value

            # Call the function and cache the result
            result = await func(*args, **kwargs)
            await cache.set(cache_key, result, ttl_seconds)

            return result

        return wrapper

    return decorator


# Cache TTL constants
CACHE_TTL_SHORT = 60  # 1 minute
CACHE_TTL_MEDIUM = 300  # 5 minutes
CACHE_TTL_LONG = 3600  # 1 hour
CACHE_TTL_STATIC = 86400  # 24 hours (for truly static data)
