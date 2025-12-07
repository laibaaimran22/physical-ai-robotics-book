"""
Simple caching layer for frequently accessed content.
In production, this would typically use Redis or another caching system.
For this implementation, we'll use a simple in-memory cache with TTL.
"""
import asyncio
import time
from typing import Any, Optional, Dict
from threading import Lock


class SimpleCache:
    """
    A simple in-memory cache with TTL (Time To Live) functionality.
    In production, consider using Redis or another distributed cache.
    """

    def __init__(self):
        self._cache: Dict[str, Dict[str, Any]] = {}
        self._lock = Lock()

    def set(self, key: str, value: Any, ttl: int = 300) -> None:
        """
        Set a value in the cache with a TTL (in seconds).

        Args:
            key: Cache key
            value: Value to cache
            ttl: Time to live in seconds (default 300 seconds = 5 minutes)
        """
        with self._lock:
            expiry_time = time.time() + ttl
            self._cache[key] = {
                "value": value,
                "expiry": expiry_time
            }

    def get(self, key: str) -> Optional[Any]:
        """
        Get a value from the cache.

        Args:
            key: Cache key

        Returns:
            Cached value or None if not found/expired
        """
        with self._lock:
            if key not in self._cache:
                return None

            item = self._cache[key]

            # Check if expired
            if time.time() > item["expiry"]:
                del self._cache[key]
                return None

            return item["value"]

    def delete(self, key: str) -> bool:
        """
        Delete a key from the cache.

        Args:
            key: Cache key

        Returns:
            True if key existed and was deleted, False otherwise
        """
        with self._lock:
            if key in self._cache:
                del self._cache[key]
                return True
            return False

    def clear(self) -> None:
        """Clear all items from the cache."""
        with self._lock:
            self._cache.clear()

    def cleanup_expired(self) -> int:
        """
        Remove all expired items from the cache.

        Returns:
            Number of expired items removed
        """
        with self._lock:
            current_time = time.time()
            expired_keys = [
                key for key, item in self._cache.items()
                if current_time > item["expiry"]
            ]

            for key in expired_keys:
                del self._cache[key]

            return len(expired_keys)

    def size(self) -> int:
        """
        Get the number of items in the cache (including expired).

        Returns:
            Number of cached items
        """
        with self._lock:
            return len(self._cache)


# Global cache instance
cache = SimpleCache()


async def get_cached_content(key: str) -> Optional[Any]:
    """
    Get content from cache asynchronously.

    Args:
        key: Cache key

    Returns:
        Cached content or None
    """
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, cache.get, key)


async def set_cached_content(key: str, value: Any, ttl: int = 300) -> None:
    """
    Set content in cache asynchronously.

    Args:
        key: Cache key
        value: Content to cache
        ttl: Time to live in seconds
    """
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, cache.set, key, value, ttl)


async def invalidate_cache_key(key: str) -> bool:
    """
    Invalidate a specific cache key asynchronously.

    Args:
        key: Cache key to invalidate

    Returns:
        True if key existed and was invalidated, False otherwise
    """
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, cache.delete, key)


async def cleanup_expired_cache() -> int:
    """
    Cleanup expired cache entries asynchronously.

    Returns:
        Number of expired entries removed
    """
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, cache.cleanup_expired)