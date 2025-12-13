import time
from typing import Dict, Optional
from fastapi import Request, HTTPException, status
from collections import defaultdict, deque
from datetime import datetime, timedelta


class RateLimiter:
    """
    Simple in-memory rate limiter middleware.
    In production, consider using Redis or other distributed storage for multi-instance deployments.
    """

    def __init__(self, requests: int = 100, window: int = 60):
        """
        Initialize the rate limiter.

        Args:
            requests: Number of requests allowed per window
            window: Time window in seconds
        """
        self.requests = requests
        self.window = window
        self.requests_store: Dict[str, deque] = defaultdict(deque)

    def check_rate_limit(self, identifier: str) -> tuple[bool, dict]:
        """
        Check if the identifier has exceeded the rate limit.

        Args:
            identifier: Unique identifier for the client (e.g., IP address or user ID)

        Returns:
            Tuple of (is_allowed, rate_limit_info)
        """
        now = time.time()
        window_start = now - self.window

        # Clean old requests outside the window
        while self.requests_store[identifier] and self.requests_store[identifier][0] < window_start:
            self.requests_store[identifier].popleft()

        # Check if limit is exceeded
        current_requests = len(self.requests_store[identifier])

        if current_requests >= self.requests:
            # Limit exceeded
            oldest_request = self.requests_store[identifier][0] if self.requests_store[identifier] else now
            reset_time = oldest_request + self.window

            return False, {
                "limit": self.requests,
                "remaining": 0,
                "reset_time": reset_time,
                "retry_after": int(reset_time - now)
            }

        # Add current request
        self.requests_store[identifier].append(now)

        return True, {
            "limit": self.requests,
            "remaining": self.requests - current_requests - 1,
            "reset_time": now + self.window,
            "retry_after": self.window
        }


# Global rate limiter instance
rate_limiter = RateLimiter(requests=100, window=60)  # 100 requests per minute


async def rate_limit_middleware(request: Request, call_next):
    """
    FastAPI middleware for rate limiting.
    """
    # Get client IP address
    client_ip = request.client.host

    # Check rate limit
    is_allowed, rate_info = rate_limiter.check_rate_limit(client_ip)

    # Add rate limit headers to response
    response = await call_next(request)

    response.headers["X-RateLimit-Limit"] = str(rate_info["limit"])
    response.headers["X-RateLimit-Remaining"] = str(rate_info["remaining"])
    response.headers["X-RateLimit-Reset"] = str(int(rate_info["reset_time"]))

    if not is_allowed:
        response.status_code = status.HTTP_429_TOO_MANY_REQUESTS
        response.body = b'{"detail": "Rate limit exceeded"}'
        response.headers["Retry-After"] = str(rate_info["retry_after"])

    return response