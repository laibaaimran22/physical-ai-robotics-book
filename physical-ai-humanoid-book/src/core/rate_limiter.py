import time
from typing import Dict, Optional
from collections import defaultdict
from datetime import datetime, timedelta
import asyncio


class RateLimiter:
    """Rate limiter for API endpoints with free tier limits."""

    def __init__(self):
        # Track requests by user ID
        self.requests_by_user: Dict[str, list] = defaultdict(list)
        # Track daily RAG queries by user ID (for free tier limits)
        self.daily_rag_queries: Dict[str, Dict[datetime, int]] = defaultdict(lambda: {})
        # Track daily search queries by user ID (for free tier limits)
        self.daily_search_queries: Dict[str, Dict[datetime, int]] = defaultdict(lambda: {})

    def is_allowed(self, user_id: str, limit: int = 100, window: int = 60) -> bool:
        """
        Check if a user is allowed to make a request based on rate limits.

        Args:
            user_id: ID of the user making the request
            limit: Number of requests allowed per window
            window: Time window in seconds

        Returns:
            True if request is allowed, False otherwise
        """
        now = time.time()

        # Clean up old requests outside the window
        self.requests_by_user[user_id] = [
            req_time for req_time in self.requests_by_user[user_id]
            if now - req_time < window
        ]

        # Check if user is within limits
        if len(self.requests_by_user[user_id]) < limit:
            # Add current request timestamp
            self.requests_by_user[user_id].append(now)
            return True

        return False

    def check_free_tier_rag_limit(self, user_id: str, limit: int = 30) -> bool:
        """
        Check if a user has exceeded their free tier RAG query limit.

        Args:
            user_id: ID of the user making the request
            limit: Daily limit for RAG queries

        Returns:
            True if within limit, False if exceeded
        """
        today = datetime.now().date()

        # Reset counter if it's a new day
        if today not in self.daily_rag_queries[user_id]:
            self.daily_rag_queries[user_id][today] = 0

        # Check if within daily limit
        if self.daily_rag_queries[user_id][today] < limit:
            # Increment the counter
            self.daily_rag_queries[user_id][today] += 1
            return True

        return False

    def check_free_tier_search_limit(self, user_id: str, limit: int = 50) -> bool:
        """
        Check if a user has exceeded their free tier search query limit.

        Args:
            user_id: ID of the user making the request
            limit: Daily limit for search queries

        Returns:
            True if within limit, False if exceeded
        """
        today = datetime.now().date()

        # Reset counter if it's a new day
        if today not in self.daily_search_queries[user_id]:
            self.daily_search_queries[user_id][today] = 0

        # Check if within daily limit
        if self.daily_search_queries[user_id][today] < limit:
            # Increment the counter
            self.daily_search_queries[user_id][today] += 1
            return True

        return False

    def reset_daily_counters(self):
        """Reset daily counters (should be called periodically)."""
        today = datetime.now().date()

        # Clean up old days except today
        for user_id in list(self.daily_rag_queries.keys()):
            for date in list(self.daily_rag_queries[user_id].keys()):
                if date != today:
                    del self.daily_rag_queries[user_id][date]

        for user_id in list(self.daily_search_queries.keys()):
            for date in list(self.daily_search_queries[user_id].keys()):
                if date != today:
                    del self.daily_search_queries[user_id][date]


# Global rate limiter instance
rate_limiter = RateLimiter()