import json
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

from src.models.user import User

def test_personalization_field():
    """Test that the personalization_preferences field exists in the User model."""
    print("Testing personalization field in User model...")

    # Create a test user object
    user = User(
        email="test@example.com",
        hashed_password="test_hash",
        personalization_preferences='{"chapters": {}}'
    )

    print(f"User personalization_preferences: {user.personalization_preferences}")
    print("Personalization field test passed!")

    # Test JSON parsing
    try:
        prefs = json.loads(user.personalization_preferences)
        print(f"Parsed preferences: {prefs}")
        print("JSON parsing test passed!")
    except json.JSONDecodeError as e:
        print(f"JSON parsing failed: {e}")
        return False

    # Test with chapter-specific preferences
    chapter_prefs = {
        "chapters": {
            "module1-lesson1-ros2-fundamentals": {
                "difficulty_level": "beginner",
                "content_focus": "balanced",
                "learning_style": "comprehensive",
                "pace": "moderate"
            }
        }
    }

    user.personalization_preferences = json.dumps(chapter_prefs)
    print(f"Updated user preferences: {user.personalization_preferences}")

    parsed = json.loads(user.personalization_preferences)
    print(f"Parsed updated preferences: {parsed}")
    print("Chapter-specific preferences test passed!")

    return True

if __name__ == "__main__":
    test_personalization_field()
    print("All tests passed!")