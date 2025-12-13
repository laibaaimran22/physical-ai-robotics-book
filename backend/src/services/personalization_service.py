from typing import Dict, Any, List
from dataclasses import dataclass
from enum import Enum


class DifficultyLevel(Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


@dataclass
class UserProfile:
    """Data class to represent user profile with background information."""
    software_level: str = "beginner"
    hardware_level: str = "beginner"
    preferred_languages: str = ""
    goals: str = ""


class PersonalizationService:
    """Service for generating personalized content based on user profile."""

    def __init__(self):
        pass

    def get_difficulty_level(self, user_profile: UserProfile) -> DifficultyLevel:
        """Determine the appropriate difficulty level based on user profile."""
        # Map string levels to enum values
        software_level = self._map_level(user_profile.software_level)
        hardware_level = self._map_level(user_profile.hardware_level)

        # Return the higher of the two levels, or beginner if both are beginner
        if software_level == DifficultyLevel.ADVANCED or hardware_level == DifficultyLevel.ADVANCED:
            return DifficultyLevel.ADVANCED
        elif software_level == DifficultyLevel.INTERMEDIATE or hardware_level == DifficultyLevel.INTERMEDIATE:
            return DifficultyLevel.INTERMEDIATE
        else:
            return DifficultyLevel.BEGINNER

    def _map_level(self, level_str: str) -> DifficultyLevel:
        """Map string level to DifficultyLevel enum."""
        level_str = level_str.lower() if level_str else "beginner"
        if level_str == "advanced":
            return DifficultyLevel.ADVANCED
        elif level_str == "intermediate":
            return DifficultyLevel.INTERMEDIATE
        else:
            return DifficultyLevel.BEGINNER

    def personalize_text(self, text: str, user_profile: UserProfile) -> str:
        """Personalize text based on user profile."""
        difficulty = self.get_difficulty_level(user_profile)

        # Add difficulty-specific annotations to the text
        if difficulty == DifficultyLevel.BEGINNER:
            # For beginners, add more explanations and simpler terms
            return self._simplify_text(text)
        elif difficulty == DifficultyLevel.ADVANCED:
            # For advanced users, add more technical depth
            return self._add_technical_depth(text, user_profile)
        else:
            # For intermediate users, provide moderate detail
            return self._moderate_detail(text)

    def _simplify_text(self, text: str) -> str:
        """Simplify text for beginner users."""
        # This is a simplified implementation
        # In a real application, this would use NLP to simplify complex concepts
        return text

    def _add_technical_depth(self, text: str, user_profile: UserProfile) -> str:
        """Add technical depth for advanced users."""
        # Add language-specific examples if preferred languages are specified
        if user_profile.preferred_languages:
            text += f"\n\nImplementation examples in {user_profile.preferred_languages} are available in the advanced section."

        # Add research-level insights for advanced users
        text += "\n\nFor cutting-edge implementations, see recent research papers on this topic."
        return text

    def _moderate_detail(self, text: str) -> str:
        """Provide moderate detail for intermediate users."""
        # Add moderate technical details
        return text

    def personalize_chapter(self, chapter_id: str, user_profile: UserProfile) -> Dict[str, Any]:
        """Generate personalized chapter content."""
        difficulty = self.get_difficulty_level(user_profile)

        # Generate personalized content based on chapter and user profile
        personalization = {
            "chapter_id": chapter_id,
            "difficulty_level": difficulty.value,
            "learning_objectives": self._get_learning_objectives(difficulty, user_profile),
            "recommended_prerequisites": self._get_prerequisites(difficulty, user_profile),
            "content_modifications": self._get_content_modifications(difficulty, user_profile),
            "practice_exercises": self._get_exercises(difficulty, user_profile),
            "additional_resources": self._get_resources(difficulty, user_profile)
        }

        return personalization

    def _get_learning_objectives(self, difficulty: DifficultyLevel, user_profile: UserProfile) -> List[str]:
        """Generate learning objectives based on difficulty and profile."""
        objectives = []

        if difficulty == DifficultyLevel.BEGINNER:
            objectives = [
                "Understand fundamental concepts",
                "Learn basic terminology",
                "Practice simple examples"
            ]
        elif difficulty == DifficultyLevel.INTERMEDIATE:
            objectives = [
                "Apply concepts to real-world scenarios",
                "Implement moderate complexity solutions",
                "Understand core algorithms"
            ]
        else:  # ADVANCED
            objectives = [
                "Master advanced implementation techniques",
                "Design custom solutions",
                "Contribute to research-level problems"
            ]

        return objectives

    def _get_prerequisites(self, difficulty: DifficultyLevel, user_profile: UserProfile) -> List[str]:
        """Generate prerequisites based on difficulty and profile."""
        prerequisites = []

        if difficulty == DifficultyLevel.BEGINNER:
            prerequisites = ["Basic programming knowledge", "Fundamentals of robotics"]
        elif difficulty == DifficultyLevel.INTERMEDIATE:
            prerequisites = ["Basic robotics concepts", "Programming experience"]
        else:  # ADVANCED
            prerequisites = ["Intermediate robotics knowledge", "Advanced programming", "Mathematics background"]

        return prerequisites

    def _get_content_modifications(self, difficulty: DifficultyLevel, user_profile: UserProfile) -> Dict[str, Any]:
        """Generate content modifications based on difficulty and profile."""
        modifications = {
            "explanation_depth": "basic" if difficulty == DifficultyLevel.BEGINNER else
                               "detailed" if difficulty == DifficultyLevel.ADVANCED else "moderate",
            "example_complexity": "simple" if difficulty == DifficultyLevel.BEGINNER else
                                "complex" if difficulty == DifficultyLevel.ADVANCED else "moderate",
            "mathematical_formalism": "minimal" if difficulty == DifficultyLevel.BEGINNER else
                                    "extensive" if difficulty == DifficultyLevel.ADVANCED else "moderate"
        }

        # Add language-specific examples if preferred languages are specified
        if user_profile.preferred_languages:
            modifications["preferred_languages"] = user_profile.preferred_languages

        return modifications

    def _get_exercises(self, difficulty: DifficultyLevel, user_profile: UserProfile) -> List[Dict[str, str]]:
        """Generate practice exercises based on difficulty and profile."""
        exercises = []

        if difficulty == DifficultyLevel.BEGINNER:
            exercises = [
                {"type": "conceptual", "description": "Explain the basic concepts in your own words"},
                {"type": "practical", "description": "Complete the provided simple coding exercise"}
            ]
        elif difficulty == DifficultyLevel.INTERMEDIATE:
            exercises = [
                {"type": "implementation", "description": "Implement a moderate complexity solution"},
                {"type": "analysis", "description": "Analyze the provided example and identify key components"}
            ]
        else:  # ADVANCED
            exercises = [
                {"type": "research", "description": "Design a novel solution to an advanced problem"},
                {"type": "optimization", "description": "Optimize the provided implementation for performance"}
            ]

        return exercises

    def _get_resources(self, difficulty: DifficultyLevel, user_profile: UserProfile) -> List[str]:
        """Generate additional resources based on difficulty and profile."""
        resources = []

        if difficulty == DifficultyLevel.BEGINNER:
            resources = [
                "Introduction to Robotics textbook",
                "Basic Python for Robotics course",
                "YouTube tutorials for beginners"
            ]
        elif difficulty == DifficultyLevel.INTERMEDIATE:
            resources = [
                "Advanced Robotics textbook",
                "Research papers on the topic",
                "Online courses with hands-on projects"
            ]
        else:  # ADVANCED
            resources = [
                "Latest research papers",
                "Conference proceedings",
                "Advanced implementation guides",
                f"Resources in {user_profile.preferred_languages} if available"
            ]

        return resources

    def get_personalized_prompt(self, base_prompt: str, user_profile: UserProfile) -> str:
        """Generate a personalized prompt for the LLM based on user profile."""
        print(f"DEBUG: Getting personalized prompt for profile: {user_profile}")
        difficulty = self.get_difficulty_level(user_profile)
        print(f"DEBUG: Determined difficulty level: {difficulty}")

        # Create a personalized instruction prefix
        if difficulty == DifficultyLevel.BEGINNER:
            instruction = "Explain concepts in simple terms with analogies and basic examples."
        elif difficulty == DifficultyLevel.INTERMEDIATE:
            instruction = "Provide moderate technical detail with practical examples."
        else:  # ADVANCED
            instruction = "Provide in-depth technical information with advanced examples and implementation details."

        # Add language preferences if specified
        language_instruction = ""
        if user_profile.preferred_languages:
            language_instruction = f"Use examples and code in {user_profile.preferred_languages} when possible."

        # Add goal alignment if specified
        goal_instruction = ""
        if user_profile.goals:
            goal_instruction = f"Connect the explanation to the user's goal of: {user_profile.goals}."

        print(f"DEBUG: Instructions - Difficulty: {instruction}, Language: {language_instruction}, Goals: {goal_instruction}")

        # Combine all instructions
        personalized_prompt = f"""
        {base_prompt}

        Personalization Instructions:
        - {instruction}
        - {language_instruction}
        - {goal_instruction}

        Provide a response that matches the user's experience level and interests.
        """

        print(f"DEBUG: Final personalized prompt length: {len(personalized_prompt)}")
        return personalized_prompt.strip()


def get_personalization_service() -> PersonalizationService:
    """Get an instance of the personalization service."""
    return PersonalizationService()